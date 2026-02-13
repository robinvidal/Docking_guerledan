# =============================================================================
# SONAR_DISPLAY.PY - Affichage du sonar brut (polaire → cartésien)
# =============================================================================
#
# CE QUE FAIT CE FICHIER :
# ------------------------
# Reçoit des données sonar POLAIRES (rayon, angle) et les convertit
# en image CARTÉSIENNE (X, Y) pour l'affichage.
#
# REPRÉSENTATION DES DONNÉES SONAR :
# ----------------------------------
#     POLAIRE (ce que le sonar envoie)      CARTÉSIEN (ce qu'on affiche)
#              
#              ↑ bearing (angle)               Y (mètres)
#              │                                      
#       ┌──────┴──────┐                           │   
#       │   Image     │   range                   │  
#       │  polaire    │   (distance)       ───────┼──────► X
#       │ (bearing,   │                           │   
#       │  range)     │                           │    
#       └─────────────┘                     ROV ici (0,0)
#
# PYQTGRAPH :
# -----------
# On utilise pyqtgraph (alias "pg") plutôt que matplotlib car :
# - Beaucoup plus rapide pour les mises à jour en temps réel
# - Optimisé pour les données scientifiques
# - S'intègre bien avec Qt
#
# =============================================================================

import numpy as np
import pyqtgraph as pg          # Bibliothèque de graphiques rapides
from PyQt5.QtCore import Qt, QRectF  # QRectF = rectangle avec coordonnées float
from scipy.ndimage import map_coordinates  # Pour la conversion polaire → cartésien


class SonarCartesianWidget(pg.PlotWidget):
    """
    Widget d'affichage 2D pour le sonar brut.
    
    Hérite de pg.PlotWidget (pas QWidget directement) car pyqtgraph
    fournit des widgets spécialisés pour les graphiques.
    
    PlotWidget = un graphique avec axes, grille, zoom, pan, etc.
    """

    def __init__(self, title="Sonar"):
        """
        Initialise le widget d'affichage sonar.
        
        Args:
            title: Titre affiché en haut du graphique
        """
        super().__init__()
        
        # Configuration du graphique
        self.setTitle(title)
        self.setLabel('bottom', 'X (latéral, m)', units='m')  # Axe X en bas
        self.setLabel('left', 'Y (frontal, m)', units='m')    # Axe Y à gauche
        self.setAspectLocked(True)  # Garde le ratio 1:1 (pas de déformation)

        # =====================================================================
        # ITEMS GRAPHIQUES - Chaque élément visuel est un "Item"
        # =====================================================================
        # Dans pyqtgraph, on ajoute des "items" au graphique :
        # - ScatterPlotItem : nuage de points
        # - ImageItem : image 2D
        # - PlotCurveItem : ligne/courbe
        # =====================================================================
        
        # Nuage de points (utilisé si use_image=False)
        self.scatter = pg.ScatterPlotItem(size=3, pen=pg.mkPen(None))  # Points de 3px, sans contour
        self.addItem(self.scatter)  # Ajoute au graphique

        # Image 2D (utilisée si use_image=True) - plus rapide que les points
        self.image_item = pg.ImageItem()
        self.addItem(self.image_item)
        self.image_item.setZValue(-10)  # ZValue = ordre d'empilement (négatif = derrière)
        self.image_item.hide()  # Cachée par défaut

        # Ligne centrale (axe Y, vers l'avant du ROV)
        # mkPen() crée un stylo: couleur 'w' (blanc), largeur 1, style pointillé
        self.center_line = pg.PlotCurveItem(pen=pg.mkPen('w', width=0.5, style=Qt.DashLine))
        self.addItem(self.center_line)

        # Limites du champ de vision (FOV = Field Of View)
        # Lignes cyan pointillées qui montrent l'angle de vision du sonar
        self.fov_left = pg.PlotCurveItem(pen=pg.mkPen('w', width=1, style=Qt.DashLine))
        self.fov_right = pg.PlotCurveItem(pen=pg.mkPen('w', width=1, style=Qt.DashLine))
        self.addItem(self.fov_left)
        self.addItem(self.fov_right)

        # Marqueur de position du ROV (triangle blanc au point (0,0))
        # symbol='t' = triangle, brush = remplissage semi-transparent
        self.rov_marker = pg.ScatterPlotItem(
            pos=[(0, 0)], size=20, symbol='t', 
            pen=pg.mkPen('w', width=2), 
            brush=pg.mkBrush(255, 255, 255, 100)  # RGBA: blanc semi-transparent
        )
        self.addItem(self.rov_marker)

        # Affiche une grille en arrière-plan (alpha = transparence)
        self.showGrid(x=True, y=True, alpha=0.3)

        # =====================================================================
        # COLORMAP - Table de couleurs pour l'intensité sonar
        # =====================================================================
        # Les données sonar sont des intensités 0-255 (niveau de gris).
        # On les convertit en couleurs pour un meilleur rendu visuel.
        #
        # positions: valeurs d'entrée normalisées [0.0 à 1.0]
        # colors: couleurs RGB correspondantes
        # =====================================================================
        positions = [0.0, 0.25, 0.5, 0.75, 1.0]
        colors = [
            (0, 0, 0),       # Très sombre (faible écho)
            (80, 60, 20),      # Brun foncé
            (180, 140, 50),    # Orange
            (230, 190, 80),    # Jaune-orange
            (255, 230, 140),   # Jaune clair (fort écho)
        ]
        self.custom_colormap = pg.ColorMap(positions, colors)
        
        # Pré-calcul pour interpolation rapide
        self._lut_positions = np.array(positions)
        self._lut_colors = np.array(colors, dtype=np.float32) / 255.0  # Normalise à [0,1]

        # =====================================================================
        # CACHE pour la conversion polaire → cartésien
        # =====================================================================
        # La conversion est coûteuse, on garde les coordonnées en cache
        # tant que les dimensions ne changent pas
        self._mapping_cache = {
            'bearing_count': None,  # Nombre d'angles
            'range_count': None,    # Nombre de distances
            'coords': None,         # Coordonnées de mapping pré-calculées
            'out_shape': None,      # Taille de l'image de sortie
        }
        
        self.use_image = True      # True = afficher comme image, False = nuage de points
        self.image_rotation = 1    # Rotation de l'image (k*90°)

    def update_image(self, frame_msg):
        """
        Met à jour l'affichage avec une nouvelle frame sonar.
        
        PROCESSUS:
        1. Reconstruire l'image polaire depuis les données plates
        2. Calculer les angles (bearings)
        3. Convertir polaire → cartésien
        4. Appliquer la colormap
        5. Afficher l'image
        
        Args:
            frame_msg: Message ROS2 de type Frame contenant:
                - intensities: liste 1D des intensités [0-255]
                - bearing_count: nombre d'angles
                - range_count: nombre de distances
                - min_range, max_range: portée en mètres
                - bearing_resolution: résolution angulaire
        """
        # =====================================================================
        # ÉTAPE 1: Reconstruire l'image polaire 2D
        # =====================================================================
        # Les données arrivent en 1D (liste plate), on les reshape en 2D
        # Shape: (bearing_count, range_count) = (angles, distances)
        img = np.array(frame_msg.intensities, dtype=np.uint8).reshape(
            (frame_msg.bearing_count, frame_msg.range_count)
        )
        
        # Calcul des distances pour chaque "bin" de range
        ranges = np.linspace(frame_msg.min_range, frame_msg.max_range, frame_msg.range_count)
        
        # Calcul de l'angle total du champ de vision
        total_angle = frame_msg.bearing_resolution * frame_msg.bearing_count
        
        # =====================================================================
        # ÉTAPE 2: Calculer les angles (bearings)
        # =====================================================================
        # INVERSION pour que gauche = gauche à l'écran
        # bearing[0] = -FOV/2 (gauche), bearing[N] = +FOV/2 (droite)
        bearings = -np.linspace(-total_angle / 2, total_angle / 2, frame_msg.bearing_count)

        # =====================================================================
        # ÉTAPE 3: Mettre à jour les lignes FOV
        # =====================================================================
        try:
            half_angle = total_angle / 2.0
            max_r = frame_msg.max_range
            
            # Calcul des extrémités des lignes FOV
            # x = r * sin(θ), y = r * cos(θ)  (conversion polaire → cartésien)
            x_left = max_r * np.sin(+half_angle)
            y_left = max_r * np.cos(+half_angle)
            x_right = max_r * np.sin(-half_angle)
            y_right = max_r * np.cos(-half_angle)
            
            # Trace les lignes de (0,0) jusqu'aux extrémités
            self.fov_left.setData([0, x_left], [0, y_left])
            self.fov_right.setData([0, x_right], [0, y_right])
            self.center_line.setData([0, 0], [0, max_r])  # Ligne verticale de (0,0) à (0,max_r)

        except Exception:
            # En cas d'erreur, cache les lignes
            self.fov_left.setData([], [])
            self.fov_right.setData([], [])
            self.center_line.setData([], [])

        # =====================================================================
        # ÉTAPE 4: Conversion POLAIRE → CARTÉSIEN (mode image)
        # =====================================================================
        # C'est la partie la plus complexe !
        # On crée une grille cartésienne et on interpole les valeurs polaires.
        if self.use_image:
            bc = frame_msg.bearing_count   # Nombre d'angles
            rc = frame_msg.range_count     # Nombre de distances
            max_r = frame_msg.max_range
            min_r = frame_msg.min_range
            total_angle = frame_msg.bearing_resolution * bc

            # Vérifie si on peut réutiliser le cache
            cache = self._mapping_cache
            if cache['bearing_count'] != bc or cache['range_count'] != rc:
                # ============================================================
                # CALCUL DES COORDONNÉES DE MAPPING (coûteux, fait une fois)
                # ============================================================
                # On crée une grille de pixels cartésiens,
                # et pour chaque pixel, on calcule d'où il vient en polaire.
                
                out_h = rc              # Hauteur de l'image de sortie
                out_w = int(2 * rc)     # Largeur = 2x hauteur (pour couvrir -max_r à +max_r)

                # Crée les grilles X et Y
                xs = np.linspace(-max_r, max_r, out_w)  # X: de -max_r à +max_r
                ys = np.linspace(0, max_r, out_h)        # Y: de 0 à max_r
                xv, yv = np.meshgrid(xs, ys)  # Grille 2D de toutes les combinaisons

                # Conversion cartésien → polaire (pour savoir où sampler)
                rr = np.sqrt(xv**2 + yv**2)      # Distance: r = √(x² + y²)
                th = np.arctan2(-xv, yv)          # Angle: θ = atan2(-x, y)

                # Conversion en indices dans l'image polaire
                # i_float = index du bearing (angle)
                # j_float = index du range (distance)
                i_float = (th + total_angle / 2.0) / total_angle * (bc - 1)
                j_float = (rr - min_r) / (max_r - min_r) * (rc - 1)

                # Empile les coordonnées pour map_coordinates
                coords = np.vstack((i_float.ravel(), j_float.ravel()))

                # Sauvegarde dans le cache
                cache['bearing_count'] = bc
                cache['range_count'] = rc
                cache['coords'] = coords
                cache['out_shape'] = (out_h, out_w)
            else:
                # Réutilise le cache
                coords = cache['coords']
                out_h, out_w = cache['out_shape']

            # =================================================================
            # INTERPOLATION - Échantillonne l'image polaire aux coordonnées calculées
            # =================================================================
            # map_coordinates() interpole les valeurs de l'image polaire
            # aux positions (i, j) calculées ci-dessus
            sampled = map_coordinates(
                img.astype(np.float32),  # Image source (polaire)
                coords,                   # Coordonnées où échantillonner
                order=1,                  # Interpolation linéaire
                mode='constant',          # Valeur constante hors limites
                cval=0.0                  # Valeur = 0 hors limites
            )
            sampled = sampled.reshape((out_h, out_w))  # Reshape en image 2D

            # =================================================================
            # APPLICATION DE LA COLORMAP
            # =================================================================
            # Normalise les intensités [0-255] → [0-1]
            v = np.clip(sampled / 255.0, 0.0, 1.0)
            
            # Interpole pour chaque canal RGB
            r_chan = np.interp(v, self._lut_positions, self._lut_colors[:, 0])
            g_chan = np.interp(v, self._lut_positions, self._lut_colors[:, 1])
            b_chan = np.interp(v, self._lut_positions, self._lut_colors[:, 2])

            # Empile les canaux en image RGB
            rgb = np.stack((r_chan, g_chan, b_chan), axis=-1)
            rgb_uint8 = (rgb * 255).astype(np.uint8)

            # Rotation pour aligner avec les axes PyQtGraph
            if self.image_rotation and self.image_rotation % 4 != 0:
                rgb_uint8 = np.rot90(rgb_uint8, k=self.image_rotation)

            # =================================================================
            # AFFICHAGE DE L'IMAGE
            # =================================================================
            self.image_item.setImage(rgb_uint8, autoLevels=False)
            
            # Positionne l'image dans le repère métrique
            # QRectF(x, y, width, height) = rectangle flottant
            try:
                self.image_item.setRect(QRectF(-max_r, 0.0, 2.0 * max_r, max_r - min_r))
            except Exception:
                # Fallback avec transform manuelle
                self.image_item.setPos(-max_r, 0.0)
                sx = (2.0 * max_r) / float(rgb_uint8.shape[1])  # Échelle X
                sy = (max_r - min_r) / float(rgb_uint8.shape[0])  # Échelle Y
                self.image_item.resetTransform()
                self.image_item.scale(sx, sy)
                
            self.image_item.show()
            self.scatter.hide()  # Cache le nuage de points
        else:
            # =================================================================
            # MODE NUAGE DE POINTS (plus lent mais parfois utile)
            # =================================================================
            # Au lieu d'une image, on affiche chaque écho comme un point coloré
            points = []
            intensities = []
            step = 2  # Sous-échantillonnage pour performance
            
            for i in range(0, frame_msg.bearing_count, step):
                for j in range(0, frame_msg.range_count, step):
                    intensity = img[i, j]
                    
                    # Ignore les échos faibles (seuil à 30)
                    if intensity > 30:
                        r_val = ranges[j]      # Distance
                        theta_val = bearings[i]  # Angle
                        
                        # Conversion polaire → cartésien
                        x = r_val * np.sin(theta_val)
                        y = r_val * np.cos(theta_val)
                        
                        points.append([x, y])
                        intensities.append(intensity)

            # Affiche les points avec des couleurs selon l'intensité
            if points:
                points = np.array(points)
                intensities = np.array(intensities)
                
                # Convertit les intensités en couleurs via la colormap
                colors = self.custom_colormap.mapToQColor(intensities / 255.0)
                brushes = [pg.mkBrush(c) for c in colors]
                
                self.scatter.setData(pos=points, brush=brushes)
            else:
                self.scatter.setData([], [])
