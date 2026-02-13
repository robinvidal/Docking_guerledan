# =============================================================================
# SONAR_CARTESIAN_DISPLAY.PY - Affichage du sonar cartésien filtré
# =============================================================================
#
# DIFFÉRENCE AVEC SONAR_DISPLAY.PY :
# -----------------------------------
# - sonar_display.py : Reçoit des données POLAIRES et les convertit
# - CE FICHIER : Reçoit des données DÉJÀ CARTÉSIENNES (pré-traitées)
#
# Le traitement (conversion + filtres) est fait par un autre nœud ROS2,
# donc ici on affiche directement l'image reçue.
#
# FONCTIONNALITÉS SUPPLÉMENTAIRES :
# ----------------------------------
# 1. Affichage des lignes détectées (Hough Lines)
# 2. Affichage du tracker (bounding box CSRT)
# 3. Affichage de la pose de la cage (centre + orientation)
# 4. Sélection de bbox par l'utilisateur (pour initialiser le tracker)
# 5. Gestion des clics souris
#
# SIGNAUX PyQt PERSONNALISÉS :
# ----------------------------
# Ce widget émet un signal quand l'utilisateur sélectionne une zone :
#   bbox_selected = pyqtSignal(int, int, int, int)  # x, y, width, height
#
# =============================================================================

"""
Widget d'affichage pour données cartésiennes (FrameCartesian).
Affichage simple et direct de l'image cartésienne.
"""

import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import Qt, QRectF, pyqtSignal, QTimer


class SonarCartesianImageWidget(pg.PlotWidget):
    """
    Widget d'affichage pour les images cartésiennes du sonar.
    
    Hérite de pg.PlotWidget (pyqtgraph) pour avoir un graphique
    avec zoom, pan, axes, etc.
    
    Ce widget gère aussi l'interaction utilisateur :
    - Clic simple : affiche un marqueur temporaire
    - Mode sélection : permet de dessiner une bbox pour le tracker
    """
    
    # =========================================================================
    # SIGNAL PERSONNALISÉ
    # =========================================================================
    # On déclare notre propre signal pour communiquer avec l'extérieur.
    # Quand l'utilisateur sélectionne une zone, ce signal est émis
    # avec les coordonnées de la bounding box (en pixels).
    #
    # pyqtSignal(int, int, int, int) = 4 entiers: x, y, width, height
    # =========================================================================
    bbox_selected = pyqtSignal(int, int, int, int)

    def __init__(self, title="Sonar Cartésien"):
        """
        Initialise le widget d'affichage cartésien.
        
        STRUCTURE :
        On crée beaucoup d'"items" graphiques (lignes, marqueurs, rectangles)
        qui seront affichés ou cachés selon les données reçues.
        
        ZVALUE :
        setZValue(n) définit l'ordre d'empilement (comme z-index en CSS).
        Plus la valeur est haute, plus l'élément est "au-dessus".
        """
        super().__init__()
        
        # Configuration de base du graphique
        self.setTitle(title)
        self.setLabel('bottom', 'X (latéral, m)', units='m')  # Axe horizontal
        self.setLabel('left', 'Y (frontal, m)', units='m')    # Axe vertical
        self.setAspectLocked(True)  # Ratio 1:1 (pas de déformation)

        # =====================================================================
        # IMAGE PRINCIPALE - L'image sonar elle-même
        # =====================================================================
        self.image_item = pg.ImageItem()  # Item pour afficher des images 2D
        self.addItem(self.image_item)     # Ajoute au graphique

        # =====================================================================
        # LIGNES DE RÉFÉRENCE
        # =====================================================================
        
        # Ligne centrale (axe Y, vers l'avant)
        # pg.mkPen() crée un "stylo" avec couleur, épaisseur, style
        self.center_line = pg.PlotCurveItem(pen=pg.mkPen('w', width=0.5, style=Qt.DashLine))
        self.addItem(self.center_line)

        # Limites du champ de vision (FOV) - lignes dorées pointillées
        self.fov_left = pg.PlotCurveItem(pen=pg.mkPen('w', width=1, style=Qt.DashLine))
        self.fov_right = pg.PlotCurveItem(pen=pg.mkPen('w', width=1, style=Qt.DashLine))
        self.addItem(self.fov_left)
        self.addItem(self.fov_right)

        # Marqueur de position du ROV (triangle blanc en (0,0))
        self.rov_marker = pg.ScatterPlotItem(
            pos=[(0, 0)],        # Position
            size=20,              # Taille en pixels
            symbol='t',           # 't' = triangle
            pen=pg.mkPen('white', width=2),  # Contour blanc
            brush=pg.mkBrush(255, 255, 255, 100)  # Remplissage blanc semi-transparent
        )
        self.addItem(self.rov_marker)

        # =====================================================================
        # LIGNES HOUGH - Lignes détectées par l'algorithme de Hough
        # =====================================================================
        self.hough_lines = []  # Liste dynamique de PlotCurveItem
        
        # =====================================================================
        # TRACKER CSRT - Affichage de la bounding box du tracker
        # =====================================================================
        # Le tracker CSRT (un autre nœud ROS2) suit la cage et envoie
        # sa position. On l'affiche ici avec un rectangle vert (lime).
        
        # Rectangle de la bounding box
        self.tracker_bbox = pg.PlotCurveItem(
            pen=pg.mkPen('lime', width=3, style=Qt.SolidLine)  # Vert lime, trait plein
        )
        self.tracker_bbox.setZValue(90)  # Au-dessus de l'image, sous les contrôles
        self.addItem(self.tracker_bbox)
        self.tracker_bbox.hide()  # Caché par défaut (affiché quand tracking actif)
        
        # Côté d'entrée de la cage (la partie ouverte du U)
        self.tracker_entry_side = pg.PlotCurveItem(
            pen=pg.mkPen('orange', width=5, style=Qt.SolidLine)  # Orange épais
        )
        self.tracker_entry_side.setZValue(95)  # Au-dessus de la bbox
        self.addItem(self.tracker_entry_side)
        self.tracker_entry_side.hide()
        
        # Centre du tracker (croix +)
        self.tracker_center = pg.ScatterPlotItem(
            size=15, symbol='+', pen=pg.mkPen('lime', width=2), brush=None
        )
        self.tracker_center.setZValue(91)
        self.addItem(self.tracker_center)
        self.tracker_center.hide()
        
        # Texte affichant l'angle de rotation
        self.tracker_text = pg.TextItem(text='', color='lime', anchor=(0, 1))
        self.tracker_text.setZValue(92)
        self.addItem(self.tracker_text)
        self.tracker_text.hide()
        
        # Résolution pour conversion mètres <-> pixels
        self.current_resolution = 0.01  # m/pixel
        
        
        # =====================================================================
        # MODE SÉLECTION DE BBOX - Pour initialiser le tracker manuellement
        # =====================================================================
        # L'utilisateur peut activer ce mode via un bouton dans l'interface.
        # Une fois activé :
        # 1. L'image se met en pause
        # 2. L'utilisateur dessine un rectangle avec la souris
        # 3. Les coordonnées sont envoyées au tracker via un signal
        
        self.bbox_selection_mode = False       # Mode activé ou non
        self.bbox_start_point = None           # Point de départ du rectangle
        self.bbox_current_point = None         # Point actuel (pendant le dessin)
        
        # Rectangle de sélection (jaune pointillé)
        self.bbox_selection_rect = pg.PlotCurveItem(
            pen=pg.mkPen('yellow', width=3, style=Qt.DashLine)
        )
        self.bbox_selection_rect.setZValue(101)  # Tout en haut
        self.addItem(self.bbox_selection_rect)
        self.bbox_selection_rect.hide()
    

        # =====================================================================
        # CENTRE DU U DÉTECTÉ - Position et orientation de la cage
        # =====================================================================
        # Quand l'algorithme de détection trouve la cage en U,
        # il envoie sa position (x, y) et son orientation (quaternion).
        # On affiche :
        # 1. Une croix X au centre
        # 2. Une flèche montrant l'orientation (direction d'entrée)
        
        # Croix au centre du U (magenta)
        self.u_center = pg.ScatterPlotItem(
            size=18, symbol='x', pen=pg.mkPen('magenta', width=2), brush=None
        )
        self.u_center.setZValue(110)
        self.addItem(self.u_center)
        self.u_center.hide()

        # Ligne d'orientation (du centre vers l'avant)
        self.u_orientation_line = pg.PlotCurveItem(
            pen=pg.mkPen('magenta', width=3, style=Qt.SolidLine)
        )
        self.u_orientation_line.setZValue(110)
        self.addItem(self.u_orientation_line)
        self.u_orientation_line.hide()

        # Tête de flèche (triangle au bout de la ligne)
        self.u_orientation_head = pg.PlotCurveItem(
            pen=pg.mkPen('magenta', width=3, style=Qt.SolidLine)
        )
        self.u_orientation_head.setZValue(111)
        self.addItem(self.u_orientation_head)
        self.u_orientation_head.hide()
        
        # Variables pour la conversion pixel <-> mètre
        self.image_width = 0    # Largeur de l'image en pixels
        self.image_height = 0   # Hauteur de l'image en pixels
        self.origin_x = 0       # Colonne correspondant à X=0
        
        # =====================================================================
        # CONNEXION DES ÉVÉNEMENTS SOURIS
        # =====================================================================
        # pyqtgraph a ses propres signaux pour les événements souris.
        # On se connecte à la "scene" (la zone de dessin) pour écouter.
        # 
        # sigMouseClicked : émis quand l'utilisateur clique
        # sigMouseMoved : émis quand la souris bouge
        self.scene().sigMouseClicked.connect(self._on_mouse_clicked)
        self.scene().sigMouseMoved.connect(self._on_mouse_moved)
        
        # Affiche une grille en arrière-plan
        self.showGrid(x=True, y=True, alpha=0.3)

        # =====================================================================
        # COLORMAP PERSONNALISÉE - Style sonar (tons chauds)
        # =====================================================================
        # Même principe que dans sonar_display.py :
        # On convertit les intensités [0-255] en couleurs pour le rendu visuel
        positions = [0.0, 0.25, 0.5, 0.75, 1.0]
        colors = [
            (0, 0, 0),       # Sombre (faible écho)
            (80, 60, 20),      # Brun
            (180, 140, 50),    # Orange
            (230, 190, 80),    # Jaune-orange  
            (255, 230, 140),   # Jaune clair (fort écho)
        ]
        self.custom_colormap = pg.ColorMap(positions, colors)
        self._lut_positions = np.array(positions)
        self._lut_colors = np.array(colors, dtype=np.float32) / 255.0  # Normalise à [0,1]

    def update_cartesian_frame(self, frame_msg):
        """
        Met à jour l'affichage avec une nouvelle image cartésienne.
        
        DIFFÉRENCE avec sonar_display.py :
        Ici, l'image est DÉJÀ en coordonnées cartésiennes !
        Pas besoin de conversion polaire → cartésien.
        
        PROCESSUS :
        1. Vérifier si on est en pause
        2. Reconstruire l'image 2D depuis les données plates
        3. Appliquer la colormap
        4. Rotation pour pyqtgraph
        5. Positionner l'image dans le repère métrique
        
        Args:
            frame_msg: Message ROS2 de type FrameCartesian contenant:
                - intensities: liste 1D des pixels
                - width, height: dimensions de l'image
                - resolution: mètres par pixel
                - min_range, max_range: portée en mètres
        """
        # Stocker les dimensions pour la conversion pixel <-> mètre
        # (utilisé quand l'utilisateur sélectionne une zone)
        self.image_width = frame_msg.width
        self.image_height = frame_msg.height
        self.origin_x = frame_msg.origin_x  # Colonne correspondant à X=0
        self.current_resolution = frame_msg.resolution  # mètres/pixel
        
        # =====================================================================
        # RECONSTRUCTION DE L'IMAGE 2D
        # =====================================================================
        # Les données arrivent en 1D (liste plate), on les reshape en 2D
        img = np.array(frame_msg.intensities, dtype=np.uint8).reshape(
            (frame_msg.height, frame_msg.width)
        )
        
        # =====================================================================
        # APPLICATION DE LA COLORMAP
        # =====================================================================
        # Normalise [0-255] → [0-1]
        v = np.clip(img / 255.0, 0.0, 1.0)
        
        # Interpole pour chaque canal RGB
        r_chan = np.interp(v, self._lut_positions, self._lut_colors[:, 0])
        g_chan = np.interp(v, self._lut_positions, self._lut_colors[:, 1])
        b_chan = np.interp(v, self._lut_positions, self._lut_colors[:, 2])
        
        # Empile en image RGB
        rgb = np.stack((r_chan, g_chan, b_chan), axis=-1)
        rgb_uint8 = (rgb * 255).astype(np.uint8)
        
        # TRANSFORMATION T5 - Rotation 90° pour PyQtGraph
        # ================================================
        # Voir COORDINATE_TRANSFORMS.md pour les détails complets.
        #
        # Notre image cartésienne (après traitement):
        #   - Axe 0 (lignes) = Y: de ROV(0) à avant(max)
        #   - Axe 1 (colonnes) = X: de gauche(-) à droite(+)
        #   - Shape: (height, width) = (n_y, n_x)
        #
        # Convention PyQtGraph ImageItem:
        #   - Axe 0 de l'array numpy → direction X (horizontal)
        #   - Axe 1 de l'array numpy → direction Y (vertical)
        #
        # np.rot90(k=1) effectue une rotation antihoraire de 90°:
        #   - Ancien axe 0 (Y) devient axe 1 (vertical dans pyqtgraph = Y) ✓
        #   - Ancien axe 1 (X) devient axe 0 (horizontal dans pyqtgraph = X) ✓
        rgb_uint8 = np.rot90(rgb_uint8, k=1)
        
        # Mémoriser la résolution pour la conversion pixel/mètre
        self.current_resolution = frame_msg.resolution
        
        self.image_item.setImage(rgb_uint8, autoLevels=False)
        
        # Positionnement de l'image (identique à sonar_display.py)
        max_r = frame_msg.max_range
        min_r = frame_msg.min_range
        
        # Rectangle: (x, y, width, height) en mètres
        # x: de -max_r à +max_r (centré)
        # y: de 0 à max_r (vers l'avant)
        # width: 2 * max_r
        # height: max_r - min_r (zone visible du sonar)
        try:
            self.image_item.setRect(QRectF(-max_r, 0.0, 2.0 * max_r, max_r - min_r))
        except Exception:
            self.image_item.setPos(-max_r, 0.0)
            sx = (2.0 * max_r) / float(rgb_uint8.shape[1])
            sy = (max_r - min_r) / float(rgb_uint8.shape[0])
            self.image_item.resetTransform()
            self.image_item.scale(sx, sy)
        
        # Mise à jour des limites FOV
        try:
            half_angle = frame_msg.total_angle / 2.0
            x_left = max_r * np.sin(+half_angle)
            y_left = max_r * np.cos(+half_angle)
            x_right = max_r * np.sin(-half_angle)
            y_right = max_r * np.cos(-half_angle)
            self.fov_left.setData([0, x_left], [0, y_left])
            self.fov_right.setData([0, x_right], [0, y_right])
        except Exception:
            self.fov_left.setData([], [])
            self.fov_right.setData([], [])
        
        # Ligne centrale
        self.center_line.setData([0, 0], [0, max_r])
    
    def update_detected_lines(self, lines_msg):
        """
        Met à jour l'affichage des lignes détectées par l'algorithme de Hough.
        
        LIGNES HOUGH :
        L'algorithme de Hough détecte des lignes droites dans l'image.
        Utilisé ici pour détecter les bords de la cage en U.
        
        GESTION DYNAMIQUE :
        Contrairement aux autres items (créés une fois dans __init__),
        les lignes Hough sont dynamiques : leur nombre change à chaque frame.
        On doit donc supprimer les anciennes et recréer les nouvelles.
        
        Args:
            lines_msg: Message contenant les lignes détectées
                - num_lines: nombre de lignes
                - x1_points, y1_points: points de départ
                - x2_points, y2_points: points d'arrivée
                - confidences: scores de confiance [0-1]
        """
        # =====================================================================
        # NETTOYAGE DES ANCIENNES LIGNES
        # =====================================================================
        for line_item in self.hough_lines:
            self.removeItem(line_item)  # Supprime du graphique
        self.hough_lines.clear()  # Vide la liste
        
        # Vérifie si on a des lignes valides
        if not lines_msg or not lines_msg.is_valid or lines_msg.num_lines == 0:
            return
        
        # =====================================================================
        # DESSIN DES NOUVELLES LIGNES
        # =====================================================================
        for i in range(lines_msg.num_lines):
            # Coordonnées des extrémités de la ligne (en mètres)
            x1 = lines_msg.x1_points[i]
            y1 = lines_msg.y1_points[i]
            x2 = lines_msg.x2_points[i]
            y2 = lines_msg.y2_points[i]
            confidence = lines_msg.confidences[i]  # Score [0-1]
            
            # ================================================================
            # COULEUR EN FONCTION DE LA CONFIANCE
            # ================================================================
            # Haute confiance = vert, Basse confiance = jaune
            # On interpole entre jaune (255, 255, 0) et vert (0, 255, 0)
            red = int(255 * (1.0 - confidence))  # Rouge diminue avec confiance
            green = 255  # Toujours vert maximum
            blue = 0
            
            # Crée un item ligne avec la couleur calculée
            line_item = pg.PlotCurveItem(
                pen=pg.mkPen((red, green, blue), width=2, style=Qt.SolidLine)
            )
            line_item.setData([x1, x2], [y1, y2])  # Définit les points
            self.addItem(line_item)                 # Ajoute au graphique
            self.hough_lines.append(line_item)      # Garde en mémoire pour suppression

    def update_cage_pose(self, pose_msg):
        """
        Met à jour l'affichage de la pose de la cage (centre + orientation).
        
        POSE = Position + Orientation
        - Position : coordonnées (x, y) du centre de la cage
        - Orientation : quaternion (qx, qy, qz, qw) → on extrait le yaw (rotation Z)
        
        QUATERNION :
        C'est une représentation mathématique des rotations en 3D.
        Pour notre cas 2D, on a juste besoin du "yaw" (rotation autour de Z).
        
        Args:
            pose_msg: Message ROS2 de type PoseStamped
                - pose.position.x, y : centre de la cage
                - pose.orientation.x, y, z, w : quaternion
        """
        # Si pas de message, cache les éléments
        if pose_msg is None:
            self.u_center.hide()
            self.u_orientation_line.hide()
            self.u_orientation_head.hide()
            return

        # Extraction des données du message
        try:
            x = float(pose_msg.pose.position.x)
            y = float(pose_msg.pose.position.y)
            qx = float(pose_msg.pose.orientation.x)
            qy = float(pose_msg.pose.orientation.y)
            qz = float(pose_msg.pose.orientation.z)
            qw = float(pose_msg.pose.orientation.w)
        except Exception:
            return

        # =====================================================================
        # CONVERSION QUATERNION → YAW (angle en radians)
        # =====================================================================
        # Formule classique pour extraire le yaw d'un quaternion
        yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

        # Affiche le centre (croix X)
        self.u_center.setData(pos=[(x, y)])
        self.u_center.show()

        # =====================================================================
        # DESSIN DE LA FLÈCHE D'ORIENTATION
        # =====================================================================
        # La flèche pointe dans la direction d'entrée de la cage
        
        # Longueur de la flèche (proportionnelle à la cage)
        L = 0.8
        
        # Point d'arrivée de la flèche
        # ex = x + L * cos(yaw), ey = y + L * sin(yaw)
        ex = x + L * np.cos(yaw)
        ey = y + L * np.sin(yaw)
        
        # Trace la ligne du centre vers l'extrémité
        self.u_orientation_line.setData([x, ex], [y, ey])
        self.u_orientation_line.show()

        # =====================================================================
        # TÊTE DE FLÈCHE (triangle)
        # =====================================================================
        head_len = 0.08   # Longueur de la pointe
        head_w = 0.06     # Largeur de la base du triangle
        
        # Vecteur direction unitaire
        dx = np.cos(yaw)
        dy = np.sin(yaw)
        
        # Point de base de la tête (reculé de head_len)
        bx = ex - head_len * dx
        by = ey - head_len * dy
        
        # Vecteur perpendiculaire (pour les côtés du triangle)
        px = -dy
        py = dx
        
        # Coins gauche et droit du triangle
        left_x = bx + (head_w / 2.0) * px
        left_y = by + (head_w / 2.0) * py
        right_x = bx - (head_w / 2.0) * px
        right_y = by - (head_w / 2.0) * py

        # Dessine le triangle (fermé)
        tx = [ex, left_x, right_x, ex]
        ty = [ey, left_y, right_y, ey]
        self.u_orientation_head.setData(tx, ty)
        self.u_orientation_head.show()
    
    def _on_mouse_clicked(self, event):
        """
        Gère les clics souris sur le graphique.
        
        DEUX MODES :
        1. Mode sélection bbox : l'utilisateur dessine un rectangle
        2. Mode normal : affiche un marqueur temporaire
        
        CONVERSION DE COORDONNÉES :
        La souris est en coordonnées "scene" (pixels écran).
        On doit convertir en coordonnées "view" (mètres dans le graphique).
        
        Args:
            event: Événement de clic pyqtgraph
        """
        # Ignore les clics autres que le bouton gauche
        if event.button() != Qt.LeftButton:
            return
        
        # =====================================================================
        # CONVERSION COORDONNÉES SCENE → VIEW
        # =====================================================================
        # scenePos() = position en pixels sur l'écran
        # mapSceneToView() = convertit en coordonnées du graphique (mètres)
        pos = event.scenePos()
        mouse_point = self.plotItem.vb.mapSceneToView(pos)
        x_m = mouse_point.x()  # Coordonnée X en mètres
        y_m = mouse_point.y()  # Coordonnée Y en mètres
        
        # =====================================================================
        # MODE SÉLECTION BBOX
        # =====================================================================
        if self.bbox_selection_mode:
            if self.bbox_start_point is None:
                # PREMIER CLIC : début du rectangle
                self.bbox_start_point = (x_m, y_m)
                self.bbox_current_point = (x_m, y_m)
                self.bbox_selection_rect.show()
                self._update_bbox_selection_display()
            else:
                # DEUXIÈME CLIC : fin du rectangle
                self.bbox_current_point = (x_m, y_m)
                self._finalize_bbox_selection()  # Émet le signal
                self.set_bbox_selection_mode(False)  # Désactive le mode
            return

    
    def _on_mouse_moved(self, pos):
        """
        Gère le mouvement de la souris pendant la sélection de bbox.
        
        Permet de voir le rectangle se dessiner en temps réel
        pendant que l'utilisateur déplace la souris.
        
        Args:
            pos: Position de la souris (coordonnées scene)
        """
        # Ignore si pas en mode sélection ou si pas encore commencé
        if not self.bbox_selection_mode or self.bbox_start_point is None:
            return
        
        # Convertit en coordonnées du graphique
        mouse_point = self.plotItem.vb.mapSceneToView(pos)
        x_m = mouse_point.x()
        y_m = mouse_point.y()
        
        # Met à jour le point courant et redessine
        self.bbox_current_point = (x_m, y_m)
        self._update_bbox_selection_display()
    
    def set_bbox_selection_mode(self, enabled):
        """
        Active ou désactive le mode sélection de bounding box.
        
        QUAND ACTIVÉ :
        - L'image se met en pause (pour sélectionner sur une frame fixe)
        - Le curseur change (pas implémenté ici mais possible)
        - Le prochain clic démarre la sélection
        
        QUAND DÉSACTIVÉ :
        - L'image reprend sa mise à jour normale
        - Les éléments de sélection sont cachés
        
        Args:
            enabled: True pour activer, False pour désactiver
        """
        self.bbox_selection_mode = enabled
        
        if enabled:
            # Pause et prépare la sélection
            self.is_paused = True           # Arrête les mises à jour
            self.bbox_start_point = None    # Réinitialise
            self.bbox_current_point = None
        else:
            # Désactive et nettoie
            self.is_paused = False           # Reprend les mises à jour
            self.bbox_start_point = None     # Réinitialise
            self.bbox_current_point = None
            self.bbox_selection_rect.hide()  # Cache le rectangle jaune
    
    def _update_bbox_selection_display(self):
        """
        Met à jour l'affichage du rectangle de sélection jaune.
        
        Appelée pendant le mouvement de la souris pour montrer
        le rectangle en cours de dessin.
        """
        if self.bbox_start_point is None or self.bbox_current_point is None:
            return
        
        # Récupère les deux coins
        x1, y1 = self.bbox_start_point
        x2, y2 = self.bbox_current_point
        
        # Dessine le rectangle (5 points pour fermer)
        x_coords = [x1, x2, x2, x1, x1]
        y_coords = [y1, y1, y2, y2, y1]
        self.bbox_selection_rect.setData(x_coords, y_coords)
    
    def _finalize_bbox_selection(self):
        """
        Finalise la sélection et émet le signal avec les coordonnées en pixels.
        
        CONVERSION MÈTRES → PIXELS :
        Le rectangle est dessiné en mètres (repère PyQtGraph),
        mais le tracker travaille en pixels (repère image).
        On doit donc convertir les coordonnées.
        
        FORMULES :
        - x_px = origin_x - (x_m / resolution)  ← X est INVERSÉ
        - y_px = y_m / resolution
        """
        if self.bbox_start_point is None or self.bbox_current_point is None:
            return
        
        # Coordonnées en mètres
        x1_m, y1_m = self.bbox_start_point
        x2_m, y2_m = self.bbox_current_point
        
        # =====================================================================
        # CONVERSION MÈTRES → PIXELS
        # =====================================================================
        # Le repère image est différent du repère PyQtGraph :
        # - Image : origine en haut-gauche, Y vers le bas
        # - PyQtGraph : origine au ROV, Y vers l'avant
        #
        # origin_x = colonne centrale (où X=0 en mètres)
        # resolution = mètres par pixel
        
        x1_px = int(self.origin_x - (x1_m / self.current_resolution))
        y1_px = int(y1_m / self.current_resolution)
        x2_px = int(self.origin_x - (x2_m / self.current_resolution))
        y2_px = int(y2_m / self.current_resolution)
        
        # Normalise : bbox = (x, y, width, height) avec x,y = coin supérieur gauche
        bbox_x = min(x1_px, x2_px)
        bbox_y = min(y1_px, y2_px)
        bbox_w = abs(x2_px - x1_px)
        bbox_h = abs(y2_px - y1_px)
        
        # =====================================================================
        # ÉMISSION DU SIGNAL
        # =====================================================================
        # Le signal bbox_selected transporte les 4 entiers vers le slot connecté
        # (dans main_window.py → on_bbox_selected → ros_node.publish_bbox_selection)
        self.bbox_selected.emit(bbox_x, bbox_y, bbox_w, bbox_h)
        
        # Réinitialise pour la prochaine sélection
        self.bbox_start_point = None
        self.bbox_current_point = None
    

    def update_tracked_object(self, tracked_msg):
        """
        Met à jour l'affichage de la bounding box du tracker CSRT.
        
        LE TRACKER CSRT :
        C'est un algorithme de suivi d'objet (OpenCV).
        Une fois initialisé avec une bbox, il suit l'objet frame par frame.
        
        BBOX ORIENTÉE :
        Si l'objet est tourné, on dessine un rectangle orienté
        (pas aligné avec les axes). Cela nécessite de calculer
        les 4 coins avec une rotation.
        
        Args:
            tracked_msg: Message contenant:
                - is_tracking: True si le tracker est actif
                - center_x, center_y: centre en mètres
                - width, height: dimensions en mètres
                - angle: rotation en radians
                - entry_p1_x/y, entry_p2_x/y: côté d'entrée
        """
        # Si pas de tracking, cache tout
        if not tracked_msg.is_tracking or not tracked_msg.is_initialized:
            self.tracker_bbox.hide()
            self.tracker_entry_side.hide()
            self.tracker_center.hide()
            self.tracker_text.hide()
            return
        
        # Position et dimensions
        cx = tracked_msg.center_x  # Centre X en mètres
        cy = tracked_msg.center_y  # Centre Y en mètres
        half_w = tracked_msg.width / 2.0
        half_h = tracked_msg.height / 2.0
        angle = tracked_msg.angle  # Angle en radians
        
        # =====================================================================
        # DESSIN DE LA BBOX (orientée si angle != 0)
        # =====================================================================
        if abs(angle) > 0.01:  # Si angle significatif (> ~0.5°)
            # =================================================================
            # CALCUL DES 4 COINS DU RECTANGLE ORIENTÉ
            # =================================================================
            # On définit les coins dans le repère LOCAL (centré, non tourné),
            # puis on applique une rotation, puis on translate au centre.
            
            # Coins dans le repère local (origine au centre)
            corners_local = np.array([
                [-half_w, -half_h],  # Coin inférieur gauche
                [+half_w, -half_h],  # Coin inférieur droit
                [+half_w, +half_h],  # Coin supérieur droit
                [-half_w, +half_h],  # Coin supérieur gauche
            ])
            
            # Matrice de rotation 2D
            cos_a = np.cos(angle)
            sin_a = np.sin(angle)
            rot_matrix = np.array([
                [cos_a, -sin_a],
                [sin_a,  cos_a]
            ])
            
            # Applique la rotation : corners_rotated = corners_local @ R^T
            corners_rotated = corners_local @ rot_matrix.T
            
            # Translate au centre (en mètres)
            corners_world = corners_rotated + np.array([cx, cy])
            
            # Ferme le rectangle (5ème point = 1er point)
            x_coords = np.append(corners_world[:, 0], corners_world[0, 0])
            y_coords = np.append(corners_world[:, 1], corners_world[0, 1])
            
            self.tracker_bbox.setData(x_coords, y_coords)
            self.tracker_bbox.show()
            
            # Dessine le côté d'entrée (ouverture du U) en orange
            if abs(tracked_msg.entry_p1_x) > 0.001 or abs(tracked_msg.entry_p1_y) > 0.001:
                entry_x = [tracked_msg.entry_p1_x, tracked_msg.entry_p2_x]
                entry_y = [tracked_msg.entry_p1_y, tracked_msg.entry_p2_y]
                self.tracker_entry_side.setData(entry_x, entry_y)
                self.tracker_entry_side.show()
            else:
                self.tracker_entry_side.hide()
        else:
            # =================================================================
            # RECTANGLE NON ORIENTÉ (angle ~ 0)
            # =================================================================
            # Plus simple : juste les 4 coins alignés avec les axes
            x_coords = [cx - half_w, cx + half_w, cx + half_w, cx - half_w, cx - half_w]
            y_coords = [cy - half_h, cy - half_h, cy + half_h, cy + half_h, cy - half_h]
            
            self.tracker_bbox.setData(x_coords, y_coords)
            self.tracker_bbox.show()
            self.tracker_entry_side.hide()
        
        # =====================================================================
        # AFFICHAGE DU CENTRE (croix +) ET DE L'ANGLE
        # =====================================================================
        self.tracker_center.setData(pos=[(cx, cy)])
        self.tracker_center.show()
        
        # Affiche l'angle en degrés si non nul
        if abs(angle) > 0.01:
            angle_deg = np.degrees(angle)  # Convertit radians → degrés
            self.tracker_text.setText(f'θ = {angle_deg:.1f}°')
            self.tracker_text.setPos(cx, cy + half_h + 0.2)  # Au-dessus de la bbox
            self.tracker_text.show()
        else:
            self.tracker_text.hide()
