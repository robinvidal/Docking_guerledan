"""
Widget d'affichage pour données cartésiennes (FrameCartesian).
Affichage simple et direct de l'image cartésienne.
"""

import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import Qt, QRectF, pyqtSignal, QTimer


class SonarCartesianImageWidget(pg.PlotWidget):
    """Vue cartésienne pour FrameCartesian avec affichage image direct."""
    
    # Signal émis lors d'un clic (x_m, y_m en mètres)
    click_position = pyqtSignal(float, float)
    
    # Signal émis lors de la sélection d'une bbox (x, y, width, height en pixels)
    bbox_selected = pyqtSignal(int, int, int, int)
    
    # Signal émis lors de la sélection rotatif (4 coins en mètres: p1_x, p1_y, p2_x, p2_y, p3_x, p3_y, p4_x, p4_y)
    rotated_bbox_selected = pyqtSignal(float, float, float, float, float, float, float, float)

    def __init__(self, title="Sonar Cartésien"):
        super().__init__()
        self.setTitle(title)
        self.setLabel('bottom', 'X (latéral, m)', units='m')
        self.setLabel('left', 'Y (frontal, m)', units='m')
        self.setAspectLocked(True)

        self.image_item = pg.ImageItem()
        self.addItem(self.image_item)

        self.borders_scatter = pg.ScatterPlotItem(
            size=15, pen=pg.mkPen(None), brush=pg.mkBrush(255, 0, 0, 255)
        )
        self.addItem(self.borders_scatter)

        self.center_line = pg.PlotCurveItem(pen=pg.mkPen('w', width=1, style=Qt.DashLine))
        self.addItem(self.center_line)

        # FOV boundaries
        self.fov_left = pg.PlotCurveItem(pen=pg.mkPen('c', width=1, style=Qt.DashLine))
        self.fov_right = pg.PlotCurveItem(pen=pg.mkPen('c', width=1, style=Qt.DashLine))
        self.addItem(self.fov_left)
        self.addItem(self.fov_right)

        self.rov_marker = pg.ScatterPlotItem(
            pos=[(0, 0)], size=20, symbol='t', 
            pen=pg.mkPen('g', width=2), brush=pg.mkBrush(0, 255, 0, 100)
        )
        self.addItem(self.rov_marker)

        # Lignes détectées par Hough
        self.hough_lines = []  # Liste de PlotCurveItem
        
        # Bounding box du tracker CSRT (rectangle)
        self.tracker_bbox = pg.PlotCurveItem(
            pen=pg.mkPen('lime', width=3, style=Qt.SolidLine)
        )
        self.tracker_bbox.setZValue(90)
        self.addItem(self.tracker_bbox)
        self.tracker_bbox.hide()
        
        # Côté d'entrée du tracker (en rouge/orange)
        self.tracker_entry_side = pg.PlotCurveItem(
            pen=pg.mkPen('orange', width=5, style=Qt.SolidLine)
        )
        self.tracker_entry_side.setZValue(95)  # Au dessus de la bbox
        self.addItem(self.tracker_entry_side)
        self.tracker_entry_side.hide()
        
        # Centre du tracker (croix)
        self.tracker_center = pg.ScatterPlotItem(
            size=15, symbol='+', pen=pg.mkPen('lime', width=2), brush=None
        )
        self.tracker_center.setZValue(91)
        self.addItem(self.tracker_center)
        self.tracker_center.hide()
        
        # Texte de confiance du tracker
        self.tracker_text = pg.TextItem(text='', color='lime', anchor=(0, 1))
        self.tracker_text.setZValue(92)
        self.addItem(self.tracker_text)
        self.tracker_text.hide()
        
        # Stocker la résolution pour la conversion
        self.current_resolution = 0.01  # Valeur par défaut
        
        # Marker de clic (point rouge temporaire)
        self.click_marker = pg.ScatterPlotItem(
            size=20, pen=pg.mkPen('r', width=2), brush=pg.mkBrush(255, 0, 0, 200)
        )
        self.click_marker.setZValue(100)  # Au dessus de tout
        self.addItem(self.click_marker)
        self.click_marker.hide()
        
        # Bounding box de clic (rectangle rouge temporaire, taille de la cage estimée)
        self.click_bbox = pg.PlotCurveItem(
            pen=pg.mkPen('r', width=2, style=Qt.SolidLine)
        )
        self.click_bbox.setZValue(99)
        self.addItem(self.click_bbox)
        self.click_bbox.hide()
        
        # Dimensions estimées de la cage (mètres) - synchronisé avec tracking_params.yaml
        self.cage_width = 0.9
        self.cage_height = 0.5
        
        # Timer pour masquer le marker après 1 seconde
        self.click_timer = QTimer()
        self.click_timer.setSingleShot(True)
        self.click_timer.timeout.connect(self._hide_click_marker)
        
        # Mode de sélection de bbox (activé via bouton dans le panneau de contrôle)
        self.bbox_selection_mode = False
        self.bbox_start_point = None
        self.bbox_current_point = None
        self.bbox_selection_rect = pg.PlotCurveItem(
            pen=pg.mkPen('yellow', width=3, style=Qt.DashLine)
        )
        self.bbox_selection_rect.setZValue(101)
        self.addItem(self.bbox_selection_rect)
        self.bbox_selection_rect.hide()
        
        # Mode de sélection rotatif (3 points)
        self.rotated_selection_mode = False
        self.rotated_points = []  # Liste de 3 points (x_m, y_m)
        self.rotated_markers = pg.ScatterPlotItem(
            size=25, pen=pg.mkPen('b', width=3), brush=pg.mkBrush(0, 0, 255, 150)
        )
        self.rotated_markers.setZValue(102)
        self.addItem(self.rotated_markers)
        self.rotated_markers.hide()
        
        # Lignes entre les points (preview du rectangle)
        self.rotated_preview = pg.PlotCurveItem(
            pen=pg.mkPen('cyan', width=3, style=Qt.SolidLine)
        )
        self.rotated_preview.setZValue(101)
        self.addItem(self.rotated_preview)
        self.rotated_preview.hide()
        
        # Pause pour la sélection
        self.is_paused = False
        self.paused_frame = None

        # Centre du U détecté et orientation
        self.u_center = pg.ScatterPlotItem(
            size=18, symbol='x', pen=pg.mkPen('magenta', width=2), brush=None
        )
        self.u_center.setZValue(110)
        self.addItem(self.u_center)
        self.u_center.hide()

        self.u_orientation_line = pg.PlotCurveItem(
            pen=pg.mkPen('magenta', width=3, style=Qt.SolidLine)
        )
        self.u_orientation_line.setZValue(110)
        self.addItem(self.u_orientation_line)
        self.u_orientation_line.hide()

        self.u_orientation_head = pg.PlotCurveItem(
            pen=pg.mkPen('magenta', width=3, style=Qt.SolidLine)
        )
        self.u_orientation_head.setZValue(111)
        self.addItem(self.u_orientation_head)
        self.u_orientation_head.hide()
        
        # Stocker les dimensions de l'image pour conversion pixels
        self.image_width = 0
        self.image_height = 0
        self.origin_x = 0
        
        # Activer les événements de clic et drag
        self.scene().sigMouseClicked.connect(self._on_mouse_clicked)
        self.scene().sigMouseMoved.connect(self._on_mouse_moved)
        
        self.showGrid(x=True, y=True, alpha=0.3)

        # Colormap personnalisée (style sonar)
        positions = [0.0, 0.25, 0.5, 0.75, 1.0]
        colors = [
            (15, 10, 5),
            (80, 60, 20),
            (180, 140, 50),
            (230, 190, 80),
            (255, 230, 140),
        ]
        self.custom_colormap = pg.ColorMap(positions, colors)
        self._lut_positions = np.array(positions)
        self._lut_colors = np.array(colors, dtype=np.float32) / 255.0

    def update_cartesian_frame(self, frame_msg):
        """Met à jour l'affichage avec un message FrameCartesian."""
        # Si en pause, ne pas mettre à jour
        if self.is_paused:
            return
        
        # Stocker les dimensions pour la conversion
        self.image_width = frame_msg.width
        self.image_height = frame_msg.height
        self.origin_x = frame_msg.origin_x
        self.current_resolution = frame_msg.resolution
        
        # Reconstruction de l'image cartésienne
        img = np.array(frame_msg.intensities, dtype=np.uint8).reshape(
            (frame_msg.height, frame_msg.width)
        )
        
        # Application du colormap
        v = np.clip(img / 255.0, 0.0, 1.0)
        r_chan = np.interp(v, self._lut_positions, self._lut_colors[:, 0])
        g_chan = np.interp(v, self._lut_positions, self._lut_colors[:, 1])
        b_chan = np.interp(v, self._lut_positions, self._lut_colors[:, 2])
        
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

    def update_borders(self, borders_msg):
        """Met à jour l'affichage des bordures."""
        if not borders_msg or not borders_msg.is_valid:
            self.borders_scatter.setData([], [])
            return

        points = []
        for r_val, theta in zip(borders_msg.ranges, borders_msg.bearings):
            x = r_val * np.sin(theta)
            y = r_val * np.cos(theta)
            points.append([x, y])

        if points:
            self.borders_scatter.setData(pos=np.array(points))
    
    def update_detected_lines(self, lines_msg):
        """Met à jour l'affichage des lignes détectées par Hough."""
        # Nettoyer les anciennes lignes
        for line_item in self.hough_lines:
            self.removeItem(line_item)
        self.hough_lines.clear()
        
        if not lines_msg or not lines_msg.is_valid or lines_msg.num_lines == 0:
            return
        
        # Afficher chaque ligne
        for i in range(lines_msg.num_lines):
            x1 = lines_msg.x1_points[i]
            y1 = lines_msg.y1_points[i]
            x2 = lines_msg.x2_points[i]
            y2 = lines_msg.y2_points[i]
            confidence = lines_msg.confidences[i]
            
            # Couleur en fonction de la confiance (vert = haute confiance, jaune = faible)
            # RGB: vert (0, 255, 0) -> jaune (255, 255, 0)
            red = int(255 * (1.0 - confidence))
            green = 255
            blue = 0
            
            line_item = pg.PlotCurveItem(
                pen=pg.mkPen((red, green, blue), width=2, style=Qt.SolidLine)
            )
            line_item.setData([x1, x2], [y1, y2])
            self.addItem(line_item)
            self.hough_lines.append(line_item)

    def update_cage_pose(self, pose_msg):
        """Met à jour l'affichage du centre du U et de son orientation (PoseStamped)."""
        if pose_msg is None:
            self.u_center.hide()
            self.u_orientation_line.hide()
            self.u_orientation_head.hide()
            return

        try:
            x = float(pose_msg.pose.position.x)
            y = float(pose_msg.pose.position.y)
            qx = float(pose_msg.pose.orientation.x)
            qy = float(pose_msg.pose.orientation.y)
            qz = float(pose_msg.pose.orientation.z)
            qw = float(pose_msg.pose.orientation.w)
        except Exception:
            return

        # Calcul du yaw à partir du quaternion
        # yaw (z) = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

        # Afficher le centre
        self.u_center.setData(pos=[(x, y)])
        self.u_center.show()

        # Ligne d'orientation
        L = max(0.2, min(0.8, self.cage_width))  # longueur de la flèche en m
        ex = x + L * np.cos(yaw)
        ey = y + L * np.sin(yaw)
        self.u_orientation_line.setData([x, ex], [y, ey])
        self.u_orientation_line.show()

        # Tête de flèche (triangle)
        head_len = 0.08
        head_w = 0.06
        # vecteur direction
        dx = np.cos(yaw)
        dy = np.sin(yaw)
        # point base de la tête
        bx = ex - head_len * dx
        by = ey - head_len * dy
        # perpendiculaire
        px = -dy
        py = dx
        left_x = bx + (head_w / 2.0) * px
        left_y = by + (head_w / 2.0) * py
        right_x = bx - (head_w / 2.0) * px
        right_y = by - (head_w / 2.0) * py

        tx = [ex, left_x, right_x, ex]
        ty = [ey, left_y, right_y, ey]
        self.u_orientation_head.setData(tx, ty)
        self.u_orientation_head.show()
    
    def _on_mouse_clicked(self, event):
        """Gère les clics souris pour sélection de bbox ou clic simple."""
        if event.button() != Qt.LeftButton:
            return
        
        pos = event.scenePos()
        mouse_point = self.plotItem.vb.mapSceneToView(pos)
        x_m = mouse_point.x()
        y_m = mouse_point.y()
        
        # Mode sélection rotatif (4 coins)
        if self.rotated_selection_mode:
            self.rotated_points.append((x_m, y_m))
            
            # Afficher les marqueurs des points cliqués
            points_array = np.array(self.rotated_points)
            self.rotated_markers.setData(pos=points_array)
            self.rotated_markers.show()
            
            # Preview du rectangle
            num_points = len(self.rotated_points)
            if num_points == 2:
                # Afficher la ligne entre P1 et P2
                p1, p2 = self.rotated_points
                self.rotated_preview.setData([p1[0], p2[0]], [p1[1], p2[1]])
                self.rotated_preview.show()
            elif num_points == 3:
                # Afficher 2 côtés
                p1, p2, p3 = self.rotated_points
                x_coords = [p1[0], p2[0], p3[0]]
                y_coords = [p1[1], p2[1], p3[1]]
                self.rotated_preview.setData(x_coords, y_coords)
                self.rotated_preview.show()
            elif num_points == 4:
                # Afficher le rectangle complet
                p1, p2, p3, p4 = self.rotated_points
                x_coords = [p1[0], p2[0], p3[0], p4[0], p1[0]]
                y_coords = [p1[1], p2[1], p3[1], p4[1], p1[1]]
                self.rotated_preview.setData(x_coords, y_coords)
                self.rotated_preview.show()
                
                # Finaliser et émettre le signal
                self._finalize_rotated_selection()
            
            return
        
        # Mode sélection de bbox activé par le bouton
        if self.bbox_selection_mode:
            if self.bbox_start_point is None:
                # Début de la sélection
                self.bbox_start_point = (x_m, y_m)
                self.bbox_current_point = (x_m, y_m)
                self.bbox_selection_rect.show()
                self._update_bbox_selection_display()
            else:
                # Fin de la sélection
                self.bbox_current_point = (x_m, y_m)
                self._finalize_bbox_selection()
                # Désactiver le mode et reprendre
                self.set_bbox_selection_mode(False)
            return
        
        # Clic normal (sans mode sélection)
        # Afficher le marker (point rouge)
        self.click_marker.setData(pos=[(x_m, y_m)])
        self.click_marker.show()
        
        # Afficher la bounding box rouge (taille cage estimée)
        half_w = self.cage_width / 2.0
        half_h = self.cage_height / 2.0
        x_coords = [x_m - half_w, x_m + half_w, x_m + half_w, x_m - half_w, x_m - half_w]
        y_coords = [y_m - half_h, y_m - half_h, y_m + half_h, y_m + half_h, y_m - half_h]
        self.click_bbox.setData(x_coords, y_coords)
        self.click_bbox.show()
        
        # Démarrer le timer pour masquer après 1 seconde
        self.click_timer.start(1000)
        
        # Émettre le signal
        self.click_position.emit(float(x_m), float(y_m))
    
    def _hide_click_marker(self):
        """Masque le marker et la bbox de clic."""
        self.click_marker.hide()
        self.click_bbox.hide()
    
    def _on_mouse_moved(self, pos):
        """Gère le mouvement de la souris pendant la sélection de bbox."""
        if not self.bbox_selection_mode or self.bbox_start_point is None:
            return
        
        mouse_point = self.plotItem.vb.mapSceneToView(pos)
        x_m = mouse_point.x()
        y_m = mouse_point.y()
        
        self.bbox_current_point = (x_m, y_m)
        self._update_bbox_selection_display()
    
    def set_bbox_selection_mode(self, enabled):
        """Active ou désactive le mode sélection de bbox."""
        self.bbox_selection_mode = enabled
        
        if enabled:
            # Désactiver le mode rotatif si activé
            if self.rotated_selection_mode:
                self.set_rotated_selection_mode(False)
            # Pause et préparer la sélection
            self.is_paused = True
            self.bbox_start_point = None
            self.bbox_current_point = None
        else:
            # Reprendre et nettoyer
            self.is_paused = False
            self.bbox_start_point = None
            self.bbox_current_point = None
            self.bbox_selection_rect.hide()
    
    def set_rotated_selection_mode(self, enabled):
        """Active ou désactive le mode sélection rotatif (3 points)."""
        self.rotated_selection_mode = enabled
        
        if enabled:
            # Désactiver le mode bbox si activé
            if self.bbox_selection_mode:
                self.set_bbox_selection_mode(False)
            # Pause et préparer la sélection
            self.is_paused = True
            self.rotated_points = []
            self.rotated_markers.hide()
            self.rotated_preview.hide()
        else:
            # Reprendre et nettoyer
            self.is_paused = False
            self.rotated_points = []
            self.rotated_markers.hide()
            self.rotated_preview.hide()
    
    def _finalize_rotated_selection(self):
        """Finalise la sélection rotatif et émet le signal avec 4 coins en mètres."""
        if len(self.rotated_points) != 4:
            return
        
        p1, p2, p3, p4 = self.rotated_points
        
        # Émettre le signal avec les 4 coins en mètres
        self.rotated_bbox_selected.emit(
            float(p1[0]), float(p1[1]),
            float(p2[0]), float(p2[1]),
            float(p3[0]), float(p3[1]),
            float(p4[0]), float(p4[1])
        )
        
        # Désactiver le mode automatiquement
        self.set_rotated_selection_mode(False)
    
    def _update_bbox_selection_display(self):
        """Met à jour l'affichage du rectangle de sélection."""
        if self.bbox_start_point is None or self.bbox_current_point is None:
            return
        
        x1, y1 = self.bbox_start_point
        x2, y2 = self.bbox_current_point
        
        # Dessiner le rectangle
        x_coords = [x1, x2, x2, x1, x1]
        y_coords = [y1, y1, y2, y2, y1]
        self.bbox_selection_rect.setData(x_coords, y_coords)
    
    def _finalize_bbox_selection(self):
        """Finalise la sélection et émet le signal avec bbox en pixels."""
        if self.bbox_start_point is None or self.bbox_current_point is None:
            return
        
        x1_m, y1_m = self.bbox_start_point
        x2_m, y2_m = self.bbox_current_point
        
        # Conversion mètres → pixels
        # ==========================
        # Le rectangle pointillé est dessiné dans le repère PyQtGraph (mètres) :
        #   - X: de -max_range à +max_range (gauche à droite)
        #   - Y: de 0 à max_range (bas=ROV vers haut=avant)
        #
        # L'image envoyée au tracker est AVANT rot90, shape (height, width) :
        #   - Ligne 0 = Y proche du ROV (Y=0 en mètres)
        #   - Ligne max = Y loin du ROV (Y=max_range en mètres)
        #   - Colonne 0 = X gauche (X=-max_range en mètres)
        #   - origin_x = colonne centrale (X=0 en mètres)
        #
        # Formules de conversion (repère PyQtGraph → pixels image AVANT rot90) :
        #   x_px = origin_x - (x_m / resolution)  ← X INVERSÉ
        #   y_px = y_m / resolution
        
        x1_px = int(self.origin_x - (x1_m / self.current_resolution))
        y1_px = int(y1_m / self.current_resolution)
        x2_px = int(self.origin_x - (x2_m / self.current_resolution))
        y2_px = int(y2_m / self.current_resolution)
        
        # Normaliser (x, y = coin supérieur gauche dans le repère image)
        bbox_x = min(x1_px, x2_px)
        bbox_y = min(y1_px, y2_px)
        bbox_w = abs(x2_px - x1_px)
        bbox_h = abs(y2_px - y1_px)
        
        # Émettre le signal
        self.bbox_selected.emit(bbox_x, bbox_y, bbox_w, bbox_h)
        
        # Réinitialiser
        self.bbox_start_point = None
        self.bbox_current_point = None
    
    def set_cage_dimensions(self, width_m: float, height_m: float):
        """Met à jour les dimensions estimées de la cage (en mètres)."""
        self.cage_width = width_m
        self.cage_height = height_m

    def update_tracked_object(self, tracked_msg):
        """Met à jour l'affichage de la bounding box du tracker (orientée si angle présent)."""
        if not tracked_msg.is_tracking or not tracked_msg.is_initialized:
            # Masquer la bounding box si pas de tracking
            self.tracker_bbox.hide()
            self.tracker_entry_side.hide()
            self.tracker_center.hide()
            self.tracker_text.hide()
            return
        
        # Position du centre en mètres
        cx = tracked_msg.center_x
        cy = tracked_msg.center_y
        
        # Dimensions en mètres
        half_w = tracked_msg.width / 2.0
        half_h = tracked_msg.height / 2.0
        
        # Angle de rotation (en radians)
        angle = tracked_msg.angle
        
        if abs(angle) > 0.01:  # Si angle significatif, dessiner bbox orientée
            # Calculer les 4 coins du rectangle orienté
            # Coins dans le repère local (non tourné)
            corners_local = np.array([
                [-half_w, -half_h],  # Coin inférieur gauche
                [+half_w, -half_h],  # Coin inférieur droit
                [+half_w, +half_h],  # Coin supérieur droit
                [-half_w, +half_h],  # Coin supérieur gauche
            ])
            
            # Matrice de rotation
            cos_a = np.cos(angle)
            sin_a = np.sin(angle)
            rot_matrix = np.array([
                [cos_a, -sin_a],
                [sin_a,  cos_a]
            ])
            
            # Appliquer la rotation
            corners_rotated = corners_local @ rot_matrix.T
            
            # Translater au centre
            corners_world = corners_rotated + np.array([cx, cy])
            
            # Fermer le rectangle
            x_coords = np.append(corners_world[:, 0], corners_world[0, 0])
            y_coords = np.append(corners_world[:, 1], corners_world[0, 1])
            
            self.tracker_bbox.setData(x_coords, y_coords)
            self.tracker_bbox.show()
            
            # Dessiner le côté d'entrée si les points sont fournis
            if abs(tracked_msg.entry_p1_x) > 0.001 or abs(tracked_msg.entry_p1_y) > 0.001:
                entry_x = [tracked_msg.entry_p1_x, tracked_msg.entry_p2_x]
                entry_y = [tracked_msg.entry_p1_y, tracked_msg.entry_p2_y]
                self.tracker_entry_side.setData(entry_x, entry_y)
                self.tracker_entry_side.show()
            else:
                self.tracker_entry_side.hide()
        else:
            # Rectangle non orienté (angle ~ 0)
            x_coords = [cx - half_w, cx + half_w, cx + half_w, cx - half_w, cx - half_w]
            y_coords = [cy - half_h, cy - half_h, cy + half_h, cy + half_h, cy - half_h]
            
            self.tracker_bbox.setData(x_coords, y_coords)
            self.tracker_bbox.show()
            self.tracker_entry_side.hide()
        
        # Afficher le centre
        self.tracker_center.setData(pos=[(cx, cy)])
        self.tracker_center.show()
        
        # Afficher l'angle si non nul
        if abs(angle) > 0.01:
            angle_deg = np.degrees(angle)
            self.tracker_text.setText(f'θ = {angle_deg:.1f}°')
            self.tracker_text.setPos(cx, cy + half_h + 0.2)
            self.tracker_text.show()
        else:
            self.tracker_text.hide()
