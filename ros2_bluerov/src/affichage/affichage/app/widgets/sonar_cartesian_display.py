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
        
        # Activer les événements de clic
        self.scene().sigMouseClicked.connect(self._on_mouse_clicked)
        
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
        
        # Rotation de 90° pour correspondre à l'orientation correcte
        # (même transformation que dans sonar_display.py)
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
    
    def _on_mouse_clicked(self, event):
        """Gère le clic souris sur le widget."""
        # Vérifier que c'est un clic gauche
        if event.button() != Qt.LeftButton:
            return
        
        # Convertir la position du clic en coordonnées du graphique (mètres)
        pos = event.scenePos()
        mouse_point = self.plotItem.vb.mapSceneToView(pos)
        x_m = mouse_point.x()
        y_m = mouse_point.y()
        
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
    
    def set_cage_dimensions(self, width_m: float, height_m: float):
        """Met à jour les dimensions estimées de la cage (en mètres)."""
        self.cage_width = width_m
        self.cage_height = height_m

    def update_tracked_object(self, tracked_msg):
        """Met à jour l'affichage de la bounding box du tracker CSRT."""
        if not tracked_msg.is_tracking or not tracked_msg.is_initialized:
            # Masquer la bounding box si pas de tracking
            self.tracker_bbox.hide()
            self.tracker_center.hide()
            self.tracker_text.hide()
            return
        
        # Position du centre en mètres
        cx = tracked_msg.center_x
        cy = tracked_msg.center_y
        
        # Dimensions en mètres
        half_w = tracked_msg.width / 2.0
        half_h = tracked_msg.height / 2.0
        
        # Dessiner le rectangle (coins)
        x_coords = [cx - half_w, cx + half_w, cx + half_w, cx - half_w, cx - half_w]
        y_coords = [cy - half_h, cy - half_h, cy + half_h, cy + half_h, cy - half_h]
        
        self.tracker_bbox.setData(x_coords, y_coords)
        self.tracker_bbox.show()
        
        # Afficher le centre
        self.tracker_center.setData(pos=[(cx, cy)])
        self.tracker_center.show()
        
        # Afficher la confiance
        confidence_pct = tracked_msg.confidence * 100
        self.tracker_text.setText(f'CSRT: {confidence_pct:.0f}%')
        self.tracker_text.setPos(cx - half_w, cy + half_h)
        self.tracker_text.show()
