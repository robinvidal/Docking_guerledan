import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import Qt, QRectF
from scipy.ndimage import map_coordinates


class SonarCartesianWidget(pg.PlotWidget):
    """2D cartesian view for sonar frames with optional image reprojection."""

    def __init__(self, title="Sonar"):
        super().__init__()
        self.setTitle(title)
        self.setLabel('bottom', 'X (latéral, m)', units='m')
        self.setLabel('left', 'Y (frontal, m)', units='m')
        self.setAspectLocked(True)

        self.scatter = pg.ScatterPlotItem(size=3, pen=pg.mkPen(None))
        self.addItem(self.scatter)

        self.image_item = pg.ImageItem()
        self.addItem(self.image_item)
        self.image_item.setZValue(-10)
        self.image_item.hide()

        self.borders_scatter = pg.ScatterPlotItem(
            size=15, pen=pg.mkPen(None), brush=pg.mkBrush(255, 0, 0, 255)
        )
        self.addItem(self.borders_scatter)

        self.center_line = pg.PlotCurveItem(pen=pg.mkPen('w', width=1, style=Qt.DashLine))
        self.addItem(self.center_line)
        self.center_line.setData([0, 0], [0, 50])

        # Sonar field-of-view boundaries (drawn from origin to max range)
        self.fov_left = pg.PlotCurveItem(pen=pg.mkPen('c', width=1, style=Qt.DashLine))
        self.fov_right = pg.PlotCurveItem(pen=pg.mkPen('c', width=1, style=Qt.DashLine))
        self.addItem(self.fov_left)
        self.addItem(self.fov_right)

        self.rov_marker = pg.ScatterPlotItem(
            pos=[(0, 0)], size=20, symbol='t', pen=pg.mkPen('g', width=2), brush=pg.mkBrush(0, 255, 0, 100)
        )
        self.addItem(self.rov_marker)

        self.showGrid(x=True, y=True, alpha=0.3)

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

        self._mapping_cache = {
            'bearing_count': None,
            'range_count': None,
            'coords': None,
            'out_shape': None,
        }
        self.use_image = True
        self.image_rotation = 1

    def update_image(self, frame_msg):
        """Met à jour l'affichage sonar polaire avec conversion vers cartésien."""
        # Reconstruction de l'image polaire depuis le format flat bearing-major
        img = np.array(frame_msg.intensities, dtype=np.uint8).reshape(
            (frame_msg.bearing_count, frame_msg.range_count)
        )
        ranges = np.linspace(frame_msg.min_range, frame_msg.max_range, frame_msg.range_count)
        total_angle = frame_msg.bearing_resolution * frame_msg.bearing_count
        
        # INVERSION DES BEARINGS pour l'affichage
        # ========================================
        # Les bearings dans l'image polaire sont ordonnés de l'indice 0 à N.
        # Par convention du message Frame, bearing[0] = -FOV/2 (gauche).
        # Pour que l'affichage montre la gauche à gauche et la droite à droite,
        # on inverse le signe des bearings calculés ici.
        # Ceci est cohérent avec arctan2(-xv, yv) utilisé dans la conversion.
        bearings = -np.linspace(-total_angle / 2, total_angle / 2, frame_msg.bearing_count)

        # Update field-of-view boundary lines using current angle and max range
        try:
            half_angle = total_angle / 2.0
            max_r = frame_msg.max_range
            x_left = max_r * np.sin(+half_angle)
            y_left = max_r * np.cos(+half_angle)
            x_right = max_r * np.sin(-half_angle)
            y_right = max_r * np.cos(-half_angle)
            self.fov_left.setData([0, x_left], [0, y_left])
            self.fov_right.setData([0, x_right], [0, y_right])
        except Exception:
            # If something goes wrong, hide the FOV indicators
            self.fov_left.setData([], [])
            self.fov_right.setData([], [])

        if self.use_image:
            bc = frame_msg.bearing_count
            rc = frame_msg.range_count
            max_r = frame_msg.max_range
            min_r = frame_msg.min_range
            total_angle = frame_msg.bearing_resolution * bc

            cache = self._mapping_cache
            if cache['bearing_count'] != bc or cache['range_count'] != rc:
                out_h = rc
                out_w = int(2 * rc)

                xs = np.linspace(-max_r, max_r, out_w)
                ys = np.linspace(0, max_r, out_h)
                xv, yv = np.meshgrid(xs, ys)

                rr = np.sqrt(xv**2 + yv**2)
                # use -xv so positive angles map to the same side as the
                # `bearings` sign convention above (fix horizontal mirroring)
                th = np.arctan2(-xv, yv)

                i_float = (th + total_angle / 2.0) / total_angle * (bc - 1)
                j_float = (rr - min_r) / (max_r - min_r) * (rc - 1)

                coords = np.vstack((i_float.ravel(), j_float.ravel()))

                cache['bearing_count'] = bc
                cache['range_count'] = rc
                cache['coords'] = coords
                cache['out_shape'] = (out_h, out_w)
            else:
                coords = cache['coords']
                out_h, out_w = cache['out_shape']

            sampled = map_coordinates(img.astype(np.float32), coords, order=1, mode='constant', cval=0.0)
            sampled = sampled.reshape((out_h, out_w))

            v = np.clip(sampled / 255.0, 0.0, 1.0)
            r_chan = np.interp(v, self._lut_positions, self._lut_colors[:, 0])
            g_chan = np.interp(v, self._lut_positions, self._lut_colors[:, 1])
            b_chan = np.interp(v, self._lut_positions, self._lut_colors[:, 2])

            rgb = np.stack((r_chan, g_chan, b_chan), axis=-1)
            rgb_uint8 = (rgb * 255).astype(np.uint8)

            if self.image_rotation and self.image_rotation % 4 != 0:
                rgb_uint8 = np.rot90(rgb_uint8, k=self.image_rotation)

            self.image_item.setImage(rgb_uint8, autoLevels=False)
            try:
                self.image_item.setRect(QRectF(-max_r, 0.0, 2.0 * max_r, max_r - min_r))
            except Exception:  # noqa: BLE001
                self.image_item.setPos(-max_r, 0.0)
                sx = (2.0 * max_r) / float(rgb_uint8.shape[1])
                sy = (max_r - min_r) / float(rgb_uint8.shape[0])
                self.image_item.resetTransform()
                self.image_item.scale(sx, sy)
            self.image_item.show()
            self.scatter.hide()
        else:
            points = []
            intensities = []
            step = 2
            for i in range(0, frame_msg.bearing_count, step):
                for j in range(0, frame_msg.range_count, step):
                    intensity = img[i, j]
                    if intensity > 30:
                        r_val = ranges[j]
                        theta_val = bearings[i]
                        x = r_val * np.sin(theta_val)
                        y = r_val * np.cos(theta_val)
                        points.append([x, y])
                        intensities.append(intensity)

            if points:
                points = np.array(points)
                intensities = np.array(intensities)
                colors = self.custom_colormap.mapToQColor(intensities / 255.0)
                brushes = [pg.mkBrush(c) for c in colors]
                self.scatter.setData(pos=points, brush=brushes)
            else:
                self.scatter.setData([], [])

    def update_borders(self, borders_msg):
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
