"""
Nœud de tracking de tache (blob) sans ML.
Approche: suivi par corrélation (template matching) dans une fenêtre de recherche autour
de la dernière position. Ré-initialisation possible via recherche globale du maximum.
Publie un message Borders avec un seul point pour visualisation dans le viewer.
"""

import rclpy
from rclpy.node import Node
from docking_msgs.msg import Frame, Borders
import numpy as np
from typing import Optional, Tuple

try:
    import cv2
except ImportError:
    cv2 = None


def _clip(a, lo, hi):
    return max(lo, min(hi, a))


class BlobTrackerNode(Node):
    def __init__(self):
        super().__init__('blob_tracker_node')

        # Paramètres
        self.declare_parameter('enable_tracker', True)
        self.declare_parameter('template_size', 21)           # impaire, px
        self.declare_parameter('search_radius', 24)           # px, autour de la position précédente
        self.declare_parameter('ncc_threshold', 0.5)          # score min TM_CCOEFF_NORMED
        self.declare_parameter('learning_rate', 0.05)         # MAJ progressive du template
        self.declare_parameter('init_mode', 'max')            # 'max' ou 'center'
        self.declare_parameter('intensity_threshold', 120)    # min intensité pour init/reinit

        # État interne
        self.template: Optional[np.ndarray] = None  # float32, zero-mean, unit-norm
        self.pos: Optional[Tuple[int, int]] = None  # (bearing_idx, range_idx)
        self.image_shape: Optional[Tuple[int, int]] = None

        # Cache des grilles métriques
        self.ranges: Optional[np.ndarray] = None
        self.bearings: Optional[np.ndarray] = None

        # Sub/Pub
        self.sub = self.create_subscription(Frame, '/docking/sonar/filtered', self.frame_cb, 10)
        # Publie sur le même topic que l'overlay du viewer
        self.pub = self.create_publisher(Borders, '/docking/tracking/borders', 10)

        self.get_logger().info('BlobTracker node démarré (template matching sans ML)')

    def _ensure_grids(self, msg: Frame):
        if (self.ranges is None or self.bearings is None or
                len(self.ranges) != msg.range_count or len(self.bearings) != msg.bearing_count):
            self.ranges = np.linspace(msg.min_range, msg.max_range, msg.range_count, dtype=np.float32)
            # Hypothèse éventail [-pi/2, pi/2]
            self.bearings = np.linspace(-np.pi/2, np.pi/2, msg.bearing_count, dtype=np.float32)

    def _extract_patch(self, img: np.ndarray, center: Tuple[int, int], k: int) -> Optional[np.ndarray]:
        bi, ri = center
        b0 = bi - k
        b1 = bi + k + 1
        r0 = ri - k
        r1 = ri + k + 1
        if b0 < 0 or r0 < 0 or b1 > img.shape[0] or r1 > img.shape[1]:
            return None
        return img[b0:b1, r0:r1]

    def _normalize_template(self, patch: np.ndarray) -> np.ndarray:
        patch = patch.astype(np.float32)
        patch = patch - patch.mean()
        n = np.linalg.norm(patch) + 1e-6
        return patch / n

    def _match_in_roi(self, img: np.ndarray, center: Tuple[int, int], templ: np.ndarray, radius: int) -> Tuple[Tuple[int, int], float]:
        k = templ.shape[0] // 2
        bi, ri = center
        b0 = _clip(bi - radius - k, 0, img.shape[0] - templ.shape[0])
        r0 = _clip(ri - radius - k, 0, img.shape[1] - templ.shape[1])
        b1 = _clip(bi + radius + k, 0, img.shape[0])
        r1 = _clip(ri + radius + k, 0, img.shape[1])
        roi = img[b0:b1, r0:r1]

        if roi.shape[0] < templ.shape[0] or roi.shape[1] < templ.shape[1]:
            # trop proche des bords
            return center, 0.0

        if cv2 is not None:
            res = cv2.matchTemplate(roi.astype(np.float32), templ, cv2.TM_CCOEFF_NORMED)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
            top_left = (max_loc[1] + b0, max_loc[0] + r0)  # (b, r)
            best_center = (top_left[0] + k, top_left[1] + k)
            return best_center, float(max_val)

        # Fallback NCC naïf (ROI restreinte donc  acceptable)
        best_score = -1.0
        best_center = center
        tb, tr = templ.shape
        for bb in range(roi.shape[0] - tb + 1):
            for rr in range(roi.shape[1] - tr + 1):
                patch = roi[bb:bb+tb, rr:rr+tr].astype(np.float32)
                patch = patch - patch.mean()
                denom = (np.linalg.norm(patch) * np.linalg.norm(templ) + 1e-6)
                score = float(np.sum(patch * templ) / denom)
                if score > best_score:
                    best_score = score
                    best_center = (b0 + bb + k, r0 + rr + k)
        return best_center, best_score

    def _init_from_image(self, img: np.ndarray, thr: int, templ_k: int) -> bool:
        # Init par maximum global
        idx = np.argmax(img)
        bi, ri = np.unravel_index(idx, img.shape)
        if img[bi, ri] < thr:
            return False
        patch = self._extract_patch(img, (bi, ri), templ_k)
        if patch is None:
            return False
        self.template = self._normalize_template(patch)
        self.pos = (int(bi), int(ri))
        return True

    def frame_cb(self, msg: Frame):
        if not self.get_parameter('enable_tracker').value:
            return

        # Reconstituer image
        img = np.array(msg.intensities, dtype=np.uint8).reshape((msg.bearing_count, msg.range_count))
        self._ensure_grids(msg)
        self.image_shape = img.shape

        ksize = int(self.get_parameter('template_size').value)
        ksize = max(3, ksize | 1)
        k = ksize // 2
        search_radius = int(self.get_parameter('search_radius').value)
        thr = int(self.get_parameter('intensity_threshold').value)
        ncc_thr = float(self.get_parameter('ncc_threshold').value)
        lr = float(self.get_parameter('learning_rate').value)

        # Initialisation si nécessaire
        if self.template is None or self.pos is None:
            mode = self.get_parameter('init_mode').value
            ok = self._init_from_image(img, thr, k)
            if not ok and mode == 'center':
                # fallback centre
                center = (img.shape[0] // 2, img.shape[1] // 2)
                patch = self._extract_patch(img, center, k)
                if patch is not None:
                    self.template = self._normalize_template(patch)
                    self.pos = center
                    ok = True
            if not ok:
                return  # rien à publier

        # Matching dans la fenêtre de recherche
        best_center, score = self._match_in_roi(img, self.pos, self.template, search_radius)

        # Validation
        if score >= ncc_thr:
            # Mise à jour position et template (EMA)
            self.pos = (int(best_center[0]), int(best_center[1]))
            patch = self._extract_patch(img, self.pos, k)
            if patch is not None:
                new_t = self._normalize_template(patch)
                self.template = (1.0 - lr) * self.template + lr * new_t
        else:
            # Ré-init si échec
            ok = self._init_from_image(img, thr, k)
            if not ok:
                # publication invalide
                self._publish(None, None, 0.0, msg)
                return

        # Publier la position courante
        bi, ri = self.pos
        rng = float(self.ranges[_clip(ri, 0, len(self.ranges)-1)])
        brg = float(self.bearings[_clip(bi, 0, len(self.bearings)-1)])
        self._publish(rng, brg, float(score), msg)

    def _publish(self, rng: Optional[float], brg: Optional[float], conf: float, msg: Frame):
        out = Borders()
        out.header = msg.header
        if rng is None or brg is None:
            out.is_valid = False
            out.ranges = []
            out.bearings = []
            out.confidences = []
            out.cage_width = 0.0
            out.cage_depth = 0.0
        else:
            out.is_valid = True
            out.ranges = [rng]
            out.bearings = [brg]
            out.confidences = [float(np.clip(conf, 0.0, 1.0))]
            out.cage_width = 0.0
            out.cage_depth = 0.0
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = BlobTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
