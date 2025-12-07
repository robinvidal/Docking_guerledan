"""
Nœud de tracking double-blobs sans ML.
Deux templates sont suivis conjointement avec un score multi-critères (NCC, stabilité
temporelle, respect de la distance attendue). Recherche locale, gating et recovery global.
Publie deux points dans Borders pour visualisation.
"""

import math
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from docking_msgs.msg import Frame, Borders

try:
    import cv2
except ImportError:
    cv2 = None


def _clip(a, lo, hi):
    return max(lo, min(hi, a))


class BlobTrackerNode(Node):
    def __init__(self):
        super().__init__('blob_tracker_node')

        # Paramètres principaux
        self.declare_parameter('enable_tracker', True)
        self.declare_parameter('template_size', 21)
        self.declare_parameter('search_radius', 24)
        self.declare_parameter('ncc_threshold', 0.5)
        self.declare_parameter('learning_rate', 0.05)
        self.declare_parameter('init_mode', 'max')            # 'max', 'center'
        self.declare_parameter('intensity_threshold', 120)

        # Contraintes géométriques et stabilité
        self.declare_parameter('distance_expected', 1.0)      # m
        self.declare_parameter('distance_epsilon', 0.3)       # m (anneau)
        self.declare_parameter('distance_sigma', 0.2)         # m (score gaussien)
        self.declare_parameter('stability_window', 5)
        self.declare_parameter('stability_threshold', 0.4)
        self.declare_parameter('max_jump_m', 1.0)             # gating mouvement

        # Pondérations score global
        self.declare_parameter('weight_ncc', 0.6)
        self.declare_parameter('weight_stability', 0.25)
        self.declare_parameter('weight_distance', 0.15)
        self.declare_parameter('min_total_score', 0.4)

        # Recovery / init avancée
        self.declare_parameter('top_k_candidates', 20)
        self.declare_parameter('recovery_enabled', True)

        # Init manuelle optionnelle (mètres/radians). -1 désactive.
        self.declare_parameter('manual_init_enabled', False)
        self.declare_parameter('manual_range_a', -1.0)
        self.declare_parameter('manual_bearing_a', 0.0)
        self.declare_parameter('manual_range_b', -1.0)
        self.declare_parameter('manual_bearing_b', 0.0)

        # État interne
        self.template_a: Optional[np.ndarray] = None
        self.template_b: Optional[np.ndarray] = None
        self.pos_a: Optional[Tuple[int, int]] = None
        self.pos_b: Optional[Tuple[int, int]] = None
        self.history_a: List[Tuple[float, float]] = []  # xy en m
        self.history_b: List[Tuple[float, float]] = []
        self.image_shape: Optional[Tuple[int, int]] = None

        # Cache des grilles métriques
        self.ranges: Optional[np.ndarray] = None
        self.bearings: Optional[np.ndarray] = None

        # Sub/Pub
        self.sub = self.create_subscription(Frame, '/docking/sonar/filtered', self.frame_cb, 10)
        self.pub = self.create_publisher(Borders, '/docking/tracking/borders', 10)

        self.get_logger().info('BlobTracker node démarré (double template matching)')

    def _ensure_grids(self, msg: Frame):
        if (self.ranges is None or self.bearings is None or
                len(self.ranges) != msg.range_count or len(self.bearings) != msg.bearing_count):
            self.ranges = np.linspace(msg.min_range, msg.max_range, msg.range_count, dtype=np.float32)
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

    def _ncc_score(self, patch: np.ndarray, templ: np.ndarray) -> float:
        patch_f = patch.astype(np.float32)
        patch_f = patch_f - patch_f.mean()
        denom = (np.linalg.norm(patch_f) * np.linalg.norm(templ) + 1e-6)
        return float(np.sum(patch_f * templ) / denom)

    def _match_in_roi(self, img: np.ndarray, center: Tuple[int, int], templ: np.ndarray, radius: int) -> Tuple[Optional[Tuple[int, int]], float]:
        k = templ.shape[0] // 2
        bi, ri = center
        b0 = _clip(bi - radius - k, 0, img.shape[0] - templ.shape[0])
        r0 = _clip(ri - radius - k, 0, img.shape[1] - templ.shape[1])
        b1 = _clip(bi + radius + k, 0, img.shape[0])
        r1 = _clip(ri + radius + k, 0, img.shape[1])
        roi = img[b0:b1, r0:r1]

        if roi.shape[0] < templ.shape[0] or roi.shape[1] < templ.shape[1]:
            return None, 0.0

        if cv2 is not None:
            res = cv2.matchTemplate(roi.astype(np.float32), templ, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, max_loc = cv2.minMaxLoc(res)
            top_left = (max_loc[1] + b0, max_loc[0] + r0)  # (b, r)
            best_center = (top_left[0] + k, top_left[1] + k)
            return best_center, float(max_val)

        best_score = -1.0
        best_center = None
        tb, tr = templ.shape
        for bb in range(roi.shape[0] - tb + 1):
            for rr in range(roi.shape[1] - tr + 1):
                patch = roi[bb:bb+tb, rr:rr+tr]
                score = self._ncc_score(patch, templ)
                if score > best_score:
                    best_score = score
                    best_center = (b0 + bb + k, r0 + rr + k)
        return best_center, best_score

    def _idx_to_xy(self, bi: int, ri: int) -> Tuple[float, float]:
        rng = float(self.ranges[_clip(ri, 0, len(self.ranges) - 1)])
        brg = float(self.bearings[_clip(bi, 0, len(self.bearings) - 1)])
        x = rng * math.cos(brg)
        y = rng * math.sin(brg)
        return x, y

    def _distance_between(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        ax, ay = self._idx_to_xy(a[0], a[1])
        bx, by = self._idx_to_xy(b[0], b[1])
        return float(math.hypot(ax - bx, ay - by))

    def _stability(self, history: List[Tuple[float, float]]) -> float:
        if len(history) < 2:
            return 0.5
        arr = np.array(history, dtype=np.float32)
        var = float(np.var(arr, axis=0).mean())
        return float(1.0 / (1.0 + var))

    def _update_history(self, history: List[Tuple[float, float]], pos_xy: Tuple[float, float], window: int):
        history.append(pos_xy)
        while len(history) > window:
            history.pop(0)

    def _distance_score(self, d_measured: float, d_ref: float, sigma: float) -> float:
        if sigma <= 0.0:
            return 0.0
        return float(math.exp(-((d_measured - d_ref) ** 2) / (2.0 * sigma * sigma)))

    def _total_score(self, ncc_pair: float, stability_pair: float, distance_score: float, w1: float, w2: float, w3: float) -> float:
        s = max(1e-6, w1 + w2 + w3)
        score = (w1 * ncc_pair + w2 * stability_pair + w3 * distance_score) / s
        return float(np.clip(score, 0.0, 1.0))

    def _match_in_ring(self, img: np.ndarray, templ: np.ndarray, mask: np.ndarray) -> Tuple[Optional[Tuple[int, int]], float]:
        k = templ.shape[0] // 2
        coords = np.argwhere(mask)
        best_center = None
        best_score = -1.0
        for bi, ri in coords:
            patch = self._extract_patch(img, (int(bi), int(ri)), k)
            if patch is None:
                continue
            score = self._ncc_score(patch, templ)
            if score > best_score:
                best_score = score
                best_center = (int(bi), int(ri))
        return best_center, best_score

    def _top_k_peaks(self, img: np.ndarray, k: int, thr: int) -> List[Tuple[int, int]]:
        flat = img.ravel()
        if flat.size == 0:
            return []
        k = min(k, flat.size)
        idxs = np.argpartition(flat, -k)[-k:]
        coords = [np.unravel_index(int(i), img.shape) for i in idxs]
        coords = [c for c in coords if img[c] >= thr]
        # Tri par intensité décroissante
        coords.sort(key=lambda c: img[c], reverse=True)
        return [(int(b), int(r)) for b, r in coords]

    def _init_manual(self, img: np.ndarray, k: int) -> bool:
        ra = float(self.get_parameter('manual_range_a').value)
        rb = float(self.get_parameter('manual_range_b').value)
        if ra < 0 or rb < 0:
            return False
        ba = float(self.get_parameter('manual_bearing_a').value)
        bb = float(self.get_parameter('manual_bearing_b').value)
        # Chercher indices les plus proches des coordonnées métriques fournies
        bi_a = int(np.argmin(np.abs(self.bearings - ba)))
        ri_a = int(np.argmin(np.abs(self.ranges - ra)))
        bi_b = int(np.argmin(np.abs(self.bearings - bb)))
        ri_b = int(np.argmin(np.abs(self.ranges - rb)))
        pa = self._extract_patch(img, (bi_a, ri_a), k)
        pb = self._extract_patch(img, (bi_b, ri_b), k)
        if pa is None or pb is None:
            return False
        self.pos_a = (bi_a, ri_a)
        self.pos_b = (bi_b, ri_b)
        self.template_a = self._normalize_template(pa)
        self.template_b = self._normalize_template(pb)
        return True

    def _init_auto(self, img: np.ndarray, k: int, thr: int, d_ref: float, eps: float) -> bool:
        topk = self._top_k_peaks(img, int(self.get_parameter('top_k_candidates').value), thr)
        if len(topk) < 2:
            return False
        best_pair = None
        best_score = -1.0
        for i in range(len(topk)):
            for j in range(i + 1, len(topk)):
                a = topk[i]
                b = topk[j]
                d = self._distance_between(a, b)
                if abs(d - d_ref) > eps:
                    continue
                ds = self._distance_score(d, d_ref, float(self.get_parameter('distance_sigma').value))
                sc = float(img[a]) + float(img[b]) + 100.0 * ds
                if sc > best_score:
                    best_score = sc
                    best_pair = (a, b)
        if best_pair is None:
            return False
        pa = self._extract_patch(img, best_pair[0], k)
        pb = self._extract_patch(img, best_pair[1], k)
        if pa is None or pb is None:
            return False
        self.pos_a = (int(best_pair[0][0]), int(best_pair[0][1]))
        self.pos_b = (int(best_pair[1][0]), int(best_pair[1][1]))
        self.template_a = self._normalize_template(pa)
        self.template_b = self._normalize_template(pb)
        return True

    def _recover(self, img: np.ndarray, k: int, thr: int, d_ref: float, eps: float) -> bool:
        if not self.get_parameter('recovery_enabled').value:
            return False
        topk = self._top_k_peaks(img, int(self.get_parameter('top_k_candidates').value), thr)
        if len(topk) < 2:
            return False
        best_pair = None
        best_score = -1.0
        sigma = float(self.get_parameter('distance_sigma').value)
        # Utilise stabilité passée pour prioriser les zones proches
        last_stab = min(self._stability(self.history_a), self._stability(self.history_b))
        for i in range(len(topk)):
            for j in range(i + 1, len(topk)):
                a = topk[i]
                b = topk[j]
                d = self._distance_between(a, b)
                if abs(d - d_ref) > eps:
                    continue
                ds = self._distance_score(d, d_ref, sigma)
                sc = ds + 0.01 * last_stab + 0.001 * (float(img[a]) + float(img[b]))
                if sc > best_score:
                    best_score = sc
                    best_pair = (a, b)
        if best_pair is None:
            return False
        pa = self._extract_patch(img, best_pair[0], k)
        pb = self._extract_patch(img, best_pair[1], k)
        if pa is None or pb is None:
            return False
        self.pos_a = (int(best_pair[0][0]), int(best_pair[0][1]))
        self.pos_b = (int(best_pair[1][0]), int(best_pair[1][1]))
        self.template_a = self._normalize_template(pa)
        self.template_b = self._normalize_template(pb)
        return True

    def frame_cb(self, msg: Frame):
        if not self.get_parameter('enable_tracker').value:
            return

        img = np.array(msg.intensities, dtype=np.uint8).reshape((msg.bearing_count, msg.range_count))
        self._ensure_grids(msg)
        self.image_shape = img.shape

        # Paramètres
        ksize = max(3, int(self.get_parameter('template_size').value) | 1)
        k = ksize // 2
        search_radius = int(self.get_parameter('search_radius').value)
        thr = int(self.get_parameter('intensity_threshold').value)
        ncc_thr = float(self.get_parameter('ncc_threshold').value)
        lr = float(self.get_parameter('learning_rate').value)
        d_ref = float(self.get_parameter('distance_expected').value)
        eps = float(self.get_parameter('distance_epsilon').value)
        sigma = float(self.get_parameter('distance_sigma').value)
        win = max(1, int(self.get_parameter('stability_window').value))
        stab_thr = float(self.get_parameter('stability_threshold').value)
        max_jump = float(self.get_parameter('max_jump_m').value)
        w1 = float(self.get_parameter('weight_ncc').value)
        w2 = float(self.get_parameter('weight_stability').value)
        w3 = float(self.get_parameter('weight_distance').value)
        min_total = float(self.get_parameter('min_total_score').value)

        # Grilles XY pour contrainte d'anneau
        br = self.bearings[:, None]
        rg = self.ranges[None, :]
        x_grid = np.cos(br) * rg
        y_grid = np.sin(br) * rg

        # Init si nécessaire
        if self.template_a is None or self.template_b is None or self.pos_a is None or self.pos_b is None:
            init_done = False
            if self.get_parameter('manual_init_enabled').value:
                init_done = self._init_manual(img, k)
            if not init_done:
                mode = self.get_parameter('init_mode').value
                if mode == 'center':
                    center = (img.shape[0] // 2, img.shape[1] // 2)
                    pa = self._extract_patch(img, center, k)
                    pb = self._extract_patch(img, (center[0], center[1] + k + 1), k)
                    if pa is not None and pb is not None:
                        self.pos_a = center
                        self.pos_b = (center[0], center[1] + k + 1)
                        self.template_a = self._normalize_template(pa)
                        self.template_b = self._normalize_template(pb)
                        init_done = True
                if not init_done:
                    init_done = self._init_auto(img, k, thr, d_ref, eps)
            if not init_done:
                self._publish(None, None, 0.0, 0.0, 0.0, msg)
                return

        # Matching blob A autour de la dernière position
        best_a, ncc_a = self._match_in_roi(img, self.pos_a, self.template_a, search_radius)
        if best_a is None:
            if self._recover(img, k, thr, d_ref, eps):
                self._after_detection(img, k, win, lr, d_ref, eps, sigma, w1, w2, w3, min_total, stab_thr, max_jump, ncc_thr, msg, allow_recover=False)
            else:
                self._publish(None, None, 0.0, 0.0, 0.0, msg)
            return

        # Recherche blob B sur l'anneau [d-eps, d+eps]
        xa, ya = self._idx_to_xy(best_a[0], best_a[1])
        dist_grid = np.sqrt((x_grid - xa) ** 2 + (y_grid - ya) ** 2)
        mask_ring = (dist_grid >= max(0.0, d_ref - eps)) & (dist_grid <= d_ref + eps)
        best_b, ncc_b = self._match_in_ring(img, self.template_b, mask_ring)
        if best_b is None:
            if self._recover(img, k, thr, d_ref, eps):
                self._after_detection(img, k, win, lr, d_ref, eps, sigma, w1, w2, w3, min_total, stab_thr, max_jump, ncc_thr, msg, allow_recover=False)
            else:
                self._publish(None, None, 0.0, 0.0, 0.0, msg)
            return

        self.pos_a = (int(best_a[0]), int(best_a[1]))
        self.pos_b = (int(best_b[0]), int(best_b[1]))

        # Gating / scoring
        self._after_detection(img, k, win, lr, d_ref, eps, sigma, w1, w2, w3, min_total, stab_thr, max_jump, ncc_thr, msg, allow_recover=True)

    def _after_detection(self, img: np.ndarray, k: int, win: int, lr: float, d_ref: float, eps: float,
                         sigma: float, w1: float, w2: float, w3: float, min_total: float,
                         stab_thr: float, max_jump: float, ncc_thr: float, msg: Frame, allow_recover: bool):
        # Recalc NCC sur positions retenues (patch direct)
        pa = self._extract_patch(img, self.pos_a, k)
        pb = self._extract_patch(img, self.pos_b, k)
        if pa is None or pb is None:
            self._publish(None, None, 0.0, 0.0, 0.0, msg)
            return
        ncc_a = self._ncc_score(pa, self.template_a)
        ncc_b = self._ncc_score(pb, self.template_b)
        ncc_pair = min(ncc_a, ncc_b)

        d_meas = self._distance_between(self.pos_a, self.pos_b)
        dist_ok = abs(d_meas - d_ref) <= eps
        dist_score = self._distance_score(d_meas, d_ref, sigma)

        # Stabilité
        xa, ya = self._idx_to_xy(self.pos_a[0], self.pos_a[1])
        xb, yb = self._idx_to_xy(self.pos_b[0], self.pos_b[1])
        self._update_history(self.history_a, (xa, ya), win)
        self._update_history(self.history_b, (xb, yb), win)
        stab_pair = min(self._stability(self.history_a), self._stability(self.history_b))

        # Mouvement (gating) en m
        move_ok = True
        if len(self.history_a) >= 2:
            prev_a = self.history_a[-2]
            move_a = math.hypot(xa - prev_a[0], ya - prev_a[1])
            if move_a > max_jump:
                move_ok = False
        if len(self.history_b) >= 2:
            prev_b = self.history_b[-2]
            move_b = math.hypot(xb - prev_b[0], yb - prev_b[1])
            if move_b > max_jump:
                move_ok = False

        total_score = self._total_score(ncc_pair, stab_pair, dist_score, w1, w2, w3)

        if (ncc_pair < ncc_thr or not dist_ok or stab_pair < stab_thr or not move_ok or total_score < min_total):
            if allow_recover and self._recover(img, k, int(self.get_parameter('intensity_threshold').value), d_ref, eps):
                self._after_detection(img, k, win, lr, d_ref, eps, sigma, w1, w2, w3, min_total, stab_thr, max_jump, ncc_thr, msg, allow_recover=False)
            else:
                self._publish(None, None, 0.0, 0.0, 0.0, msg)
            return

        # MAJ templates (EMA)
        self.template_a = (1.0 - lr) * self.template_a + lr * self._normalize_template(pa)
        self.template_b = (1.0 - lr) * self.template_b + lr * self._normalize_template(pb)

        # Publication
        rng_a = float(self.ranges[_clip(self.pos_a[1], 0, len(self.ranges) - 1)])
        brg_a = float(self.bearings[_clip(self.pos_a[0], 0, len(self.bearings) - 1)])
        rng_b = float(self.ranges[_clip(self.pos_b[1], 0, len(self.ranges) - 1)])
        brg_b = float(self.bearings[_clip(self.pos_b[0], 0, len(self.bearings) - 1)])
        self._publish(rng_a, brg_a, rng_b, brg_b, total_score, msg)

    def _publish(self, rng_a: Optional[float], brg_a: Optional[float], rng_b: Optional[float], brg_b: Optional[float], conf: float, msg: Frame):
        out = Borders()
        out.header = msg.header
        if rng_a is None or brg_a is None or rng_b is None or brg_b is None:
            out.is_valid = False
            out.ranges = []
            out.bearings = []
            out.confidences = []
            out.cage_width = 0.0
            out.cage_depth = 0.0
        else:
            out.is_valid = True
            out.ranges = [rng_a, rng_b]
            out.bearings = [brg_a, brg_b]
            out.confidences = [float(np.clip(conf, 0.0, 1.0)), float(np.clip(conf, 0.0, 1.0))]
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
