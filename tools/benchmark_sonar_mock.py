#!/usr/bin/env python3
"""
Micro-benchmark pour la pipeline du `sonar_mock` (downsample/filter/upscale/blend)
Exécutable sans ROS. Mesure ms/frame pour N itérations.
"""
import os
import sys
import time
import numpy as np

# Ajouter le src ROS local au path pour importer docking_utils
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
SRC_PATH = os.path.join(REPO_ROOT, 'ros2_bluerov', 'src')
if SRC_PATH not in sys.path:
    sys.path.insert(0, SRC_PATH)

try:
    from docking_utils import filters
except Exception:
    # Fallback: charger le fichier filters.py directement par chemin absolu
    import importlib.util
    filters_path = os.path.join(REPO_ROOT, 'ros2_bluerov', 'src', 'docking_utils', 'docking_utils', 'filters.py')
    if not os.path.exists(filters_path):
        raise RuntimeError(f"Impossible de trouver {filters_path}; vérifiez l'arborescence du dépôt.")
    spec = importlib.util.spec_from_file_location('du_filters', filters_path)
    filters = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(filters)

import cv2

# Paramètres (identiques à sonar_mock par défaut)
BEARING_COUNT = 256
RANGE_COUNT = 512
DS_FACTOR = 2
NOISE_LEVEL = 20.0
MEDIAN_KERNEL = 3
GAUSSIAN_SIGMA = 1.5
ITERATIONS = 200

rng = np.random.default_rng(12345)

# Buffers préalloués
float_buf = np.empty((BEARING_COUNT, RANGE_COUNT), dtype=np.float32)
int_buf = np.empty((BEARING_COUNT, RANGE_COUNT), dtype=np.uint8)

# Pré-calcul ranges (pour range_comp si utilisé)
ranges = np.linspace(1.0, 40.0, RANGE_COUNT)

# Fonction qui simule génération rapide puis application du pipeline optimisé
def pipeline_once():
    std = max(1.0, float(NOISE_LEVEL))
    mean_bg = std * 0.2
    float_buf[:] = rng.normal(loc=mean_bg, scale=std, size=float_buf.shape)

    # speckle
    sp_prob = min(0.12, NOISE_LEVEL / 400.0)
    if sp_prob > 0.0:
        rand_vals = rng.random(float_buf.shape)
        speckle_mask = rand_vals < sp_prob
        if speckle_mask.any():
            high_mask = rand_vals[speckle_mask] < (sp_prob * 0.7)
            speckle_vals = np.empty(high_mask.shape, dtype=np.uint8)
            speckle_vals[high_mask] = rng.integers(200, 255, size=high_mask.sum(), dtype=np.uint8)
            speckle_vals[~high_mask] = 0
            float_buf[speckle_mask] = speckle_vals

    np.clip(float_buf, 0, 255, out=float_buf)
    int_buf[:] = float_buf.astype(np.uint8, copy=False)

    filtered = int_buf

    # downsample path
    if DS_FACTOR > 1:
        ds = DS_FACTOR
        small = filtered[::ds, ::ds]
        small_filtered = small
        small_filtered = filters.median_filter(small_filtered, MEDIAN_KERNEL)
        small_filtered = filters.gaussian_filter(small_filtered, GAUSSIAN_SIGMA)
        up = cv2.resize(small_filtered, (RANGE_COUNT, BEARING_COUNT), interpolation=cv2.INTER_NEAREST)
        alpha = 0.6
        filtered = cv2.addWeighted(filtered, float(alpha), up, float(1.0 - alpha), 0)

    # optional: range compensation + contrast (skipped for raw speed measurement)
    return filtered

# Warm-up
for _ in range(10):
    _ = pipeline_once()

# Timed runs
times = []
for i in range(ITERATIONS):
    t0 = time.perf_counter()
    _ = pipeline_once()
    t1 = time.perf_counter()
    times.append((t1 - t0) * 1000.0)

print('Benchmark sonar mock pipeline (downsample + median + gaussian + resize+blend)')
print(f'Parameters: BEARING={BEARING_COUNT}, RANGE={RANGE_COUNT}, DS={DS_FACTOR}, iters={ITERATIONS}')
print(f'Mean ms/frame: {np.mean(times):.3f} ms')
print(f'Std ms/frame:  {np.std(times):.3f} ms')
print(f'Percentiles (50/90/99): {np.percentile(times, [50,90,99]).round(3)} ms')

# Quick sample memory of the arrays
print('Buffers sizes (bytes):', float_buf.nbytes, int_buf.nbytes)

# Suggestion: to compare, set DS_FACTOR=1 and re-run

if __name__ == '__main__':
    pass
