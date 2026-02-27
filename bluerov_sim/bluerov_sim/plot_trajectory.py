#!/usr/bin/env python3
"""
plot_trajectory.py — Trajectoire du BlueROV dans un repère cage-fixe
====================================================================

Lit un rosbag ROS 2 contenant le topic /docking/tracking/tracked_object
(docking_msgs/msg/TrackedObject) et trace la trajectoire du BlueROV dans
un repère « monde » centré sur la cage (supposée immobile).

Le cap (heading) du ROV n'est pas enregistré dans ce bag.  Il est estimé
par optimisation : on cherche le profil ψ(t) qui minimise un coût combinant
  • la régularité du cap        Σ (Δψ)²
  • la régularité de la trajectoire  Σ ‖accélération‖²
  • la pénalisation du déplacement latéral par rapport au cap

Usage
-----
    python plot_trajectory.py /chemin/vers/dossier_rosbag [options]

Options
-------
    --no-animate       Ne pas afficher l'animation (seulement le plot statique)
    --no-sonar-cone    Ne pas afficher le cône sonar sur l'animation
    --save-fig PATH    Sauvegarder la figure statique (png/pdf/svg)
    --save-anim PATH   Sauvegarder l'animation (mp4/gif)
    --sonar-fov DEG    Demi-angle du cône sonar en degrés (défaut: 65)
    --smooth-sigma N   Sigma du filtre gaussien sur range/bearing (défaut: 3)
    --heading-weight F Poids de la régularité du cap (défaut: 5.0)
    --accel-weight F   Poids de la régularité d'accélération (défaut: 0.5)
    --lateral-weight F Poids de la pénalisation latérale (défaut: 1.0)
    --topic TOPIC      Nom du topic tracked_object (défaut: /docking/tracking/tracked_object)

Dépendances : numpy, matplotlib, scipy, sqlite3 (std)
"""

import argparse
import math
import sqlite3
import struct
import sys
from pathlib import Path

import matplotlib.cm as cm
import matplotlib.colors as mcolors
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection
from scipy.ndimage import gaussian_filter1d
from scipy.optimize import minimize


# ═══════════════════════════════════════════════════════════════════════
# 1. Lecture du rosbag  (SQLite + désérialisation CDR manuelle)
# ═══════════════════════════════════════════════════════════════════════

def parse_tracked_object_cdr(data: bytes) -> dict:
    """
    Désérialise un message docking_msgs/msg/TrackedObject en CDR little-endian.

    Layout CDR (après 4 octets d'en-tête) :
        std_msgs/Header   : stamp(int32 sec, uint32 nsec)  + string frame_id
        bool is_tracking, bool is_initialized
        7 × float32       : center_x, center_y, range, bearing, angle, width, height
        4 × int32         : bbox_x, bbox_y, bbox_width, bbox_height
        9 × float32       : entry_p1_(x,y,range,bearing), entry_p2_(x,y,range,bearing), confidence
    """
    offset = 4  # Skip CDR header [0x00 0x01 0x00 0x00]

    sec = struct.unpack_from('<i', data, offset)[0]; offset += 4
    nsec = struct.unpack_from('<I', data, offset)[0]; offset += 4

    str_len = struct.unpack_from('<I', data, offset)[0]; offset += 4
    offset += str_len  # saute frame_id + null terminator

    is_tracking = bool(data[offset]); offset += 1
    is_initialized = bool(data[offset]); offset += 1

    offset = (offset + 3) & ~3  # padding → alignement 4 octets

    vals = struct.unpack_from('<7f', data, offset); offset += 28
    ints = struct.unpack_from('<4i', data, offset); offset += 16
    evals = struct.unpack_from('<9f', data, offset)

    return {
        'sec': sec, 'nsec': nsec,
        'is_tracking': is_tracking,
        'is_initialized': is_initialized,
        'center_x': vals[0], 'center_y': vals[1],
        'range': vals[2], 'bearing': vals[3],
        'angle': vals[4], 'width': vals[5], 'height': vals[6],
        'bbox_x': ints[0], 'bbox_y': ints[1],
        'bbox_w': ints[2], 'bbox_h': ints[3],
        'confidence': evals[8],
    }


def read_bag(bag_dir: Path, topic: str) -> dict:
    """
    Lit tous les messages TrackedObject d'un rosbag ROS 2 (format SQLite).

    Retourne un dict de numpy arrays :
        timestamps (s), center_x (m), center_y (m), ranges (m), bearings (rad)
    Seuls les messages avec is_tracking=True sont conservés.
    """
    db_files = sorted(bag_dir.glob('*.db3'))
    if not db_files:
        sys.exit(f"Erreur : aucun fichier .db3 dans {bag_dir}")

    all_ts, all_cx, all_cy, all_r, all_b = [], [], [], [], []

    for db_file in db_files:
        conn = sqlite3.connect(str(db_file))
        cur = conn.cursor()

        cur.execute("SELECT id FROM topics WHERE name = ?", (topic,))
        row = cur.fetchone()
        if row is None:
            conn.close()
            continue
        topic_id = row[0]

        cur.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
            (topic_id,),
        )
        for ts_ns, raw in cur.fetchall():
            msg = parse_tracked_object_cdr(bytes(raw))
            if not msg['is_tracking']:
                continue
            all_ts.append(ts_ns * 1e-9)
            all_cx.append(msg['center_x'])
            all_cy.append(msg['center_y'])
            all_r.append(msg['range'])
            all_b.append(msg['bearing'])

        conn.close()

    if not all_ts:
        sys.exit("Erreur : aucun message TrackedObject avec is_tracking=True trouvé.")

    ts = np.array(all_ts)
    ts -= ts[0]  # temps relatif depuis le début

    return {
        'timestamps': ts,
        'center_x': np.array(all_cx),
        'center_y': np.array(all_cy),
        'ranges': np.array(all_r),
        'bearings': np.array(all_b),
    }


# ═══════════════════════════════════════════════════════════════════════
# 2. Estimation du cap (heading) par optimisation
# ═══════════════════════════════════════════════════════════════════════

def _world_pos(psi, r, beta):
    """Position du ROV en repère monde (cage à l'origine)."""
    angle = psi + beta
    px = -r * np.sin(angle)
    py = -r * np.cos(angle)
    return px, py


def _cost_heading(psi, r, beta, w_h, w_a, w_l):
    """
    Coût pour l'estimation du cap.

    Termes :
        C_h  : régularité du cap            Σ (Δψ)²
        C_a  : régularité d'accélération     Σ ‖a‖²
        C_l  : pénalisation latérale         Σ (v_lat / |v|)²
    """
    N = len(psi)
    px, py = _world_pos(psi, r, beta)

    # --- Régularité du cap ---
    dpsi = np.diff(psi)
    C_h = np.sum(dpsi ** 2)

    # --- Régularité d'accélération (trajectoire lisse) ---
    if N >= 3:
        ax = px[2:] - 2.0 * px[1:-1] + px[:-2]
        ay = py[2:] - 2.0 * py[1:-1] + py[:-2]
        C_a = np.sum(ax ** 2 + ay ** 2)
    else:
        C_a = 0.0

    # --- Pénalisation du déplacement latéral ---
    vx = np.diff(px)
    vy = np.diff(py)
    speed_sq = vx ** 2 + vy ** 2 + 1e-12
    # vecteur « avant » du ROV dans le monde : (sin(ψ), cos(ψ))
    psi_mid = 0.5 * (psi[:-1] + psi[1:])
    # composante latérale = (cos(ψ), −sin(ψ)) · v
    v_lat = np.cos(psi_mid) * vx - np.sin(psi_mid) * vy
    C_l = np.sum(v_lat ** 2 / speed_sq)

    return w_h * C_h + w_a * C_a + w_l * C_l


def _grad_heading(psi, r, beta, w_h, w_a, w_l):
    """Gradient analytique (partiel) pour les termes C_h et C_a.
    Le terme C_l utilise un gradient numérique (petit)."""
    N = len(psi)
    px, py = _world_pos(psi, r, beta)
    dpx = -r * np.cos(beta + psi)   # ∂px/∂ψ
    dpy = r * np.sin(beta + psi)     # ∂py/∂ψ

    grad = np.zeros(N)

    # --- ∂C_h / ∂ψ ---
    dpsi = np.diff(psi)
    grad[:-1] -= 2.0 * dpsi
    grad[1:] += 2.0 * dpsi
    grad *= w_h

    # --- ∂C_a / ∂ψ  (accélération) ---
    if N >= 3:
        ax = px[2:] - 2.0 * px[1:-1] + px[:-2]
        ay = py[2:] - 2.0 * py[1:-1] + py[:-2]
        # ∂C_a/∂ψ_s   pour chaque a(t), t=1..N-2
        ga = np.zeros(N)
        for t_idx in range(N - 2):
            s = t_idx  # index dans ax/ay : a(t_idx) correspond à t = t_idx+1
            coef_x = 2.0 * ax[s]
            coef_y = 2.0 * ay[s]
            # ∂a_x(t)/∂ψ_{t-1} = dpx[t-1],  ∂a_x(t)/∂ψ_t = -2·dpx[t], ∂a_x(t)/∂ψ_{t+1} = dpx[t+1]
            t = t_idx + 1
            ga[t - 1] += coef_x * dpx[t - 1] + coef_y * dpy[t - 1]
            ga[t]     += coef_x * (-2.0 * dpx[t]) + coef_y * (-2.0 * dpy[t])
            ga[t + 1] += coef_x * dpx[t + 1] + coef_y * dpy[t + 1]
        grad += w_a * ga

    # --- ∂C_l / ∂ψ  (approximation par différences finies) ---
    if w_l > 0:
        eps = 1e-5
        C_l0 = _cost_heading(psi, r, beta, 0, 0, w_l)
        # Calcul vectorisé : ψ + eps·e_i
        # On fait un gradient numérique clairsemé (sous-échantillonné) pour la vitesse
        step = max(1, N // 200)
        for i in range(0, N, step):
            psi_p = psi.copy(); psi_p[i] += eps
            C_p = _cost_heading(psi_p, r, beta, 0, 0, w_l)
            grad[i] += (C_p - C_l0) / eps

    return grad


def estimate_heading(ranges, bearings, w_h=5.0, w_a=0.5, w_l=1.0, verbose=True):
    """
    Estime le cap ψ(t) du ROV par optimisation.

    Étapes :
        1. Recherche exhaustive du meilleur cap constant ψ₀ (360 angles)
        2. Affinage par L-BFGS-B avec cap variable, initialisé depuis ψ₀

    Retourne ψ (array, radians) — 0 ↔ direction de l'approche initiale.
    """
    N = len(ranges)
    r = ranges.copy()
    b = bearings.copy()

    # ---------- Étape 1 : meilleur cap constant ----------
    psi_candidates = np.linspace(0, 2 * np.pi, 360, endpoint=False)
    best_cost, best_psi0 = np.inf, 0.0
    for p0 in psi_candidates:
        psi_const = np.full(N, p0)
        c = _cost_heading(psi_const, r, b, w_h, w_a, w_l)
        if c < best_cost:
            best_cost = c
            best_psi0 = p0
    if verbose:
        print(f"  Cap constant optimal : {np.degrees(best_psi0):.1f}°  (coût = {best_cost:.2f})")

    # ---------- Étape 2 : optimisation L-BFGS-B ----------
    psi0 = np.full(N, best_psi0)

    def fun(psi):
        return _cost_heading(psi, r, b, w_h, w_a, w_l)

    def jac(psi):
        return _grad_heading(psi, r, b, w_h, w_a, w_l)

    result = minimize(
        fun, psi0, jac=jac, method='L-BFGS-B',
        options={'maxiter': 300, 'ftol': 1e-12, 'gtol': 1e-8},
    )
    if verbose:
        conv = "convergé" if result.success else f"non convergé ({result.message})"
        print(f"  Optimisation : {conv}, {result.nit} itérations, coût final = {result.fun:.2f}")

    psi_opt = result.x

    # ---------- Recaler : ψ(0) = 0  (direction initiale = vers le haut) ----------
    psi_opt -= psi_opt[0]

    return psi_opt


# ═══════════════════════════════════════════════════════════════════════
# 3. Dessin du BlueROV stylisé (adapté de simulation/plot_bluerov.py)
# ═══════════════════════════════════════════════════════════════════════

def _rot2d(pts, theta):
    """Rotation 2D de points Nx2."""
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s], [s, c]])
    return pts @ R.T


def draw_bluerov(ax, x, y, theta, size=0.6, color='#2196F3', alpha=0.85, zorder=10):
    """
    Dessine un BlueROV stylisé (vue du dessus).

    theta : cap en radians (0 = vers Y+).
    """
    h = size / 2.0
    # Carré du corps — le corps a son « avant » vers X+ en repère local,
    # donc on tourne de (θ + π/2) pour que heading 0 → Y+.
    rot_angle = theta + np.pi / 2.0
    corners = np.array([[-h, -h], [h, -h], [h, h], [-h, h]])
    corners = _rot2d(corners, rot_angle) + [x, y]
    body = mpatches.Polygon(corners, closed=True,
                            facecolor=color, edgecolor='k',
                            alpha=alpha, lw=0.8, zorder=zorder)
    ax.add_patch(body)

    # Cercles aux coins (propulseurs)
    r_prop = size * 0.15
    for cx, cy in corners:
        prop = mpatches.Circle((cx, cy), r_prop,
                               facecolor='white', edgecolor='k',
                               lw=0.6, alpha=alpha, zorder=zorder + 1)
        ax.add_patch(prop)

    # Arc avant (indicate direction)
    front_mid = _rot2d(np.array([[h, 0.0]]), rot_angle).ravel() + [x, y]
    arc_r = size * 0.22
    arc = mpatches.Arc(front_mid, 2 * arc_r, 2 * arc_r,
                       angle=np.degrees(rot_angle), theta1=-90, theta2=90,
                       edgecolor='k', lw=1.0, alpha=alpha, zorder=zorder + 2)
    ax.add_patch(arc)

    return body


def draw_sonar_cone(ax, x, y, theta, fov_half_deg=65.0, max_range=3.0,
                    color='gold', alpha=0.12, zorder=3):
    """Dessine le cône sonar (secteur angulaire).

    theta : heading en radians (0 = vers Y+, sens horaire).
    """
    fov_half = np.radians(fov_half_deg)
    # Heading θ (0=Y+, CW) → angle math (0=X+, CCW) : α = π/2 − θ
    center_angle = np.pi / 2.0 - theta
    angles = np.linspace(center_angle - fov_half,
                         center_angle + fov_half, 60)
    xs = x + max_range * np.cos(angles)
    ys = y + max_range * np.sin(angles)
    verts = np.vstack([[x, y], np.column_stack([xs, ys]), [x, y]])
    wedge = mpatches.Polygon(verts, closed=True,
                             facecolor=color, edgecolor='none',
                             alpha=alpha, zorder=zorder)
    ax.add_patch(wedge)
    return wedge


def draw_cage(ax, x=0.0, y=0.0, width=0.82, depth=0.60, lw=3, zorder=12):
    """Dessine la cage en U à la position (x,y)."""
    hw = width / 2.0
    # U ouvert vers le bas (Y−)
    cage_pts = np.array([
        [-hw, -depth / 2],
        [-hw,  depth / 2],
        [ hw,  depth / 2],
        [ hw, -depth / 2],
    ])
    cage_pts[:, 0] += x
    cage_pts[:, 1] += y
    ax.plot(cage_pts[:, 0], cage_pts[:, 1], 'k-', lw=lw, solid_capstyle='round', zorder=zorder)
    # Remplissage léger
    cage_fill = mpatches.Polygon(cage_pts, closed=False,
                                 facecolor='none', edgecolor='k',
                                 lw=lw, zorder=zorder)
    ax.add_patch(cage_fill)
    # Petite cible au centre
    ax.plot(x, y, 'k+', ms=10, mew=2, zorder=zorder + 1)


# ═══════════════════════════════════════════════════════════════════════
# 4. Tracé statique
# ═══════════════════════════════════════════════════════════════════════

def plot_static(traj_x, traj_y, headings, timestamps, ranges, bearings,
                sonar_fov_half=65.0, show_cone=True, save_path=None):
    """Figure statique multi-panneaux."""
    fig = plt.figure(figsize=(18, 10), facecolor='white')
    fig.suptitle('Trajectoire de docking du BlueROV  —  repère cage-fixe',
                 fontsize=14, fontweight='bold', y=0.98)

    # --- Layout : 2 lignes, 3 colonnes ---
    gs = fig.add_gridspec(2, 3, hspace=0.32, wspace=0.30,
                          left=0.06, right=0.97, top=0.92, bottom=0.08)

    # ====== A. Trajectoire 2D (grande) ======
    ax_traj = fig.add_subplot(gs[:, 0:2])
    t_norm = (timestamps - timestamps[0]) / (timestamps[-1] - timestamps[0])
    cmap = cm.get_cmap('viridis')

    # Colorbar de la trajectoire
    points = np.column_stack([traj_x, traj_y]).reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    lc = LineCollection(segments, cmap=cmap, norm=mcolors.Normalize(0, timestamps[-1]))
    lc.set_array(timestamps)
    lc.set_linewidth(2.5)
    ax_traj.add_collection(lc)

    # Colorbar
    cbar = fig.colorbar(lc, ax=ax_traj, shrink=0.7, pad=0.02)
    cbar.set_label('Temps (s)', fontsize=10)

    # Quelques cônes sonar le long de la trajectoire
    if show_cone:
        n_cones = 8
        indices = np.linspace(0, len(traj_x) - 1, n_cones, dtype=int)
        for idx in indices:
            r_cone = min(ranges[idx] * 0.4, 2.5)
            draw_sonar_cone(ax_traj, traj_x[idx], traj_y[idx], headings[idx],
                            fov_half_deg=sonar_fov_half, max_range=r_cone,
                            alpha=0.08, color='gold')

    # Départ et arrivée
    ax_traj.plot(traj_x[0], traj_y[0], 'o', color=cmap(0.0), ms=10, zorder=15,
                 markeredgecolor='k', markeredgewidth=1.5, label='Départ')
    ax_traj.plot(traj_x[-1], traj_y[-1], 's', color=cmap(1.0), ms=10, zorder=15,
                 markeredgecolor='k', markeredgewidth=1.5, label='Arrivée')

    # BlueROV au départ et à l'arrivée
    draw_bluerov(ax_traj, traj_x[0], traj_y[0], headings[0], size=0.7,
                 color='#64B5F6', alpha=0.7)
    draw_bluerov(ax_traj, traj_x[-1], traj_y[-1], headings[-1], size=0.5,
                 color='#1565C0', alpha=0.9)

    # Cage
    draw_cage(ax_traj)

    ax_traj.set_xlabel('X  (m)  —  latéral', fontsize=11)
    ax_traj.set_ylabel('Y  (m)  —  longitudinal', fontsize=11)
    ax_traj.set_title('Trajectoire 2D  (repère cage-fixe)', fontsize=12)
    ax_traj.set_aspect('equal', 'datalim')
    ax_traj.legend(loc='upper left', fontsize=9)
    ax_traj.grid(True, alpha=0.3)

    # Marges
    margin = 1.5
    ax_traj.set_xlim(traj_x.min() - margin, traj_x.max() + margin)
    ax_traj.set_ylim(traj_y.min() - margin, traj_y.max() + margin)

    # ====== B. Range vs temps ======
    ax_range = fig.add_subplot(gs[0, 2])
    ax_range.plot(timestamps, ranges, '-', color='#E53935', lw=1.5, label='Range')
    ax_range.set_xlabel('Temps (s)')
    ax_range.set_ylabel('Distance à la cage (m)')
    ax_range.set_title('Range vs temps')
    ax_range.grid(True, alpha=0.3)
    ax_range.legend(fontsize=9)

    # ====== C. Bearing vs temps ======
    ax_bearing = fig.add_subplot(gs[1, 2])
    ax_bearing.plot(timestamps, np.degrees(bearings), '-', color='#1E88E5', lw=1.5, label='Bearing')
    ax_bearing.axhline(0, color='gray', ls='--', lw=0.8)
    ax_bearing.set_xlabel('Temps (s)')
    ax_bearing.set_ylabel('Bearing (°)')
    ax_bearing.set_title('Bearing vs temps')
    ax_bearing.grid(True, alpha=0.3)
    ax_bearing.legend(fontsize=9)

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Figure sauvegardée : {save_path}")

    return fig


# ═══════════════════════════════════════════════════════════════════════
# 5. Animation
# ═══════════════════════════════════════════════════════════════════════

def animate_trajectory(traj_x, traj_y, headings, timestamps, ranges, bearings,
                       sonar_fov_half=65.0, show_cone=True, save_path=None):
    """Animation matplotlib de la trajectoire de docking."""

    fig, ax = plt.subplots(figsize=(10, 10), facecolor='white')
    ax.set_aspect('equal', 'datalim')
    ax.set_xlabel('X  (m)  —  latéral', fontsize=11)
    ax.set_ylabel('Y  (m)  —  longitudinal', fontsize=11)
    ax.set_title('Animation docking BlueROV', fontsize=13, fontweight='bold')
    ax.grid(True, alpha=0.3)

    margin = 1.5
    ax.set_xlim(traj_x.min() - margin, traj_x.max() + margin)
    ax.set_ylim(traj_y.min() - margin, traj_y.max() + margin)

    # Cage (fixe)
    draw_cage(ax)

    # Traces de trajectoire
    trail_line, = ax.plot([], [], '-', color='#90CAF9', lw=1.5, alpha=0.6, zorder=2)
    trail_points = ax.scatter([], [], c=[], cmap='viridis', s=8,
                              norm=mcolors.Normalize(0, timestamps[-1]),
                              zorder=4, edgecolors='none')

    # Texte temps / range
    time_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                        fontsize=11, verticalalignment='top',
                        fontfamily='monospace',
                        bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))

    # Éléments dynamiques (recréés à chaque frame)
    dynamic_patches = []

    # Sous-échantillonnage pour fluidité (~20 fps visuel)
    step = max(1, len(traj_x) // 500)
    idxs = list(range(0, len(traj_x), step))
    if idxs[-1] != len(traj_x) - 1:
        idxs.append(len(traj_x) - 1)

    def init():
        trail_line.set_data([], [])
        return [trail_line, time_text]

    def update(frame_idx):
        i = idxs[frame_idx]

        # Nettoyage patches dynamiques
        for p in dynamic_patches:
            p.remove()
        dynamic_patches.clear()

        # Trail
        trail_line.set_data(traj_x[:i + 1], traj_y[:i + 1])

        # Scatter trail
        offsets = np.column_stack([traj_x[:i + 1], traj_y[:i + 1]])
        trail_points.set_offsets(offsets)
        trail_points.set_array(timestamps[:i + 1])

        # Cône sonar
        if show_cone:
            r_cone = min(ranges[i] * 0.5, 4.0)
            w = draw_sonar_cone(ax, traj_x[i], traj_y[i], headings[i],
                                fov_half_deg=sonar_fov_half, max_range=r_cone,
                                alpha=0.15, color='gold', zorder=5)
            dynamic_patches.append(w)

        # BlueROV
        rov_size = max(0.35, min(0.7, ranges[i] * 0.08))
        body = draw_bluerov(ax, traj_x[i], traj_y[i], headings[i],
                            size=rov_size, color='#2196F3', alpha=0.9, zorder=10)
        # Collecter tous les patches du ROV
        # draw_bluerov ajoute directement via ax.add_patch → on doit les retrouver
        # Simplification : on les retrouve depuis la fin de ax.patches
        n_rov_patches = 6  # body + 4 propellers + arc
        for p in ax.patches[-n_rov_patches:]:
            dynamic_patches.append(p)

        # Info texte
        time_text.set_text(
            f't = {timestamps[i]:5.1f} s\n'
            f'range = {ranges[i]:5.2f} m\n'
            f'bearing = {np.degrees(bearings[i]):+5.1f}°'
        )

        return [trail_line, trail_points, time_text] + dynamic_patches

    anim = FuncAnimation(fig, update, init_func=init,
                         frames=len(idxs), interval=40, blit=False, repeat=True)

    if save_path:
        ext = Path(save_path).suffix.lower()
        if ext == '.gif':
            anim.save(save_path, writer='pillow', fps=25)
        else:
            anim.save(save_path, writer='ffmpeg', fps=25)
        print(f"Animation sauvegardée : {save_path}")

    return fig, anim


# ═══════════════════════════════════════════════════════════════════════
# 6. Programme principal
# ═══════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description='Trace la trajectoire du BlueROV dans un repère cage-fixe '
                    'à partir d\'un rosbag ROS 2.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('bag_dir', type=str,
                        help='Chemin vers le dossier du rosbag (contenant le .db3)')
    parser.add_argument('--no-animate', action='store_true',
                        help='Ne pas afficher l\'animation')
    parser.add_argument('--no-sonar-cone', action='store_true',
                        help='Cacher le cône sonar dans l\'animation')
    parser.add_argument('--save-fig', type=str, default=None,
                        help='Chemin pour sauvegarder la figure statique (png/pdf)')
    parser.add_argument('--save-anim', type=str, default=None,
                        help='Chemin pour sauvegarder l\'animation (mp4/gif)')
    parser.add_argument('--sonar-fov', type=float, default=65.0,
                        help='Demi-angle du cône sonar en degrés (défaut: 65)')
    parser.add_argument('--smooth-sigma', type=float, default=3.0,
                        help='Sigma du filtre gaussien sur range/bearing (défaut: 3)')
    parser.add_argument('--heading-weight', type=float, default=5.0,
                        help='Poids régularité du cap (défaut: 5.0)')
    parser.add_argument('--accel-weight', type=float, default=0.5,
                        help='Poids régularité d\'accélération (défaut: 0.5)')
    parser.add_argument('--lateral-weight', type=float, default=1.0,
                        help='Poids pénalisation latérale (défaut: 1.0)')
    parser.add_argument('--topic', type=str,
                        default='/docking/tracking/tracked_object',
                        help='Nom du topic TrackedObject')

    args = parser.parse_args()
    bag_dir = Path(args.bag_dir)
    if not bag_dir.is_dir():
        sys.exit(f"Erreur : {bag_dir} n'est pas un répertoire valide.")

    # ---- Lecture du bag ----
    print(f"Lecture du rosbag : {bag_dir}")
    data = read_bag(bag_dir, args.topic)
    N = len(data['timestamps'])
    print(f"  {N} messages de tracking chargés "
          f"({data['timestamps'][-1]:.1f} s, {N / data['timestamps'][-1]:.1f} Hz)")
    print(f"  Range : {data['ranges'][0]:.2f} → {data['ranges'][-1]:.2f} m")

    # ---- Lissage ----
    sigma = args.smooth_sigma
    if sigma > 0:
        ranges_s = gaussian_filter1d(data['ranges'], sigma)
        bearings_s = gaussian_filter1d(data['bearings'], sigma)
        print(f"  Lissage gaussien appliqué (σ = {sigma})")
    else:
        ranges_s = data['ranges'].copy()
        bearings_s = data['bearings'].copy()

    # ---- Estimation du cap ----
    print("Estimation du cap (heading) par optimisation...")
    headings = estimate_heading(
        ranges_s, bearings_s,
        w_h=args.heading_weight,
        w_a=args.accel_weight,
        w_l=args.lateral_weight,
    )

    # ---- Calcul de la trajectoire monde ----
    traj_x, traj_y = _world_pos(headings, ranges_s, bearings_s)
    print(f"  Trajectoire : départ ({traj_x[0]:.1f}, {traj_y[0]:.1f}) → "
          f"arrivée ({traj_x[-1]:.1f}, {traj_y[-1]:.1f}) m")

    # ---- Tracé statique ----
    print("Tracé de la figure statique...")
    fig_static = plot_static(
        traj_x, traj_y, headings,
        data['timestamps'], data['ranges'], data['bearings'],
        sonar_fov_half=args.sonar_fov,
        show_cone=not args.no_sonar_cone,
        save_path=args.save_fig,
    )

    # ---- Animation ----
    anim = None
    if not args.no_animate or args.save_anim:
        print("Préparation de l'animation...")
        fig_anim, anim = animate_trajectory(
            traj_x, traj_y, headings,
            data['timestamps'], data['ranges'], data['bearings'],
            sonar_fov_half=args.sonar_fov,
            show_cone=not args.no_sonar_cone,
            save_path=args.save_anim,
        )

    print("Affichage... (fermer les fenêtres pour quitter)")
    plt.show()


if __name__ == '__main__':
    main()
