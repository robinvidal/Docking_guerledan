import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def _rot(points, theta):
    """Rotate Nx2 points by theta (radians) around origin."""
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s], [s, c]])
    return points @ R.T

def plot_bluerov(ax=None, x=0.0, y=0.0, theta=0.0, size=1.0,
                 color='C0', alpha=0.8, label=None, linewidth=1.0):
    """
    Dessine un BlueROV stylisé sur l'axe 2D donné.

    - Le corps est un carré de côté `size` centré en (x,y) et orienté par `theta` (radians).
    - Chaque coin du carré a un petit cercle (hélice/point dur).
    - Le côté "avant" (direction +x dans le repère local) porte un arc de cercle
      dont le centre est le milieu du segment avant.

    Paramètres :
    - ax : matplotlib.axes.Axes (optionnel). Si None, on crée une figure.
    - x, y : coordonnées du centre du robot.
    - theta : angle d'orientation en radians (0 = avant vers +x).
    - size : longueur du côté du carré.
    - color : couleur principale.
    - alpha : opacité.
    - label : texte pour la légende (optionnel).
    - linewidth : épaisseur du contour du carré/arc.

    Retourne : list de patches ajoutés (Polygon, Circle, Arc).
    """
    created_ax = False
    if ax is None:
        fig, ax = plt.subplots()
        created_ax = True

    # 1) Corps : carré centré en (0,0) dans le repère local
    h = size / 2.0
    corners_body = np.array([
        [-h, -h],
        [ h, -h],
        [ h,  h],
        [-h,  h]
    ])  # ordre pour polygon

    # 2) Appliquer rotation + translation
    corners_world = _rot(corners_body, theta) + np.array([x, y])

    # Corps -> dessiner 3 bandes avant-arrière (division selon l'axe y local)
    # bandes : left, center, right (seuls left et right sont colorés)
    band_w = size / 3.0
    y0 = -h
    bands_body = [
        (y0, y0 + band_w),               # left
        (y0 + band_w, y0 + 2 * band_w),  # center
        (y0 + 2 * band_w, h)             # right
    ]

    patches_added = []
    band_patches = []
    for i, (yb, yt) in enumerate(bands_body):
        band_corners_body = np.array([
            [-h, yb],
            [ h, yb],
            [ h, yt],
            [-h, yt]
        ])
        band_corners_world = _rot(band_corners_body, theta) + np.array([x, y])
        # colorer seulement les bandes latérales (i == 0 or 2)
        if i == 0 or i == 2:
            face = color
        else:
            face = 'none'
        band_poly = patches.Polygon(band_corners_world, closed=True,
                                    facecolor=face, edgecolor='k',
                                    alpha=alpha if face != 'none' else 1.0,
                                    linewidth=linewidth)
        ax.add_patch(band_poly)
        patches_added.append(band_poly)
        band_patches.append(band_poly)

    # dessiner le contour du carré (bord uniquement)
    square_border = patches.Polygon(corners_world, closed=True,
                                    facecolor='none', edgecolor='k',
                                    linewidth=linewidth)
    ax.add_patch(square_border)
    patches_added.append(square_border)

    # 3) Cercles aux coins
    corner_radius = 2.0 * max(size * 0.12, 0.01 * size)  # ajustable (doublé)
    for (cx, cy) in corners_world:
        c = patches.Circle((cx, cy), radius=corner_radius,
                           facecolor='white', edgecolor='k',
                           linewidth=linewidth*1.5, alpha=alpha)
        ax.add_patch(c)
        patches_added.append(c)

    # 4) Arc sur le côté avant
    # côté avant en repère local = segment entre (h, -h) et (h, h)
    front_mid_body = np.array([h, 0.0])
    front_mid_world = _rot(front_mid_body.reshape(1,2), theta).ravel() + np.array([x,y])

    # rayon d'arc : proportion du size (ajustable)
    # réduit par 3 selon la demande
    arc_radius = size * 0.65 / 3.0

    # Matplotlib Arc : center = front_mid_world, width/height = diameter
    # On veut une demi-cercle "vers l'extérieur" du côté avant.
    # On prend un arc non-roté de -90..90 deg puis on le fait tourner de theta en degrés.
    arc = patches.Arc(front_mid_world, width=2*arc_radius, height=2*arc_radius,
                      angle=np.degrees(theta), theta1=-90, theta2=90,
                      edgecolor='k', lw=linewidth*1.2, alpha=alpha, zorder=square_border.get_zorder()+1)
    ax.add_patch(arc)
    patches_added.append(arc)

    # 5) Option label : placer un petit point de légende/handle
    if label:
        # On attache le label à la première bande colorée (gauche) afin
        # que la légende affiche la couleur du robot et non un fond vide.
        try:
            band_patches[0].set_label(label)
        except Exception:
            # fallback: use the square border
            square_border.set_label(label)

    # Réglages axes si on a créé un ax
    if created_ax:
        ax.set_aspect('equal', 'box')
        margin = size * 1.2
        ax.set_xlim(x - margin, x + margin)
        ax.set_ylim(y - margin, y + margin)
        ax.grid(True)

    return patches_added


# --------------------------
# Exemple d'utilisation
# --------------------------
if __name__ == '__main__':
    fig, ax = plt.subplots(figsize=(6,6))

    # Trois robots exemples
    plot_bluerov(ax, x=0.0, y=0.0, theta=0.0, size=1.0, color='C0', alpha=0.9, label='BR-0')
    plot_bluerov(ax, x=2.0, y=1.0, theta=np.pi/4, size=0.9, color='C1', alpha=0.9, label='BR-1')
    plot_bluerov(ax, x=-1.5, y=1.5, theta=-np.pi/6, size=0.7, color='C2', alpha=0.9, label='BR-2')

    ax.set_title('BlueROV stylisé')
    ax.legend()
    # Ensure view isn't too zoomed: autoscale then expand margins
    ax.relim()
    ax.autoscale_view()
    # expand_factor > 1 zooms out (1.0 = no change)
    expand_factor = 2.0
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    xmid = 0.5 * (x0 + x1)
    ymid = 0.5 * (y0 + y1)
    xhalf = 0.5 * (x1 - x0) * expand_factor + 1e-9
    yhalf = 0.5 * (y1 - y0) * expand_factor + 1e-9
    ax.set_xlim(xmid - xhalf, xmid + xhalf)
    ax.set_ylim(ymid - yhalf, ymid + yhalf)
    ax.set_aspect('equal', 'box')
    plt.show()