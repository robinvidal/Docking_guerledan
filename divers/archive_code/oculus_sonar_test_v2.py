#! /usr/bin/python3

import oculus_python
import time
import numpy as np
import math
import argparse
import pylab
import matplotlib.pyplot as plt

master_mode = 1
range_max = 5.5
gain = 50
ping_rate = 0

parser = argparse.ArgumentParser(
    prog='OculusRecord',
    description='Example of how to record data from a connected Oculus sonar')
parser.add_argument('-o', '--output', type=str, default='output.oculus',
                    help='Path where to save the output file.')
parser.add_argument('-m', '--masterMode', type=int, default=master_mode,
                    help='Master mode (1 LF, 2 HF)')
parser.add_argument('-r', '--range', type=float, default=range_max,
                    help='Sonar range in meters')
parser.add_argument('-g', '--gainPercent', type=int, default=gain,
                    help='Gain in percent')
parser.add_argument('-ping', '--pingRate', type=int, default=ping_rate,
                    help='Ping rate setting')
args = parser.parse_args()

configSet = False

def message_callback(msg):
    global sonar
    global configSet
    global args
    if configSet:
        return

    currentConfig = sonar.current_config()
    currentConfig.range       = args.range
    currentConfig.masterMode  = args.masterMode
    currentConfig.gainPercent = args.gainPercent
    currentConfig.pingRate    = args.pingRate

    print("\n[CONFIG] Envoi de la configuration suivante au sonar :")
    print(currentConfig)

    sonar.send_config(currentConfig)

    nom = args.output
    print(f"[RECORDER] Start recording to : {nom}")
    sonar.recorder_start(nom, True)

    configSet = True


c = None 
pingData = None
bearings = None
linearAngles = None
rawPingData = None
gains = None
test = None
range_sonar = None
master = None  # master mode actuel

# Grilles pré-calculées pour la visualisation
grid_X = None
grid_Y = None
grid_shape = None

new_frame = False   # flag : un nouveau ping vient d'arriver

def ping_callback(metadata):
    global pingData
    global bearings
    global linearAngles
    global rawPingData
    global gains
    global test
    global range_sonar
    global master
    global new_frame

    msg = metadata

    bearings     = 0.01 * np.array(msg.bearing_data())  # degrés
    linearAngles = np.linspace(bearings[0], bearings[-1], len(bearings))

    rawPing = np.array(msg.raw_ping_data()).astype(np.float32)
    rawPingData = rawPing  # si tu veux y accéder globalement

    gains = np.ones([msg.range_count()], dtype=np.float32)
    if msg.has_gains():
        gains = np.array(msg.gains(), dtype=np.float32)

    pingData = np.array(msg.ping_data()).astype(np.float32) / np.sqrt(gains)[:, np.newaxis]
    range_sonar = msg.range()
    master      = msg.master_mode()

    test = 1
    new_frame = True


def status_callback(status):
    print("[STATUS] Got status :", status)


def switch_master_mode(sonar):
    c = sonar.current_config()
    print("[SWITCH] Current config :")
    print(c)
    if c.masterMode == 1:
        c.masterMode = 2
    else:
        c.masterMode = 1
    c.pingRate = 0
    c.range = 8
    print("[SWITCH] Config request :")
    print(c)
    print(sonar.send_config(c))


def print_config(sonar):
    print("[CONFIG] Current config :")
    print(sonar.current_config())


def Affichage_oculus():
    """Affiche un résumé du ping courant + quelques points forts dans le terminal."""

    global pingData
    global bearings
    global rawPingData
    global gains
    global range_sonar
    global master

    if pingData is None or rawPingData is None or bearings is None:
        return

    # Dimensions de la matrice sonar : (range, bearing)
    n_range, n_bearing = rawPingData.shape

    # Stats basiques
    inten_min = float(rawPingData.min())
    inten_max = float(rawPingData.max())
    inten_mean = float(rawPingData.mean())

    aperture_deg = bearings[-1] - bearings[0]

    print("\n================= NOUVEAU PING =================")
    print(f"  Master mode       : {master}")
    print(f"  Range (m)         : {range_sonar:.2f}")
    print(f"  Nb ranges (bins)  : {n_range}")
    print(f"  Nb bearings       : {n_bearing}")
    print(f"  Aperture (deg)    : {aperture_deg:.2f}")
    print(f"  Intensité min/max : {inten_min:.1f} / {inten_max:.1f}")
    print(f"  Intensité moyenne : {inten_mean:.1f}")
    if gains is not None:
        print(f"  Gains min/max     : {float(gains.min()):.2f} / {float(gains.max()):.2f}")

    # ====== Quelques points “physiques” intéressants ======
    # On cherche les N échos les plus forts
    N = 10  # nombre de points à afficher

    # On travaille sur une copie aplatie
    data_flat = rawPingData.ravel()
    # on prend les indices des N plus grandes valeurs
    if N < data_flat.size:
        top_idx_flat = np.argpartition(-data_flat, N)[:N]
    else:
        top_idx_flat = np.argsort(-data_flat)

    # Conversion indices 1D -> 2D (range_idx, bearing_idx)
    top_range_idx, top_bearing_idx = np.unravel_index(top_idx_flat, rawPingData.shape)

    # Construction de l’échelle de distance (0 → range_max)
    ranges = np.linspace(0.0, range_sonar, n_range)
    bearings_deg = bearings  # déjà en degrés

    print("\n  Points les plus forts :")
    for k in range(len(top_range_idx)):
        i = int(top_range_idx[k])    # index de range
        j = int(top_bearing_idx[k])  # index de bearing
        d = ranges[i]                # distance en m
        ang = bearings_deg[j]        # angle en degrés
        inten = float(rawPingData[i, j])
        print(f"    - d = {d:5.2f} m,  angle = {ang:6.2f} deg,  intensité = {inten:.1f}")

    

    print("================================================")


def Affichage_graphique_oculus(ax):
    """Visualisation graphique en temps réel du sonar."""
    
    global bearings
    global rawPingData
    global range_sonar
    global master
    global grid_X, grid_Y, grid_shape

    if rawPingData is None or bearings is None:
        return

    n_range, n_bearing = rawPingData.shape
    
    # Interpolation des angles si nécessaire
    if len(bearings) != n_bearing:
        bearings_interp = np.linspace(bearings[0], bearings[-1], n_bearing)
    else:
        bearings_interp = bearings
    
    # Pré-calcul de la grille (x, y) si nécessaire
    if grid_X is None or grid_Y is None or grid_shape != (n_range, n_bearing):
        print("[INFO] Pré-calcul de la grille de coordonnées...")
        
        # Conversion bearings degrés → radians + rotation de π/2
        bearings_rad = bearings_interp * np.pi / 180.0 + math.pi / 2
        
        # Construction de l'échelle de distance
        ranges = np.linspace(0.0, range_sonar, n_range)
        
        # Création des grilles avec meshgrid
        R, B = np.meshgrid(ranges, bearings_rad, indexing='ij')
        
        # Conversion polaire → cartésien
        grid_X = R * np.cos(B)
        grid_Y = R * np.sin(B)
        grid_shape = (n_range, n_bearing)
        
        print(f"[INFO] Grille pré-calculée : {grid_shape}")
    
    ax.clear()
    
    # Aplatissement des données pour scatter plot
    Lx = grid_X.ravel()
    Ly = grid_Y.ravel()
    Lintensite_xy = rawPingData.ravel()
    
    # Analyse des bords : afficher les stats des premières/dernières colonnes
    if new_frame:  # Seulement au premier affichage
        print(f"[DEBUG] Intensité colonne 0 (gauche) : min={rawPingData[:,0].min():.1f}, max={rawPingData[:,0].max():.1f}, mean={rawPingData[:,0].mean():.1f}")
        print(f"[DEBUG] Intensité colonne -1 (droite): min={rawPingData[:,-1].min():.1f}, max={rawPingData[:,-1].max():.1f}, mean={rawPingData[:,-1].mean():.1f}")
        print(f"[DEBUG] Intensité globale : min={rawPingData.min():.1f}, max={rawPingData.max():.1f}, mean={rawPingData.mean():.1f}")
    
    # Filtrage : enlever les valeurs nulles ou très faibles (bruit de fond)
    # Et limiter les valeurs extrêmes qui créent des artefacts
    seuil_min = np.percentile(Lintensite_xy, 10)  # Enlever le bruit de fond
    seuil_max = np.percentile(Lintensite_xy, 98)  # Limiter les pics extrêmes
    
    mask = (Lintensite_xy >= seuil_min) & (Lintensite_xy <= seuil_max)
    
    # Affichage scatter plot avec colormap adaptée
    ax.set_aspect('equal', 'box')
    ax.scatter(x=Lx[mask], y=Ly[mask], c=Lintensite_xy[mask], 
               cmap="viridis", s=2, vmin=seuil_min, vmax=seuil_max)
    
    # Ajout de cercles de référence pour la distance
    for r in [2, 4, 6, 8]:
        if r <= range_sonar:
            circle = plt.Circle((0, 0), r, color='k', fill=False, linewidth=0.5)
            ax.add_patch(circle)
    
    # Limites du graphique
    ax.set_xlim(-range_sonar, range_sonar)
    ax.set_ylim(-0.1, range_sonar)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'Sonar Oculus - Range: {range_sonar:.1f}m - Mode: {master}')
    ax.grid(True, alpha=0.3)


def main(args=None):

    global sonar
    global new_frame

    sonar = oculus_python.OculusSonar()
    sonar.start()
    
    sonar.add_message_callback(message_callback)
    sonar.add_ping_callback(ping_callback)
    # sonar.add_status_callback(status_callback)

    print("[MAIN] Sonar démarré, attente des pings (Ctrl+C pour quitter)...")

    # Configuration de la figure matplotlib en mode interactif
    plt.ion()
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111)

    try:
        while True:
            if new_frame:
                Affichage_oculus()  # Affichage terminal
                Affichage_graphique_oculus(ax)  # Affichage graphique
                plt.pause(0.001)  # Permet la mise à jour du graphique
                new_frame = False
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n[MAIN] Arrêt demandé par l'utilisateur.")
    finally:
        plt.close(fig)
        try:
            sonar.recorder_stop()
        except Exception:
            pass
        sonar.stop()
        print("[MAIN] Sonar arrêté proprement.")


if __name__ == '__main__':
    main()
