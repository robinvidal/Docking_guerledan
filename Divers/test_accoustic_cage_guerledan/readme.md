# GOAL :Transformer un fichier `.oculus` en format **NetCDF4** exploitable :

# Qu'est ce que bps_oculus_io ?

bps_oculus_io = Blueprint Subsea Oculus Input/Output

C’est un convertisseur et parseur de fichiers .oculus :

il lit les fichiers d’enregistrement du logiciel Oculus ViewPoint (formats V1 et V2),

il décode les trames sonar binaires (backscatter, angle, portée, etc.),

et il peut exporter les données dans plusieurs formats faciles à exploiter, dont :

NetCDF4 (--output netcdf4) → pour traitement scientifique (xarray, numpy…),

Vidéo (.avi) ou images (selon les options disponibles),

ROS bag (--output rosbags) pour intégration dans ROS.


# Installation du module Python contenant bps_oculus_io
pip install --user --upgrade oculus-python

# Vérification que le module est bien installé et reconnu
python3 - <<'PY'
import bps_oculus, importlib.metadata as im
print("module:", bps_oculus.__file__)
print("version:", im.version("oculus-python"))
PY

# Modifier bibliotèque 

trouver le fichier qui finit par /bps_oculus/core.py en lancant cette commmande bps_oculus_io pack_mousse.oculus --output netcdf4
# remplacer NAN par nan dans 
gain_table = np.frombuffer(gain_payload, np.uint32) if gain_payload is not None else np.full((item_message.nRanges,), np.NAN, np.float32)



# Conversion du fichier sonar .oculus en .nc (format NetCDF4)
cd ~/Documents/test_accoustic_cage_guerledan  
bps_oculus_io pack_mousse.oculus --output netcdf4

# Un fichier .nc a été créé

# Lecture et affichage en Python
python3 - <<'PY'
import xarray as xr
import matplotlib.pyplot as plt

ds = xr.open_dataset("pack_mousse.nc")
I = ds["backscatter"].values  # (ping, sample, beam)
plt.imshow(I[0], cmap="gray", aspect="auto")
plt.title("Oculus – trame 0 (backscatter)")
plt.show()
PY

