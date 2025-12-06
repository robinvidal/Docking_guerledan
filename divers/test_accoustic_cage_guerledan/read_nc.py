import xarray as xr
import matplotlib.pyplot as plt

# Charger le fichier
ds = xr.open_dataset("pack_mousse.nc")
print(ds)

# Les données sonar sont dans 'backscatter'
I = ds["backscatter"].values  # shape: (ping, sample, beam)
print("Shape:", I.shape)

# Affiche la première trame
frame0 = I[0]
plt.imshow(frame0, cmap="gray", aspect="auto")
plt.title("Oculus – trame 0 (backscatter)")
plt.xlabel("Beam index")
plt.ylabel("Sample index (distance)")
plt.show()
