import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

##############################################################################
# Chargement
##############################################################################

df = pd.read_csv("log.csv")

##############################################################################
# Suppression des lignes invalides
##############################################################################

# Norme du quaternion GMV
norm = np.sqrt(
    df["gqw"]**2 +
    df["gqx"]**2 +
    df["gqy"]**2 +
    df["gqz"]**2
)

# Conserver uniquement les lignes valides
mask = (
    (df["t"] > 0) &
    (norm > 1e-6)
)

df = df.loc[mask].reset_index(drop=True)

# Temps en secondes
t = (df["t"] - df["t"].iloc[0]) / 1000.0


##############################################################################
# Quaternion -> angles de Tait-Bryan
##############################################################################

# scipy attend x,y,z,w
quat = np.column_stack([
    df["gqx"],
    df["gqy"],
    df["gqz"],
    df["gqw"]
])

rot = R.from_quat(quat)

# convention aéronautique ZYX
angles = rot.as_euler("ZYX", degrees=True)

yaw = angles[:,0]
pitch = angles[:,1]
roll = angles[:,2]

##############################################################################
# Figure
##############################################################################

fig, axs = plt.subplots(
    5,
    1,
    figsize=(16,12),
    sharex=True
)

##############################################################################
# Accélérations
##############################################################################

axs[0].plot(t, df["ax"], label="Ax")
axs[0].plot(t, df["ay"], label="Ay")
axs[0].plot(t, df["az"], label="Az")
axs[0].set_ylabel("Acceleration")
axs[0].grid(True)
axs[0].legend()

##############################################################################
# Angles
##############################################################################

axs[1].plot(t, yaw, label="Yaw")
axs[1].plot(t, pitch, label="Pitch")
axs[1].plot(t, roll, label="Roll")
axs[1].set_ylabel("Angle (°)")
axs[1].grid(True)
axs[1].legend()

##############################################################################
# Erreur quaternion MV -> GMV
##############################################################################

quat_mv = np.column_stack([
    df["qx"],
    df["qy"],
    df["qz"],
    df["qw"]
])

quat_gmv = np.column_stack([
    df["gqx"],
    df["gqy"],
    df["gqz"],
    df["gqw"]
])

r_mv = R.from_quat(quat_mv)
r_gmv = R.from_quat(quat_gmv)

# Quaternion d'erreur
r_err = r_mv.inv() * r_gmv

# Rotation vector (rad)
rotvec = r_err.as_rotvec()

# degrés
rotvec = np.degrees(rotvec)

err_roll  = rotvec[:,0]
err_pitch = rotvec[:,1]
err_yaw   = rotvec[:,2]

# norme de l'erreur
err_norm = np.linalg.norm(rotvec, axis=1)

##############################################################################
# Erreur quaternion
##############################################################################

axs[2].plot(t, err_roll,  label="ΔRoll")
axs[2].plot(t, err_pitch, label="ΔPitch")
axs[2].plot(t, err_yaw,   label="ΔYaw")
axs[2].plot(t, err_norm, '--k', linewidth=1, label="Norme")

axs[2].set_ylabel("Erreur (°)")
axs[2].grid(True)
axs[2].legend()

##############################################################################
# Pression
##############################################################################

axs[3].plot(t, df["altitude"], label="Pression")
axs[3].set_ylabel("Pression")
axs[3].grid(True)
axs[3].legend()

##############################################################################
# Commandes
##############################################################################

axs[4].plot(t, df["ctrl_yaw"], label="Yaw")
axs[4].plot(t, df["ctrl_pitch"], label="Pitch")
axs[4].plot(t, df["ctrl_roll"], label="Roll")
axs[4].plot(t, df["esc"], label="ESC")
axs[4].set_ylabel("Commandes")
axs[4].set_xlabel("Temps (s)")
axs[4].grid(True)
axs[4].legend()

plt.tight_layout()
plt.show()