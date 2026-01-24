 
import cv2
import numpy as np
import time
import matplotlib.pyplot as plt

# ---------------------------
# Paramètres
# ---------------------------
CAMERA_INDEX = 0
WIDTH = 160
HEIGHT = 120
DURATION_SEC = 10.0          # 10 à 60 s selon besoin
THRESH_FACTOR = 0.6         # seuil relatif (adaptatif)

# ---------------------------
# Initialisation caméra
# ---------------------------
cap = cv2.VideoCapture(CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

# Désactiver automatismes
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)   # ou 0.25 selon OS
cap.set(cv2.CAP_PROP_AUTO_WB, 0)

# Fixer valeurs
cap.set(cv2.CAP_PROP_EXPOSURE, -6)
cap.set(cv2.CAP_PROP_GAIN, 0)
cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 4500)

if not cap.isOpened():
    raise RuntimeError("Impossible d'ouvrir la caméra")

mean_signal = []
timestamps = []

print("===== Demarrage =====")
# ---------------------------
# Acquisition temporelle
# ---------------------------

# attend une touche (Entrée)
input("Appuyez sur une touche pour continuer...")
t_start = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    mean_val = gray.mean()

    mean_signal.append(mean_val)
    timestamps.append(time.time())
    if timestamps[-1] - t_start >= DURATION_SEC:
        break

cap.release()

timestamps = np.array(timestamps)
mean_signal = np.array(mean_signal)

# ---------------------------
# Framerate Python réel
# ---------------------------
dt = np.diff(timestamps)
fps_python = 1.0 / dt.mean()

# ---------------------------
# Détection des flashs
# ---------------------------
# Normalisation
sig = (mean_signal - mean_signal.min())
sig /= sig.max()

# Seuil adaptatif
threshold = THRESH_FACTOR * np.mean(sig)

binary = sig > threshold

# Détection fronts montants
edges = np.diff(binary.astype(int))
flash_indices = np.where(edges == 1)[0]

n_flashes = len(flash_indices)

# Fréquence flash estimée
duration = timestamps[-1] - timestamps[0]
flash_freq = n_flashes / duration

# ---------------------------
# Affichage résultats
# ---------------------------
print("===== Résultats =====")
print(f"Durée observation      : {duration:.2f} s")
print(f"FPS Python mesuré      : {fps_python:.2f} Hz")
print(f"Nombre de flashs       : {n_flashes}")
print(f"Fréquence flash estimée: {flash_freq:.2f} Hz")

# ---------------------------
# Visualisation
# ---------------------------
plt.figure(figsize=(10, 5))

plt.subplot(2, 1, 1)
plt.title("Signal temporel moyenne_image(t)")
plt.plot(timestamps - timestamps[0], sig, label="Signal normalisé")
plt.axhline(threshold, color='r', linestyle='--', label="Seuil")
plt.legend()
plt.xlabel("Temps (s)")
plt.ylabel("Luminance")

plt.subplot(2, 1, 2)
plt.title("Détection des flashs")
plt.plot(timestamps[:-1] - timestamps[0], binary[:-1], label="Signal binaire")
plt.scatter(timestamps[flash_indices] - timestamps[0],
            np.ones_like(flash_indices),
            color='r', label="Flashs détectés")
plt.ylim(-0.2, 1.2)
plt.legend()
plt.xlabel("Temps (s)")

plt.tight_layout()
plt.show()
