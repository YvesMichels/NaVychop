import os
import csv
import numpy as np
from PIL import Image
import math
import matplotlib.pyplot as plt
from scipy.ndimage import convolve
from scipy.ndimage import gaussian_filter


IDIM = 320
JDIM = 240



def load_image_u8(path):
    img = Image.open(path).convert('L')
    return np.array(img, dtype=np.int32)

def norme_l1_gradient(image):
    """
    Calcule la norme L1 du gradient d'une image
    en utilisant les filtres [-1, 1] horizontal et vertical.

    :param image: image 2D (niveaux de gris) sous forme de tableau NumPy
    :return: norme L1 du gradient
    """
    # Filtres de dérivation
    filtre_horizontal = np.array([[-1, 1]])
    filtre_vertical = np.array([[-1], [1]])

    # Calcul des gradients
    grad_x = convolve(image, filtre_horizontal, mode='reflect')
    grad_y = convolve(image, filtre_vertical, mode='reflect')

    # Norme L1 du gradient
    norme_l1 = np.abs(grad_x) + np.abs(grad_y)

    return norme_l1

index = 10

img0 = load_image_u8(f"images/{index:04d}.png")



huber_delta_q16 = int(round(10 *65536))
plt.ion()
fig = plt.figure(1)
ax = fig.add_subplot(111)
image = gaussian_filter(img0, sigma=3)
image = norme_l1_gradient(image)

ax.clear()
ax.imshow(image, cmap="gray")

plt.xlim(0, image.shape[1])
plt.ylim(image.shape[0], 0)
plt.tight_layout()
plt.show()