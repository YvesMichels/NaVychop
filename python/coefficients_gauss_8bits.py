import numpy as np 
import matplotlib.pyplot as plt

x = np.array(range(-4, 5))
sigma = 3

fig, ax = plt.subplots()
y = np.exp(-(np.multiply(x,x)/(sigma*sigma)))
cum_gauss = sum(y)
y = np.round(y*256/cum_gauss)
cum_gauss = sum(y)
ax.plot(x, y)
plt.show()

print(cum_gauss)
print(y)

# x = np.array(range(-7, 8))
# sigma = 4

# fig, ax = plt.subplots()
# y = np.exp(-(np.multiply(x,x)/(sigma*sigma)))
# cum_gauss = sum(y)
# y = np.round(y*256/cum_gauss)
# cum_gauss = sum(y)
# ax.plot(x, y)
# plt.show()

# print(cum_gauss)
# print(y)

# # reduction des coeffs avec un masque [1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1]
# y[0] = y[0] + np.floor(y[1]/2) # prio à droite
# y[2] = y[2] + np.ceil(y[1]/2)
# y[1] = 0

# y[2] = y[2] + np.floor(y[3]/2) # prio à droite
# y[4] = y[4] + np.ceil(y[3]/2)
# y[3] = 0

# y[4] = y[4] + np.ceil(y[5]/2) # prio à gauche
# y[6] = y[6] + np.floor(y[5]/2)
# y[5] = 0

# y[8] = y[8] + np.floor(y[9]/2) # prio à droite
# y[10] = y[10] + np.ceil(y[9]/2)
# y[9] = 0

# y[10] = y[10] + np.ceil(y[11]/2) # prio à gauche
# y[12] = y[12] + np.floor(y[11]/2)
# y[11] = 0

# y[12] = y[12] + np.ceil(y[13]/2) # prio à gauche
# y[14] = y[14] + np.floor(y[13]/2)
# y[13] = 0

# cum_gauss = sum(y)
# ax.plot(x, y)
# plt.show()

# print(cum_gauss)
# print(y)

# # test coefficients proposés par chatGPT (pour une gaussienne sigma = 4): 
# y = np.array([4,  0, 18, 0, 29, 0, 41, 75, 41, 0, 27, 0, 17, 0, 4])
# cum_gauss = sum(y)
# ax.plot(x, y)
# plt.show()

# print(cum_gauss)
# print(y)

# # test coefficients adaptés manuellement (pour une gaussienne sigma = 4): 
# y = np.array([ 4,  0, 16,  0, 36,  0, 47, 50, 47,  0, 36,  0, 16,  0,  4])
# cum_gauss = sum(y)
# ax.plot(x, y)
# plt.show()

# print(cum_gauss)
# print(y)

# # test gaussienne à trou
# x_trou = [-7, -5, -3, -1, 0, 1, 3, 5, 7]
# y_trou = np.exp(-(np.multiply(x_trou,x_trou)/(sigma*sigma)))
# cum_gauss = sum(y_trou)
# y_trou = np.round(y_trou*256/cum_gauss)
# cum_gauss = sum(y_trou)
# print(cum_gauss)
# print(y_trou)
