"""
Affichage des courbes de vol à partir de log.csv.

Deux figures sont produites :
1. Orientation (quaternion), accélération et magnétomètre, en sous-graphiques
   empilés partageant le même axe temporel, pour comparer facilement leurs
   dynamiques respectives.
2. La chaîne d'asservissement, de la commande au sol jusqu'aux moteurs, en
   passant par les erreurs de projection utilisées par le PID puis par les
   intégrales bornées (eta) : cmd_* -> proj_* -> eta_* -> ctrl_* -> esc.
3. Une trajectoire 3D obtenue en faisant tourner l'accélération mesurée dans
   le repère avion vers le repère monde grâce au quaternion d'orientation,
   puis en l'intégrant deux fois (vitesse puis position).

Note : les accélérations a_x, a_y, a_z sont telles que mesurées par l'IMU et
peuvent inclure la gravité. Aucune compensation n'est faite par défaut
(remove_gravity=False dans integrate_trajectory/plot_trajectory_3d) : une
double intégration sans compensation dérive rapidement, la trajectoire 3D est
donc surtout utile pour observer la forme/direction du mouvement sur des
segments courts, pas pour une position absolue fiable sur toute la durée du
vol.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (enregistre la projection 3D)

AXES = ["yaw", "pitch", "roll"]


def load_log(path="log.csv"):
    df = pd.read_csv(path)
    df["t"] = df["t"] / 1000.0  # ms -> s, plus lisible sur les graphiques
    return df


def plot_orientation_accel_mag(df):
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 9))
    fig.suptitle("Orientation, accélération et magnétomètre")

    ax = axes[0]
    for col in ["gqw", "gqx", "gqy", "gqz"]:
        ax.plot(df["t"], df[col], label=col)
    ax.set_ylabel("Quaternion (orientation)")
    ax.legend(loc="upper right")
    ax.grid(True)

    ax = axes[1]
    for col in ["a_x", "a_y", "a_z"]:
        ax.plot(df["t"], df[col], label=col)
    ax.set_ylabel("Accélération")
    ax.legend(loc="upper right")
    ax.grid(True)

    ax = axes[2]
    for col in ["mag_x", "mag_y", "mag_z"]:
        ax.plot(df["t"], df[col], label=col)
    ax.set_ylabel("Magnétomètre")
    ax.set_xlabel("Temps (s)")
    ax.legend(loc="upper right")
    ax.grid(True)

    fig.tight_layout()
    return fig


def plot_control_chain(df):
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(12, 11))
    fig.suptitle(
        "Asservissement : commande au sol -> erreur de projection -> PID -> moteurs"
    )

    ax = axes[0]
    for name in AXES:
        ax.plot(df["t"], df[f"cmd_{name}"], label=f"cmd_{name}")
    ax.set_ylabel("Commande au sol")
    ax.legend(loc="upper right")
    ax.grid(True)

    ax = axes[1]
    for name in AXES:
        ax.plot(df["t"], df[f"proj_{name}"], label=f"proj_{name}")
    ax.set_ylabel("Erreur de projection (PID)")
    ax.legend(loc="upper right")
    ax.grid(True)

    ax = axes[2]
    for name in AXES:
        ax.plot(df["t"], df[f"eta_{name}"], label=f"eta_{name}")
    ax.set_ylabel("Intégrale bornée (PID)")
    ax.legend(loc="upper right")
    ax.grid(True)

    ax = axes[3]
    for name in AXES:
        ax.plot(df["t"], df[f"ctrl_{name}"], label=f"ctrl_{name}")
    ax.plot(df["t"], df["esc"], label="esc", linestyle="--", color="black")
    ax.set_ylabel("Commandes moteurs / ESC")
    ax.set_xlabel("Temps (s)")
    ax.legend(loc="upper right")
    ax.grid(True)

    fig.tight_layout()
    return fig


def _rotate_body_to_world(df):
    """Fait tourner les vecteurs (a_x, a_y, a_z), repère avion, vers le repère
    monde à l'aide du quaternion (qw, qx, qy, qz) échantillon par échantillon.
    Retourne un tableau (N, 3).
    """
    qw = df["gqw"].to_numpy()
    qx = df["gqx"].to_numpy()
    qy = df["gqy"].to_numpy()
    qz = df["gqz"].to_numpy()
    norm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    qw, qx, qy, qz = qw / norm, qx / norm, qy / norm, qz / norm

    v = df[["a_x", "a_y", "a_z"]].to_numpy()
    vx, vy, vz = v[:, 0], v[:, 1], v[:, 2]

    r00 = 1 - 2 * (qy**2 + qz**2)
    r01 = 2 * (qx * qy - qz * qw)
    r02 = 2 * (qx * qz + qy * qw)
    r10 = 2 * (qx * qy + qz * qw)
    r11 = 1 - 2 * (qx**2 + qz**2)
    r12 = 2 * (qy * qz - qx * qw)
    r20 = 2 * (qx * qz - qy * qw)
    r21 = 2 * (qy * qz + qx * qw)
    r22 = 1 - 2 * (qx**2 + qy**2)

    wx = r00 * vx + r01 * vy + r02 * vz
    wy = r10 * vx + r11 * vy + r12 * vz
    wz = r20 * vx + r21 * vy + r22 * vz
    return np.column_stack([wx, wy, wz])


def _cumulative_trapz(y, t):
    """Intégrale cumulée de y (N, 3) par rapport à t (N,), méthode des
    trapèzes, sans dépendance à scipy. Renvoie un tableau (N, 3) démarrant à 0.
    """
    dt = np.diff(t)[:, None]
    avg = (y[1:] + y[:-1]) / 2.0
    increments = avg * dt
    return np.vstack([np.zeros((1, y.shape[1])), np.cumsum(increments, axis=0)])


def integrate_trajectory(df, remove_gravity=False, gravity_world=(0.0, 0.0, -9.81)):
    """Calcule vitesse et position dans le repère monde par double intégration
    de l'accélération, orientée via le quaternion.

    remove_gravity : si True, soustrait gravity_world (dans le repère monde,
    après rotation) avant intégration. À ajuster selon l'unité et le signe
    réels de a_x/a_y/a_z, qui ne sont pas garantis ici.
    """
    t = df["t"].to_numpy()
    accel_world = _rotate_body_to_world(df)

    if remove_gravity:
        accel_world = accel_world - np.array(gravity_world)

    vel = _cumulative_trapz(accel_world, t)
    pos = _cumulative_trapz(vel, t)
    return pos, vel


def plot_trajectory_3d(df, remove_gravity=False, gravity_world=(0.0, 0.0, -9.81)):
    pos, _ = integrate_trajectory(df, remove_gravity=remove_gravity, gravity_world=gravity_world)

    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(pos[:, 0], pos[:, 1], pos[:, 2], label="trajectoire intégrée")
    ax.scatter(*pos[0], color="green", s=40, label="départ")
    ax.scatter(*pos[-1], color="red", s=40, label="arrivée")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("Déplacement intégré (accélération + orientation)")
    ax.legend()
    fig.tight_layout()
    return fig


def main():
    df = load_log("log.csv")
    plot_orientation_accel_mag(df)
    plot_control_chain(df)
    plot_trajectory_3d(df)
    plt.show()


if __name__ == "__main__":
    main()