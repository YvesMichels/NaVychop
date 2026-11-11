import os
import csv
import numpy as np
from PIL import Image
import math
import matplotlib.pyplot as plt

Q16_SHIFT = 16
Q16_ONE   = 1 << Q16_SHIFT
Q16 = 1 << 16


IDIM = 320
JDIM = 240
Y_SHIFT     = 23
X_SHIFT     = 14
PIX_SHIFT   = 6
COORD_MASK  = 0x1FF
PIX_MASK    = 0xFF
GRAD_MASK  = 0x3F

class TransformParamQ16:
    def __init__(self, div, theta, tx, ty, vp_x, vp_y):
        self.div = div
        self.theta = theta
        self.tx = tx
        self.ty = ty
        self.vp_x = vp_x
        self.vp_y = vp_y


class Grad4Q16:
    def __init__(self):
        self.div = 0
        self.theta = 0
        self.tx = 0
        self.ty = 0
        
        
def pack_q16(x, y):
    return ((y << 16) << 32) | (x << 16)

def unpack_q16(wpt):
    x_q16 =  wpt & 0xFFFFFFFF
    y_q16 = (wpt >> 32) & 0xFFFFFFFF

    x = x_q16 >> 16
    y = y_q16 >> 16
    return x, y


def q16_mul(a, b):
    return (a * b) >> Q16_SHIFT


def q16_sin(theta_q16):
    theta = theta_q16 / Q16_ONE
    return int(math.sin(theta) * Q16_ONE)

def q16_cos(theta_q16):
    theta = theta_q16 / Q16_ONE
    return int(math.cos(theta) * Q16_ONE)


def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s,  c]])

def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[ c, 0, s],
                     [ 0, 1, 0],
                     [-s, 0, c]])

def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]])


def load_image_u8(path):
    img = Image.open(path).convert('L')
    return np.array(img, dtype=np.int32)


def load_points_csv_as_uint32(csv_path):
    points = []
    with open(csv_path, newline='') as f:
        reader = csv.DictReader(f, delimiter=',')
        for row in reader:
            y = int(row['y']) & COORD_MASK
            x = int(row['x']) & COORD_MASK
            pix = int(row['pixel']) & PIX_MASK
            grad = int(row['grad']) & GRAD_MASK

            word = (y << Y_SHIFT) | (x << X_SHIFT) | (pix << PIX_SHIFT) | grad
            points.append(word)
    return points

def read_bilineaire(image, x_q, y_q):
    """
    Lecture bilinéaire équivalente à la version C++.
    Entrées :
        image : image uint8, taille IDIMxJDIM
        x_q, y_q : coordonnées en Q16.16 (int)
    Sortie :
        pixel interpolé en Q16.16 (int)
    """

    # Partie entière
    x0 = x_q >> Q16_SHIFT
    y0 = y_q >> Q16_SHIFT

    # Gestion des bords (strictement identique au C++)
    if x0 < 0 or x0 >= IDIM - 1 or y0 < 0 or y0 >= JDIM - 1:
        return 0

    # Partie fractionnaire Q16.16
    fx = x_q & (Q16_ONE - 1)
    fy = y_q & (Q16_ONE - 1)

    fx1 = Q16_ONE - fx
    fy1 = Q16_ONE - fy

    # Accès pixels
    I00 = int(image[y0, x0])
    I10 = int(image[y0, x0+1])
    I01 = int(image[y0+1, x0])
    I11 = int(image[y0+1, x0+1])

    # Interpolation factorisée (int32 exact)
    i0 = I00 * fx1 + I10 * fx    # Q16.16
    i1 = I01 * fx1 + I11 * fx    # Q16.16

    i  = i0 * fy1 + i1 * fy      # Q32.32

    return i >> Q16_SHIFT        # Q16.16

def tranform_point_q16(x, y, p):
    
    x_s = []
    y_s = []
    c = q16_cos(p.theta)
    s = q16_sin(p.theta)
    scale = Q16_ONE + p.div

    for i in range(len(x)):

        x_q = x[i]
        y_q = y[i]

        dx = x_q - p.vp_x
        dy = y_q - p.vp_y

        dxp = q16_mul(dx, scale)
        dyp = q16_mul(dy, scale)

        dxr = q16_mul(dxp, c) - q16_mul(dyp, s)
        dyr = q16_mul(dxp, s) + q16_mul(dyp, c)

        xw = p.vp_x + dxr + p.tx
        yw = p.vp_y + dyr + p.ty
        x_s.append(xw)
        y_s.append(yw)
        
    return x_s, y_s

def integrate_camera_motion(
    p_list,
    ifov_rad_per_pix,
    Z_ref,
    show=True
):
    """
    p_list : liste de TransformParamQ16 successifs
    ifov_rad_per_pix : IFOV caméra (rad / pixel)
    Z_ref : profondeur effective moyenne (m ou unité arbitraire)
    """

    # --- État initial ---
    C = np.zeros(3)          # position caméra monde
    R = np.eye(3)            # orientation caméra → monde

    traj = [C.copy()]
    z_axes = [R[:, 2].copy()]

    for k, p in enumerate(p_list):

        # --- Paramètres image -> angles caméra ---
        yaw   = (p.tx / Q16) * ifov_rad_per_pix
        pitch = (p.ty / Q16) * ifov_rad_per_pix
        roll  = (p.theta / Q16)

        # --- Rotation incrémentale caméra ---
        R_inc = rot_z(roll) @ rot_x(pitch) @ rot_y(yaw)

        # --- Mise à jour orientation monde ---
        R = R @ R_inc

        # --- Translation caméra (dans repère caméra courant) ---
        dZ = - Z_ref * (p.div / Q16)
        t_cam = np.array([0, 0, dZ])

        # --- Passage en monde ---
        t_world = R @ t_cam
        C = C + t_world

        traj.append(C.copy())
        z_axes.append(R[:, 2].copy())

    traj = np.array(traj)
    z_axes = np.array(z_axes)

    # --- Affichage ---
    if show:
        fig = plt.figure(20)
        ax = fig.add_subplot(111, projection="3d")

        ax.plot(-traj[:, 2], traj[:, 1], traj[:, 0], "-o", label="Trajectoire caméra")

        # Affichage de l’axe optique
        for i in range(0, len(traj), max(1, len(traj)//10)):
            ax.quiver(
                -traj[i, 2], traj[i, 1], traj[i, 0],
                z_axes[i, 2], -z_axes[i, 1], -z_axes[i, 0],
                length=Z_ref * 0.2,
                color="r"
            )

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title("Déplacement caméra (axe optique variable)")
        ax.legend()
        ax.set_zlim(traj[:,2].min(), traj[:,2].max())
        ax.set_xlim(traj[:,2].min(), -traj[:,2].min())
        ax.set_ylim(traj[:,2].min(), -traj[:,2].min())
        #ax.axis("equal")
        plt.tight_layout()
        plt.show()

    return traj, z_axes

    
def ssd_huber_gradient_q16_py(points_ref, img_cur, p, huber_delta_q16):
    h, w = img_cur.shape
    h = h - 15
    w = w - 15
    g = Grad4Q16()

    c = q16_cos(p.theta)
    s = q16_sin(p.theta)
    scale = Q16_ONE + p.div

    for wpt in points_ref:
        y = (wpt >> Y_SHIFT) & COORD_MASK
        x = (wpt >> X_SHIFT) & COORD_MASK
        pix_ref = (wpt >> PIX_SHIFT) & PIX_MASK

        if x <= 0 or x >= w-1 or y <= 0 or y >= h-1:
            continue

        x_q = x << Q16_SHIFT
        y_q = y << Q16_SHIFT

        dx = x_q - p.vp_x
        dy = y_q - p.vp_y

        dxp = q16_mul(dx, scale)
        dyp = q16_mul(dy, scale)

        dxr = q16_mul(dxp, c) - q16_mul(dyp, s)
        dyr = q16_mul(dxp, s) + q16_mul(dyp, c)

        xw = p.vp_x + dxr + p.tx
        yw = p.vp_y + dyr + p.ty

        xi = xw >> Q16_SHIFT
        yi = yw >> Q16_SHIFT

        if xi <= 0 or xi >= w-1 or yi <= 0 or yi >= h-1:
            continue

        Ic = read_bilineaire(img_cur, xw, yw)
        e_q = Ic - (pix_ref << Q16_SHIFT)

        abs_e = abs(e_q)
        if abs_e <= huber_delta_q16:
            e_h = e_q
        else:
            e_h = huber_delta_q16 if e_q >= 0 else -huber_delta_q16

        Ix = (img_cur[yi, xi+1] - img_cur[yi, xi-1]) << Q16_SHIFT
        Iy = (img_cur[yi+1, xi] - img_cur[yi-1, xi]) << Q16_SHIFT

        dxd_div = q16_mul(dx, c) - q16_mul(dy, s)
        dyd_div = q16_mul(dx, s) + q16_mul(dy, c)

        dxd_th = -q16_mul(dxp, s) - q16_mul(dyp, c)
        dyd_th =  q16_mul(dxp, c) - q16_mul(dyp, s)
        Ix = int(Ix)
        Iy = int(Iy)
        dxd_div = int(dxd_div)
        dyd_div = int(dyd_div)
        dxd_th = int(dxd_th)
        dyd_th = int(dyd_th)
        e_h = int(e_h)
        g.div   += (e_h * (Ix*dxd_div + Iy*dyd_div)) >> Q16_SHIFT
        g.theta += (e_h * (Ix*dxd_th  + Iy*dyd_th )) >> Q16_SHIFT
        g.tx    +=  e_h * Ix
        g.ty    +=  e_h * Iy

    g.div   >>= Q16_SHIFT
    g.theta >>= Q16_SHIFT
    g.tx    >>= Q16_SHIFT
    g.ty    >>= Q16_SHIFT

    return g

def ssd_huber_gradient_float(
    image,
    points_ref,
    nb_points,
    params_q16,
    huber_delta=10.0
):
    # --- Conversion paramètres Q16.16 -> float ---
    div   = params_q16.div   / Q16
    theta = params_q16.theta / Q16
    tx    = params_q16.tx    / Q16
    ty    = params_q16.ty    / Q16
    vp_x  = params_q16.vp_x  / Q16
    vp_y  = params_q16.vp_y  / Q16

    c = math.cos(theta)
    s = math.sin(theta)

    g_div   = 0.0
    g_theta = 0.0
    g_tx    = 0.0
    g_ty    = 0.0

    h, w = image.shape
    h = h - 15
    w = w - 15

    for p in points_ref[:nb_points]:

        # --- Dépack ---
        y = (p >> 23) & 0x1FF
        x = (p >> 14) & 0x1FF
        pix_ref = (p >> 6) & 0xFF

        # --- Coordonnées float ---
        xf = float(x)
        yf = float(y)

        dx = xf - vp_x
        dy = yf - vp_y

        # --- Divergence ---
        scale = 1.0 + div
        dx *= scale
        dy *= scale

        # --- Rotation ---
        dxr =  c * dx - s * dy
        dyr =  s * dx + c * dy

        # --- Translation ---
        xw = vp_x + dxr + tx
        yw = vp_y + dyr + ty

        xi = int(round(xw))
        yi = int(round(yw))

        if xi < 1 or xi >= w-1 or yi < 1 or yi >= h-1:
            continue

        # --- Intensité courante ---
        I = float(read_bilineaire(image, int(round(xw*65536)), int(round(yw*65536)) ))/65536
        r = I - float(pix_ref)

        # --- Huber ---
        if abs(r) <= huber_delta:
            e = r
        else:
            e = huber_delta * (1.0 if r > 0 else -1.0)

        # --- Gradient spatial ---
        Ix = 0.5 * (float(image[yi, xi+1]) - float(image[yi, xi-1]))
        Iy = 0.5 * (float(image[yi+1, xi]) - float(image[yi-1, xi]))

        # --- Jacobien géométrique ---
        dxd_div = dxr
        dyd_div = dyr

        dxd_th  = -s * dx - c * dy
        dyd_th  =  c * dx - s * dy

        # --- Accumulation gradient ---
        g_div   += e * (Ix * dxd_div + Iy * dyd_div)
        g_theta += e * (Ix * dxd_th  + Iy * dyd_th)
        g_tx    += e * Ix
        g_ty    += e * Iy

    # --- Normalisation ---
    if nb_points > 0:
        inv_n = 1.0 / nb_points
        g_div   *= inv_n
        g_theta *= inv_n
        g_tx    *= inv_n
        g_ty    *= inv_n

    # --- Retour Q16.16 ---
    class grad4_q16_t:
        pass

    g = grad4_q16_t()
    g.div   = int(round(g_div   * Q16))
    g.theta = int(round(g_theta * Q16))
    g.tx    = int(round(g_tx    * Q16))
    g.ty    = int(round(g_ty    * Q16))

    return g

def unpack_points(points):
    xs = []
    ys = []
    pix_s = []
    grad_s = []
    for p in points:
        y = (p >> 23) & 0x1FF
        x = (p >> 14) & 0x1FF
        val_pix = (p >> 6) & 0xFF
        val_grad = p & 0x3F
        xs.append(x)
        ys.append(y)
        pix_s.append(val_pix)
        grad_s.append(val_grad)
    return np.array(xs), np.array(ys), np.array(pix_s), np.array(grad_s)

def transform_points_float(points, params):
    xs_t = []
    ys_t = []

    div   = params.div   / Q16
    theta = params.theta / Q16
    tx    = params.tx    / Q16
    ty    = params.ty    / Q16
    vp_x  = params.vp_x  / Q16
    vp_y  = params.vp_y  / Q16

    c = np.cos(theta)
    s = np.sin(theta)

    for p in points:
        y = (p >> 23) & 0x1FF
        x = (p >> 14) & 0x1FF

        dx = x - vp_x
        dy = y - vp_y

        scale = 1.0 + div
        dx *= scale
        dy *= scale

        dxr =  c * dx - s * dy
        dyr =  s * dx + c * dy

        xt = vp_x + dxr + tx
        yt = vp_y + dyr + ty

        xs_t.append(xt)
        ys_t.append(yt)

    return np.array(xs_t), np.array(ys_t)


def show_image_with_points(image, points_ref, params, ax, title=""):
    xs, ys, pix_s, grad_s = unpack_points(points_ref)
    xs_t, ys_t = transform_points_float(points_ref, params)

    ax.clear()
    ax.imshow(image, cmap="gray", vmin=0, vmax=255)
    ax.scatter(xs, ys, s=(3*grad_s+1), marker='+', c="lime", label="ref", alpha=0.7)
    ax.scatter(xs_t, ys_t, s=(3*grad_s+1), marker='+', c="red", label="transformed", alpha=0.7)
    ax.legend()
    plt.title(title)
    plt.xlim(0, image.shape[1])
    plt.ylim(image.shape[0], 0)
    plt.tight_layout()
    #plt.show()
    
def show_dual_scale(img_d, pts_d, p_d, img_f, pts_f, p_f, fig):
    plt.figure(1)
    fig.clf()

    ax1 = fig.add_subplot(121)
    ax2 = fig.add_subplot(122)

    show_image_with_points(img_d, pts_d, p_d, ax1, "Décimée")
    show_image_with_points(img_f, pts_f, p_f, ax2, "Pleine résolution")

    plt.tight_layout()
    
def run_gradient_on_pair(index, nb_iter=100, alpha_q16=1<<8):
    img0 = load_image_u8(f"images/{index:04d}.png")
    points = load_points_csv_as_uint32(f"points/{index+1:04d}.csv")

    p = TransformParamQ16(
        div=0,
        theta=0,
        tx=0,
        ty=0,
        vp_x=(160-8) << Q16_SHIFT,
        vp_y=(120-8) << Q16_SHIFT
    )

    huber_delta_q16 = int(round(10 *65536))
    plt.ion()
    fig = plt.figure(1)
    ax = fig.add_subplot(111)
    for it in range(nb_iter):
        #g = ssd_huber_gradient_q16_py(points, img0, p, huber_delta_q16)
        g = ssd_huber_gradient_float(img0, points, 300, p, huber_delta_q16/65536)

        # descente de gradient (simple)
        p.div   -= q16_mul(1, g.div)/16
        p.theta -= q16_mul(1, g.theta)/16
        p.tx    -= q16_mul(1500, g.tx)
        p.ty    -= q16_mul(1500, g.ty)

        if it % 10 == 0:
            print(
                f"it {it:3d} | "
                f"div={p.div / Q16:+.6f}  "
                f"th={p.theta / Q16:+.6f}  "
                f"tx={p.tx / Q16:+.3f}  "
                f"ty={p.ty / Q16:+.3f}")
            show_image_with_points(
                img0,
                points,
                p,
                ax, title=f"it={it}")
            plt.pause(0.1)

    return p

def ssd_huber_cost_q16(points_ref, img_cur, p, huber_delta_q16):
    h, w = img_cur.shape
    h -= 7 # 7 pour l'image décimée
    w -= 7 # 7 pour l'image décimée

    c = q16_cos(p.theta)
    s = q16_sin(p.theta)
    scale = Q16_ONE + p.div

    cost = 0
    n = 0

    for wpt in points_ref:
        y = (wpt >> Y_SHIFT) & COORD_MASK
        x = (wpt >> X_SHIFT) & COORD_MASK
        pix_ref = (wpt >> PIX_SHIFT) & PIX_MASK

        if x <= 0 or x >= w-1 or y <= 0 or y >= h-1:
            continue

        x_q = x << Q16_SHIFT
        y_q = y << Q16_SHIFT

        dx = x_q - p.vp_x
        dy = y_q - p.vp_y

        dxp = q16_mul(dx, scale)
        dyp = q16_mul(dy, scale)

        dxr = q16_mul(dxp, c) - q16_mul(dyp, s)
        dyr = q16_mul(dxp, s) + q16_mul(dyp, c)

        xw = p.vp_x + dxr + p.tx
        yw = p.vp_y + dyr + p.ty

        xi = xw >> Q16_SHIFT
        yi = yw >> Q16_SHIFT

        if xi <= 0 or xi >= w-1 or yi <= 0 or yi >= h-1:
            continue

        Ic = read_bilineaire(img_cur, xw, yw)
        e_q = Ic - (pix_ref << Q16_SHIFT)
        ae = abs(e_q)
        if ae > huber_delta_q16:
            ae = q16_mul(ae, 2*huber_delta_q16)
        else:
            ae = q16_mul(ae, ae)

        cost += ae
        n += 1

    return cost // max(1, n)

def exhaustive_init_with_display(points, img, p_ref, huber_delta_q16, fig):
    theta_vals = [
        int(-0.05 * Q16),
        0,
        int(+0.05 * Q16)
    ]

    costs = np.zeros((3, 7, 7), dtype=np.int64)

    best_cost = None
    best_p = None

    # --- Calcul exhaustif ---
    for k, th in enumerate(theta_vals):
        for iy, ty in enumerate(range(-3, 4)):
            for ix, tx in enumerate(range(-3, 4)):

                p_test = TransformParamQ16(
                    div=0,
                    theta=th,
                    tx=tx * Q16,
                    ty=ty * Q16,
                    vp_x=p_ref.vp_x,
                    vp_y=p_ref.vp_y
                )

                c = ssd_huber_cost_q16(
                    points,
                    img,
                    p_test,
                    huber_delta_q16
                )

                costs[k, iy, ix] = c

                if best_cost is None or c < best_cost:
                    best_cost = c
                    best_p = p_test

    # --- Affichage ---
    fig.clf()
    vmin = 0
    vmax = costs.max()/65536

    titles = [
        "theta = -0.05 rad",
        "theta =  0.00 rad",
        "theta = +0.05 rad"
    ]

    for k in range(3):
        ax = fig.add_subplot(1, 3, k+1)
        im = ax.imshow(
            costs[k]/65536,
            cmap="viridis",
            vmin=vmin,
            vmax=vmax,
            origin="upper"
        )

        ax.set_title(titles[k])
        ax.set_xticks(range(5))
        ax.set_yticks(range(5))
        ax.set_xticklabels([-3, -2, -1, 0, 1, 2, 3])
        ax.set_yticklabels([-3, -2, -1, 0, 1, 2, 3])
        ax.set_xlabel("tx")
        ax.set_ylabel("ty")

    #fig.colorbar(im, ax=fig.axes, shrink=0.8)
    plt.tight_layout()
    plt.pause(0.1)

    return best_p, best_cost

    
def gradient_huber_2ech(
    index,
    nb_iter=100,
    step0_tx=1.0,
    step0_ty=1.0,
    step0_theta=0.02,
    step0_div=0.01
):
    # --- Chargement ---
    img_full = load_image_u8(f"images/{index+1:04d}.png")
    img_dec  = load_image_u8(f"images_r/{index+1:04d}.png")

    points_full = load_points_csv_as_uint32(f"points/{index:04d}.csv")
    points_dec  = load_points_csv_as_uint32(f"points_r/{index:04d}.csv")


    #plt.ion()
    fig = plt.figure(1)

    # =====================
    # Paramètres initiaux (pleine résolution)
    # =====================
    p = TransformParamQ16(
        div=0,
        theta=0,
        tx=0,
        ty=0,
        vp_x=(160 - 8) << Q16_SHIFT,
        vp_y=(120 - 8) << Q16_SHIFT
    )

    # =====================
    # ÉCHELLE DÉCIMÉE (COARSE)
    # =====================
    p_d = TransformParamQ16(
        div=p.div,
        theta=p.theta,
        tx=p.tx >> 3,
        ty=p.ty >> 3,
        vp_x=(p.vp_x >> 3)-(7<< Q16_SHIFT),
        vp_y=(p.vp_y >> 3)-(7<< Q16_SHIFT)
    )
    
    # =====================
    # INITIALISATION ROBUSTE + VISUALISATION
    # =====================
    fig_init = plt.figure(10)
    p_best, best_cost = exhaustive_init_with_display(
        points_dec,
        img_dec,
        p_d,
        huber_delta_q16=10 * Q16,
        fig=fig_init
    )

    p_d.div   = p_best.div
    p_d.theta = p_best.theta
    p_d.tx    = p_best.tx
    p_d.ty    = p_best.ty

    print(
        "[init coarse] "
        f"cost={best_cost} "
        f"th={p_d.theta/Q16:+.3f} "
        f"tx={p_d.tx/Q16:+.1f} "
        f"ty={p_d.ty/Q16:+.1f}"
    )

    alpha = 1
    for it in range(nb_iter):
        # --- Pas affine ---
        
        alpha = alpha * 0.9

        step_tx    = step0_tx    * alpha
        step_ty    = step0_ty    * alpha
        step_theta = step0_theta * alpha
        step_div   = step0_div   * alpha

        # --- Gradient ---
        g = ssd_huber_gradient_q16_py(points_dec, img_dec, p_d, 10*65536)
        # g = ssd_huber_gradient_float(
        #     img_dec,
        #     points_dec,
        #     len(points_dec),
        #     p_d,
        #     huber_delta=10.0
        # )

        # --- Mise à jour par signe ---
        if g.tx > 0:
            p_d.tx -= int(step_tx * Q16)
        elif g.tx < 0:
            p_d.tx += int(step_tx * Q16)

        if g.ty > 0:
            p_d.ty -= int(step_ty * Q16)
        elif g.ty < 0:
            p_d.ty += int(step_ty * Q16)

        if g.theta > 0:
            p_d.theta -= int(step_theta * Q16)
        elif g.theta < 0:
            p_d.theta += int(step_theta * Q16)

        if g.div > 0:
            p_d.div -= int(step_div * Q16)
        elif g.div < 0:
            p_d.div += int(step_div * Q16)
            
        if it % 50 == 0:
            print(
                f"[fine] it {it:3d} | "
                f"div={p.div/Q16:+.6f} "
                f"th={p.theta/Q16:+.6f} "
                f"tx={p.tx/Q16:+.3f} "
                f"ty={p.ty/Q16:+.3f}"
                f"alpha={alpha:+.3f}"
            )

            show_dual_scale(
                img_dec, points_dec, p_d,
                img_full, points_full, p,
                fig
            )
            plt.pause(0.1)

    # =====================
    # REMONTÉE D'ÉCHELLE
    # =====================
    p = TransformParamQ16(
        div=p_d.div,
        theta=p_d.theta,
        tx=p_d.tx << 3,
        ty=p_d.ty << 3,
        vp_x=p_d.vp_x << 3,
        vp_y=p_d.vp_y << 3
    )

    # =====================
    # ÉCHELLE FINE
    # =====================
    
    alpha = 1
    for it in range(nb_iter):
        alpha = alpha * 0.9

        step_tx    = step0_tx    * alpha
        step_ty    = step0_ty    * alpha
        step_theta = step0_theta * alpha
        step_div   = step0_div   * alpha

        g = ssd_huber_gradient_q16_py(points_full, img_full, p, 10*65536)
        # g = ssd_huber_gradient_float(
        #     img_full,
        #     points_full,
        #     min(300, len(points_full)),
        #     p,
        #     huber_delta=10.0
        # )

        if g.tx > 0:
            p.tx -= int(step_tx * Q16)
        elif g.tx < 0:
            p.tx += int(step_tx * Q16)

        if g.ty > 0:
            p.ty -= int(step_ty * Q16)
        elif g.ty < 0:
            p.ty += int(step_ty * Q16)

        if g.theta > 0:
            p.theta -= int(step_theta * Q16)
        elif g.theta < 0:
            p.theta += int(step_theta * Q16)

        if g.div > 0:
            p.div -= int(step_div * Q16)
        elif g.div < 0:
            p.div += int(step_div * Q16)


        if it % 100 == 0:
            print(
                f"[fine] it {it:3d} | "
                f"div={p.div/Q16:+.6f} "
                f"th={p.theta/Q16:+.6f} "
                f"tx={p.tx/Q16:+.3f} "
                f"ty={p.ty/Q16:+.3f}"
                f"alpha={alpha:+.3f}"
            )

            show_dual_scale(
                img_dec, points_dec, p_d,
                img_full, points_full, p,
                fig
            )
            plt.pause(0.1)

    return p


x0 = 182
y0 = 70
d  = 10

x_q16 = [
    (x0 + d) * Q16,
    (x0 - d) * Q16,
    x0 * Q16,
    x0 * Q16,
    x0 * Q16,
]

y_q16 = [
    y0 * Q16,
    y0 * Q16,
    y0 * Q16,
    (y0 + d) * Q16,
    (y0 - d) * Q16,
]

plt.ion()
fig2 = plt.figure(2)
ax = fig2.subplots()

plt.pause(5)

p_history = []
for it in range(1, 29):

    img_full = load_image_u8(f"images/{it+1:04d}.png")

    # Cette fonction vole la figure active
    p = gradient_huber_2ech(it, 100, 1, 1, 0.02, 0.01)
    p_history.append(p)

    # Reprendre explicitement la main
    plt.figure(fig2.number)

    #x_q16, y_q16 = tranform_point_q16(x_q16, y_q16, p)

    ax.clear()
    ax.imshow(img_full, cmap="gray", vmin=0, vmax=255)

    xc = np.array([x >> 16 for x in x_q16])
    yc = np.array([y >> 16 for y in y_q16])

    ax.plot(xc, yc, c="red", alpha=0.7)

    ax.set_title("tracking")
    ax.set_xlim(0, 320)
    ax.set_ylim(240, 0)
    
    traj, z_axes = integrate_camera_motion(p_history, ifov_rad_per_pix=0.003627, Z_ref=50.0)
    
    plt.pause(0.1)


