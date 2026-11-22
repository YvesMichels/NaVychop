#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
navychop_hybrid.py - Outil d'aide a l'hybridation multi-capteurs (Navychop)

Entrees : les CSV issus de decode_logs.py (dans un meme dossier)
    logs_fast.csv    t, qw,qx,qy,qz, altitude, ...          (25 Hz)
    logs_acc.csv     t, a_x,a_y,a_z                         (~8 Hz, repere IMU, m/s2 lineaire)
    logs_mag.csv     t, mag_x,mag_y,mag_z                   (~1 Hz, repere IMU, uT)
    logs_vision.csv  t, theta, tx, ty, div, cost            (~8 Hz, mouvement entre 2 images)
    logs_gps.csv     (ignore pour l'instant)

Ce que fait l'outil
    1. Synchronisation : rotation du gyro (quaternion GRV) entre 2 images successives
       (interpolation SLERP) -> increment de rotation phi (3 axes, repere IMU).
    2. Calibration vision <-> inertiel : retard temporel (tau), matrice A telle que
       [tx, ty, theta] ~= A . phi  (axes camera, signes, echelle pixels/rad), ajustee de
       facon robuste (le module de vision fournit des estimations aberrantes).
    3. Detection des aberrations : residu robuste (MAD), valeurs "collees" aux butees,
       et indicateur de qualite par image (a defaut de `cost` fourni par le module).
    4. Flux derote : mouvement image restant une fois la rotation gyro retiree
       (= information de translation, la seule que la vision apporte en plus du gyro).
    5. Controles complementaires : coherence magnetometre / quaternion (derive en cap),
       bruit du barometre, correlation flux derote / vitesse issue de l'accelerometre.
    6. Exports : CSV enrichi, rapport JSON (parametres a reporter dans le code embarque)
       et figures PNG.

Usage :
    python navychop_hybrid.py DOSSIER_CSV [--out DOSSIER_SORTIE] [--no-show]
    (sous Spyder : modifier DEFAULT_DIR ci-dessous puis F5)
"""
import argparse
import glob
import json
import os
import sys

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as Rot, Slerp
from scipy.signal import butter, filtfilt

import matplotlib
if "--no-show" in sys.argv:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt

DEFAULT_DIR = "."          # dossier contenant les logs_*.csv (usage Spyder)
LAG_RANGE = (-0.10, 0.50)  # plage de recherche du retard vision/inertiel (s)
LAG_STEP = 0.01
OUT_K = 3.0                # seuil aberration : |residu| > OUT_K * sigma_robuste
CH = ["tx", "ty", "theta"]
AXES_PRIOR = {"tx": "y", "ty": "x", "theta": "z"}   # a priori physique (camera solidaire de l'IMU) ; "auto" pour chercher
AXES = AXES_PRIOR


# ----------------------------------------------------------------------------
# Chargement
# ----------------------------------------------------------------------------
def find_csv(folder, key):
    """Trouve <prefixe>logs_<key>.csv (tolere un prefixe ajoute par un telechargement)."""
    c = sorted(glob.glob(os.path.join(folder, f"*logs_{key}.csv")))
    if not c:
        raise FileNotFoundError(f"logs_{key}.csv introuvable dans {folder}")
    return c[0]


def load(folder):
    d = {k: pd.read_csv(find_csv(folder, k)) for k in ["fast", "acc", "mag", "vision", "gps"]}
    f = d["fast"]
    f = f[np.r_[True, np.diff(f.t.values) > 1e-6]].reset_index(drop=True)  # temps strictement croissants
    d["fast"] = f
    return d


# ----------------------------------------------------------------------------
# Rotation gyro entre images
# ----------------------------------------------------------------------------
def attitude(fast):
    R = Rot.from_quat(fast[["qx", "qy", "qz", "qw"]].values)
    return Slerp(fast.t.values, R), fast.t.values


def regularize_times(tv):
    """Les dates de reception sont quantifiees par la boucle (40 ms). Ajuste t_k = t0 + k*T."""
    dt = np.diff(tv)
    T = np.median(dt)
    k = np.round((tv - tv[0]) / T)               # numero d'image (detecte les images perdues)
    A = np.c_[np.ones_like(k), k]
    t0, Tf = np.linalg.lstsq(A, tv, rcond=None)[0]
    return t0 + Tf * k, Tf, k


def rot_increments(S, tf, tv, tau, T):
    """Increment de rotation (rotvec, repere IMU) entre image k-1 et k, decale de tau."""
    t1 = np.clip(tv - tau, tf[0], tf[-1])
    t0 = np.clip(tv - T - tau, tf[0], tf[-1])
    return (S(t0).inv() * S(t1)).as_rotvec()


# ----------------------------------------------------------------------------
# Ajustement robuste  y = phi . a   (IRLS Huber puis moindres carres sur les meilleurs residus)
# ----------------------------------------------------------------------------
MOVE_DEG = 0.5   # une image est "en mouvement" si le gyro tourne de plus de MOVE_DEG entre 2 images


def mad_sigma(r):
    return 1.4826 * np.median(np.abs(r - np.median(r))) + 1e-9


def robust_fit(P, y, n_iter=30):
    """Moindres carres robustes (Huber puis 60 % meilleurs residus). P : (n, p)."""
    w = np.ones(len(y))
    a = np.zeros(P.shape[1])
    for _ in range(n_iter):
        W = np.sqrt(w)[:, None]
        a = np.linalg.lstsq(P * W, y * W[:, 0], rcond=None)[0]
        r = y - P @ a
        c = 1.345 * mad_sigma(r)
        w = np.where(np.abs(r) <= c, 1.0, c / np.maximum(np.abs(r), 1e-12))
    for _ in range(5):
        r = y - P @ a
        keep = np.abs(r) <= np.quantile(np.abs(r), 0.6)
        a = np.linalg.lstsq(P[keep], y[keep], rcond=None)[0]
    return a


def fit_channels(P, Y, valid, axes=None):
    """Modele structure : chaque canal vision = gain * (un seul axe gyro), axe choisi automatiquement.
    valid : (n,3) booleens, images utilisables pour l'ajustement de chaque canal (mouvement, non bloquees).
    Retourne A (3x3, une entree non nulle par ligne), residus (n,3), sigma robuste (3,), axes (3,)."""
    A = np.zeros((3, 3)); res = np.zeros_like(Y); sig = np.zeros(3); axis = [0, 0, 0]
    for k in range(3):
        m = valid[:, k]
        best = None
        cand = range(3) if axes is None else ["xyz".index(axes[CH[k]])]
        for j in cand:
            a = robust_fit(P[m][:, [j]], Y[m, k])
            r = Y[m, k] - P[m][:, [j]] @ a
            c = np.median(np.abs(r))                 # critere de choix de l'axe
            if best is None or c < best[0]:
                best = (c, j, a[0])
        _, j, g = best
        A[k, j] = g; axis[k] = j
        res[:, k] = Y[:, k] - P[:, j] * g
        r = res[m, k]
        sig[k] = mad_sigma(r[np.abs(r) <= np.quantile(np.abs(r), 0.7)]) * 1.2
    return A, res, sig, axis


def full_matrix(P, Y, valid):
    """Matrice 3x3 libre (diagnostic des couplages / defauts d'alignement camera)."""
    return np.array([robust_fit(P[valid[:, k]], Y[valid[:, k], k]) for k in range(3)])


def lag_scan(S, tf, tv, Y, T, stuck, axes=None):
    """Retard tau minimisant la mediane des residus (normalisee), images en mouvement et non bloquees."""
    taus = np.arange(LAG_RANGE[0], LAG_RANGE[1] + 1e-9, LAG_STEP)
    cost = []
    for tau in taus:
        P = rot_increments(S, tf, tv, tau, T)
        move = np.linalg.norm(P, axis=1) > np.radians(MOVE_DEG)
        valid = move[:, None] & ~stuck
        if valid.sum(axis=0).min() < 20:
            cost.append(np.inf); continue
        _, res, _, _ = fit_channels(P, Y, valid, axes)
        c = 0.0
        for k in range(3):
            m = valid[:, k]
            c += np.median(np.abs(res[m, k])) / mad_sigma(Y[m, k])
        cost.append(c)
    cost = np.array(cost)
    return taus, cost, taus[int(np.argmin(cost))]


# ----------------------------------------------------------------------------
# Aberrations
# ----------------------------------------------------------------------------
def stuck_values(y, min_count=3, thr=0.5):
    """Valeurs identiques (a 1e-6) repetees >= min_count fois et |v| > thr (px ou deg) :
    le recalage retombe sur des valeurs discretes (grille / butee) quand il n'accroche pas."""
    flag = np.zeros(len(y), bool)
    vals, cnt = np.unique(np.round(y, 6), return_counts=True)
    for v, c in zip(vals, cnt):
        if c >= min_count and abs(v) > thr:
            flag |= np.isclose(y, v, atol=1e-6)
    return flag


# ----------------------------------------------------------------------------
# Magnetometre / baro / accelerometre
# ----------------------------------------------------------------------------
def mag_check(S, tf, mag):
    t = mag.t.values
    m = mag[["mag_x", "mag_y", "mag_z"]].values
    ok = (t > tf[0]) & (t < tf[-1]) & (np.linalg.norm(m, axis=1) > 1e-6)
    R = S(t[ok])
    me = R.apply(m[ok])               # champ exprime dans le repere terrestre du GRV
    norm = np.linalg.norm(m[ok], axis=1)
    incl = np.degrees(np.arcsin(me[:, 2] / norm))
    head = np.degrees(np.arctan2(me[:, 1], me[:, 0]))   # direction du champ horizontal (cap relatif)
    return t[ok], me, norm, incl, head


def lowpass_v_from_acc(S, tf, acc, fc=0.3):
    """Vitesse terrestre = integrale de l'acceleration (derive retiree par passe-haut)."""
    t = acc.t.values
    a = acc[["a_x", "a_y", "a_z"]].values
    ok = (t > tf[0]) & (t < tf[-1])
    t, a = t[ok], a[ok]
    ae = S(t).apply(a)
    dt = np.median(np.diff(t))
    v = np.cumsum(ae, axis=0) * dt
    b, aa = butter(2, fc / (0.5 / dt), btype="high")
    return t, filtfilt(b, aa, v, axis=0), ae


# ----------------------------------------------------------------------------
# Programme principal
# ----------------------------------------------------------------------------
def main(folder, out, show=True, axes=None):
    global AXES
    AXES = axes
    os.makedirs(out, exist_ok=True)
    d = load(folder)
    S, tf = attitude(d["fast"])
    vis = d["vision"].copy()
    tv_raw = vis.t.values
    tv, T, kimg = regularize_times(tv_raw)
    n_gap = int(kimg[-1] - len(kimg) + 1)
    Y = vis[CH].values

    # --- 1) retard + calibration ---------------------------------------------------------
    stuck = np.column_stack([stuck_values(Y[:, k]) for k in range(3)])    # (n,3) par canal
    taus, cost, tau = lag_scan(S, tf, tv, Y, T, stuck, AXES)
    P = rot_increments(S, tf, tv, tau, T)
    move = np.linalg.norm(P, axis=1) > np.radians(MOVE_DEG)
    valid = move[:, None] & ~stuck
    A, res, sig, axis = fit_channels(P, Y, valid, AXES)
    Afull = full_matrix(P, Y, valid)
    # seuil d'aberration par canal : OUT_K sigma, avec un plancher = quantum de la mesure
    thr = np.maximum(OUT_K * sig, 4 / 256.)
    inl_ch = (np.abs(res) < thr) & ~stuck                                  # (n,3) canal exploitable
    inlier = inl_ch.all(axis=1)                                            # image entierement exploitable
    quality = np.clip(1 - np.max(np.abs(res) / thr, axis=1), 0, 1) * (~stuck.any(axis=1))

    # correlation (Spearman) canal x axe gyro sur les valeurs non bloquees en mouvement : sert a verifier l'a priori
    from scipy.stats import spearmanr
    corr_axes = np.array([[spearmanr(Y[valid[:, k], k], P[valid[:, k], j])[0] for j in range(3)] for k in range(3)])

    # part de variance expliquee par le gyro, canal par canal (images en mouvement retenues)
    expl = []
    for k in range(3):
        sel = move & inl_ch[:, k]
        expl.append(1 - np.var(res[sel, k]) / np.var(Y[sel, k]))

    # --- 2) flux derote -------------------------------------------------------------------
    resid = np.where(inl_ch, res, np.nan)   # mouvement image non explique par la rotation (par canal)

    # --- 3) controles annexes -------------------------------------------------------------
    tm, me, mnorm, incl, head = mag_check(S, tf, d["mag"])
    ta, vacc, ae = lowpass_v_from_acc(S, tf, d["acc"])
    # correlation flux derote (par seconde) / vitesse accelerometre (repere IMU)
    rate = resid / T
    ia = np.interp(tv, ta, np.arange(len(ta)))
    vI = np.array([np.interp(tv, ta, vacc[:, j]) for j in range(3)]).T     # terrestre
    vIMU = np.array([S(np.clip(tv, tf[0], tf[-1]))[i].inv().apply(vI[i]) for i in range(len(tv))])
    corr_v = np.full((3, 3), np.nan)
    for k in range(3):
        ok = np.isfinite(rate[:, k])
        for j in range(3):
            corr_v[k, j] = np.corrcoef(rate[ok, k], vIMU[ok, j])[0, 1]

    alt = d["fast"].altitude.values
    # --- 4) rapport -----------------------------------------------------------------------
    rep = {
        "periode_vision_s": float(T),
        "images_perdues_estimees": n_gap,
        "gigue_reception_ms_rms": float(1e3 * np.std(tv_raw - tv)),
        "retard_vision_vs_gyro_s": float(tau),
        "images_en_mouvement": int(move.sum()),
        "axe_IMU_par_canal": {CH[k]: "xyz"[axis[k]] for k in range(3)},
        "gain_par_canal_[px_ou_deg]_par_rad": {CH[k]: round(float(A[k, axis[k]]), 2) for k in range(3)},
        "correlation_canal_x_axe_gyro_(lignes tx,ty,theta ; colonnes x,y,z)": corr_axes.round(2).tolist(),
        "matrice_A_modele_structure": A.round(2).tolist(),
        "matrice_A_libre_diagnostic": Afull.round(1).tolist(),
        "sigma_residu_robuste": dict(zip(CH, sig.round(3).tolist())),
        "seuil_aberration": dict(zip(CH, thr.round(3).tolist())),
        "variance_expliquee_par_gyro_images_retenues": dict(zip(CH, np.round(expl, 2).tolist())),
        "taux_canal_utilisable_pct_images_en_mouvement": dict(zip(CH, (100 * inl_ch[move].mean(axis=0)).round(1).tolist())),
        "taux_valeurs_collees_pct_images_en_mouvement": dict(zip(CH, (100 * stuck[move].mean(axis=0)).round(1).tolist())),
        "taux_image_entierement_utilisable_pct": float(100 * inlier[move].mean()),
        "valeurs_collees_frequentes": {CH[k]: sorted({round(abs(float(x)), 3) for x in Y[stuck[:, k], k]})[:12] for k in range(3)},
        "magnetometre": {
            "norme_uT_moy": float(mnorm.mean()), "norme_uT_std": float(mnorm.std()),
            "inclinaison_deg_moy": float(incl.mean()), "inclinaison_deg_std": float(incl.std()),
            "cap_champ_deg_std": float(np.degrees(np.std(np.unwrap(np.radians(head))))),
        },
        "baro_altitude_m": {"moy": float(alt.mean()), "std": float(alt.std()), "min": float(alt.min()), "max": float(alt.max())},
        "correlation_flux_derote_vs_vitesse_acc": corr_v.round(2).tolist(),
        "lignes": "lignes = tx,ty,theta ; colonnes = vitesse IMU x,y,z",
    }
    with open(os.path.join(out, "hybrid_report.json"), "w", encoding="utf-8") as fh:
        json.dump(rep, fh, indent=2, ensure_ascii=False)

    out_df = vis.copy()
    out_df["t_reg"] = tv
    for j, n in enumerate("xyz"):
        out_df[f"gyro_phi_{n}_rad"] = P[:, j]
    for k, n in enumerate(CH):
        out_df[f"res_{n}"] = res[:, k]
    out_df["inlier"] = inlier.astype(int)
    for k, n in enumerate(CH):
        out_df[f"stuck_{n}"] = stuck[:, k].astype(int)
        out_df[f"ok_{n}"] = inl_ch[:, k].astype(int)
    out_df["quality"] = quality
    out_df.to_csv(os.path.join(out, "vision_annotated.csv"), index=False)

    # --- 5) figures -----------------------------------------------------------------------
    t0 = tv[0]
    fig, ax = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    names = ["tx (px)", "ty (px)", "theta (deg)"]
    for k in range(3):
        pred = P @ A[k]
        ax[k].plot(tv - t0, Y[:, k], ".", ms=3, color="tab:red", alpha=.5, label="vision (brut)")
        ax[k].plot(tv[inl_ch[:, k]] - t0, Y[inl_ch[:, k], k], ".", ms=4, color="k", label="vision (retenu)")
        ax[k].plot(tv - t0, pred, "-", color="tab:blue", lw=1, label="prediction gyro")
        ax[k].set_ylabel(names[k]); ax[k].grid(alpha=.3)
    ax[0].legend(ncol=3, loc="upper right")
    ax[3].plot(tv - t0, quality, color="tab:green"); ax[3].set_ylabel("qualite (0-1)")
    ax[3].set_xlabel("t (s)"); ax[3].grid(alpha=.3)
    fig.suptitle(f"Vision vs rotation gyro  (retard = {tau*1e3:.0f} ms)"); fig.tight_layout()
    fig.savefig(os.path.join(out, "fig1_vision_vs_gyro.png"), dpi=130)

    fig, ax = plt.subplots(1, 3, figsize=(14, 4.2))
    ax[0].plot(taus * 1e3, cost); ax[0].axvline(tau * 1e3, color="r", ls="--")
    ax[0].set_xlabel("retard (ms)"); ax[0].set_ylabel("residu robuste normalise"); ax[0].set_title("Recherche du retard")
    for k in range(3):
        ax[1].plot(Y[:, k], res[:, k], ".", ms=3, label=CH[k])
    ax[1].set_xlabel("mesure vision"); ax[1].set_ylabel("residu apres gyro"); ax[1].legend(); ax[1].set_title("Residus")
    ax[2].hist(np.abs(Y[:, 0]), bins=60, alpha=.6, label="|tx|"); ax[2].hist(np.abs(Y[:, 2]), bins=60, alpha=.6, label="|theta|")
    ax[2].set_title("Valeurs collees (pics)"); ax[2].legend()
    for a_ in ax: a_.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(os.path.join(out, "fig2_diagnostic_vision.png"), dpi=130)

    fig, ax = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    ax[0].plot(tm - t0, mnorm, "o-"); ax[0].set_ylabel("|B| (uT)")
    ax[1].plot(tm - t0, incl, "o-"); ax[1].set_ylabel("inclinaison champ (deg)")
    ax[2].plot(tf - t0, alt, label="baro"); ax[2].set_ylabel("altitude baro (m)")
    ax2 = ax[2].twinx(); ax2.plot(ta - t0, vacc[:, 2], color="tab:orange", lw=1); ax2.set_ylabel("vz acc (m/s, passe-haut)")
    ax[2].set_xlabel("t (s)")
    for a_ in ax: a_.grid(alpha=.3)
    fig.suptitle("Magnetometre (repere terrestre du GRV), baro, vitesse verticale accel"); fig.tight_layout()
    fig.savefig(os.path.join(out, "fig3_mag_baro_acc.png"), dpi=130)

    if show:
        plt.show()
    plt.close("all")
    print(json.dumps(rep, indent=2, ensure_ascii=False))
    return rep


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("folder", nargs="?", default=DEFAULT_DIR)
    ap.add_argument("--out", default=None)
    ap.add_argument("--no-show", action="store_true")
    ap.add_argument("--axes", default="prior",
                    help="'prior' (tx<-y, ty<-x, theta<-z), 'auto' (recherche) ou 'tx=y,ty=x,theta=z'")
    a = ap.parse_args()
    if a.axes == "prior":
        ax_ = AXES_PRIOR
    elif a.axes == "auto":
        ax_ = None
    else:
        ax_ = dict(kv.split("=") for kv in a.axes.split(","))
    main(a.folder, a.out or os.path.join(a.folder, "hybrid_out"), show=not a.no_show, axes=ax_)
