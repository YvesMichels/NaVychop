# Simulateur navychop : modèle physique et tutoriel de test d'algorithmes

Ce document a deux parties :

- **Partie A – Modèle physique** : ce que le simulateur calcule exactement (équations, paramètres, hypothèses, limites).
- **Partie B – Tutoriel** : comment tester un algorithme de guidage et une hybridation de capteurs, mesurer ses performances et éviter les pièges classiques.

Toutes les valeurs chiffrées ont été obtenues en exécutant le code livré (graine de bruit 0, pas de temps 10 ms), sauf mention contraire.

**Sommaire**

- [Partie A – Modèle physique](#partie-a--modèle-physique)
  - [A1. Vue d'ensemble](#a1-vue-densemble)
  - [A2. Repères et conventions](#a2-repères-et-conventions)
  - [A3. Équations du mouvement](#a3-équations-du-mouvement)
  - [A4. Efforts aérodynamiques et poussée](#a4-efforts-aérodynamiques-et-poussée)
  - [A5. Autopilote bas niveau : de la commande haut niveau aux couples](#a5-autopilote-bas-niveau--de-la-commande-haut-niveau-aux-couples)
  - [A6. Intégration numérique et ordre des opérations](#a6-intégration-numérique-et-ordre-des-opérations)
  - [A7. Capteurs simulés](#a7-capteurs-simulés)
  - [A8. Paramètres par défaut](#a8-paramètres-par-défaut)
  - [A9. Comportement caractéristique (valeurs mesurées)](#a9-comportement-caractéristique-valeurs-mesurées)
  - [A10. Hypothèses, limites, et comment recaler sur navychop](#a10-hypothèses-limites-et-comment-recaler-sur-navychop)
- [Partie B – Tutoriel](#partie-b--tutoriel)
  - [B0. Prise en main](#b0-prise-en-main)
  - [B1. Anatomie d'un algorithme : ce que voit et ce que renvoie le code](#b1-anatomie-dun-algorithme)
  - [B2. Définir des balises et des conditions de passage](#b2-définir-des-balises-et-des-conditions-de-passage)
  - [B3. Écrire un algorithme de guidage](#b3-écrire-un-algorithme-de-guidage)
  - [B4. Mesurer : métriques, benchmark, Monte-Carlo](#b4-mesurer--métriques-benchmark-monte-carlo)
  - [B5. Hybridation de capteurs (GPS + IMU + baromètre)](#b5-hybridation-de-capteurs-gps--imu--baromètre)
  - [B6. Dégrader ou ajouter un capteur](#b6-dégrader-ou-ajouter-un-capteur)
  - [B7. Exercices proposés](#b7-exercices-proposés)
  - [B8. Pièges courants](#b8-pièges-courants)
  - [B9. Simulation interactive : voir en direct et prendre la main](#b9-simulation-interactive--voir-en-direct-et-prendre-la-main)

---

# Partie A – Modèle physique

## A1. Vue d'ensemble

L'avion est un **solide rigide à 6 degrés de liberté** (3 en position, 3 en rotation) soumis à la gravité, à des efforts aérodynamiques simples et à une poussée. Il ne se pilote pas par gouvernes mais par **quatre commandes haut niveau** : angle de roulis, angle de tangage, vitesse de lacet, poussée. Une boucle d'attitude interne (l'« autopilote bas niveau ») les convertit en couples.

Ce découpage reproduit la structure réelle de navychop : le projet à développer est la couche de **navigation et guidage**, qui ne voit que des mesures capteurs et ne parle à l'avion que par ces quatre commandes.

```
                       état VRAI (inaccessible à l'algorithme)
                                   │
   ┌───────────────┐   mesures     ▼          ┌─────────────────┐
   │   Capteurs    │◄──────────────────────── │     Avion       │
   │ GPS IMU baro  │  (bruit, cadence,        │  solide 6 ddl   │
   │ Pitot         │   biais, retard)         │  + autopilote   │
   └──────┬────────┘                          │  d'attitude     │
          │ dict de mesures                   └────────▲────────┘
          ▼                                            │ Command
   ┌───────────────┐   NavState   ┌──────────────────┐ │ (roll, pitch,
   │  Estimateur   │ ───────────► │    Guidage /     │─┘  yaw_rate,
   │ (hybridation) │              │    contrôleur    │    throttle)
   └───────────────┘              │ + Mission/balises│
                                  └──────────────────┘
```

À chaque pas de 10 ms : **1)** les capteurs échantillonnent l'état vrai, **2)** l'estimateur et le contrôleur calculent une commande, **3)** l'avion intègre sa dynamique avec cette commande. Les coordonnées GPS sont donc produites au fil de la simulation, pas pré-calculées.

| Fichier | Contenu |
|---|---|
| `aircraft.py` | Dynamique, efforts, autopilote d'attitude, classe `Command` |
| `sensors.py` | `GPS`, `IMU`, `Barometer`, `Pitot`, `SensorSuite` |
| `geo.py` | Conversion repère local NED ↔ latitude/longitude |
| `mission.py` | `Waypoint`, `Mission`, générateurs `circle()` et `figure_eight()` |
| `estimators.py` | Interface `Estimator`, estimateur de référence, filtre de Kalman GPS/IMU/baro |
| `controller.py` | Interface `Controller`, guidage L1 |
| `simulator.py` | Boucle capteurs → contrôleur → dynamique : `run()` (simulation complète, journal `SimLog`) et `step()` (un pas, pour l'interactif) |
| `live_sim.py` | Fenêtre interactive : trajectoire en direct et pilotage manuel au clavier (voir B9) |
| `benchmark.py` | Scénarios et métriques chiffrées pour comparer des algorithmes |
| `exemples/` | Trois scripts du tutoriel |

## A2. Repères et conventions

- **Repère monde NED** : x = Nord, y = Est, z = Bas. L'altitude vaut donc `−z`.
- **Repère avion FRD** : x = avant, y = droite, z = bas.
- **Angles d'Euler (ZYX)** : roulis φ > 0 = aile droite vers le bas ; tangage θ > 0 = nez vers le haut ; cap ψ > 0 = nez vers la droite (ψ = 0 : Nord, ψ = 90° : Est).
- **Quaternion** `q = [w, x, y, z]` : rotation repère avion → repère monde. `R(q)` est la matrice associée.
- **Vitesse de rotation** `ω = (p, q, r)` exprimée dans le repère avion.
- **Géographie** : terre plate autour d'une origine `GeoOrigin(lat0, lon0, alt0)` ; `Nord = Δlat · R_T`, `Est = Δlon · R_T · cos(lat0)`, avec `R_T = 6 378 137 m`. Valable pour des vols de quelques kilomètres.
- **Dans `NavState`** (ce que reçoit le guidage) : `alt` est positive **vers le haut**, mais `vd` (vitesse « down ») est positive **vers le bas**. C'est la source d'erreur de signe la plus fréquente.

## A3. Équations du mouvement

État à 13 variables : `x = [p (3), v (3), q (4), ω (3)]`, avec `p` et `v` la position et la vitesse sol en NED.

```
ṗ = v
v̇ = R(q) · F_b / m  +  [0, 0, g]            g = 9,81 m/s²
q̇ = ½ · q ⊗ (0, p, q, r)
ω̇ = I⁻¹ · ( τ − ω × I·ω )                    I = diag(Ixx, Iyy, Izz)
```

`F_b` est la résultante des forces **hors gravité** exprimée dans le repère avion (poussée + aérodynamique), `τ` la somme des couples (autopilote + aérodynamique). Le quaternion est renormalisé après chaque pas.

La **force spécifique** `F_b / m` est ce que mesure un accéléromètre. En vol en palier stabilisé, elle vaut environ `[0, 0, −g]` : un accéléromètre mesure l'opposé de la gravité, pas l'accélération.

## A4. Efforts aérodynamiques et poussée

**Vitesse air, incidence, dérapage** (le vent `w` est un vecteur NED constant) :

```
v_a = R(q)ᵀ · (v − w)              vitesse de la masse d'air dans le repère avion
V   = max(‖v_a‖, 1)                m/s (plancher à 1 pour éviter les divisions par zéro)
α   = atan2(v_az, max(v_ax, 1e-3))
β   = asin(v_ay / V)
q̄   = ½ · ρ · V²                   pression dynamique, ρ = 1,225 kg/m³
```

**Coefficients et forces** :

```
CL = clip( CL0 + CLα·α , −CLmax , +CLmax )       décrochage simplifié : saturation de CL
CD = CD0 + k · CL²                               polaire parabolique
Portance  L = q̄·S·CL
Traînée   D = q̄·S·CD
Latérale  Y = q̄·S·CYβ·β
Poussée   T = δ · Tmax · max(0, 1 − V/V0)        δ ∈ [0,1] = commande de poussée
```

**Résultante dans le repère avion** (la portance, perpendiculaire à la vitesse air, est projetée avec l'incidence ; la traînée est opposée à la vitesse air) :

```
F_b = [ T + L·sin α ,  Y ,  −L·cos α ]  −  D · v_a / V
```

**Couples aérodynamiques** (`b` envergure, `c` corde, `S` surface) :

```
τ_roulis  = q̄·S·b · Clp · (b / 2V) · p            amortissement en roulis
τ_tangage = q̄·S·c · [ Cmα·α + Cmq · (c / 2V) · q ] stabilité statique + amortissement
τ_lacet   = q̄·S·b · [ Cnβ·β + Cnr · (b / 2V) · r ] stabilité de route (girouette) + amortissement
```

Avec `Cmα < 0` (stabilité statique en tangage), `Cnβ > 0` (effet girouette), et `Clp, Cmq, Cnr < 0` (amortissements), l'avion est naturellement stable.

## A5. Autopilote bas niveau : de la commande haut niveau aux couples

L'algorithme de navigation envoie une `Command` :

| Champ | Signification | Limite appliquée par le simulateur |
|---|---|---|
| `roll` | angle de roulis demandé (rad) | ±45° |
| `pitch` | angle de tangage demandé (rad) | ±25° |
| `yaw_rate` | vitesse de variation du cap demandée (rad/s) | ±60°/s |
| `throttle` | poussée, 0 à 1 | [0, 1] |

La commande est **saturée** avant usage, puis convertie en couples par des PD dimensionnés avec l'inertie :

```
τ_x = Ixx · [ ωφ² · (φ_c − φ)  −  2·ζ·ωφ · p ]
τ_y = Iyy · [ ωθ² · (θ_c − θ)  −  2·ζ·ωθ · q ]
τ_z = Izz · kr · [ ψ̇_c · cos φ · cos θ  −  r ]

puis saturation de chaque couple à ±(4 ; 3 ; 1,5) N·m
```

`φ` et `θ` sont les angles d'Euler courants. Les pulsations propres valent `ωφ = 12 rad/s` (roulis) et `ωθ = 8 rad/s` (tangage), l'amortissement `ζ = 0,8`, et `kr = 5 s⁻¹`. Les couples sont ensuite additionnés aux couples aérodynamiques de A4.

Cette boucle **n'est pas un modèle de gouvernes** : elle représente le comportement vu de l'extérieur d'un autopilote d'attitude réel (réponse d'ordre 2 en roulis et en tangage, ordre 1 en lacet), saturé par la puissance de commande disponible. C'est le bon niveau d'abstraction pour développer de la navigation.

**Virage coordonné.** Pour qu'un virage soit propre, la consigne de lacet doit suivre le roulis : `ψ̇_c = g · tan(φ_c) / V`. Le guidage L1 fourni le fait. Envoyer `yaw_rate` seul avec `roll = 0` ne fait presque pas virer l'avion : dans un essai avec une consigne de 20 °/s, la vitesse de lacet atteinte est de 1,5 °/s, car l'effet girouette (`Cnβ`) s'y oppose et le couple de lacet est saturé à 1,5 N·m.

## A6. Intégration numérique et ordre des opérations

- **Intégrateur** : Runge-Kutta d'ordre 4, pas `dt = 0,01 s` (100 Hz).
- **Commande maintenue constante** sur le pas (bloqueur d'ordre zéro).
- **Ordre dans un pas** : (1) les capteurs lisent l'état à l'instant `t` ; (2) le contrôleur calcule la commande à partir de ces mesures ; (3) l'avion avance de `t` à `t + dt`. Il y a donc **un pas de latence** entre mesure et effet de la commande, comme dans un système embarqué réel.
- **Cadences des capteurs** : chaque capteur ne délivre une mesure qu'à sa période (GPS 5 Hz, IMU 100 Hz, baro/Pitot 20 Hz). Le contrôleur reçoit `None` pour un capteur sans nouvelle mesure à ce pas.

## A7. Capteurs simulés

Chaque capteur lit l'état vrai, y ajoute ses erreurs, et le délivre à sa cadence, éventuellement avec un retard.

| Capteur | Cadence | Modèle d'erreur par défaut |
|---|---|---|
| **GPS** | 5 Hz | Position : bruit gaussien σ = 1,5 m par axe horizontal, 3 m en altitude. Vitesse sol : bruit blanc σ = 0,15 m/s par axe. Option `corr_time_s` : bruit corrélé. Option `delay_s` : retard de livraison. |
| **IMU** | 100 Hz | Accéléromètre : force spécifique + biais constant (tiré en N(0 ; 0,05) m/s² par axe) + bruit blanc σ = 0,05 m/s². Gyromètre : ω + biais constant (N(0 ; 0,002) rad/s) + bruit blanc σ = 0,005 rad/s. |
| **Baromètre** | 20 Hz | Altitude relative + marche aléatoire (dérive 0,02 m/√s) + bruit blanc σ = 0,3 m. |
| **Pitot** | 20 Hz | Vitesse par rapport à la masse d'air + bruit blanc σ = 0,3 m/s. |

**Bruit corrélé (Gauss-Markov du 1er ordre)**, plus réaliste pour un GPS dont l'erreur varie lentement :

```
e_k = a · e_{k−1} + √(1 − a²) · σ · w_k ,    a = exp( −1 / (f · τ) ) ,    w_k ~ N(0, 1)
```

avec `f` la cadence et `τ = corr_time_s`. Pour `τ = 0`, le bruit est blanc. L'écart-type en régime établi reste `σ`.

Le GPS délivre **latitude, longitude, altitude absolue** et vitesse NED, comme un vrai récepteur. L'algorithme doit reconvertir avec `GeoOrigin`.

**Non modélisés** : facteur d'échelle et désalignement de l'IMU, bruit en densité spectrale (`m/s²/√Hz`), magnétomètre, coupures GPS spontanées (à ajouter soi-même, voir B6).

## A8. Paramètres par défaut

Ordres de grandeur d'un avion RC de 2 kg et 1,6 m d'envergure, **à remplacer par les valeurs de navychop** (classe `AircraftParams`).

| Groupe | Paramètre | Valeur |
|---|---|---|
| Inertie | masse `m` | 2,0 kg |
| | `Ixx, Iyy, Izz` | 0,08 ; 0,12 ; 0,18 kg·m² |
| Géométrie | surface alaire `S`, envergure `b`, corde `c` | 0,4 m², 1,6 m, 0,25 m |
| Portance | `CL0`, `CLα`, `CLmax` | 0,2 ; 5,0 /rad ; 1,2 |
| Traînée | `CD0`, `k` | 0,03 ; 0,06 |
| Latéral | `CYβ` | −0,3 |
| Moments | `Cmα`, `Cmq` | −0,2 ; −3,0 |
| | `Clp` | −0,1 |
| | `Cnβ`, `Cnr` | 0,06 ; −0,1 |
| Propulsion | `Tmax`, `V0` | 12 N ; 35 m/s |
| Autopilote | `ωφ`, `ωθ`, `ζ`, `kr` | 12 ; 8 rad/s ; 0,8 ; 5 s⁻¹ |
| | couples max (x, y, z) | 4 ; 3 ; 1,5 N·m |

L'état initial est un **vol en palier approché** : `CL = 2·m·g / (ρ·V²·S)`, `α = (CL − CL0) / CLα`, tangage = α, vitesse air `V` donnée, poussée à 0,46.

## A9. Comportement caractéristique (valeurs mesurées)

Valeurs calculées ou mesurées sur le code avec les paramètres par défaut, pour se repérer.

| Grandeur | Valeur |
|---|---|
| Équilibre à 18 m/s : `CL`, incidence | 0,247 ; 0,54° |
| Traînée à 18 m/s | 2,67 N |
| Poussée maximale à 18 m/s | 5,83 N, soit une poussée d'équilibre ≈ 0,46 |
| Vitesse de décrochage (`CLmax`) | 8,2 m/s |
| Vitesse max en palier, plein gaz | ≈ 23 m/s |
| Rayon de virage à 18 m/s, roulis 20° | 91 m (facteur de charge 1,06) |
| Rayon de virage à 18 m/s, roulis 35° | 47 m (facteur de charge 1,22) |
| Rayon de virage à 18 m/s, roulis 45° | 33 m (facteur de charge 1,41) |
| Échelon de roulis 20° | 90 % atteint en 0,36 s, sans dépassement |
| Échelon de tangage 10° (poussée fixe) | 90 % atteint en 0,7 s ; le tangage reste ensuite **entre 9,5° (à 2 s) et 9,15° (à 10 s)**, la vitesse chutant de 18 à 12,6 m/s en montée |
| Virage coordonné, roulis 30°, 20 s | taux de cap 15,4 °/s, contre 16,1 °/s par `g·tan φ / V` avec la vitesse moyenne atteinte (20,1 m/s), soit 4 % d'écart ; dérapage moyen 0,7° |
| Même virage sans commande de lacet | taux de cap 14,1 °/s (15,9 °/s attendus, soit 11 % d'écart), dérapage moyen 1,9° |
| Vol libre 30 s, poussée 0,46 | l'altitude passe de 80 à 74 m : l'équilibre initial n'est qu'approché |

Deux enseignements :

1. **Le tangage n'atteint pas exactement sa consigne** (entre 9,2° et 9,5° pour 10°) : la boucle est un PD sans terme intégral, donc elle laisse un écart statique, et la vitesse qui change modifie aussi l'équilibre. Un contrôleur de navigation qui boucle sur l'altitude corrige cet écart naturellement. Le virage coordonné montre le même type d'écart : la commande de lacet réduit le dérapage (0,7° contre 1,9°) et rapproche le taux de cap de la valeur théorique (4 % d'écart contre 11 %), sans l'atteindre exactement.
2. **Le simulateur ne tient ni l'altitude ni la vitesse tout seul** : ces boucles appartiennent à l'algorithme de navigation (le L1 fourni les contient).

## A10. Hypothèses, limites, et comment recaler sur navychop

**Hypothèses simplificatrices**

- Terre plate, gravité constante, atmosphère uniforme (ρ constant).
- Vent constant et uniforme ; **pas de rafales ni de turbulence**.
- Autopilote d'attitude idéalisé (pas de gouvernes, pas de servos, pas de retard actionneur).
- Décrochage réduit à une saturation de `CL` (pas de chute de portance, pas de traînée de décrochage).
- Pas de dynamique moteur ni batterie : la poussée suit instantanément `δ` et ne dépend que de la vitesse.
- Pas de couple d'hélice ni d'effet gyroscopique du rotor, pas de sol (l'altitude négative arrête la simulation en `crashed`).
- Modèle longitudinal et latéral découplés seulement par les termes simples ci-dessus.

**Conséquence pour l'usage** : le simulateur est adapté pour développer, comparer et valider des algorithmes de **navigation et guidage**. Il n'est pas adapté pour dimensionner l'autopilote d'attitude, la structure, ni l'énergie.

**Recaler le modèle sur l'avion réel**, dans l'ordre d'importance pour le guidage :

1. **Masse, surface, vitesse de croisière, poussée max** : pesée, plan, essai au banc. Ce sont eux qui fixent les rayons de virage et les vitesses atteignables.
2. **Dynamique d'attitude (`ωφ`, `ωθ`, `ζ`, couples max)** : à identifier sur un enregistrement de vol, en mesurant la réponse du roulis et du tangage à un échelon de consigne et en ajustant `ωφ`, `ζ` pour retrouver le temps de montée et le dépassement observés.
3. **Inerties** : estimation par pendule bifilaire ou calcul CAO ; leur effet est surtout visible sur la rapidité de la boucle d'attitude.
4. **Coefficients aérodynamiques** : XFLR5 ou formules de dimensionnement ; peu sensibles pour le guidage tant que le vol reste loin du décrochage.
5. **Bruits capteurs** : relever les écarts-types réels sur banc statique (GPS immobile, IMU posée, baromètre) et reporter dans les constructeurs de `GPS`, `IMU`, `Barometer`.

---

# Partie B – Tutoriel

## B0. Prise en main

```bash
pip install -r requirements.txt          # numpy et matplotlib
python run_sim.py                        # cercle de 100 m, guidage L1, GPS brut
python run_sim.py --traj eight           # trajectoire en 8
python run_sim.py --traj eight --wind 5 --wind-from 270 --duration 200
python run_sim.py --estimator kalman     # même scénario avec l'hybridation GPS/IMU/baro
python run_sim.py --no-show --out res.png
```

Les trois exemples du tutoriel se lancent depuis la racine du projet :

```bash
python exemples/01_controleur_perso.py   # écrire un guidage et le comparer à L1
python exemples/02_hybridation.py        # GPS brut vs Kalman, GPS dégradé, retardé, coupé
python exemples/03_monte_carlo.py        # comparer sur plusieurs tirages de bruit
```

Une exécution de 150 s simulées prend environ 5 s de calcul avec le GPS brut et 9 s avec le filtre de Kalman (mesuré sur le conteneur de test ; cela dépend de la machine).

## B1. Anatomie d'un algorithme

Un algorithme de navigation se décompose en **deux briques interchangeables** :

```
mesures capteurs ──► Estimateur ──► NavState ──► Contrôleur ──► Command
  (dict)                                                         (roll, pitch, yaw_rate, throttle)
```

**Ce que le code reçoit** : un dictionnaire `meas` à chaque pas de 10 ms.

```python
meas = {"gps": GPSMeasurement | None,      # lat, lon, alt, vel_ned (None sauf à 5 Hz)
        "imu": IMUMeasurement | None,      # accel (force spécifique), gyro
        "baro": BaroMeasurement | None,    # altitude relative
        "pitot": AirspeedMeasurement | None}
```

**La règle d'or** : un algorithme n'a **jamais** le droit de lire l'état vrai (`ac.pos`, `ac.euler`…). Tout ce qu'il sait vient de `meas`. L'état vrai ne sert qu'à évaluer a posteriori (c'est ce que fait `benchmark.py`).

**Interface de l'estimateur** (`estimators.py`) : renvoie un `NavState` ou `None` tant qu'il n'est pas prêt.

```python
class Estimator:
    def update(self, t: float, meas: dict) -> NavState | None: ...

# NavState : t, north, east, alt (haut positif), vn, ve, vd (bas positif), airspeed
```

**Interface du contrôleur** (`controller.py`) :

```python
class Controller:
    debug: dict                                   # valeurs à journaliser (facultatif)
    def update(self, t: float, meas: dict) -> Command: ...
```

Le contrôleur L1 fourni accepte un estimateur : `L1WaypointController(mission, origin, estimator=...)`. On peut donc changer l'un sans toucher à l'autre, ce qui permet de **comparer deux estimateurs avec le même guidage**, ou deux guidages avec le même estimateur.

Les valeurs du dictionnaire `debug` sont journalisées dans `log.debug[...]` (utile pour les graphes). Les clés `est_n` et `est_e` (position estimée) permettent à `benchmark.py` de calculer l'erreur d'estimation.

## B2. Définir des balises et des conditions de passage

Les balises sont exprimées en mètres dans le repère local (Nord, Est) avec une altitude positive vers le haut.

```python
from mission import Waypoint, Mission, circle, figure_eight

wps = circle(center_north=0, center_east=100, radius=100, alt=80, n_points=16)   # cercle
wps = figure_eight(0, 0, half_width=150, alt=80, n_points=24)                    # 8

wp = Waypoint(north=200, east=50, alt=90, radius=15,
              speed=20.0,                                  # vitesse air demandée jusqu'à cette balise
              max_alt_error=5.0,                           # condition verticale
              condition=lambda nav: nav.airspeed > 15.0)   # condition libre sur l'état estimé

mission = Mission(wps, loop=True)      # loop=False : se termine à la dernière balise
```

**Une balise est franchie quand les trois conditions sont réunies** :

1. **Horizontale** : distance à la balise < `radius`, **ou** plan perpendiculaire à la branche dépassé à moins de `3 × radius` (évite de tourner indéfiniment autour d'une balise manquée de peu).
2. **Verticale** (optionnelle) : `|alt − alt_balise| ≤ max_alt_error`.
3. **Libre** (optionnelle) : `condition(nav)` renvoie `True` ; `nav` est le `NavState` **estimé**.

Si une condition échoue, la balise reste active. Attention : une condition jamais satisfaite bloque la mission (voir B8).

Le contrôle de passage s'appuie sur l'état **estimé**. Une mauvaise estimation peut donc déclarer une balise franchie alors que l'avion en est loin, ou l'inverse. C'est une des raisons de soigner l'estimateur (voir B5).

## B3. Écrire un algorithme de guidage

Fichier : `exemples/01_controleur_perso.py`. Le contrôleur « poursuite pure » vise directement la balise courante, avec une loi proportionnelle sur l'écart de cap. Il tient en une quinzaine de lignes utiles :

```python
class PursuitController(Controller):
    def __init__(self, mission, origin, k_heading=1.2, max_bank=math.radians(35), estimator=None):
        self.mission = mission
        self.estimator = estimator or ExtrapolationEstimator(origin)
        self.k, self.max_bank = k_heading, max_bank
        self.debug = {}
        self._started = False

    def update(self, t, meas):
        nav = self.estimator.update(t, meas)               # 1) estimation (jamais l'état vrai)
        if nav is None:                                    #    pas encore de fix GPS : palier
            return Command(throttle=0.46)
        if not self._started:                              # 2) mission et conditions de passage
            self.mission.start(nav); self._started = True
        self.mission.check(nav)
        wp = self.mission.target

        course = math.atan2(nav.ve, nav.vn)                # 3) guidage latéral
        bearing = math.atan2(wp.east - nav.east, wp.north - nav.north)
        roll = max(-self.max_bank, min(self.max_bank, self.k * wrap_pi(bearing - course)))
        yaw_rate = G * math.tan(roll) / max(nav.airspeed, 5.0)          # virage coordonné

        pitch = 0.01 + 0.02 * (wp.alt - nav.alt) + 0.05 * nav.vd        # 4) altitude
        throttle = 0.46 + 0.12 * ((wp.speed or 18.0) - nav.airspeed)    # 5) vitesse
        # (écriture simplifiée ; le fichier borne pitch à ±0,2 rad et throttle à [0, 1])
        return Command(roll=roll, pitch=pitch, yaw_rate=yaw_rate, throttle=throttle)
```

Pour écrire le vôtre, la structure est toujours la même : **estimer → gérer la mission → calculer les quatre commandes**. Seule l'étape 3 (la loi latérale) change d'un algorithme de guidage à l'autre.

**Le guidage L1 fourni** (`L1WaypointController`) vise un point de référence situé à une distance `L1` en avant de l'avion sur la branche courante :

```
L1    = max( L1min , amortissement · période · V_sol / π )
η     = angle entre la route sol et la direction du point de référence
a_lat = 2 · V_sol² / L1 · sin η
roll  = atan( a_lat / g )                      limité à ±35°
```

Il tient l'altitude par un PI avec amortissement sur la vitesse verticale (consigne de tangage), et la vitesse par un PI (poussée).

**Comparaison obtenue** avec le même GPS brut (150 s simulées, graine 0) :

| Scénario | Algorithme | Écart moy. | Écart max | Passage moy. | Passage max | Nervosité du roulis |
|---|---|---|---|---|---|---|
| Cercle 100 m | poursuite pure | 3,8 m | 5,3 m | 15,3 m | 18,0 m | 98 °/s |
| Cercle 100 m | L1 | 5,1 m | 6,5 m | 15,0 m | 18,3 m | 74 °/s |
| Huit 300 m | poursuite pure | 4,2 m | 9,8 m | 15,5 m | 19,1 m | 102 °/s |
| Huit 300 m | L1 | 5,8 m | 12,3 m | 15,3 m | 18,4 m | 79 °/s |

La lecture honnête de ce tableau : **sur la distance à la ligne des balises, l'algorithme le plus simple fait mieux que L1**. Cela ne veut pas dire que L1 est mauvais : cette métrique avantage un avion qui passe au plus près des balises, alors que L1 est conçu pour suivre un segment sans osciller. Elle montre surtout qu'**il faut choisir ses métriques en fonction de ce qu'on veut** (voir B4). À noter aussi : avec un GPS quasi parfait (bruit de l'ordre du centimètre), L1 reste à 5,0 m du polygone sur ce cercle, donc ces 5 m sont un biais du guidage et pas un effet du bruit.

## B4. Mesurer : métriques, benchmark, Monte-Carlo

`benchmark.py` fournit `run_scenario(...)`, qui construit l'avion, les capteurs et le contrôleur, lance la simulation et renvoie un dictionnaire de métriques :

```python
from benchmark import run_scenario, print_table
from controller import L1WaypointController

res = run_scenario(lambda mission, origin: L1WaypointController(mission, origin),
                   traj="eight", wind=5.0, duration=150, seed=0)
print(res["xtrack_mean"], res["xtrack_max"], res["est_rms"])
```

Le premier argument est une **fabrique** `(mission, origin) → Controller`. Paramètres utiles : `traj` (`"circle"`/`"eight"`), `size`, `wind` et `wind_from`, `duration`, `seed`, `speed`, `gps_cls` (classe de GPS à utiliser) et `gps_kwargs` (réglages du GPS).

| Métrique | Définition | Ce qu'elle dit |
|---|---|---|
| `xtrack_mean`, `xtrack_max` | Distance (m) de la position vraie au polygone des balises, après 20 s | Qualité du suivi de ligne. Biaisée par la forme de la trajectoire (un cercle n'est pas son polygone). |
| `miss_mean`, `miss_max` | Distance (m) entre la position **vraie** et le centre de la balise à l'instant où le contrôleur la déclare franchie | Fiabilité du franchissement. Vaut environ le rayon de capture (15 m) si l'estimation est bonne ; s'écarte quand elle est fausse. |
| `est_rms`, `est_max` | Erreur (m) entre position estimée et vraie (si le contrôleur publie `est_n`/`est_e` dans `debug`) | Qualité de l'estimateur, indépendamment du guidage. |
| `roll_jitter` | RMS de la dérivée de la consigne de roulis (°/s) | « Nervosité » de la commande : ce que subiraient les servos. |
| `alt_std`, `laps`, `crashed` | Écart-type d'altitude, nombre de tours, chute au sol | Santé générale de la simulation. |

**Un seul tirage de bruit ne prouve rien.** Pour comparer deux algorithmes, répétez sur plusieurs graines (`exemples/03_monte_carlo.py`) :

```python
res = [run_scenario(factory, traj="eight", wind=5.0, duration=120, seed=s) for s in range(5)]
np.mean([r["xtrack_mean"] for r in res]), np.std([r["xtrack_mean"] for r in res])
```

Résultat sur le huit avec vent de 5 m/s (5 graines, moyenne ± écart-type) :

| Algorithme | Écart moyen | Écart max | Erreur d'estimation RMS |
|---|---|---|---|
| L1 + GPS brut | 5,74 ± 0,07 m | 12,12 ± 0,41 m | 2,08 ± 0,02 m |
| L1 + Kalman GPS/IMU/baro | 5,91 ± 0,02 m | 13,96 ± 0,34 m | 0,49 ± 0,08 m |

Les écarts-types sont petits devant les différences, donc ces différences sont réelles (pour ces scénarios et ces réglages).

## B5. Hybridation de capteurs (GPS + IMU + baromètre)

### Ce que fournit `GpsImuBaroEstimator`

Un filtre de Kalman **faiblement couplé** : le GPS fournit des mesures de position et de vitesse, l'IMU sert à propager l'état entre deux fix GPS, le baromètre corrige l'altitude.

**1. Estimation d'attitude (AHRS de type Mahony)**, nécessaire pour exprimer l'accélération mesurée dans le repère NED :

```
a_cin  = ω × [V, 0, 0]                      accélération centripète (V = vitesse air)
g_mes  = a_cin − f                          gravité mesurée, repère avion (f = accéléromètre)
e      = (g_mes / ‖g_mes‖) × p_prédit       erreur d'orientation, p_prédit = 3e ligne de R(q)
        + (k_yaw/kp) · (route_GPS − ψ) · p_prédit        recalage du cap sur la route sol
ω_corrigé = ω − biais + kp · e ,   biais_dot = −ki · e
```

Pourquoi `a_cin` : en virage, l'accéléromètre ne mesure **pas** la gravité seule. Sans cette compensation, l'avion « croirait » être à plat en plein virage. Le recalage du cap sur la route GPS est désactivé si le dernier cap GPS a plus d'une seconde (coupure GPS).

**2. Filtres de Kalman** (un par axe : Nord, Est, altitude), état `[position, vitesse]` :

```
prédiction (100 Hz, IMU)   :  a_NED = R·f + [0, 0, g]
                              p += v·dt + ½·a·dt² ;  v += a·dt
                              Q = σa² · [½dt², dt]ᵀ · [½dt², dt]
correction (5 Hz, GPS)     :  position et vitesse
correction (20 Hz, baro)   :  altitude (axe vertical uniquement)
```

**Réglages principaux** (constructeur de `GpsImuBaroEstimator`) :

| Paramètre | Défaut | Rôle |
|---|---|---|
| `sigma_a` | 0,6 m/s² | Confiance dans l'IMU. Plus grand : on suit davantage le GPS. Doit couvrir les erreurs d'attitude, pas seulement le bruit de l'accéléromètre. |
| `sigma_gps_h`, `sigma_gps_v`, `sigma_gps_vel` | 1,5 ; 3,0 ; 0,15 | Doivent refléter le bruit réel du GPS simulé. |
| `sigma_baro` | 0,3 | Idem pour le baromètre. |
| `AttitudeEstimator(kp, ki, k_yaw)` | 2,0 ; 0,02 ; 0,4 | Gains de l'AHRS. |

### Utilisation

```python
from estimators import GpsImuBaroEstimator
ctrl = L1WaypointController(mission, origin, estimator=GpsImuBaroEstimator(origin))
# ou : python run_sim.py --estimator kalman
```

### Résultats (exemples/02_hybridation.py)

Cercle de 100 m, 150 s, guidage L1 identique, graine 0.

| Scénario | Estimateur | Écart moy. | Écart max | Passage max | Erreur d'estimation RMS | Nervosité |
|---|---|---|---|---|---|---|
| Nominal | GPS brut | 5,1 m | 6,5 m | 18,3 m | 2,10 m | 74 °/s |
| Nominal | Kalman | 5,5 m | 6,9 m | 15,5 m | 0,27 m | 6 °/s |
| GPS 5 m corrélé | GPS brut | 4,9 m | 13,4 m | 25,8 m | 6,69 m | 69 °/s |
| GPS 5 m corrélé | Kalman | 5,2 m | 8,8 m | 17,9 m | 2,40 m | 7 °/s |
| GPS retardé de 0,5 s | GPS brut | 2,0 m | 3,6 m | 18,2 m | 2,19 m | 87 °/s |
| GPS retardé de 0,5 s | Kalman | 5,4 m | 7,1 m | 7,6 m | 8,94 m | 6 °/s |
| **Coupure GPS de 20 s** | GPS brut | 15,4 m | 95,2 m | 18,4 m | 97,09 m | 66 °/s |
| **Coupure GPS de 20 s** | Kalman | 5,3 m | 9,2 m | 15,6 m | 3,78 m | 13 °/s |

**Ce qu'il faut en retenir**

1. **L'hybridation n'améliore pas le suivi quand le GPS est bon.** À GPS nominal, l'écart de suivi est même légèrement plus élevé avec le Kalman (5,5 m contre 5,1 m ; 5,91 contre 5,74 m en Monte-Carlo sur le huit, avec un maximum de 13,96 m contre 12,12 m). La cause n'a pas été étudiée : un léger retard de l'estimation dans les changements de virage est une piste, à vérifier.
2. **Elle améliore fortement l'estimation et la douceur des commandes** : erreur d'estimation divisée par environ 8 (2,10 → 0,27 m) et nervosité du roulis divisée par environ 12 (74 → 6 °/s).
3. **Elle apporte la robustesse** : avec un GPS dégradé, le franchissement erroné d'une balise passe de 25,8 m à 17,9 m ; pendant une coupure GPS de 20 s, l'erreur de position reste à 3,8 m au lieu de 97 m.
4. **Elle ne gère pas le retard GPS** : avec 0,5 s de retard, l'erreur d'estimation monte à 8,94 m, soit à peu près `0,5 s × 18 m/s`. L'estimateur de référence, lui, compense ce retard en extrapolant la position avec la vitesse. C'est un défaut connu de ce filtre et un bon exercice (voir B7).
5. **Le recalage du cap suppose peu de vent** : l'erreur d'estimation passe de 0,27 m sans vent à 0,46 m avec 5 m/s de vent, parce que la route sol diffère du cap quand l'avion « crabe ». Une estimation du vent corrigerait cela.
6. **À propos de la métrique « Écart moy. » du tableau** : elle ne classe pas les cas de façon fiable (la ligne « retard 0,5 s » donne le meilleur écart pour le GPS brut). Lisez-la avec l'erreur d'estimation et la nervosité, pas seule.

### Écrire son propre estimateur

Il suffit d'hériter de `Estimator` et de renvoyer un `NavState`. Squelette d'un filtre d'altitude baro + GPS :

```python
class MonEstimateur(Estimator):
    def __init__(self, origin):
        self.origin = origin
        # ... état interne du filtre

    def update(self, t, meas):
        if meas["gps"] is not None:
            n, e = self.origin.to_local(meas["gps"].lat, meas["gps"].lon)
            # ... correction GPS
        if meas["imu"] is not None:
            # ... prédiction avec accel / gyro
            pass
        if not self.pret:
            return None                          # le contrôleur attend
        return NavState(t, north, east, alt, vn, ve, vd, airspeed)
```

À publier dans `controller.debug` : `est_n` et `est_e`, pour que `benchmark.py` calcule l'erreur d'estimation (le contrôleur L1 le fait déjà).

## B6. Dégrader ou ajouter un capteur

**Dégrader un GPS existant** : tout passe par `gps_kwargs`.

```python
run_scenario(factory, gps_kwargs=dict(sigma_h=5.0, sigma_v=8.0, corr_time_s=5.0))   # bruité, corrélé
run_scenario(factory, gps_kwargs=dict(delay_s=0.5))                                  # retardé de 0,5 s
```

**Coupure GPS** : ajoutez une classe qui hérite de `GPS` (exemple de `exemples/02_hybridation.py`).

```python
class GPSWithOutage(GPS):
    def __init__(self, *args, outage=(60.0, 80.0), **kwargs):
        super().__init__(*args, **kwargs)
        self.outage = outage
    def read(self, t, ac):
        m = super().read(t, ac)
        return None if self.outage[0] <= t < self.outage[1] else m

run_scenario(factory, gps_cls=GPSWithOutage)
```

**Nouveau capteur** (magnétomètre, télémètre…) : héritez de `Sensor`, implémentez `measure(t, ac)` qui renvoie une mesure à partir de l'état vrai `ac`, et ajoutez-le au dictionnaire de `SensorSuite`. La clé choisie est celle que recevra l'algorithme dans `meas`.

## B7. Exercices proposés

Du plus simple au plus ambitieux :

1. **Régler la poursuite pure** : faire varier `k_heading`, tracer l'écart de suivi et la nervosité du roulis. Y a-t-il un compromis ?
2. **Condition de passage** : ajouter une condition `nav.airspeed > 16` sur une balise, et observer ce que fait l'avion quand elle n'est pas satisfaite (voir B8).
3. **Nouvelle trajectoire** : écrire un générateur de balises pour un hippodrome (deux droites reliées par deux demi-cercles) ou un carré, puis tester les deux guidages.
4. **Autre loi de guidage** : implémenter un suivi de chemin (par exemple *carrot chasing* ou *NLGL*) et le comparer à L1 en Monte-Carlo, avec vent.
5. **Compensation du retard GPS** dans `GpsImuBaroEstimator` : propager l'état filtré de l'instant de la mesure jusqu'à l'instant courant. Objectif : retrouver une erreur d'estimation proche de celle sans retard.
6. **Estimation du vent** : estimer le vecteur vent à partir de la différence entre vitesse sol (GPS) et vitesse air (Pitot, cap estimé), puis s'en servir pour recaler le cap sans biais de crabe.
7. **Biais d'accéléromètre** : ajouter un état de biais au filtre et mesurer le gain pendant une longue coupure GPS (60 s).
8. **Rafales** : ajouter une perturbation de vent variable dans `Aircraft` (par exemple un processus de Gauss-Markov) et évaluer la robustesse des deux guidages.

## B8. Pièges courants

- **Lire l'état vrai dans le contrôleur.** Tout algorithme qui utilise `ac.pos` ou `ac.euler` donne des résultats trop beaux. Il ne doit utiliser que `meas`.
- **Signes d'altitude.** `NavState.alt` est positive vers le haut, mais `vd` est positive vers le bas. Une montée à 2 m/s correspond à `vd = −2`.
- **Mesure à `None`.** À 100 Hz, le GPS (5 Hz) n'est présent qu'un pas sur vingt. Toujours tester `meas["gps"] is not None` avant de l'utiliser.
- **Origine du temps de mesure.** Les mesures portent leur propre instant `t` ; avec un retard, il diffère de l'instant courant. Un estimateur qui ignore cette différence se trompe d'autant (c'est le défaut du filtre fourni, voir B5).
- **Condition de passage jamais satisfaite.** La balise reste active et l'avion tourne autour indéfiniment. Prévoir un délai maximal ou une condition de repli.
- **Conclure sur un seul tirage.** Toujours répéter sur plusieurs graines avant de classer deux algorithmes (B4).
- **Une métrique unique.** La distance au polygone ne mesure ni la fiabilité du franchissement ni la douceur des commandes. Utiliser plusieurs métriques et les lire ensemble.
- **Bruit réel et réglages du filtre incohérents.** Si vous changez le bruit du GPS simulé, changez `sigma_gps_h` du filtre en conséquence, sinon le filtre pondère mal ses sources.
- **Avion trop sollicité.** Un guidage qui demande un roulis au-delà de 45° est saturé par le simulateur : le comportement observé n'est alors plus celui de votre loi de commande.
- **Clés de `debug`.** Pour tracer une grandeur interne, publiez-la dans `self.debug` ; elle est ensuite disponible dans `log.debug["ma_cle"]`, une valeur par échantillon journalisé (un pas sur deux).

## B9. Simulation interactive : voir en direct et prendre la main

```bash
python live_sim.py                                   # cercle, guidage L1, GPS brut
python live_sim.py --traj eight --estimator kalman --wind 4
python live_sim.py --timescale 2                     # 2x plus vite que le temps réel
```

Une fenêtre affiche la vue de dessus (balises, branche courante, balise visée, mesures GPS, position estimée, trajectoire réelle), l'altitude sur les 60 dernières secondes et un panneau d'état (mode, altitude, vitesse, roulis/tangage, écart au tracé, erreur d'estimation).

**Touches** (la fenêtre doit avoir le focus) :

| Touche | Action |
|---|---|
| `←` / `→` | roulis à gauche / à droite, avec virage coordonné |
| `↑` / `↓` | nez en haut / nez en bas |
| `espace` | pause / reprise |
| `+` / `-` | simulation plus rapide / plus lente (×0,25 à ×16) |
| `r` | recommencer |
| `q` | quitter |

**Comment fonctionne la prise de contrôle.** L'autopilote (estimateur + guidage) tourne en permanence et ne sait pas qu'il est surchargé. Tant qu'une flèche est enfoncée, la commande de l'axe correspondant est remplacée par la vôtre : roulis et lacet pour `←` `→`, tangage pour `↑` `↓`. La consigne manuelle part de l'attitude réelle de l'avion (pas de saut) et évolue en rampe : 60°/s en roulis jusqu'à ±40°, 30°/s en tangage jusqu'à ±20°. **Dès qu'on relâche, l'autopilote reprend la main**, et le tracé redevient bleu ; le pilotage manuel est tracé en rouge. La poussée reste gérée par l'autopilote.

C'est le protocole de test le plus direct pour la question « que fait l'algorithme quand l'avion est loin de la trajectoire ? » : on pousse l'avion à l'écart, on relâche, on regarde le retour.

**Temps réel.** À chaque image (environ 25 par seconde), la simulation avance du temps réellement écoulé multiplié par `--timescale`. Si l'ordinateur ne suit pas, la simulation ralentit au lieu de rattraper le retard (une image ne fait jamais avancer la simulation de plus de 0,1 s de temps réel). Dans le conteneur de test, un run complet de `run_sim.py` (tracé final inclus) va environ 28 fois plus vite que le temps réel avec le GPS brut et 16 fois avec le Kalman : il y a donc de la marge pour le calcul, mais le coût de l'affichage en direct dépend de votre machine et n'a pas été mesuré.

**Mesures de récupération** (cercle de 100 m, GPS brut, sans vent, graine 0, perturbation commençant à t = 20 s ; « retour » = écart au tracé resté sous 10 m pendant 10 s) :

| Perturbation | Écart max | Retour sous 10 m |
|---|---|---|
| virage à gauche 2 s | 43 m | 8 s |
| virage à gauche 4 s | 126 m | 22 s |
| virage à gauche 8 s | 90 m | 16 s |
| virage à droite 8 s | 77 m | 11 s |
| cabré 10 s | 9 m | immédiat |
| piqué 5 s | 11 m | 1 s |
| gauche + cabré 6 s | 75 m | 24 s |
| huit, vent 5 m/s, virage à gauche 4 s | 160 m | 24 s |
| Kalman, virage à gauche 4 s | 125 m | 18 s |

Le guidage L1 a récupéré dans tous les cas testés. À lire avec précaution : (1) un virage maintenu est un cercle de rayon environ `V² / (g·tan φ)`, soit 39 m à 40° de roulis, donc **l'écart maximal n'augmente pas avec la durée** (4 s donne plus que 12 s) ; (2) ces essais sont des perturbations scriptées, et un pilotage manuel au clavier donnera des résultats différents ; (3) en piqué prolongé, l'avion finit par toucher le sol (« CRASH », `r` pour recommencer).

**Sans écran.** On peut rejouer des appuis de touches simulés et enregistrer une image, utile pour un rapport ou un test automatique :

```bash
python live_sim.py --snapshot res.png --seconds 90 --script "20-24:left,50-53:up"
# format : début-fin:touches, séparés par des virgules ; plusieurs touches avec « + » (ex. 20-26:left+up)
```

L'image `exemples/live_poussee_manuelle.png` en est le résultat : virage à gauche de 4 s (écart de 126 m) puis cabré de 3 s, avec retour sur la trajectoire.

**Si la fenêtre ne s'ouvre pas.** Matplotlib a besoin d'un backend graphique. Il est généralement fourni avec Python sous Windows et macOS ; sous Linux, installez `python3-tk` (paquet système) ou `pip install PyQt5`. Le script le signale si aucun n'est disponible.

**Pour aller plus loin** : la méthode `Simulator.step(override)` est le seul changement apporté au cœur du simulateur. `override(cmd)` reçoit la commande de l'autopilote et renvoie celle qui sera appliquée, ce qui permet aussi de brancher une manette, un script de perturbation ou un enregistrement.
