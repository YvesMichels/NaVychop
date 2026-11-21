# Simulateur de vol « dans la boucle » – navychop

Simulateur Python (numpy + matplotlib) pour développer et tester un algorithme de **suivi de trajectoire GPS par balises**.
L'avion est un solide rigide à 6 degrés de liberté piloté par des commandes haut niveau ; les capteurs (GPS, IMU, baromètre, Pitot)
sont simulés pas à pas à partir de l'état vrai, et le contrôleur ne voit que ces mesures.

> **Documentation détaillée** : [`DOCUMENTATION.md`](DOCUMENTATION.md) décrit le modèle physique (équations, paramètres, limites)
> et propose un tutoriel pour tester des algorithmes de guidage et d'hybridation de capteurs.

## Démarrage rapide

```bash
pip install -r requirements.txt
python run_sim.py                               # cercle de 100 m de rayon
python run_sim.py --traj eight                  # huit
python run_sim.py --traj eight --wind 4 --wind-from 270 --duration 240
python run_sim.py --estimator kalman            # avec hybridation GPS + IMU + baro
python live_sim.py                              # trajectoire EN DIRECT + pilotage manuel aux flèches du clavier
python run_sim.py --no-show --out res.png       # sans fenêtre
```

Le script affiche un bilan (tours, écart à la trajectoire, altitude, vitesse) et trace la vue de dessus (balises, mesures GPS bruitées,
trajectoire réelle) ainsi que l'altitude, la vitesse, le roulis et l'écart de suivi.

## Architecture

| Fichier | Rôle |
|---|---|
| `aircraft.py` | Solide rigide (masse, inertie, position, quaternion, vitesse angulaire), efforts aéro simples, autopilote bas niveau. Classe `Command` = commandes haut niveau. |
| `sensors.py` | `GPS`, `IMU`, `Barometer`, `Pitot` (cadence, bruit, biais, retard) + `SensorSuite`. |
| `geo.py` | Conversion repère local NED ↔ latitude/longitude. |
| `mission.py` | `Waypoint` (balise + conditions de passage), `Mission`, générateurs `circle()` et `figure_eight()`, `distance_to_path()`. |
| `estimators.py` | Interface `Estimator`, estimateur de référence (GPS extrapolé) et filtre de Kalman GPS + IMU + baro avec AHRS. |
| `controller.py` | Interface `Controller` + premier algorithme `L1WaypointController` (estimateur interchangeable). |
| `benchmark.py` | `run_scenario()` : scénarios et métriques chiffrées pour comparer des algorithmes. |
| `exemples/` | Trois scripts du tutoriel : contrôleur perso, hybridation, Monte-Carlo. |
| `simulator.py` | Boucle SIL : capteurs → contrôleur → dynamique, et journalisation (`SimLog`). |
| `run_sim.py` | Exemple complet avec graphiques. |
| `live_sim.py` | Simulation interactive : trajectoire en direct, prise de contrôle manuelle au clavier (← → ↑ ↓), pause, vitesse, reset. |

## Conventions

- Repère monde **NED** : x = Nord, y = Est, z = Bas (altitude = −z). Repère avion **FRD** : avant, droite, bas.
- Roll > 0 : aile droite vers le bas. Pitch > 0 : nez vers le haut. Cap/yaw > 0 : nez vers la droite (0 = Nord, 90° = Est).
- Les balises sont données en mètres (Nord, Est) et altitude positive vers le haut, relativement à l'origine `GeoOrigin`.

## Le modèle d'avion

1. **Solide** : `x = [position, vitesse sol, quaternion, p/q/r]`, intégré en RK4 (dt = 10 ms).
2. **Efforts** : portance `CL0 + CLa·alpha` (saturée), traînée parabolique, force latérale en `beta`, poussée `T = throttle·Tmax·(1 − V/V0)`. Le vent est un vecteur constant.
3. **Commandes haut niveau** (`Command`) : `roll`, `pitch` (angles, rad), `yaw_rate` (rad/s), `throttle` (0..1).
   Une boucle interne PD, dimensionnée avec l'inertie (`wn_roll`, `wn_pitch`, `zeta`), transforme ces consignes en couples.
   Elle représente l'autopilote d'attitude bas niveau de l'avion réel.

Les paramètres sont dans `AircraftParams` (ordres de grandeur d'un avion de 2 kg) : **remplace-les par ceux de navychop**
(masse, inertie, surface, vitesse de croisière, poussée max…).

## Capteurs

Chaque capteur échantillonne l'état vrai à sa propre cadence et ajoute son erreur :

- **GPS** (5 Hz) : position vraie + bruit gaussien (σ = 1,5 m horizontal, 3 m vertical), vitesse + bruit. Options : `corr_time_s` (bruit corrélé, plus réaliste), `delay_s` (latence). Il fournit **latitude/longitude/altitude** : le contrôleur reconvertit avec `GeoOrigin`, comme sur le vrai système.
- **IMU** (100 Hz) : force spécifique et vitesses angulaires, bruit blanc + biais constant.
- **Baromètre** (20 Hz) : altitude + bruit + dérive lente. **Pitot** (20 Hz) : vitesse air + bruit.

Ajouter un capteur : hériter de `Sensor`, écrire `measure(t, aircraft)`, l'ajouter au dict de `SensorSuite`.

## Balises et conditions de passage

```python
from mission import Waypoint, Mission, circle, figure_eight

wps = circle(center_north=0, center_east=100, radius=100, alt=80, n_points=16)
wps = figure_eight(0, 0, half_width=150, alt=80, n_points=24)

# balise à condition de passage personnalisée
wp = Waypoint(north=200, east=50, alt=90, radius=15,
              max_alt_error=5,                              # condition verticale
              condition=lambda nav: nav.airspeed > 15.0)    # condition libre sur l'état estimé
mission = Mission([...], loop=True)                         # loop=False : s'arrête à la dernière balise
```

Une balise est franchie si : (distance < `radius` **ou** plan perpendiculaire franchi à moins de 3·radius) **et** condition verticale **et** condition libre. Si une condition échoue, la balise reste active et l'avion revient la chercher.

## Premier algorithme de suivi (`L1WaypointController`)

- **Estimation** : position GPS extrapolée avec la vitesse GPS (compense la cadence et le retard), altitude = baromètre filtré, vitesse air = Pitot filtré.
- **Guidage latéral** : loi L1 sur la branche *balise précédente → balise courante* ; `a_cmd = 2·V²/L1·sin(eta)` → consigne de roulis.
- **Vertical** : PI + amortissement sur la vitesse verticale GPS → consigne de tangage ; l'altitude consigne est interpolée le long de la branche.
- **Vitesse** : PI sur la vitesse air → poussée. **Virage coordonné** : `yaw_rate = g·tan(roll)/V`.

Résultats obtenus avec les réglages par défaut (GPS σ = 1,5 m) : écart moyen ≈ 5 m, altitude tenue à ±1 m, vitesse à ±0,3 m/s, y compris avec 5 m/s de vent. Le roulis demandé est bruité car il reflète le bruit GPS : une piste d'amélioration (filtre de Kalman).

Écrire son propre algorithme : hériter de `Controller` et implémenter `update(t, meas) -> Command`. `meas` est un dict `{'gps','imu','baro','pitot'}` dont chaque valeur est `None` s'il n'y a pas de nouvelle mesure à ce pas. Puis : `Simulator(ac, sensors, MonControleur(...), origin).run(150)`.

## Pistes pour la suite du projet

- Filtre de Kalman (GPS + IMU + baro) à la place de l'extrapolation simple, puis estimation du vent.
- Autres lois de guidage (pure pursuit, Dubins, suivi de chemin NLGL) et comparaison avec L1 sur des critères chiffrés (écart moyen/max, nombre de balises manquées).
- Scénarios dégradés : perte de GPS, GPS corrélé/retardé, rafales (turbulence), moteur faible.
- Passage en *hardware-in-the-loop* : remplacer le contrôleur par un pont (série/UDP) vers le vrai firmware.

## Limites du modèle

Modèle volontairement simple : pas de gouvernes explicites (l'autopilote d'attitude est idéalisé), pas de décrochage détaillé, pas de dynamique moteur/batterie, pas de rafales, terre plate. C'est suffisant pour développer et comparer des algorithmes de navigation ; à raffiner si besoin.
