# Simulateur de vol « dans la boucle » – navychop

Simulateur Python (numpy + matplotlib) pour développer et tester un algorithme de **suivi de trajectoire GPS par balises**.
L'avion est un solide rigide à 6 degrés de liberté piloté par des commandes haut niveau ; les capteurs (GPS, IMU, baromètre, Pitot)
sont simulés pas à pas à partir de l'état vrai, et le contrôleur ne voit que ces mesures.

## Démarrage rapide

```bash
pip install -r requirements.txt
python run\_sim.py                               # cercle de 100 m de rayon
python run\_sim.py --traj eight                  # huit
python run\_sim.py --traj eight --wind 4 --wind-from 270 --duration 240
python run\_sim.py --no-show --out res.png       # sans fenêtre
```

Le script affiche un bilan (tours, écart à la trajectoire, altitude, vitesse) et trace la vue de dessus (balises, mesures GPS bruitées,
trajectoire réelle) ainsi que l'altitude, la vitesse, le roulis et l'écart de suivi.

## Architecture

|Fichier|Rôle|
|-|-|
|`aircraft.py`|Solide rigide (masse, inertie, position, quaternion, vitesse angulaire), efforts aéro simples, autopilote bas niveau. Classe `Command` = commandes haut niveau.|
|`sensors.py`|`GPS`, `IMU`, `Barometer`, `Pitot` (cadence, bruit, biais, retard) + `SensorSuite`.|
|`geo.py`|Conversion repère local NED ↔ latitude/longitude.|
|`mission.py`|`Waypoint` (balise + conditions de passage), `Mission`, générateurs `circle()` et `figure\_eight()`, `distance\_to\_path()`.|
|`controller.py`|Interface `Controller` + premier algorithme `L1WaypointController`.|
|`simulator.py`|Boucle SIL : capteurs → contrôleur → dynamique, et journalisation (`SimLog`).|
|`run\_sim.py`|Exemple complet avec graphiques.|

## Conventions

* Repère monde **NED** : x = Nord, y = Est, z = Bas (altitude = −z). Repère avion **FRD** : avant, droite, bas.
* Roll > 0 : aile droite vers le bas. Pitch > 0 : nez vers le haut. Cap/yaw > 0 : nez vers la droite (0 = Nord, 90° = Est).
* Les balises sont données en mètres (Nord, Est) et altitude positive vers le haut, relativement à l'origine `GeoOrigin`.

## Le modèle d'avion

1. **Solide** : `x = \[position, vitesse sol, quaternion, p/q/r]`, intégré en RK4 (dt = 10 ms).
2. **Efforts** : portance `CL0 + CLa·alpha` (saturée), traînée parabolique, force latérale en `beta`, poussée `T = throttle·Tmax·(1 − V/V0)`. Le vent est un vecteur constant.
3. **Commandes haut niveau** (`Command`) : `roll`, `pitch` (angles, rad), `yaw\_rate` (rad/s), `throttle` (0..1).
Une boucle interne PD, dimensionnée avec l'inertie (`wn\_roll`, `wn\_pitch`, `zeta`), transforme ces consignes en couples.
Elle représente l'autopilote d'attitude bas niveau de l'avion réel.

Les paramètres sont dans `AircraftParams` (ordres de grandeur d'un avion de 2 kg) : **remplace-les par ceux de navychop**
(masse, inertie, surface, vitesse de croisière, poussée max…).

## Capteurs

Chaque capteur échantillonne l'état vrai à sa propre cadence et ajoute son erreur :

* **GPS** (5 Hz) : position vraie + bruit gaussien (σ = 1,5 m horizontal, 3 m vertical), vitesse + bruit. Options : `corr\_time\_s` (bruit corrélé, plus réaliste), `delay\_s` (latence). Il fournit **latitude/longitude/altitude** : le contrôleur reconvertit avec `GeoOrigin`, comme sur le vrai système.
* **IMU** (100 Hz) : force spécifique et vitesses angulaires, bruit blanc + biais constant.
* **Baromètre** (20 Hz) : altitude + bruit + dérive lente. **Pitot** (20 Hz) : vitesse air + bruit.

Ajouter un capteur : hériter de `Sensor`, écrire `measure(t, aircraft)`, l'ajouter au dict de `SensorSuite`.

## Balises et conditions de passage

```python
from mission import Waypoint, Mission, circle, figure\_eight

wps = circle(center\_north=0, center\_east=100, radius=100, alt=80, n\_points=16)
wps = figure\_eight(0, 0, half\_width=150, alt=80, n\_points=24)

# balise à condition de passage personnalisée
wp = Waypoint(north=200, east=50, alt=90, radius=15,
              max\_alt\_error=5,                              # condition verticale
              condition=lambda nav: nav.airspeed > 15.0)    # condition libre sur l'état estimé
mission = Mission(\[...], loop=True)                         # loop=False : s'arrête à la dernière balise
```

Une balise est franchie si : (distance < `radius` **ou** plan perpendiculaire franchi à moins de 3·radius) **et** condition verticale **et** condition libre. Si une condition échoue, la balise reste active et l'avion revient la chercher.

## Premier algorithme de suivi (`L1WaypointController`)

* **Estimation** : position GPS extrapolée avec la vitesse GPS (compense la cadence et le retard), altitude = baromètre filtré, vitesse air = Pitot filtré.
* **Guidage latéral** : loi L1 sur la branche *balise précédente → balise courante* ; `a\_cmd = 2·V²/L1·sin(eta)` → consigne de roulis.
* **Vertical** : PI + amortissement sur la vitesse verticale GPS → consigne de tangage ; l'altitude consigne est interpolée le long de la branche.
* **Vitesse** : PI sur la vitesse air → poussée. **Virage coordonné** : `yaw\_rate = g·tan(roll)/V`.

Résultats obtenus avec les réglages par défaut (GPS σ = 1,5 m) : écart moyen ≈ 5 m, altitude tenue à ±1 m, vitesse à ±0,3 m/s, y compris avec 5 m/s de vent. Le roulis demandé est bruité car il reflète le bruit GPS : une piste d'amélioration (filtre de Kalman).

Écrire son propre algorithme : hériter de `Controller` et implémenter `update(t, meas) -> Command`. `meas` est un dict `{'gps','imu','baro','pitot'}` dont chaque valeur est `None` s'il n'y a pas de nouvelle mesure à ce pas. Puis : `Simulator(ac, sensors, MonControleur(...), origin).run(150)`.

## Pistes pour la suite du projet

* Filtre de Kalman (GPS + IMU + baro) à la place de l'extrapolation simple, puis estimation du vent.
* Autres lois de guidage (pure pursuit, Dubins, suivi de chemin NLGL) et comparaison avec L1 sur des critères chiffrés (écart moyen/max, nombre de balises manquées).
* Scénarios dégradés : perte de GPS, GPS corrélé/retardé, rafales (turbulence), moteur faible.
* Passage en *hardware-in-the-loop* : remplacer le contrôleur par un pont (série/UDP) vers le vrai firmware.

## Limites du modèle

Modèle volontairement simple : pas de gouvernes explicites (l'autopilote d'attitude est idéalisé), pas de décrochage détaillé, pas de dynamique moteur/batterie, pas de rafales, terre plate. C'est suffisant pour développer et comparer des algorithmes de navigation ; à raffiner si besoin.

