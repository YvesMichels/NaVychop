# Estimation de mouvement par vision — fonctionnement et intégration dans l'hybridation multi-capteurs

Projet Navychop — module vision (ESP32-CAM, caméra OV3660, mono8 QVGA 320×240)
Version du 2026-11-23. Basé sur la lecture de `main.cpp` (vision), `image_process.cpp/.h`, `transformation.cpp/.h` et sur l'analyse des logs du salon (`logs_*.csv`) avec `navychop_hybrid.py`.

Légende : **[État actuel]** = ce que fait le code aujourd'hui ; **[Mesuré]** = constaté sur les logs du salon ; **[Proposé]** = piste non encore implémentée ni validée sur la carte.

---

## 1. Ce que le module mesure

Entre deux images successives, le module estime la transformation 2D qui amène l'image précédente (référence) sur l'image courante :

```
q = vp + (1 + div) · R(θ) · (p − vp) + t          p : pixel de l'image de référence
                                                   q : position correspondante dans l'image courante
```

| Paramètre | Signification | Unité dans le code | Unité dans la trame / CSV |
|---|---|---|---|
| `tx`, `ty` | translation de l'image | pixel QVGA (Q16.16) | pixels (int16 en Q8, soit px × 256) |
| `theta` | rotation de l'image autour de `vp` | radian (Q16.16) | radian × 256 (voir § 4.7) |
| `div` | dilatation radiale (zoom) autour de `vp` | sans unité (Q16.16) | **non transmis** |
| `vp` | centre de rotation et de dilatation (« point de fuite ») | pixel, constant | (152, 112) en QVGA, (16, 11) en image réduite |

C'est un mouvement **apparent** de l'image. Il mélange la rotation de la caméra, sa translation et la structure de la scène (voir § 8).

---

## 2. Vue d'ensemble du pipeline [État actuel]

```
 caméra OV3660 (QVGA mono8, 320×240)
        │
        ├─► réduction 8×8 → 40×30 ─► flou gaussien 7×7 ─► im_conv_red
        │                                                      │
        │      points de référence réduits (204)  ◄── image précédente
        │                                                      │
        │            ┌─ recherche exhaustive (125 candidats) ──┤
        │            └─ descente de gradient Huber (25 it.)  ──┤  → tx, ty, θ, div (échelle 1/8)
        │                                                      │
        │      extraction des points réduits pour la trame suivante
        │
        ├─► flou gaussien 9×9 ─► im_conv ─► gradient L1 ─► img (image de gradient)
        │
        │      tx, ty × 8 ; vp mis à l'échelle QVGA
        │      points de référence pleins (266)  ◄── image précédente
        │                                                      │
        │            └─ descente de gradient Huber (25 it.)  ──┤  → tx, ty, θ, div (QVGA)
        │
        │      extraction des 266 points pour la trame suivante (max de gradient par bloc 16×16)
        │
        └─► trame série : θ, tx, ty + 16 mots d'instrumentation (76 octets, 115200 bauds)
```

Principe général : estimation **dense-éparse** (« direct method »). On ne calcule ni descripteurs ni correspondances. On cherche le mouvement qui rend les pixels de la nouvelle image égaux à ceux mémorisés à des endroits choisis de l'ancienne. Le calcul se fait en deux échelles : une estimation approchée sur l'image réduite, puis un raffinement en pleine résolution.

---

## 3. Modèle de mouvement et coût

### 3.1 Coût photométrique de Huber

Pour un jeu de paramètres `p = (tx, ty, θ, div)`, chaque point de référence `(x, y, I_ref)` est transformé en `(xw, yw)`. On lit l'image courante par interpolation bilinéaire (précision 1/256 de pixel) et on forme l'erreur :

```
e = I_cur(xw, yw) − I_ref
coût(p) = moyenne sur les points valides de  ρ(e)

ρ(e) = e²                       si |e| < δ
     = 2δ|e| − δ²               sinon                  δ = 10 niveaux de gris
```

Les points dont la position transformée sort de l'image (marge de 8 ou 16 px) sont ignorés. Le coût de Huber limite l'influence des points aberrants (zones qui bougent, reflets, bords).

### 3.2 Pourquoi les points sont pris sur les forts gradients

Un pixel dans une zone uniforme ne contraint pas le mouvement : le coût y est plat. Le code prend donc, par bloc de 16×16 pixels, le pixel de plus fort gradient L1 (|∂x| + |∂y|, calculé sur l'image floutée 9×9). Il en résulte 14 × 19 = **266 points** répartis presque uniformément. Chaque point est stocké sur 32 bits :

```
bits 31..23 : y (9 bits) | 22..14 : x (9 bits) | 13..6 : valeur du pixel (8 bits) | 5..0 : gradient (6 bits, saturé à 63)
```

Sur l'image réduite, les points sont pris sur une grille régulière (un point sur deux, de 0 à 22 en y et de 0 à 32 en x), soit 12 × 17 = **204 points**. Le gradient n'est pas utilisé.

---

## 4. Étapes de l'algorithme [État actuel]

### 4.1 Acquisition

Capture en niveaux de gris `PIXFORMAT_GRAYSCALE`, `FRAMESIZE_QVGA`, deux tampons caméra (`fb_count = 2`). La période mesurée entre deux estimations est d'environ **120 ms** (8 Hz) **[Mesuré]**.

### 4.2 Prétraitement

- **Réduction** : moyenne de blocs 8×8 → image 40×30.
- **Flou gaussien** : 7×7 (σ = 2) sur l'image réduite, 9×9 (σ = 3) en pleine résolution. Il élargit le bassin de convergence de la descente de gradient et réduit le bruit.
- Les filtres ne calculent que la zone valide : la sortie est **décalée d'un demi-noyau** et ses derniers pixels sont à zéro. Ce décalage est le même pour la référence et l'image courante. Il s'annule dans les translations, mais il fixe la place de `vp`.

### 4.3 Estimation approchée sur l'image réduite : recherche exhaustive

`exhaustive_init` évalue le coût de Huber sur une grille de candidats (`div` = 0) :

- `theta` ∈ {−0,2 ; −0,1 ; 0 ; +0,1 ; +0,2} rad,
- `tx`, `ty` ∈ {−4, −2, 0, +2, +4} px de l'image réduite.

Soit 5 × 5 × 5 = **125 candidats**, en n'utilisant qu'un point de référence sur deux. Le meilleur sert de point de départ.

**Conséquence importante :** le pas de 2 px sur l'image réduite vaut **16 px en QVGA**, et 0,1 rad vaut environ 20 px au bord de l'image. Une erreur d'une case de la grille produit un saut de ±16 px (voir § 5).

### 4.4 Raffinement : descente de gradient

`optim_huber_grad_q16` fait 25 itérations. À chaque itération, le gradient du coût par rapport à `(div, θ, tx, ty)` est accumulé sur tous les points. Les dérivées géométriques sont analytiques, et le gradient d'image est une différence finie sur le pixel entier. Les paramètres bougent ensuite d'un pas fixe dans la direction du signe du gradient. Le pas décroît à chaque itération (`alpha *= 0.9`).

| Étape | Pas initiaux (tx, ty, θ, div) en Q16 | Équivalent |
|---|---|---|
| Image réduite | 10000, 10000, 1000, 500 | 0,15 px, 0,15 px, 0,015 rad, 0,008 |
| Image pleine | 30000, 30000, 500, 200 | 0,46 px, 0,46 px, 0,008 rad, 0,003 |

### 4.5 Passage à l'échelle supérieure

`tx` et `ty` sont multipliés par 8, `vp` est recalculé en QVGA, `theta` et `div` sont inchangés. La descente se poursuit sur l'image pleine, avec les 266 points.

### 4.6 Préparation de la trame suivante

Une fois l'image courante traitée, ses points sont extraits (204 réduits, 266 pleins) et deviennent la référence de l'image suivante. Le module ne garde donc qu'**une image de mémoire**.

### 4.7 Trame envoyée au module principal

76 octets : synchro `AA 55 CC 33`, format `0x42`, puis :

- `theta` : 16 bits de poids faible du Q16.16, soit radian × 65536 modulo 65536. D'après les valeurs observées dans les logs, le CSV exprime `theta` en **radian × 256** (le décodeur divise l'entier 16 bits par 256, ce que je n'ai pas relu). Le champ est borné à **±0,5 rad** (±128 dans le CSV) ;
- `tx`, `ty` : octets 1 et 2 du Q16.16, soit pixel × 256 sur 16 bits, borné à ±128 px ;
- 16 mots int32 d'instrumentation (horodatages `millis()` des étapes, 11 utilisés) ;
- checksum XOR.

Ni `div`, ni le coût, ni un numéro ou une date de capture ne sont transmis. Le coût `init_cost` est passé à la fonction d'envoi mais n'est pas écrit dans la trame. La colonne `cost` de `logs_vision.csv` est donc vide.

---

## 5. Limites constatées sur les logs du salon

Essai : avion déplacé à la main à 1–2 m du sol dans un salon. Caméra, gyroscope, accéléromètre, magnétomètre et baromètre sont solidaires. Analyse avec `navychop_hybrid.py`.

**[Mesuré]**

- **Estimations exploitables :** en mouvement, seulement environ 25 % des mesures de `tx`, 24 % de `ty` et 44 % de `theta` concordent avec le gyroscope. Environ 4 % des images sont perdues (17 sur 398) et la date de réception a une gigue d'environ 32 ms RMS.
- **Retard :** la vision est en retard d'environ **160 ms** sur le gyroscope (à re-mesurer à chaque modification de l'algorithme).
- **Focale :** le canal `ty` suit la rotation autour de l'axe x de l'IMU avec un gain de −281 px/rad, soit une focale d'environ 281 px et un champ horizontal d'environ 59°. 70 % de sa variance s'explique par le gyroscope quand l'estimation est correcte. `theta` suit l'axe z avec un gain de −283 en rad × 256 (théorie : 256). `tx` n'a pas pu être identifié (corrélation 0,37) : il faudra une rotation autour de cet axe.
- **Valeurs répétées :** `tx` et `ty` retombent sur ±1,56 et ±0,71 px, ainsi que sur ces valeurs décalées de ±14,4 / 15,3 / 17,6 / 30,4 px. `theta` retombe sur ±20,05, ±5,55 et ±1,82 (en rad × 256).

**[Analyse du code]**

- **Descente de gradient quasi inactive :** dans `optim_huber_grad_q16`, `alpha_q16 * 58982` déborde en `int32` dès la première itération. `alpha_q16` devient négatif (−0,1, −0,09, …) et la descente bouge au total de moins d'un pas initial (environ ±0,14 px sur l'image réduite, ±0,43 px sur l'image pleine). Une émulation en entier 32 bits retrouve les valeurs répétées observées : 1,136 ± 0,426 = 1,562 et 0,710 pour `tx`/`ty`, 3,684 + 1,867 = 5,551 pour `theta`.
- **Sens de la mise à jour :** le code fait `p += pas` quand `g > 0`, alors que `g` est le vrai gradient du coût (vérifié en simulation sur des scènes synthétiques). Il monte. Ce sens est compensé par le signe négatif d'`alpha` : **corriger le débordement seul aggraverait le problème**.
- **Grille exhaustive trop grossière :** un pas de 2 px réduit équivaut à 16 px en QVGA, ce qui correspond aux sauts de ±14,4 / 15,3 / 17,6 px observés.
- **Pas de mesure de qualité** (coût, `div`, nombre de points, texture) transmise.

---

## 6. Améliorations proposées [Proposé]

Priorités, dans l'ordre conseillé :

1. **Corriger `alpha_q16`** en Q15 (`alpha = (alpha * 29491) >> 15`, tout en 32 bits) **et inverser le sens** de la mise à jour (`p -= pas` quand `g > 0`).
2. **Remplacer la descente par le signe** par une descente de type **Gauss-Newton diagonale** (voir `proposition_gn_diag.cpp`) :
   - pas de chaque paramètre = −g / (Σ w·J² + H_MIN), indépendant du contraste de l'image ;
   - boucle sur les points en 32 bits ;
   - en simulation flottante (scènes synthétiques, départ à ±6 px et ±0,03 rad), 100 % de réussite en 15 itérations pour les contrastes moyens et faibles, médiane de 4 à 9 itérations, arrêt anticipé sous 1/32 px ;
   - sous-produit : `h_tx`, `h_ty` (information sur la texture), coût moyen et nombre de points valides, utilisables comme **indicateur de qualité**.
3. **Initialiser avec le gyroscope** : le module principal envoie à l'ESP32-CAM l'incrément de rotation entre les deux captures. Le départ de la descente devient `(f · φ)` au lieu de 0, et la recherche exhaustive ne sert plus que de secours. Cela supprime presque tous les sauts de 16 px.
4. **Transmettre `div`, le coût, la qualité et un numéro d'image**, avec la date de capture côté ESP32-CAM. Cela supprime la gigue de réception.
5. **Rejeter ou pondérer les points sans gradient** : dans `extract_points`, écarter les blocs dont le gradient maximal est faible (par exemple < 5).
6. **Raffiner la grille** : pas de 1 px réduit, ou deuxième passe ±1 px autour du meilleur candidat.
7. **Convertir la géométrie en 32 bits** : `read_bilineaire` n'utilise que 8 bits de fraction. Positions en Q8, sinus et cosinus en Q14. À tester.

Ces propositions viennent d'une simulation en flottant et d'un calcul des plages. Elles n'ont pas été compilées ni mesurées sur la carte.

---

## 7. Intégration dans le système hybridé

### 7.1 Ce que la vision apporte par rapport au gyroscope

Le mouvement image mesuré est la somme de deux contributions :

```
(tx, ty, θ)_mesuré  =  A · φ           +   flux de translation
                       (rotation caméra)   (vitesse / profondeur)
```

- Pour une **rotation caméra** φ (rad), le déplacement image vaut environ `f · φ` pour les axes perpendiculaires à l'axe optique, avec `f ≈ 281 px`. La rotation autour de l'axe optique donne `θ ≈ φ`. Cette partie est déjà connue par le gyroscope : elle est **redondante**, et sert à calibrer et à valider.
- La **translation** de la caméra donne un déplacement image proportionnel à `v / Z` (vitesse divisée par la distance à la scène). C'est la seule information **nouvelle** de la vision.
- La **dilatation** `div` est liée à `v_axiale / Z` (temps avant contact). Elle est actuellement estimée mais pas transmise.

Il faut donc **dérotater** : retirer la contribution du gyroscope, puis exploiter le résidu.

### 7.2 Chaîne de traitement côté module principal [Proposé]

```
trame vision (tx, ty, θ, [div, qualité])
   │
   ├─ 1. datation : t_capture = t_réception − τ_vision      (τ ≈ 160 ms, à calibrer)
   │                ou date de capture envoyée par l'ESP32-CAM
   ├─ 2. rotation gyroscope φ entre t_capture − T et t_capture     (interpolation des quaternions)
   ├─ 3. dérotation : r = y − A · φ        (A : gains et axes identifiés, § 7.5)
   ├─ 4. porte de validité (§ 7.3)
   └─ 5. mise à jour du filtre : r → mesure de vitesse / profondeur (§ 7.4)
```

Les étapes 1 à 3 sont déjà réalisées dans `navychop_hybrid.py` (recalage des dates, estimation du retard, ajustement robuste des gains). Elles peuvent être reprises telles quelles dans le code embarqué.

### 7.3 Porte de validité

Une mesure est rejetée si l'une de ces conditions est vraie :

- valeur **répétée** (même valeur à 1e-6 près au moins 3 fois et |v| > 0,5) : signature d'une estimation non convergée ;
- résidu `|r| > 3 σ` (σ robuste par canal, estimé à partir de la médiane des écarts) ;
- qualité faible : `h_tx` ou `h_ty` sous un seuil (peu de texture, problème d'ouverture), coût moyen élevé, peu de points valides ;
- rotation gyroscope trop forte entre deux images (flou de bougé, sortie de la zone de capture) : seuil à mesurer en fonction de la vitesse de rotation ;
- image perdue (numéro d'image non consécutif) : la rotation gyroscope doit couvrir l'intervalle réel.

### 7.4 Modèle de mesure pour un filtre (ESKF/EKF) [Proposé]

On suppose ici une caméra regardant vers le bas, au-dessus d'un sol à peu près plan. L'orientation réelle de la caméra sur l'avion est à renseigner.

```
flux_xy (px)  ≈  − f · dt · (R_cam←corps · v_corps)_xy / Z
Z             =  altitude sol / cos(inclinaison)        (baromètre + attitude)
```

- **État** : attitude (déjà issue du GRV hybridé avec le magnétomètre), vitesse (3), position, biais accéléromètre.
- **Prédiction** : accéléromètre et gyroscope à 25 Hz.
- **Correction vision** : mesure `flux_xy` de covariance `σ²` (champ `σ` du rapport, ou fonction de `h_tx`, `h_ty`), avec rejet par test de l'innovation (χ²).
- **Correction baromètre** : altitude et vitesse verticale, ce qui fournit `Z`.
- **Retard** : la mesure arrive environ 160 ms après la capture. Garder un historique des états (environ 13 échantillons à 25 Hz), appliquer la correction à l'état daté de `t_capture`, puis repropager jusqu'à l'instant courant.
- **GPS** : en extérieur, mesure de position et de vitesse absolues. La vision comble les trous et améliore la vitesse à courte échelle.
- **Asservissement d'attitude** : ne pas le boucler sur la vision. La rotation vient du gyroscope et du magnétomètre, la vision n'intervient que sur vitesse et position.

**Résolution en vitesse (ordre de grandeur, à vérifier) :** `δv ≈ σ_flux · Z / (f · dt)`. Avec σ_flux = 0,3 px, f = 281 px, dt = 0,12 s : δv ≈ 0,02 m/s à Z = 2 m et ≈ 0,27 m/s à Z = 30 m. Plus l'avion vole haut, moins la vision est informative.

**Capture :** à 15 m/s et Z = 30 m, le flux est d'environ 0,5 rad/s soit 140 px/s, donc environ 17 px par image de 120 ms. Cela reste dans la plage de la grille exhaustive (±32 px en QVGA), mais l'erreur résiduelle de la grille (jusqu'à 8 px) dépasse ce que la descente actuelle peut rattraper. Il faut prédire le départ à partir du gyroscope, de la vitesse estimée et de l'altitude (§ 6, point 3).

### 7.5 Calibration vision ↔ inertiel

À chaque modification de l'algorithme ou du montage de la caméra, lancer :

```
python navychop_hybrid.py DOSSIER_CSV --out sortie
```

À reporter dans le code embarqué à partir de `hybrid_report.json` :

| Valeur du rapport | Usage |
|---|---|
| `retard_vision_vs_gyro_s` | décalage de datation τ |
| `axe_IMU_par_canal`, `gain_par_canal_*` | matrice A (axes et gains de dérotation) |
| `sigma_residu_robuste`, `seuil_aberration` | bruit de mesure et seuils de rejet |
| `taux_canal_utilisable_pct_images_en_mouvement` | indicateur de santé de l'estimateur |
| `periode_vision_s`, `images_perdues_estimees` | gestion des trames manquantes |

Procédure d'essai conseillée :

1. rotation pure autour de chacun des trois axes (identifie les gains, en particulier `tx` qui n'est pas identifié aujourd'hui) ;
2. translation pure à vitesse et hauteur connues (valide le modèle `v / Z`) ;
3. mouvement libre à la main (mesure du taux d'estimations exploitables avant / après chaque amélioration du § 6).

### 7.6 Interface proposée entre les deux modules

| Sens | Champ | Rôle |
|---|---|---|
| vision → principal | `tx`, `ty`, `θ` (actuel) | mouvement image |
| vision → principal | `div` | dilatation (temps avant contact) |
| vision → principal | `h_tx`, `h_ty`, coût moyen, nombre de points | qualité / covariance |
| vision → principal | numéro d'image + date de capture | datation exacte, détection des pertes |
| principal → vision | incrément de rotation gyroscope entre deux captures (3 valeurs) | initialisation de la descente |
| principal → vision | altitude et vitesse estimées (optionnel) | prédiction de la translation |

---

## 8. Hypothèses et domaine de validité

- **Scène texturée** : les points sont pris sur les forts gradients, mais une scène sans texture (ciel uniforme, mur lisse) rend l'estimation indéterminée. Le critère `h_tx`, `h_ty` doit le détecter.
- **Scène rigide et à peu près plane** pour le modèle `v / Z`. Un sol très accidenté ou des objets proches violent cette hypothèse.
- **Éclairage stable** : le coût compare des niveaux de gris, donc un changement de gain ou d'exposition entre deux images biaise l'estimation. Le module relève déjà `aec_value`, `agc_gain` et `ae_level` (variable `cam_param`, non transmise aujourd'hui), qui pourraient servir à compenser ou à rejeter.
- **Faible rotation entre deux images** : la plage de convergence est de l'ordre de la largeur du flou gaussien (quelques pixels). Au-delà, une prédiction par le gyroscope devient nécessaire.
- **Flou de bougé** à fort mouvement : à surveiller avec le temps d'exposition.
- **Le GPS n'est pas pris en compte** dans les essais actuels (aucun satellite en intérieur).

---

## 9. Annexes

### 9.1 Fichiers sources

| Fichier | Rôle |
|---|---|
| `main.cpp` (vision) | boucle principale, enchaînement réduit → plein, envoi de la trame |
| `image_process.cpp/.h` | réduction, flous gaussiens, gradient, extraction des points, envoi de debug |
| `transformation.cpp/.h` | transformation de points, coût de Huber, recherche exhaustive, gradient, descente |

### 9.2 Colonnes de `logs_vision.csv`

| Colonne | Contenu |
|---|---|
| `t` | date de réception (s) sur le module principal |
| `theta` | rotation de l'image, **rad × 256** |
| `tx`, `ty` | translation de l'image, pixels QVGA |
| `div` | actuellement 0 (non transmis) |
| `cost` | actuellement vide (non transmis) |

### 9.3 Outils associés

- `navychop_hybrid.py` : recalage des dates, estimation du retard et des gains, détection des mesures aberrantes, rapport JSON et figures.
- `proposition_gn_diag.cpp` : descente Gauss-Newton diagonale en 32 bits, avec indicateur de qualité.
- `sim_descent.py` : simulation en flottant qui compare les stratégies de pas (signe, `grad·λ`, Rprop, Gauss-Newton).
