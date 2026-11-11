# Navychop — Dépôt du projet

Ce dépôt contient **tous les éléments nécessaires pour construire un prototype d'avion autonome** : modèles 3D, plans de découpe et d'assemblage, carte électronique de navigation, codes embarqués (navigation, vision, cartes périphériques), outils Python et documentation.

La carte électronique de navigation a fait l'objet d'une **production testée avec la plateforme PCBGogo** (fichiers Gerber et de perçage fournis).

Une documentation adaptée aux **développeurs** et aux **IA** est également disponible pour comprendre, intégrer et améliorer l'aéronef.

---

## Vue d'ensemble de l'arborescence

| Dossier | Contenu |
|---|---|
| `common/` | Code partagé entre les différentes cartes |
| `documentation/` | Documentation pour les IA et les développeurs |
| `electronics/` | Projet KiCad de la carte de navigation et fichiers de fabrication |
| `matlab/` | Scripts de prototypage (filtres) |
| `mechanical/` | Modèles 3D, plans de découpe et d'assemblage du fuselage |
| `nav/` | Firmware de la carte de navigation (PlatformIO) |
| `vision/` | Firmware de vision embarquée (PlatformIO) |
| `peripherals/` | Codes des cartes périphériques (contrôle moteurs, station au sol) |
| `parameters/` | Définition centralisée des paramètres de l'avion |
| `python/` | Outils côté PC : génération de code, analyse de logs, commande au sol |

---

## `common/` — Code commun

- `communication/communication.cpp` / `.h` : code de communication partagé entre les cartes (liaisons entre la navigation, les périphériques et le sol).

## `documentation/` — Documentation

### `documentation/IA/`
Documents rédigés pour être exploitables par des assistants IA comme par des humains :

- `AIRFRAME.md` : description de la cellule (structure de l'avion).
- `ARCHITECTURE.md` : architecture logicielle et matérielle globale.
- `COMMANDE_SOL.md` : commande depuis la station au sol.
- `LOGGING.md` : journalisation des données de vol.
- `LORA_PROTOCOL.md` : protocole de liaison radio LoRa.
- `MESURES.md` : mesures et capteurs.
- `NAV_MOTOR_PROTOCOL.md` : protocole entre la carte de navigation et la carte de contrôle des moteurs.
- `synthese_robustesse_brouillage_radio.md` : synthèse sur la robustesse face au brouillage radio.

### `documentation/developpeurs/`
- `navychop_liaisons_commande.docx` : document sur les liaisons de commande, destiné aux développeurs.

### Autres
- `parameters_responsibility.ods` : tableau de répartition des responsabilités des paramètres entre les différents composants.

## `electronics/` — Électronique

- `connections_carte.png` : schéma des connexions de la carte.
- `navychop/` : projet KiCad de la **carte de navigation**
  - `navychop.kicad_sch`, `navychop.kicad_pcb`, `navychop.kicad_pro`, `navychop.kicad_prl` : schéma, circuit imprimé et projet.
  - `navychop_alim.kicad_pcb` / `.kicad_pro` : carte d'alimentation associée.
  - `navychop_Library.kicad_sym`, `navychop.pretty/` : symboles et empreintes personnalisés (BMP390, BNO085, ESP32-CAM, ESP32 30 broches, LoRa SX1278, NEO-N8M).
  - `V00R00/` : fichiers de fabrication (Gerber, perçage, `navychop.zip`) de la version V00R00, **utilisés pour la production via PCBGogo**.
  - `DRC.rpt` : rapport de vérification des règles de conception.
  - `navychop-backups/` : sauvegardes automatiques KiCad.
  - `org.openscopeproject.InteractiveHtmlBom_v2.10.0.zip` : plugin de génération de BOM interactive.
  - `fp-lib-table`, `sym-lib-table`, `fp-info-cache` : tables de bibliothèques KiCad.

## `mechanical/` — Mécanique

- `CAD/` : modèles 3D FreeCAD (`.FCStd`) et fichiers d'impression 3D (`.stl`) : attache de gouverne, pointe avant.
- `assembly/` : plans de découpe et d'assemblage
  - `decoupe_plaque_fuselage.py` : génération du patron de découpe des plaques de polystyrène du fuselage.
  - `structure_avion.py` : assemblage 3D de la structure de l'avion sous FreeCAD.
  - `pieces_fuselage.json` : description des pièces du fuselage.

## `nav/` — Firmware de navigation

Projet PlatformIO pour la carte de navigation :

- `src/main.cpp` : programme principal.
- `include/` : `config.h`, `default_parameters.h`, `project_types.h`.
- `lib/control/` : asservissement et contrôle de l'avion.
- `lib/datalink/` : liaison de données (trames LoRa), généré en partie par `python/gen_datalink.py`.
- `lib/gps/` : gestion du GPS.
- `lib/navigation/` : algorithmes de navigation.
- `platformio.ini` : configuration de compilation.

## `vision/` — Firmware de vision

Projet PlatformIO pour la vision embarquée :

- `src/main.cpp`, `gaussian_dual_core.cpp/.h` : traitement d'image (filtre gaussien sur les deux cœurs).
- `lib/image_process/` : traitement d'image.
- `lib/transformation/` : transformations géométriques.
- `lib/verification/` : vérifications et contrôles.
- `platformio.ini` : configuration de compilation.

## `peripherals/` — Cartes périphériques

- `motor_control/motor_control.ino` : code de la carte de **contrôle des moteurs**.
- `radio_control/radio_control.ino` : code de la **station au sol** (radiocommande).

## `parameters/` — Paramètres

- `parameters.json` : source unique des paramètres de l'avion, utilisée pour générer le code de la liaison de données.

## `matlab/`

- `test_filtres.m` : essais et validation de filtres.

## `python/` — Outils PC

| Script | Rôle |
|---|---|
| `Navychop.py` | Outil principal côté PC |
| `gen_datalink.py` | Génère `datalink.cpp/.h` à partir de `parameters.json` |
| `affiche_log.py` (`affiche_log_old.py`) | Affichage et analyse des logs de vol (CSV) |
| `read_log_nav.py` | Lecture des logs de la carte de navigation |
| `affiche_orientation.py`, `visu_R.py` | Visualisation de l'orientation / matrices de rotation |
| `commandes_sol_ps4.py` | Commande depuis le sol avec une manette PS4 |
| `lecture_mouvement.py` | Lecture et analyse de mouvements |
| `coefficients_gauss_8bits.py` | Calcul des coefficients du filtre gaussien en 8 bits |
| `generate_LUT_sin.py` | Génération d'une table de sinus (LUT) |
| `gradient_tranformation.py`, `test_contours.py` | Essais de traitement d'image (gradients, contours) |
| `estimate_framerate_flash.py` | Estimation de la cadence d'images |
| `save_images.py` | Sauvegarde d'images issues de la caméra |

---

## Par où commencer ?

1. **Comprendre le projet** : lire `documentation/IA/ARCHITECTURE.md` puis `AIRFRAME.md`.
2. **Construire la cellule** : utiliser `mechanical/` (pièces 3D à imprimer, patrons de découpe, assemblage).
3. **Fabriquer la carte** : envoyer `electronics/navychop/V00R00/navychop.zip` chez un fabricant de PCB (testé avec PCBGogo).
4. **Programmer les cartes** : compiler `nav/` et `vision/` avec PlatformIO, téléverser `peripherals/*.ino` avec l'IDE Arduino.
5. **Modifier les paramètres** : éditer `parameters/parameters.json`, puis régénérer le code avec `python/gen_datalink.py`.
6. **Analyser les vols** : exploiter les logs avec `python/affiche_log.py` (voir `documentation/IA/LOGGING.md`).
