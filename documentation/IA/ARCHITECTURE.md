\# Architecture globale — `navychop`



> \*\*Projet :\*\* `navychop`

> \*\*Nature :\*\* projet amateur scientifique et ludique d'autonomie pour aéronef RC

> \*\*Architecture :\*\* avion électrique à voilure fixe, piloté par un système embarqué basé sur ESP32

> \*\*Objectif :\*\* expérimenter les capacités techniques d'un pilote autonome développé par une seule personne, avec l'assistance d'IA génératives.



\---



\## 1. Objet du projet



`navychop` est un projet expérimental visant à développer progressivement un \*\*pilote autonome pour aéronef RC\*\*.



Le projet poursuit deux objectifs complémentaires :



1\. \*\*Technique et scientifique\*\*



&#x20;  \* expérimenter la perception, l'estimation d'état, la commande et la navigation autonomes ;

&#x20;  \* évaluer différentes architectures algorithmiques avec des ressources embarquées limitées ;

&#x20;  \* caractériser expérimentalement les performances et les limites d'un système autonome amateur.



2\. \*\*Méthodologique\*\*



&#x20;  \* explorer jusqu'où peut aller un projet développé par \*\*une seule personne assistée par des IA génératives\*\* ;

&#x20;  \* comparer indirectement cette approche avec les pratiques et contraintes d'un développement industriel ;

&#x20;  \* conserver une architecture suffisamment modulaire pour pouvoir faire évoluer indépendamment le matériel, les algorithmes et les logiciels.



Le système est conçu comme une plateforme expérimentale : les choix actuels ne constituent donc pas nécessairement l'architecture finale.



\---



\# 2. Vue système



L'architecture générale peut être représentée comme suit :



```text

&#x20;                          ┌─────────────────────┐

&#x20;                          │    Pilote humain    │

&#x20;                          │     PS4 / RC        │

&#x20;                          └──────────┬──────────┘

&#x20;                                     │

&#x20;                                     │ Commandes

&#x20;                                     ▼

┌─────────────────────────────────────────────────────────────────┐

│                           AVION                                 │

│                                                                 │

│  ┌──────────────────┐       ┌──────────────────────────────┐    │

│  │      Vision      │──────▶│                              │    │

│  │ caméra / système │ UART  │                              │    │

│  └──────────────────┘       │                              │    │

│                             │            ESP32              │    │

│  ┌──────────────────┐       │                              │    │

│  │     BNO085       │──────▶│  Estimation d'état           │    │

│  │ IMU / attitude   │ I²C  │       +                      │    │

│  └──────────────────┘       │  Commande de vol             │    │

│                             │       +                      │    │

│  ┌──────────────────┐       │  Gestion des modes            │    │

│  │     BMP390       │──────▶│       +                      │    │

│  │   pression       │ I²C  │  Acquisition / logging        │    │

│  └──────────────────┘       │                              │    │

│                             └───────┬───────────┬──────────┘    │

│  ┌──────────────────┐               │           │               │

│  │    NEO-N8M       │───────────────┘           │               │

│  │      GNSS        │ UART                      │ LoRa          │

│  └──────────────────┘                           ▼               │

│                                        ┌─────────────────┐       │

│                                        │ Télécommande /  │       │

│                                        │ télémétrie      │       │

│                                        └─────────────────┘       │

│                                                                 │

│                              │                                   │

│                              ▼                                   │

│                    ┌──────────────────┐                          │

│                    │  Commandes servo │                          │

│                    │  + propulsion    │                          │

│                    └────────┬─────────┘                          │

│                             ▼                                    │

│                       Surfaces de vol                            │

└─────────────────────────────────────────────────────────────────┘

```



L'ESP32 constitue le \*\*calculateur central du système\*\*. Les capteurs fournissent les informations nécessaires à l'estimation de l'état de l'avion, tandis que les algorithmes de commande produisent les consignes destinées aux actionneurs.



\---



\# 3. Architecture physique



\## 3.1 Cellule



Configuration actuelle :



| Paramètre                 |                       Valeur |

| ------------------------- | ---------------------------: |

| Envergure                 |                       \~1,4 m |

| Longueur                  |                       \~0,9 m |

| Surface alaire            |                      \~40 dm² |

| Masse en ordre de vol     |                      \~1090 g |

| Construction              | plaques de polystyrène \~3 mm |

| Propulsion                |                   électrique |

| Configuration de commande |                      élevons |

| Axe X                     |                        avant |

| Axe Y                     |                       gauche |

| Axe Z                     |                         haut |



La cellule est une plateforme expérimentale plutôt qu'une cellule optimisée pour la performance.



Le positionnement du centre de gravité fait partie intégrante de la mise au point du système de commande. La configuration initiale autour de 40 % de corde a été corrigée vers une position plus avant, avec une cible actuelle de l'ordre de \*\*25–30 % de corde\*\*.



\---



\# 4. Propulsion



Configuration actuelle :



\* moteur \*\*A2212 1000 KV, 13T\*\* ;

\* batterie \*\*LiPo 3S 2200 mAh, 11,1 V\*\* ;

\* ESC \*\*40 A\*\*.



La propulsion est actuellement commandée séparément des surfaces de vol.



L'architecture logicielle considère donc au minimum :



```text

Commande throttle

&#x20;      │

&#x20;      ▼

&#x20;     ESC

&#x20;      │

&#x20;      ▼

&#x20;    Moteur

```



Le comportement exact du système de propulsion et les limites associées devront être documentés dans le Markdown dédié.



\---



\# 5. Architecture électronique



\## 5.1 Calculateur principal



Le calculateur embarqué est un :



\*\*ESP32 DevKit 30 broches\*\*



Il assure notamment :



\* acquisition des capteurs ;

\* estimation de l'état de l'avion ;

\* calcul des commandes ;

\* gestion des modes de vol ;

\* communication LoRa ;

\* communication avec le système de vision ;

\* acquisition GNSS ;

\* enregistrement des données ;

\* gestion des paramètres.



L'ESP32 constitue donc le \*\*nœud central de l'architecture avionique\*\*.



\---



\## 5.2 Capteurs



\### BNO085



Le BNO085 fournit principalement les informations inertielle et d'attitude.



Interface :



```text

I²C

Adresse : 0x4B

```



Bibliothèque actuellement utilisée :



```text

Adafruit\_BNO08x v1.2.3

```



Les données utilisées ou étudiées comprennent notamment :



\* quaternion ;

\* accélération linéaire ;

\* champ magnétique ;

\* autres sorties SH-2 selon les besoins.



Les quaternions sont enregistrés sous la forme :



```text

gqx

gqy

gqz

gqw

```



à environ \*\*25 Hz\*\*.



Le yaw n'est actuellement pas destiné à être directement contrôlé par la boucle de commande principale.



\---



\### BMP390



Le BMP390 fournit principalement la pression atmosphérique permettant notamment d'estimer les variations d'altitude.



Interface :



```text

I²C

Adresse : 0x76

```



Une pression de référence au sol `P0` est utilisée.



Le système ne dispose actuellement pas d'une mesure de vitesse air dédiée.



\---



\### GNSS — NEO-N8M



Le module GNSS est un :



\*\*NEO-N8M\*\*



Interface :



```text

UART

TX → GPIO17

```



L'antenne GNSS est actuellement inactive dans la configuration expérimentale connue.



Le GNSS est destiné à fournir à terme une information de position et/ou de navigation.



\---



\### Vision



Un système de vision externe communique avec l'ESP32 par UART :



```text

UART

GPIO25

115200 bauds

```



La vision est considérée comme un \*\*sous-système indépendant\*\* dont les données sont transmises au calculateur principal.



La nature exacte des traitements de vision et du format des données doit rester documentée dans un fichier spécifique.



\---



\# 6. Communications



\## 6.1 LoRa



Le système utilise un module \*\*SX1278\*\*.



Connexion SPI :



| Signal | GPIO |

| ------ | ---: |

| SCK    |   18 |

| MISO   |   19 |

| MOSI   |   23 |

| NSS    |    5 |

| RST    |   27 |

| DIO0   |   26 |



LoRa est utilisé pour les communications entre le système embarqué et l'extérieur.



Une partie importante du protocole consiste à transmettre des commandes compactes.



\---



\## 6.2 Encodage des commandes



Certaines commandes sont codées sur \*\*6 bits\*\*.



Quatre valeurs de commande sont regroupées dans trois octets :



```text

4 × 6 bits = 24 bits = 3 octets

```



Le codage actuel utilise notamment une valeur neutre :



```text

neutre = 31

```



Le protocole doit être considéré comme une interface système : toute modification du format doit être répercutée simultanément dans les émetteurs et récepteurs.



\---



\# 7. Commandes utilisateur



Une manette PS4 est actuellement utilisée comme interface utilisateur.



Mapping actuel :



| Commande PS4            | Fonction      |

| ----------------------- | ------------- |

| △                       | SAFE / MANUAL |

| ✕                       | HOLD          |

| □                       | Logging       |

| ○                       | Save          |

| R2                      | Throttle      |

| Stick gauche horizontal | Yaw           |

| Stick droit vertical    | Pitch         |

| D-pad                   | Trims         |



Le rôle exact des différents modes et la logique de priorité entre commande humaine et autonomie sont décrits dans le document consacré aux \*\*modes de vol et au contrôle manuel\*\*.



\---



\# 8. Architecture logicielle



L'architecture logicielle peut être vue comme une chaîne :



```text

&#x20;                 ┌─────────────────┐

&#x20;                 │     Capteurs    │

&#x20;                 └────────┬────────┘

&#x20;                          │

&#x20;                          ▼

&#x20;                 ┌─────────────────┐

&#x20;                 │ Acquisition /   │

&#x20;                 │ conversion      │

&#x20;                 └────────┬────────┘

&#x20;                          │

&#x20;                          ▼

&#x20;                 ┌─────────────────┐

&#x20;                 │ Estimation état │

&#x20;                 │ attitude        │

&#x20;                 │ altitude        │

&#x20;                 │ position        │

&#x20;                 └────────┬────────┘

&#x20;                          │

&#x20;                          ▼

&#x20;                 ┌─────────────────┐

&#x20;                 │ Gestion modes   │

&#x20;                 │ de vol          │

&#x20;                 └────────┬────────┘

&#x20;                          │

&#x20;                          ▼

&#x20;                 ┌─────────────────┐

&#x20;                 │ Contrôleur      │

&#x20;                 │ de vol          │

&#x20;                 └────────┬────────┘

&#x20;                          │

&#x20;                          ▼

&#x20;                 ┌─────────────────┐

&#x20;                 │ Mixer / limites │

&#x20;                 │ actionneurs     │

&#x20;                 └────────┬────────┘

&#x20;                          │

&#x20;                          ▼

&#x20;               ┌──────────────────────┐

&#x20;               │ Servos / ESC / moteur│

&#x20;               └──────────────────────┘

```



Les communications et le logging fonctionnent en parallèle de cette chaîne.



\---



\# 9. Commande de vol



\## 9.1 Axes



Le repère avion est défini par :



```text

X → avant

Y → gauche

Z → haut

```



Les principales variables de commande sont :



\* roll ;

\* pitch ;

\* yaw ;

\* throttle.



La commande actuelle est particulièrement centrée sur le \*\*roll et le pitch\*\*, les deux axes étant commandés par les surfaces de l'avion.



\---



\## 9.2 Élevons



Les gouvernes principales sont deux élevons.



Les commandes de vol doivent donc être transformées en commandes individuelles de servos par un \*\*mixage\*\*.



Conceptuellement :



```text

&#x20;             Roll

&#x20;               │

&#x20;               ▼

&#x20;            ┌─────┐

&#x20;            │     │

Pitch ──────▶│Mixer│──────▶ Servo gauche

&#x20;            │     │

&#x20;            └─────┘──────▶ Servo droit

&#x20;               ▲

&#x20;               │

&#x20;             Roll

```



Le sens et les coefficients exacts du mixage doivent être documentés dans le Markdown dédié à la commande de vol.



\---



\# 10. Paramétrage



Une partie importante des paramètres du système est externalisée dans une configuration JSON.



Les paramètres comprennent notamment :



\* gains de contrôleurs ;

\* paramètres de commande ;

\* trims ;

\* limites de servos ;

\* autres paramètres d'exploitation.



Les paramètres de servos utilisent notamment :



```text

type : uint8\_t

binpt : 0

index : ≥ 12

```



Le JSON constitue donc une \*\*interface de configuration\*\*, distincte du code de contrôle.



Les règles de conversion entre représentation JSON, variables C/C++ et valeurs physiques doivent être documentées séparément.



\---



\# 11. Limitation et sécurité des actionneurs



Les commandes de servos sont soumises à des limites configurables.



L'architecture doit garantir :



```text

Commande calculée

&#x20;      │

&#x20;      ▼

Saturation / limites

&#x20;      │

&#x20;      ▼

Commande servo valide

```



Cette séparation est importante : un algorithme de commande ne doit pas pouvoir générer directement une commande physiquement hors plage.



\---



\# 12. Failsafe



Le système possède un mécanisme de \*\*failsafe\*\* destiné à gérer une perte ou une invalidité des commandes reçues.



Le principe général est :



```text

Communication valide ?

&#x20;      │

&#x20;  ┌───┴───┐

&#x20; OUI      NON

&#x20;  │        │

&#x20;  ▼        ▼

Commandes  FAILSAFE

normales

```



Le failsafe agit notamment sur :



\* le flag de commande ;

\* le throttle ;

\* les commandes de vol.



Les valeurs binaires exactes du protocole doivent être documentées dans le fichier consacré au protocole LoRa.



\---



\# 13. Logging



Le système possède une fonction d'enregistrement des données de vol.



Les données enregistrées peuvent notamment inclure :



\* attitude ;

\* quaternion ;

\* commandes ;

\* états des modes ;

\* mesures capteurs ;

\* paramètres utiles au diagnostic.



Le logging est une fonction importante du projet car l'objectif est de pouvoir analyser quantitativement les essais en vol.



Le principe général est :



```text

Capteurs ───────┐

&#x20;               │

Commandes ──────┤

&#x20;               │

États ──────────┤

&#x20;               ▼

&#x20;            Logger

&#x20;               │

&#x20;               ▼

&#x20;         Données de vol

```



Le format exact des logs doit être décrit dans un Markdown spécifique.



\---



\# 14. Philosophie de développement



Le projet privilégie une architecture permettant de séparer :



\### Matériel



```text

Cellule

&#x20; ↓

Actionneurs

&#x20; ↓

Électronique

```



\### Acquisition



```text

Capteurs

&#x20; ↓

Drivers

&#x20; ↓

Données normalisées

```



\### Algorithmes



```text

Données capteurs

&#x20; ↓

Estimation

&#x20; ↓

Commande

```



\### Infrastructure



```text

Communication

Logging

Configuration

Failsafe

```



Cette séparation doit permettre de remplacer progressivement un élément sans réécrire l'ensemble du système.



\---



\# 15. Architecture fonctionnelle cible



À terme, le système doit pouvoir évoluer vers :



```text

&#x20;                      ┌───────────────────┐

&#x20;                      │     Navigation    │

&#x20;                      │  mission / route  │

&#x20;                      └─────────┬─────────┘

&#x20;                                │

&#x20;                                ▼

&#x20;                      ┌───────────────────┐

&#x20;                      │ Guidance /       │

&#x20;                      │ consignes        │

&#x20;                      └─────────┬─────────┘

&#x20;                                │

&#x20;                                ▼

┌──────────────┐       ┌───────────────────┐

│ Vision       │──────▶│                   │

├──────────────┤       │   Flight Control  │

│ IMU          │──────▶│                   │

├──────────────┤       └─────────┬─────────┘

│ Baromètre    │──────▶          │

├──────────────┤                  │

│ GNSS         │──────▶          ▼

└──────────────┘          ┌────────────────┐

&#x20;                         │ Actuators      │

&#x20;                         │ mixer / limits │

&#x20;                         └───────┬────────┘

&#x20;                                 ▼

&#x20;                        ┌─────────────────┐

&#x20;                        │     Avion       │

&#x20;                        └─────────────────┘

```



L'architecture actuelle constitue donc une première étape vers un système complet de \*\*perception → estimation → décision → commande\*\*.



\---



\# 16. Documentation spécialisée



Afin d'éviter que `ARCHITECTURE.md` devienne une copie de l'ensemble du projet, les détails doivent être répartis dans des Markdown spécialisés.



\## Documentation système



\### `README.md`



Présentation générale du projet :



\* philosophie ;

\* objectifs ;

\* esprit expérimental ;

\* organisation du dépôt ;

\* état général du projet.



\### `ARCHITECTURE.md`



Le présent document :



\* architecture globale ;

\* interfaces entre sous-systèmes ;

\* dépendances ;

\* vue système ;

\* orientation vers les documents spécialisés.



\---



\## Documentation matériel



\### `AIRFRAME.md`



À documenter :



\* géométrie ;

\* dimensions ;

\* masse ;

\* matériaux ;

\* surfaces de contrôle ;

\* centre de gravité ;

\* modifications de la cellule ;

\* comportement en vol observé.



\### `PROPULSION.md`



À documenter :



\* moteur ;

\* hélice ;

\* batterie ;

\* ESC ;

\* alimentation ;

\* consommation ;

\* performances ;

\* limites.



\---



\## Documentation électronique



\### `ELECTRONICS.md`



À documenter :



\* ESP32 ;

\* alimentation ;

\* câblage ;

\* GPIO ;

\* I²C ;

\* UART ;

\* SPI ;

\* alimentation des servos ;

\* masse ;

\* contraintes électriques.



\### `SENSORS.md`



Document commun ou index vers les capteurs :



\* BNO085 ;

\* BMP390 ;

\* NEO-N8M ;

\* système de vision ;

\* fréquences d'acquisition ;

\* unités ;

\* conventions ;

\* calibration.



Des fichiers indépendants peuvent ensuite être créés si nécessaire :



```text

BNO085.md

BMP390.md

GNSS.md

VISION.md

```



\---



\## Documentation communication



\### `LORA\_PROTOCOL.md`



À documenter :



\* SX1278 ;

\* paramètres radio ;

\* structure des paquets ;

\* encodage 6 bits ;

\* packing ;

\* valeur neutre ;

\* checksum éventuel ;

\* gestion des pertes ;

\* failsafe ;

\* compatibilité des versions.



\### `PS4\_CONTROL.md`



À documenter :



\* mapping des boutons ;

\* conversion des sticks ;

\* trims ;

\* modes ;

\* commandes manuelles ;

\* comportement en cas de perte de liaison.



\---



\## Documentation logiciel



\### `SOFTWARE\_ARCHITECTURE.md`



À documenter :



\* organisation des fichiers ;

\* modules ;

\* responsabilités ;

\* cycle principal ;

\* fréquence des tâches ;

\* dépendances ;

\* échanges entre modules.



\### `FLIGHT\_CONTROL.md`



À documenter :



\* définition des axes ;

\* roll ;

\* pitch ;

\* yaw ;

\* throttle ;

\* PID / contrôleurs ;

\* gains ;

\* mixage des élevons ;

\* saturation ;

\* limites ;

\* conventions de signe.



\### `STATE\_ESTIMATION.md`



À documenter :



\* attitude ;

\* quaternion ;

\* altitude ;

\* pression ;

\* GNSS ;

\* filtrage ;

\* calibration ;

\* fusion de capteurs ;

\* références et conventions.



\### `FLIGHT\_MODES.md`



À documenter :



\* SAFE ;

\* MANUAL ;

\* HOLD ;

\* futur mode autonome ;

\* transitions entre modes ;

\* priorités ;

\* conditions d'entrée/sortie ;

\* comportement en cas d'erreur.



\### `CONFIGURATION.md`



À documenter :



\* JSON ;

\* paramètres ;

\* types ;

\* unités ;

\* `binpt` ;

\* index ;

\* valeurs par défaut ;

\* validation ;

\* compatibilité des versions.



\### `LOGGING.md`



À documenter :



\* données enregistrées ;

\* fréquence ;

\* format ;

\* synchronisation temporelle ;

\* stockage ;

\* analyse post-vol.



\---



\## Documentation autonomie



\### `AUTONOMY.md`



À créer lorsque l'autonomie sera suffisamment avancée.



À documenter :



\* architecture perception → décision → commande ;

\* navigation ;

\* suivi de trajectoire ;

\* évitement ;

\* logique mission ;

\* supervision ;

\* interactions avec le pilote.



\### `VISION.md`



À documenter :



\* capteur caméra ;

\* traitement ;

\* algorithmes ;

\* format des observations ;

\* fréquence ;

\* latence ;

\* communication avec l'ESP32.



\### `NAVIGATION.md`



À documenter :



\* position ;

\* trajectoire ;

\* waypoint ;

\* altitude ;

\* vitesse ;

\* guidage ;

\* navigation GNSS / vision / inertielle.



\---



\## Documentation essais



\### `FLIGHT\_TESTS.md`



À documenter :



\* protocole d'essai ;

\* configuration utilisée ;

\* conditions météo ;

\* CG ;

\* paramètres ;

\* résultats ;

\* incidents ;

\* modifications décidées après chaque vol.



\### `KNOWN\_ISSUES.md`



À maintenir comme liste des problèmes connus :



\* problèmes de roll ;

\* comportement des élevons ;

\* limitations des capteurs ;

\* problèmes de communication ;

\* problèmes logiciels ;

\* améliorations envisagées.



\---



\# 17. Règle d'utilisation de la documentation par les IA



L'architecture documentaire doit permettre de sélectionner uniquement les informations nécessaires à une question.



Principe :



```text

Question utilisateur

&#x20;      │

&#x20;      ▼

ARCHITECTURE.md

&#x20;      │

&#x20;      ├── mécanique ───────▶ AIRFRAME.md

&#x20;      │

&#x20;      ├── propulsion ──────▶ PROPULSION.md

&#x20;      │

&#x20;      ├── câblage ─────────▶ ELECTRONICS.md

&#x20;      │

&#x20;      ├── capteur ─────────▶ SENSORS.md

&#x20;      │

&#x20;      ├── LoRa ────────────▶ LORA\_PROTOCOL.md

&#x20;      │

&#x20;      ├── PS4 ─────────────▶ PS4\_CONTROL.md

&#x20;      │

&#x20;      ├── commande ────────▶ FLIGHT\_CONTROL.md

&#x20;      │

&#x20;      ├── estimation ─────▶ STATE\_ESTIMATION.md

&#x20;      │

&#x20;      ├── modes ───────────▶ FLIGHT\_MODES.md

&#x20;      │

&#x20;      ├── configuration ──▶ CONFIGURATION.md

&#x20;      │

&#x20;      ├── logs ────────────▶ LOGGING.md

&#x20;      │

&#x20;      ├── vision ──────────▶ VISION.md

&#x20;      │

&#x20;      ├── navigation ─────▶ NAVIGATION.md

&#x20;      │

&#x20;      └── autonomie ──────▶ AUTONOMY.md

```



L'objectif est d'éviter de fournir systématiquement l'intégralité du dépôt à une IA.



`ARCHITECTURE.md` doit donc rester \*\*stable, synthétique et orienté interfaces\*\*, tandis que les documents spécialisés contiennent les détails susceptibles d'évoluer.



\---



\# 18. Principe de cohérence documentaire



Chaque information technique importante doit avoir idéalement \*\*une seule source de vérité\*\*.



Par exemple :



```text

Valeur neutre LoRa

&#x20;       │

&#x20;       └──▶ LORA\_PROTOCOL.md

```



et non une copie de cette valeur dans cinq documents différents.



De même :



```text

Limites servo

&#x20;       │

&#x20;       └──▶ CONFIGURATION.md

```



```text

Gains contrôleur

&#x20;       │

&#x20;       └──▶ FLIGHT\_CONTROL.md / CONFIGURATION.md

```



`ARCHITECTURE.md` ne doit conserver que l'information nécessaire pour comprendre \*\*où se trouve cette donnée et pourquoi elle est importante\*\*.



\---



\# 19. État de maturité



Le projet est développé progressivement par essais expérimentaux.



Les fonctions actuellement validées ou partiellement validées comprennent notamment :



\* pilotage manuel ;

\* commande de pitch ;

\* acquisition inertielle ;

\* acquisition barométrique ;

\* communication radio ;

\* transmission compacte de commandes ;

\* logging ;

\* configuration paramétrable ;

\* mécanisme de failsafe.



Le comportement du \*\*roll\*\* reste notamment un axe d'amélioration important identifié lors des essais en vol.



Les fonctions d'autonomie complète sont destinées à être développées progressivement au-dessus de cette infrastructure.



\---



\# 20. Résumé architectural



`navychop` peut être résumé comme une architecture embarquée distribuée autour d'un \*\*ESP32 central\*\* :



```text

&#x20;                   ┌───────────────────┐

&#x20;                   │      Pilote       │

&#x20;                   │      / mission    │

&#x20;                   └─────────┬─────────┘

&#x20;                             │

&#x20;                        LoRa / PS4

&#x20;                             │

&#x20;                             ▼

┌──────────────────────────────────────────────────────┐

│                        ESP32                         │

│                                                      │

│  ┌──────────┐   ┌──────────────┐   ┌─────────────┐ │

│  │ Sensors  │──▶│ State        │──▶│ Flight      │ │

│  │          │   │ estimation   │   │ control     │ │

│  └──────────┘   └──────────────┘   └──────┬──────┘ │

│                                           │         │

│  ┌──────────┐   ┌──────────────┐          │         │

│  │ Vision   │──▶│ Flight modes │──────────┤         │

│  └──────────┘   └──────────────┘          │         │

│                                           ▼         │

│                                    ┌─────────────┐  │

│                                    │ Mixer /     │  │

│                                    │ limits      │  │

│                                    └──────┬──────┘  │

│                                           │         │

│  ┌──────────┐   ┌──────────────┐          │         │

│  │ Logging  │◀──│ Telemetry    │          │         │

│  └──────────┘   └──────────────┘          ▼         │

└───────────────────────────────────────────┬──────────┘

&#x20;                                           │

&#x20;                             ┌─────────────┴─────────────┐

&#x20;                             ▼                           ▼

&#x20;                          Servos                      ESC/Motor

&#x20;                             │

&#x20;                             ▼

&#x20;                           Avion

```



L'architecture est volontairement \*\*modulaire, expérimentale et évolutive\*\* : la plateforme matérielle fournit les interfaces nécessaires à l'expérimentation d'algorithmes de perception, d'estimation, de commande et, à terme, d'autonomie complète.



