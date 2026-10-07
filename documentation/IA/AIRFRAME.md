# AIRFRAME — Cellule et architecture mécanique

> Documentation de référence de la cellule de `navychop`.
>
> Ce document décrit la géométrie, la structure mécanique, les surfaces de contrôle, le centrage et les caractéristiques physiques de l'aéronef.
> Les paramètres électroniques, logiciels et algorithmiques sont documentés dans les fichiers spécialisés correspondants.

---

## 1. Vue générale

`navychop` est un aéronef électrique à voilure fixe de construction amateur, utilisant une cellule légère principalement réalisée en polystyrène renforcé par des éléments en carbone.

La cellule est constituée :

* d'une aile principale rectangulaire ;
* de deux élevons occupant la majeure partie du bord de fuite ;
* d'un fuselage central ;
* d'un empennage arrière ;
* d'une gouverne de profondeur horizontale ;
* de deux gouvernes de direction verticales ;
* d'une motorisation électrique placée à l'avant ;
* d'une pointe avant imprimée en PLA intégrant le support moteur et le logement de la batterie et de l'ESC.

L'architecture mécanique privilégie la simplicité, la légèreté, la réparabilité et la résistance aux expérimentations et aux crashs.

---

# 2. Caractéristiques générales

| Paramètre                   |           Valeur |
| --------------------------- | ---------------: |
| Envergure totale            |          ~1,37 m |
| Longueur totale             |          ~0,98 m |
| Surface alaire              |        ~40,6 dm² |
| Corde aile avec gouverne    |          29,5 cm |
| Corde partie fixe           |            26 cm |
| Flèche                      |               0° |
| Dièdre                      |               0° |
| Profil                      |     Plan-convexe |
| Matériau principal          | Polystyrène 3 mm |
| Renfort principal           |          Carbone |
| Masse ailes                 |            370 g |
| Masse fuselage              |            423 g |
| Masse centrale navigation   |             63 g |
| Masse antenne GPS céramique |             13 g |
| Masse batterie              |            142 g |
| Masse équilibrage           |             67 g |

### Masse

La somme des masses détaillées ci-dessus est d'environ :

**1078 g**

Elle est cohérente avec la masse en ordre de vol d'environ **1,09 kg** précédemment mesurée.

La masse exacte en vol dépend notamment de la configuration de navigation et de la présence de l'antenne GPS.

---

# 3. Système de coordonnées

Le repère avion utilisé par le projet est :

```text
              X
              ↑
              │
              │ direction du vol
              │
        ┌─────┴─────┐
        │           │
        │   AVION   │
        └───────────┘
             Y →
```

Convention complète :

```text
X : avant
Y : gauche
Z : haut
```

Cette convention est également utilisée par les traitements logiciels et doit être conservée lors de toute documentation de géométrie ou de commande.

---

# 4. Aile principale

## 4. Aile principale

### 4.1 Géométrie générale

L'aile est constituée de deux demi-ailes symétriques séparées par le fuselage.

Chaque demi-aile mesure :

* longueur : **65 cm** ;
* corde totale : **29,5 cm**, gouverne comprise ;
* corde de la partie fixe : **26 cm** ;
* flèche : **0°** ;
* dièdre : **0°**.

Le fuselage crée une interruption centrale de **6,5 cm** permettant le passage des deux barres carbone à travers les fentes du fuselage.

L'envergure totale nominale est donc :

```text
65 cm + 6,5 cm + 65 cm = 136,5 cm
```

soit environ **1,365 m**.

La surface projetée correspondant à cette géométrie est approximativement :

```text
1,365 × 0,295 = 0,402 m²
```

soit environ **40,2 dm²**.

Cette valeur est cohérente avec la surface alaire d'environ **40 dm²** retenue pour le projet.

### 4.2 Partie fixe

Vue du dessus, chaque demi-aile possède une partie fixe approximativement rectangulaire de :

```text
65 cm × 26 cm
```

La différence entre les 29,5 cm de corde totale et les 26 cm de partie fixe correspond à la zone occupée par l'élevon.

### 4.3 Élevons

Chaque demi-aile possède un élevon occupant la totalité de la longueur de la demi-aile.

La géométrie nominale est approximativement :

* largeur : **3,5 cm** ;
* longueur à la base : **65 cm** ;
* longueur à l'extrémité : **~58 cm** ;
* forme : trapèze ;
* extrémités : angles d'environ **45°**.

Schématiquement :

```text
                 ~58 cm
             ┌────────────┐
            /              \
           /                \
          /                  \
         └────────────────────┘
                65 cm
                ↑
             base

             largeur ≈ 3,5 cm
```

Pour les calculs et les documents de référence, la valeur nominale à retenir est donc **~58 cm** à l'extrémité.

### 4.4 Surface alaire de référence

Pour les calculs de premier ordre, la surface alaire totale peut être considérée comme :

**~40 dm²**.

La valeur géométrique obtenue à partir de l'envergure et de la corde totale est d'environ **40,2 dm²**.


# 5. Profil aérodynamique

L’aile est constituée de deux plaques de polystyrène de 3 mm d’épaisseur, dont la géométrie et la courbure sont maintenues par deux renforts longitudinaux en carbone : un tube creux de 12 mm de diamètre et une tige pleine de 5 mm de diamètre.

Ces deux éléments en carbone définissent la forme du profil de l’aile. Le longeron principal de 12 mm est en outre renforcé par un empilement de quatre plaques de polystyrène, collé directement sur celui-ci sur une largeur d’environ 1 cm et prolongé sur l’ensemble de la demi-aile (65 cm). Cette structure forme ainsi un profil plan-convexe léger, dont la géométrie est principalement donnée par la position des renforts en carbone et l’empilement de polystyrène.

Caractéristiques mesurées :

* épaisseur maximale : environ **2 cm**, située à ~6 cm du bord d'attaque ;
* épaisseur proche du bord de fuite : environ **3 mm** avant l'élevon ;
* courbure du bord / raccord avant : rayon d'environ **5 mm**.

La forme du profil est partiellement assurée par la structure en carbone, notamment par la barre arrière de 5 mm.

Le profil n'est pas issu d'un profil aérodynamique normalisé identifié ; il s'agit d'une géométrie de construction expérimentale défini par le procédé de construction.

---

# 6. Élevons

Chaque demi-aile possède un élevon sur la majeure partie de son bord de fuite.

Caractéristiques :

* largeur : **3,5 cm** ;
* longueur : environ **57 cm** sur le bord arrière ;
* forme : trapézoïdale ;
* extrémités avec angles d'environ **45°**.

Les deux élevons constituent les principales surfaces de commande en roulis.

Ils interviennent également dans la commande de tangage selon le mixage logiciel.

La commande et le mixage des élevons sont documentés dans :

`FLIGHT_CONTROL.md`

---

# 7. Renfort et fixation de l'aile

Chaque aile est renforcée et fixée au fuselage par deux éléments en carbone continus entre les deux demi-ailes.

## 7.1 Renfort avant

* type : tube carbone creux ;
* diamètre : **12 mm** ;
* longueur : **50 cm** ;
* position : environ **7 cm de la corde**.

Cette barre constitue également le principal élément mécanique de fixation de l'aile au fuselage.

## 7.2 Renfort arrière

* type : tige carbone pleine ;
* diamètre : **5 mm** ;
* longueur : **50 cm** ;
* position : environ **19 cm de la corde**.

Cette tige joue deux rôles :

1. renforcement mécanique ;
2. contribution au maintien de la forme du profil plan-convexe.

Les deux éléments traversent le fuselage et assurent la continuité mécanique entre les deux demi-ailes.

---

# 8. Fixation de l'aile au fuselage

L'aile est démontable afin de faciliter le transport.

Les deux éléments carbone sont insérés dans des fentes de dimensions adaptées dans le fuselage.

Le maintien est réalisé par du **scratch/Velcro** enroulé au-dessus des barres.

La résistance annoncée du système de fixation est supérieure à :

**10 kg**

Cette architecture permet de combiner :

* démontabilité ;
* simplicité ;
* absence de visserie lourde ;
* résistance suffisante en utilisation expérimentale.

### Retour d'expérience

Plusieurs crashs à pleine vitesse ont été réalisés sans défaillance structurelle significative au niveau de la fixation de l'aile.

La structure présente donc jusqu'à présent une résistance mécanique jugée satisfaisante pour l'utilisation expérimentale prévue.

---

# 9. Incidence / position de l'aile

L'aile est installée avec un angle mesuré d'environ :

**2,5°**

pour le plat inférieur de l'aile.

Le point haut de l'aile est quasiment tangent à la partie supérieure du fuselage.

La barre carbone principale de 12 mm est située à environ **25 cm du moteur**.

---

# 10. Fuselage

## 10.1 Dimensions principales

Le fuselage présente une section approximativement carrée :

* largeur : **6,5 cm** ;
* hauteur : **6,5 cm**.

La longueur totale est d'environ **90 cm**.

## 10.2 Géométrie longitudinale

La partie supérieure du fuselage est approximativement alignée sur les **80 premiers centimètres**.

La partie inférieure est alignée sur environ **40 cm**, puis s'incline sur environ **30 cm** vers l'arrière afin de former une section arrière d'environ :

**4 × 4 cm**

Cette géométrie donne au fuselage une forme légèrement pyramidale vers la queue.

Une pyramide finale sous la queue assure une forme en pointe à l'arrière. 

Elle permet notamment :

* de réduire progressivement la section arrière ;
* de créer le support des servomoteurs arrière ;
* d'intégrer les mécanismes de commande de profondeur et de direction.

---

# 11. Pointe avant

La partie avant est constituée d'une pièce imprimée en **PLA**.

Elle assure plusieurs fonctions :

* arrondi aérodynamique du nez ;
* support du moteur ;
* logement de la batterie ;
* logement de l'ESC ;
* interface mécanique avec le fuselage.

Le nez présente un arrondi d'environ :

**5 cm**

La motorisation est alignée avec l'axe longitudinal du fuselage.

---

# 12. Batterie et équilibrage

La batterie est positionnée dans la pointe avant imprimée en PLA.

Elle repose sur une plaque vissée constituée d'une matière de type carte électronique perforée au pas de :

**2,54 mm**

Cette solution permet une fixation modulable et potentiellement un ajustement longitudinal de la position de la batterie et son refroidissement.

La batterie participe donc directement au réglage du centre de gravité.

Une masse d'équilibrage d'environ **67 g** est actuellement présente.

---

# 13. Empennage

L'empennage arrière est aligné avec la partie supérieure du fuselage.

Il présente une géométrie trapézoïdale.

Dimensions approximatives :

* largeur : **40 cm** ;
* profondeur au bord : **12 cm** ;
* profondeur au centre : **15 cm** ;
* épaisseur : **3 mm**.

La géométrie exacte du bord d'attaque et du bord de fuite devra être précisée si une modélisation aérodynamique détaillée est réalisée.

---

# 14. Gouverne de profondeur

Une gouverne horizontale est située à l'arrière de l'empennage.

Dimensions :

**40 × 5 cm**

Elle constitue la principale surface de commande du tangage.

La commande est réalisée par :

* servo moteur ~9 g ;
* tige métallique de liaison.

Le comportement en tangage observé lors des premiers essais est actuellement très réactif.

---

# 15. Gouvernes de direction

Deux gouvernes verticales sont installées au-dessus de la queue.

Dimensions approximatives de chaque gouverne :

**6 × 7,5 cm**

Configuration :

* deux gouvernes parallèles ;
* espacement à leur base : **5 cm**.

Elles sont commandées par un mécanisme de direction dédié.

La présence de deux surfaces permet de conserver une configuration symétrique autour du plan formé par les axes longitudinal et vertical.

---

# 16. Commandes mécaniques

Les servomoteurs utilisés pour les commandes sont de l'ordre de :

**9 g**

Les liaisons avec les gouvernes sont réalisées par des **tiges métalliques**.

Les surfaces de commande sont :

| Axe      | Surface                                        |
| -------- | ---------------------------------------------- |
| Roll     | 2 élevons                                      |
| Pitch    | gouverne horizontale arrière + mixage éventuel |
| Yaw      | 2 gouvernes verticales                         |
| Throttle | moteur électrique                              |

Les paramètres de débattement et le mixage électronique ne sont pas définis dans ce document.

Référence :

`FLIGHT_CONTROL.md`

---

# 17. Centre de gravité

Le centre de gravité est un paramètre critique de la cellule.

## Configuration actuelle

Le CG est actuellement visé dans une plage d'environ :

**25–30 % de corde**

La position exacte varie en fonction de :

* la position de la batterie ;
* la configuration de la centrale de navigation ;
* la présence ou non de l'antenne GPS ;
* la masse d'équilibrage.

## Configuration initiale

Les premiers essais ont été réalisés avec un CG approximativement situé autour de :

**35–40 % de corde**

Cette position s'est révélée trop arrière et a été corrigée rapidement.

Le déplacement vers l'avant a amélioré la stabilité longitudinale.

---

# 18. Répartition des masses

Répartition connue :

| Sous-ensemble                    |      Masse |
| -------------------------------- | ---------: |
| Ailes                            |      370 g |
| Fuselage                         |      423 g |
| Centrale de navigation, hors GPS |       63 g |
| Antenne GPS céramique            |       13 g |
| Batterie                         |      142 g |
| Équilibrage                      |       67 g |
| **Total**                        | **1078 g** |

Cette répartition devra être mise à jour si la configuration matérielle évolue.

Le CG ne doit pas être déduit uniquement de cette table : les positions longitudinales des masses doivent également être prises en compte.

---

# 19. Construction

## Matériau principal

La cellule est principalement réalisée en :

**polystyrène de 3 mm**

Le matériau est utilisé sous forme de plaques découpées et assemblées pour constituer :

* l'aile ;
* le fuselage ;
* l'empennage ;
* les surfaces de contrôle.

## Renforts

Les ailes utilisent deux renforts longitudinaux en carbone :

```text
Avant
 │
 │   Ø12 mm carbone creux
 │
 ├───────────────────────────
 │
 │   Ø5 mm carbone plein
 │
Arrière
```

Les deux barres font environ 50 cm de long et apportent chacune environ 20 cm de portée mécanique de part et d'autre du fuselage.

La barre arrière contribue également à la géométrie du profil.

## Assemblage

Les principaux moyens d'assemblage sont :

* colle à chaud ;
* ruban adhésif ;
* éléments carbone ;
* pièces imprimées en PLA ;
* Velcro pour la fixation démontable de l'aile ;
* visserie pour certains éléments internes.

---

# 20. Résistance mécanique et réparabilité

La cellule a été conçue dans une logique expérimentale privilégiant la simplicité de réparation.

Le retour d'expérience disponible indique :

> Plusieurs crashs à pleine vitesse n'ont pas provoqué de défaillance de la fixation principale des ailes.

La cellule doit donc être considérée comme suffisamment robuste pour permettre une phase d'expérimentation en vol comportant des atterrissages ou crashs potentiellement destructifs.

Cette propriété est importante dans le contexte du projet : la cellule constitue une plateforme d'expérimentation et non une cellule optimisée uniquement pour la performance aérodynamique.

---

# 21. Retours d'essais en vol

Les observations suivantes doivent être distinguées des caractéristiques géométriques.

### Premier retour majeur

Lors d'un premier essai en vol :

* le **pitch s'est révélé très réactif** ;
* la stabilisation longitudinale était rapide ;
* le **roll était en revanche faiblement efficace** malgré des élevons relativement larges.

Les élevons ont une largeur d'environ 3,5 cm et peuvent présenter des débattements importants.

Le problème de roll n'est donc pas attribué a priori à une seule cause.

Les hypothèses peuvent notamment concerner :

* efficacité aérodynamique ;
* moment de commande ;
* différentiel ;
* sens de mixage ;
* transmission mécanique ;
* flexion ;
* jeu des commandes ;
* interaction avec la géométrie de la cellule.

Ces hypothèses doivent être testées expérimentalement.

Référence :

`FLIGHT_CONTROL.md`

et, pour les résultats d'essais :

`FLIGHT_TESTS.md`

---

# 22. Paramètres mécaniques à surveiller

Les paramètres suivants sont particulièrement importants pour les futurs essais :

* position du CG ;
* masse totale ;
* position longitudinale de la batterie ;
* masse et position de la centrale de navigation ;
* présence de l'antenne GPS ;
* débattement des gouvernes ;
* rigidité des élevons ;
* jeu des tiges de commande ;
* rigidité des fixations carbone ;
* incidence de l'aile ;
* intégrité du profil plan-convexe.

Toute modification importante de ces paramètres doit idéalement être associée à un nouvel enregistrement dans `FLIGHT_TESTS.md`.

---

# 23. Paramètres encore à préciser

Les valeurs suivantes ne sont pas encore suffisamment documentées pour être considérées comme des données de référence :

* profil exact ou géométrie complète de la section d'aile ;
* position exacte de la charnière des élevons ;
* débattement mécanique réel des élevons ;
* dimensions exactes de la gouverne de profondeur au niveau de sa charnière ;
* géométrie détaillée des deux gouvernes de direction ;
* position longitudinale exacte du CG en mm ;
* position longitudinale exacte des différents éléments lourds ;
* dimensions détaillées du fuselage au niveau du nez et de la queue ;
* géométrie exacte de l'empennage ;
* modèle exact des servomoteurs ;
* géométrie précise des palonniers et guignols.

Ces paramètres ne doivent pas être inventés dans les documents dérivés.

---

# 24. Relations avec les autres documents

| Sujet                 | Document de référence |
| --------------------- | --------------------- |
| Architecture générale | `ARCHITECTURE.md`     |
| Propulsion            | `PROPULSION.md`       |
| Électronique          | `ELECTRONICS.md`      |
| Capteurs              | `SENSORS.md`          |
| Commande de vol       | `FLIGHT_CONTROL.md`   |
| Modes de vol          | `FLIGHT_MODES.md`     |
| Protocole radio       | `LORA_PROTOCOL.md`    |
| Configuration         | `CONFIGURATION.md`    |
| Journal des essais    | `FLIGHT_TESTS.md`     |
| Problèmes connus      | `KNOWN_ISSUES.md`     |

---

# 25. Résumé mécanique

La cellule de `navychop` est un avion expérimental léger d'environ **1,1 kg**, construit principalement en polystyrène de 3 mm et renforcé par deux éléments carbone.

L'aile rectangulaire d'environ **1,37 m × 29,5 cm**, sans flèche ni dièdre, utilise un profil plan-convexe artisanal et deux élevons occupant la majeure partie du bord de fuite.

L'aile est démontable et fixée au fuselage par deux barres carbone de 12 mm et 5 mm maintenues par Velcro. La structure a jusqu'à présent résisté à plusieurs crashs à pleine vitesse.

Le fuselage de section principale 6,5 × 6,5 cm intègre une pointe avant en PLA portant le moteur et la batterie. L'arrière se réduit progressivement afin d'intégrer les mécanismes de commande de l'empennage.

Le centrage actuel est recherché autour de **25–30 % de corde**, après une première configuration à **35–40 %** qui s'est révélée trop arrière.

La cellule constitue avant tout une **plateforme expérimentale robuste, légère, démontable et facilement réparable**, destinée à permettre l'évolution progressive du système de pilotage autonome.
