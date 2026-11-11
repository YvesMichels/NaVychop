# navychop

**Autonomous flight experimentation platform — RC aircraft / embedded navigation / AI-assisted development**

`navychop` est un projet **scientifique, technique et ludique** consacré à l'expérimentation autour des pilotes autonomes pour aéronefs.

L'objectif est de construire, avec des moyens essentiellement amateurs, une chaîne complète de navigation et de pilotage autonome : **capteurs → estimation d'état → contrôle → actionneurs → télémétrie**, tout en explorant les limites de ce qui peut être réalisé avec une architecture embarquée légère.

Le projet sert également d'expérimentation sur une autre question :

> **Jusqu'où un projet technique développé par une seule personne, assistée par des IA génératives, peut-il aller face aux méthodes et moyens traditionnellement disponibles dans l'industrie ?**

Il ne s'agit pas de reproduire une architecture avionique industrielle certifiée, mais de mesurer concrètement les capacités, les limites, les compromis et les difficultés rencontrées lorsqu'une approche individuelle s'appuie fortement sur les outils modernes d'IA.

---

## Objectifs

Le projet poursuit plusieurs objectifs complémentaires :

* expérimenter les techniques de **navigation inertielle et de pilotage automatique** ;
* développer une centrale de navigation embarquée autour d'un microcontrôleur ;
* exploiter et fusionner plusieurs capteurs : IMU, baromètre, magnétomètre, GPS et éventuellement vision ;
* développer des lois de commande adaptées à un aéronef léger ;
* expérimenter différentes architectures d'estimation et de contrôle ;
* étudier les contraintes de calcul, de mémoire, de latence et de robustesse d'un système embarqué ;
* développer une chaîne complète allant du matériel embarqué au poste sol ;
* réaliser des essais en conditions réelles sur un aéronef radiocommandé ;
* utiliser les IA génératives comme **assistant de développement et de recherche**, et mesurer concrètement leur apport.

Une attention particulière est portée à la distinction entre :

1. ce qui est **théoriquement possible** ;
2. ce qui est **réalisable avec le matériel disponible** ;
3. ce qui fonctionne réellement **en simulation ou sur banc** ;
4. ce qui fonctionne **en vol**.

---

## Philosophie du projet

`navychop` est volontairement développé comme un projet personnel à vocation expérimentale.

Le but n'est pas de produire immédiatement un système industriel, certifiable ou commercialisable. Le but est au contraire de pouvoir tester rapidement des idées, constater leurs limites et conserver les résultats des expériences.

Le projet cherche notamment à répondre expérimentalement à des questions telles que :

* Quelle complexité de pilote automatique peut être embarquée sur une architecture très contrainte ?
* Quelle précision peut-on obtenir avec des capteurs grand public ?
* Jusqu'où peut-on pousser une architecture développée par une seule personne ?
* Quelles parties d'un système avionique nécessitent réellement une expertise spécialisée ?
* Quelles tâches peuvent être efficacement déléguées à une IA générative ?
* Où l'IA générative atteint-elle ses limites lorsqu'elle doit raisonner sur un système physique réel ?
* Dans quelle mesure une personne assistée par IA peut-elle atteindre une productivité comparable à celle d'une petite équipe technique sur certaines tâches ?

L'IA est donc considérée comme un **outil d'ingénierie**, et non comme une autorité technique.

Les hypothèses proposées par les modèles sont vérifiées autant que possible par le calcul, la simulation, les mesures sur banc et les essais en vol.

---

# Architecture

Le projet est organisé autour de plusieurs éléments :

```text
                    ┌──────────────────────┐
                    │     Poste sol        │
                    │                      │
                    │ Configuration        │
                    │ Commandes            │
                    │ Télémétrie           │
                    │ Enregistrement        │
                    └──────────┬───────────┘
                               │
                             LoRa
                               │
                    ┌──────────▼───────────┐
                    │ Centrale de navigation│
                    │                       │
                    │ ESP32                 │
                    │                       │
                    │ ┌───────────────────┐ │
                    │ │ Estimation d'état │ │
                    │ └───────────────────┘ │
                    │          │            │
                    │ ┌────────▼───────────┐│
                    │ │ Contrôle de vol    ││
                    │ └────────┬───────────┘│
                    │          │            │
                    │      Actionneurs      │
                    │          │            │
                    │      Capteurs         │
                    └───────────────────────┘
```

La centrale de navigation est basée sur un **ESP32** et communique avec différents capteurs et périphériques.

L'architecture est conçue pour pouvoir faire évoluer séparément :

* l'acquisition des capteurs ;
* l'estimation d'attitude et d'état ;
* la navigation ;
* les lois de commande ;
* la communication ;
* l'enregistrement des données ;
* le poste sol.

---

# Matériel

Le système est actuellement validé sur un aéronef radiocommandé expérimental.

L'avion est volontairement réalisé avec des matériaux très simples, principalement des **plaques de polystyrène fines (~2 mm)**.

Ordres de grandeur :

| Paramètre                            |                               Valeur |
| ------------------------------------ | -----------------------------------: |
| Envergure                            |                               ~1,4 m |
| Longueur                             |                                 ~1 m |
| Masse à vide / configuration de base |                                ~1 kg |
| Structure                            |                      Polystyrène fin |
| Motorisation                         |                           Électrique |
| Pilotage                             | Elevons / commandes conventionnelles |
| Centrale de navigation               |                                ESP32 |

Ces caractéristiques ne constituent pas une spécification figée : l'aéronef et son électronique sont susceptibles d'évoluer au cours des expérimentations.

---

# Centrale de navigation

La centrale de navigation constitue une partie importante du projet.

Elle regroupe notamment :

* microcontrôleur **ESP32** ;
* IMU ;
* baromètre ;
* magnétomètre ;
* GPS ;
* liaison radio longue portée 433MHz ;
* interface avec un système de vision ;
* sorties de commande des servomoteurs ;
* enregistrement des données de vol.

Le brochage, les paramètres et les interfaces peuvent évoluer avec les versions matérielles.

Les informations relatives au câblage et aux versions de la carte doivent donc être considérées comme **versionnées avec le matériel correspondant**.

---

# Bibliothèques embarquées

Les bibliothèques présentes dans ce dépôt sont principalement des composants **spécifiques à la centrale de navigation développée dans le cadre de `navychop`**.

Elles ne doivent donc pas nécessairement être considérées comme des bibliothèques génériques ou autonomes.

Elles peuvent dépendre :

* du brochage de la carte ;
* des périphériques effectivement présents ;
* de la structure des données du projet ;
* du protocole de communication interne ;
* du format des données de télémétrie ;
* des contraintes de calcul et de mémoire de l'ESP32.

Lorsqu'une bibliothèque tierce est utilisée, elle doit autant que possible être clairement identifiée comme telle, avec sa version et sa licence.

Les bibliothèques développées spécifiquement pour `navychop` sont distinguées des dépendances externes afin de faciliter leur réutilisation.

---

# Développement assisté par IA

Une caractéristique importante du projet est l'utilisation intensive d'**IA génératives comme assistants de développement**.

Elles peuvent notamment être utilisées pour :

* proposer des architectures logicielles ;
* générer ou modifier du code ;
* expliquer des algorithmes ;
* rechercher des erreurs ;
* analyser des résultats expérimentaux ;
* produire de la documentation ;
* challenger certaines hypothèses techniques ;
* générer des outils de test.

Cependant, le code généré par une IA n'est pas considéré comme automatiquement correct.

Pour les fonctions ayant un impact sur le comportement de l'aéronef, les propositions sont confrontées autant que possible à :

* la documentation des composants ;
* les équations et modèles physiques ;
* des tests unitaires ;
* des simulations ;
* des essais sur banc ;
* des mesures instrumentées ;
* des essais en vol progressifs.

Le projet constitue ainsi également une expérimentation sur le **développement logiciel humain + IA dans un système cyber-physique réel**.

---

# Expérimentation et validation

Le développement suit autant que possible une progression :

```text
Théorie
   ↓
Simulation / calcul
   ↓
Test logiciel
   ↓
Test sur banc
   ↓
Test moteur arrêté
   ↓
Essai en conditions contrôlées
   ↓
Essai en vol
   ↓
Analyse des données
   ↓
Modification
```

Les résultats d'un essai réel peuvent donc conduire à remettre en question une hypothèse précédemment considérée comme correcte.

Une fonctionnalité présente dans le code ne doit pas être considérée comme **validée en vol** tant qu'un essai approprié ne l'a pas démontré.

---

# Données de vol

Les données enregistrées pendant les expérimentations sont utilisées pour :

* analyser le comportement de l'aéronef ;
* caractériser les capteurs ;
* analyser les boucles de contrôle ;
* identifier les instabilités ;
* comparer différentes configurations ;
* reproduire certains problèmes ;
* mesurer les performances du système.

Lorsque cela est pertinent, les expérimentations peuvent être accompagnées de leur configuration matérielle et logicielle afin de permettre leur reproduction.

---

# État du projet

Le projet est **expérimental et en développement actif**.

Les interfaces, protocoles, paramètres, algorithmes et structures de données peuvent changer sans garantie de compatibilité avec les versions précédentes.

Les fonctionnalités peuvent avoir différents niveaux de maturité :

* **Prototype** — implémentation initiale, non validée ;
* **Bench tested** — testé sur banc ;
* **Flight tested** — testé en vol ;
* **Experimental** — fonction opérationnelle mais encore en cours d'étude.

Une fonctionnalité fonctionnant sur l'aéronef ne doit pas pour autant être considérée comme adaptée à une utilisation critique ou à un autre aéronef sans nouvelle validation.

---

# Sécurité

`navychop` est un projet expérimental concernant un aéronef radiocommandé.

**Le logiciel et le matériel fournis dans ce dépôt ne sont pas des systèmes avioniques certifiés.**

Toute utilisation sur un aéronef doit être précédée de validations adaptées au matériel, à l'environnement et au niveau de risque considéré.

Les essais doivent notamment prendre en compte :

* la perte de liaison radio ;
* les erreurs de capteurs ;
* les erreurs d'estimation ;
* les saturations des commandes ;
* les défauts logiciels ;
* les pertes d'alimentation ;
* les comportements inattendus du pilote automatique ;
* la possibilité de reprendre le contrôle manuellement.

Le système doit toujours disposer d'un moyen approprié de récupération ou d'interruption du contrôle automatique lors des essais.

**Ne pas utiliser ce projet comme base directe pour le contrôle d'un aéronef habité ou d'un système dont la défaillance pourrait mettre des personnes en danger.**

**Tout vol doit être effectué dans des conditions autorisée par la loi en vigueur.**

---

# Reproductibilité

L'objectif est de documenter suffisamment le projet pour qu'une autre personne puisse reproduire une partie significative des expérimentations.

Lorsque cela est pertinent, les commits, versions matérielles, paramètres et résultats d'essais doivent être associés aux expérimentations correspondantes.

Les changements susceptibles de modifier le comportement en vol doivent idéalement être identifiables dans l'historique Git.

---

# Git et GitLab

Le dépôt utilise Git afin de conserver l'historique du développement et GitLab pour l'hébergement du projet.

Les contributions doivent autant que possible :

* utiliser des commits atomiques ;
* employer des messages de commit explicites ;
* éviter de mélanger refactoring, nouvelle fonctionnalité et correction expérimentale dans un même commit ;
* documenter les changements ayant un impact sur le comportement de l'aéronef ;
* conserver les configurations nécessaires à la reproduction d'une expérience ;
* utiliser les **Issues** pour les bugs, idées et problèmes identifiés ;
* utiliser les **Merge Requests** lorsque plusieurs branches ou contributeurs sont impliqués ;
* utiliser les **Tags/Releases** pour identifier les versions importantes du logiciel ou du matériel.

Pour les modifications importantes, une Issue peut servir à documenter :

```text
Problème
↓
Hypothèse
↓
Modification
↓
Test
↓
Résultat
↓
Conclusion
```

Cette approche permet de conserver une trace expérimentale plutôt qu'un simple historique de code.

---

# Dépendances

Les dépendances externes doivent être identifiées explicitement lorsqu'elles sont nécessaires à la compilation ou au fonctionnement du projet.

Pour chaque dépendance importante, il est recommandé de conserver :

* nom ;
* version ;
* source officielle ;
* licence ;
* raison de son utilisation.

Les bibliothèques développées spécifiquement pour `navychop` sont séparées conceptuellement des dépendances tierces.

---

# Licence

| Contenu | Licence |
|---|---|
| Code (`nav/`, `vision/`, `peripherals/`, `common/`, `python/`, `matlab/`) | MIT (`LICENSE.md`) |
| Matériel (`electronics/`, `mechanical/`) | CERN-OHL-P v2 (`LICENSE-HARDWARE.md`) |
| Documentation (`documentation/`) | CC BY 4.0 (`LICENSE-DOCS.md`) |

Voir aussi [DISCLAIMER.md](DISCLAIMER.md) : avertissement sur la sécurité et la réglementation.

---

# Contribution

Les contributions, corrections et discussions techniques sont les bienvenues.

Avant de proposer une modification importante, il est recommandé d'ouvrir une Issue afin de discuter de l'approche envisagée.

Les contributions doivent privilégier :

* la reproductibilité ;
* la simplicité ;
* la documentation ;
* les tests ;
* l'explicitation des hypothèses ;
* la séparation entre fonctionnalités expérimentales et fonctionnalités validées.

---

# Pourquoi « navychop » ?

`navychop` est le nom du projet et de son environnement logiciel. Ce nom provien de l'association du pseudo Vychop et 'nav' pour navigation. 

Le nom n'a pas vocation à constituer une marque commerciale ou une désignation aéronautique officielle.

---

## Avertissement

Ce dépôt documente une expérimentation personnelle.

Les résultats présentés sont dépendants du matériel, du logiciel, de la configuration de l'aéronef et des conditions expérimentales.

**Un résultat obtenu dans le cadre de `navychop` ne constitue pas une garantie de comportement dans une autre configuration.**

---

## Auteur

Projet développé individuellement par Vychop (Yves MICHELS) avec l'assistance d'outils d'IA générative.

Le projet cherche notamment à documenter ce qu'une personne seule peut réaliser en combinant :

* connaissances scientifiques et techniques ;
* prototypage électronique ;
* développement logiciel ;
* expérimentation physique ;
* outils open source ;
* IA génératives.

L'objectif n'est pas de démontrer que l'IA remplace l'ingénierie, mais de **mesurer expérimentalement ce qu'elle permet de réaliser lorsqu'elle est utilisée comme outil par un ingénieur.**
