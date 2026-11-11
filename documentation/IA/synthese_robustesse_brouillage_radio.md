# Étude exploratoire — Robustesse des communications d’un avion autonome face au brouillage

**Date :** 10 octobre 2026  
**Contexte :** prototype d’avion RC/autonome.  
**Objet :** conserver une synthèse des risques de brouillage, des architectures de communication et des pistes de validation théorique et expérimentale.

> \*\*Statut du document :\*\* note exploratoire, pas une spécification de conception ni une garantie de résistance à un brouilleur donné. Les puissances et portées publiées varient fortement selon les appareils et les conditions. Les ordres de grandeur ci-dessous servent à classer les risques ; ils ne permettent pas de calculer une portée de brouillage universelle.

## 1\. Contexte et objectifs de communication

Deux liaisons distinctes doivent être étudiées, car leurs exigences et leurs conséquences en cas de perte ne sont pas les mêmes.

### 1.1 Avion → sol : images / vidéo

* Transfert périodique d’une image d’environ **20 kB en 1 à 2 s**.
* Débit utile correspondant : **environ 80 à 160 kbit/s**, avant ajout des en-têtes, du contrôle d’intégrité, du codage correcteur, des retransmissions et des marges.
* Liaison radio envisagée en **2,4 GHz**, avec antenne directive embarquée orientée vers une station au sol et antenne directive au sol orientée vers l’avion.
* La station au sol est prévue à environ **1 km de la cible observée**.

Le critère ne doit pas être uniquement le débit PHY annoncé : il faut caractériser le débit utile, le délai réel de livraison, les pertes d’images et l’âge des données à leur réception.

### 1.2 Sol → avion : commandes

* Liaison à faible débit, actuellement envisagée via **LoRa**.
* Le volume de données est faible, mais la fraîcheur et la continuité des commandes sont importantes.
* La perte de la liaison doit être traitée comme un événement attendu : détection, invalidation des commandes périmées et transition vers un mode de vol sûr.

## 2\. Classement qualitatif des risques de brouillage

Les catégories ci-dessous sont des niveaux de risque, pas des probabilités chiffrées. La puissance annoncée seule ne suffit pas : l’effet dépend du niveau d’interférence reçu dans la bande utile, de la largeur spectrale, des antennes, de la géométrie, des pertes de propagation et du comportement du récepteur.

|Niveau|Scénario|Ordres de grandeur / caractéristiques publiques|Effet possible|Implication pour le projet|
|-|-|-|-|-|
|**R0 — Coexistence radio ordinaire**|Wi-Fi, télécommandes et autres émetteurs légitimes partageant la bande|Équipements grand public : puissances typiquement de l’ordre du mW à quelques centaines de mW, selon l’équipement et la réglementation applicable|Collisions, baisse de débit, latence variable et pertes de paquets|À intégrer au dimensionnement nominal et aux essais de coexistence.|
|**R1 — Interférence accidentelle ou parasite**|Émetteur défectueux, émission hors bande ou source radio proche du récepteur|Pas de plage universelle ; la proximité et le niveau reçu sont déterminants|Dégradation du récepteur, erreurs de paquets ou désensibilisation|Scénario plausible, à étudier par simulation et essais de banc contrôlés.|
|**R2 — Brouilleur pirate local, large bande ou multibande**|Appareil illégal visant plusieurs services radio|Les annonces commerciales peuvent aller de puissances modestes à des puissances revendiquées de l’ordre du watt ou davantage ; les fiches sont souvent difficiles à vérifier indépendamment|Perturbation d’un ou plusieurs liens si le niveau reçu est suffisant et recouvre la bande utile|Menace plausible, mais impossible à quantifier correctement à partir d’une seule annonce commerciale.|
|**R3 — Brouillage ciblé en fréquence**|Énergie concentrée dans le canal ou la bande du récepteur visé|À puissance totale égale, la concentration spectrale peut augmenter la densité de puissance dans la bande utile par rapport à une émission large bande. Il n’existe pas de seuil universel.|Dégradation ou perte de décodage du signal utile|À considérer explicitement pour le canal de commande.|
|**R4 — Brouillage directionnel et ciblé**|Émission orientée vers l’aéronef ou la station réceptrice|Les études publiques de systèmes anti-UAS caractérisent séparément puissance RF, densité spectrale, gain et largeur de faisceau. Les systèmes sont très hétérogènes.|Augmentation de l’interférence dans la direction visée, avec des effets hors axe dépendant des lobes secondaires et de la géométrie|Scénario conservateur pour l’analyse de perte de liaison, sans supposer qu’un système précis sera rencontré.|
|**R5 — Système anti-drone institutionnel ou militaire**|Équipement portatif, embarqué sur véhicule ou fixe, multi-bandes et doté d’antennes adaptées|Les sources ouvertes décrivent des systèmes de catégories et de dimensions très différentes ; les portées annoncées ne sont pas comparables sans conditions d’essai|Perturbation de liaisons de commande et/ou de navigation, avec risques d’interférences collatérales|Menace de niveau élevé à considérer dans l’analyse de risques, sans reproduire ses effets par des essais opérationnels sur le terrain.|
|**R6 — Défaillance hors enveloppe spécifiée**|Interférence très forte, saturation du frontal RF, perte simultanée de plusieurs canaux, panne d’antenne ou d’alimentation|Non borné sans hypothèses explicites|Perte complète de communication ou de plusieurs fonctions|Impossible à exclure par un simple bilan de liaison ; la sécurité doit inclure un mode autonome de repli.|

### 2.1 Pourquoi il n’existe pas de « portée de brouillage » universelle

Le paramètre pertinent est le rapport entre le signal utile et l’interférence reçue, avec les effets de modulation, de codage, de filtrage et de saturation du frontal analogique.

* Une émission **large bande** répartit son énergie sur une largeur de spectre plus importante.
* Une émission **ciblée en fréquence** peut concentrer davantage d’énergie dans la bande utile.
* Une antenne **directive** peut concentrer l’énergie dans une direction, mais le gain dépend de son diagramme réel, de son orientation et de ses lobes secondaires.
* Une interférence suffisamment forte peut saturer le récepteur avant qu’un filtrage numérique puisse la rejeter.

Il n’est donc pas rigoureux d’affirmer qu’une puissance donnée neutralisera systématiquement une liaison à 1 km. Il faut spécifier la bande, la géométrie, le gain des antennes, les pertes, la sensibilité du récepteur et le critère de perte de liaison.

## 3\. Analyse directionnelle des communications

## 3.1 Avion → sol : transfert d’images

### Architecture étudiée

* Émetteur radio embarqué en 2,4 GHz.
* Antenne directive sur l’avion, orientée vers la station sol.
* Antenne directive au sol, orientée vers l’avion.
* Station sol positionnée à environ 1 km de la cible observée.

### Menaces à considérer

1. Coexistence radio et congestion de la bande.
2. Brouillage ciblé du récepteur sol.
3. Mauvais alignement, masquage par l’avion ou le relief, et trajets multiples.
4. Perte de paquets ou retransmissions entraînant un dépassement du délai de livraison de 1 à 2 s.
5. Désensibilisation ou saturation du récepteur en présence d’une émission très forte.

### Hypothèse de risque : brouillage du récepteur au sol jugé très peu probable

Dans le scénario d’usage envisagé, la station sol se trouve à environ 1 km de la cible observée et dispose d’une antenne directive pointée vers l’avion. Le brouillage intentionnel du récepteur au sol est donc considéré comme **un scénario de probabilité plus faible**, notamment si l’on suppose un perturbateur non directif situé à proximité de la cible : celui-ci ne bénéficie pas nécessairement d’un couplage favorable vers le récepteur au sol, tandis que la liaison utile bénéficie du gain directionnel des antennes.

Cette hypothèse est utile pour hiérarchiser les risques, mais elle reste conditionnelle et ne constitue pas une preuve de protection :

* la distance entre la station et la cible n’est pas nécessairement la distance entre la station et le brouilleur ;
* un émetteur non directif peut malgré tout produire un niveau d’interférence important s’il est proche du récepteur ;
* le gain des antennes directives est fini et les lobes secondaires existent ;
* un brouilleur directionnel pointé vers la station est un scénario différent d’un brouilleur non directif ;
* un récepteur peut être désensibilisé ou saturé si l’interférence est suffisamment forte.

**Conclusion de travail :** classer le brouillage intentionnel du récepteur sol comme moins prioritaire que la perte du lien de commande, mais conserver une vérification du bilan de liaison et des scénarios de perturbation dans l’analyse de risques.

## 3.2 Sol → avion : commandes

### Architecture actuelle

Liaison LoRa à faible débit. Sa sensibilité et sa modulation peuvent procurer une bonne marge dans un environnement radio ordinaire, mais ne garantissent pas l’immunité à un brouillage ciblé suffisamment puissant dans la bande utilisée.

### Menaces à considérer

1. Interférence sur le canal de commande.
2. Brouillage ciblé en fréquence.
3. Perte de liaison due à l’orientation, au masquage ou à une panne.
4. Commandes retardées, répétées ou devenues obsolètes.
5. Perte simultanée du canal de commande et d’une aide à la navigation, selon l’architecture embarquée.

### Conséquence d’architecture

La priorité est de définir et de valider :

* un contrôle d’intégrité et un numéro de séquence ;
* l’âge maximal d’une commande autorisée à agir sur les gouvernes ;
* une détection de perte de liaison ;
* un mode autonome sûr après dépassement d’un délai défini ;
* une reprise contrôlée lorsque le lien revient.

Le maintien indéfini de la dernière commande reçue ne doit pas être considéré comme un comportement sûr par défaut.

## 3.3 Tableau récapitulatif des deux directions

|Flux|Risque principal|Apport des antennes directives|Risques résiduels|Exigence de sécurité|
|-|-|-|-|-|
|**Avion → sol : images**|Coexistence radio, perte du débit ou dépassement du délai de livraison|Augmentation du niveau utile dans la direction de la station|Désalignement, masquage, interférence dans la bande, saturation du récepteur|Signaler les images perdues ou périmées ; ne pas faire dépendre la stabilité du vol de la vidéo|
|**Sol → avion : commandes**|Brouillage ciblé et perte des commandes|Amélioration possible du bilan selon le diagramme d’antenne, l’orientation et la géométrie|Perte du canal, commandes périmées ou absence de retour|Détection temporisée de perte de lien et stratégie autonome de repli|
|**Optique avion → sol**|Erreur de pointage, lumière ambiante, occultation et météo|Faisceau étroit possible ; un baffle peut limiter les lumières hors axe|Perte de ligne de visée et mouvement de l’avion|Mesure de qualité et de fraîcheur des données, sans dépendre du lien optique seul|
|**Optique sol → avion**|Perte de visée et suivi insuffisant|Support distinct de la RF, mais dépendant de l’alignement|Mouvements, vibrations, occultation et météo|Repli automatique vers un mode sûr si les commandes optiques deviennent trop anciennes|

## 4\. Architectures à comparer

### Architecture A — Radio seule avec protocole renforcé

* Conserver la radio pour les images et les commandes.
* Ajouter numéros de séquence, contrôle d’intégrité, horodatage et mesure de l’âge des commandes.
* Détecter les pertes de paquets, les retards excessifs et la dégradation du lien.
* Définir un mode autonome de repli en cas de perte prolongée des commandes.
* Mesurer le débit utile et le délai applicatif, pas seulement le débit radio théorique.

**Avantages :** simplicité, faible masse et consommation.  
**Limite :** les deux flux restent exposés aux interférences radio dans leurs bandes respectives.

### Architecture B — Radio pour les images, radio + optique complémentaire pour les commandes

* Garder la radio 2,4 GHz pour le transfert d’images.
* Conserver LoRa comme canal de commande principal ou de secours, selon les résultats de l’analyse.
* Étudier un canal optique complémentaire comme moyen de diversité physique.
* Ne pas déclarer l’optique fiable comme secours tant que le pointage, le champ de vue, la lumière ambiante, la météo et la continuité de la liaison n’ont pas été caractérisés.
* Définir une logique explicite de sélection ou de bascule entre les canaux.

**Avantage :** un brouillage RF ne perturbe pas directement le canal optique.  
**Limite :** l’optique peut être perdue pour des raisons géométriques ou environnementales ; la redondance n’est utile que si la perte de chaque canal est détectée et gérée.

### Architecture C — Autonomie de sécurité indépendante des communications

Cette couche doit être présente quelle que soit l’architecture radio ou optique.

* Vérifier l’intégrité et la fraîcheur des commandes.
* Détecter la dégradation et la perte de liaison.
* Ignorer les commandes trop anciennes.
* Appliquer un comportement de vol sûr défini à l’avance.
* Organiser la reprise du lien sans changement brutal des commandes.
* Journaliser les pertes, les délais, les mesures de qualité et les changements de mode.

Pour un avion à voilure fixe, la stratégie de repli doit être adaptée à la dynamique réelle, à la navigation disponible et à l’environnement de vol. « Couper le moteur », « maintenir la dernière commande » ou « revenir au point de départ » ne sont pas universellement sûrs : chaque option demande une analyse et une validation spécifiques.

## 5\. Quantification et validation sans brouilleur opérationnel

Le but est de définir une **enveloppe de fonctionnement mesurable**, et non de promettre une immunité absolue à une menace non bornée.

### 5.1 Vidéo avion → sol

À mesurer :

* débit utile soutenu avec les en-têtes et le contrôle d’erreur ;
* délai de livraison et taux d’images perdues ;
* âge des images à réception ;
* comportement en cas de retard supérieur à 1–2 s ;
* sensibilité à l’orientation et aux occultations ;
* taux d’erreur selon le niveau de signal utile et des interférences simulées.

Pour un flux d’images périodiques, il peut être préférable d’abandonner une image devenue obsolète plutôt que de retransmettre indéfiniment une ancienne image.

### 5.2 Commandes sol → avion

À spécifier et mesurer :

* fréquence des commandes ;
* latence maximale admissible ;
* âge maximal d’une commande applicable ;
* nombre ou proportion de commandes perdues sur une fenêtre temporelle définie ;
* délai de détection de perte de liaison ;
* délai de transition vers le mode autonome sûr ;
* comportement lors de la réacquisition du lien.

Les essais RF peuvent être réalisés sur banc, par atténuation et injection contrôlée d’interférences **par voie conduite**, sans brouilleur opérationnel rayonnant dans l’environnement. Des simulations logicielles permettent également d’injecter pertes de paquets, retards, interruptions et rétablissements.

### 5.3 Liaison optique

Le bilan optique doit intégrer :

* puissance optique réellement émise (à distinguer de la puissance électrique consommée) ;
* divergence du faisceau ;
* ouverture et sensibilité du récepteur ;
* pertes optiques et atmosphériques ;
* erreurs de pointage, vibrations et mouvements ;
* champ de vue et lumière ambiante.

Les essais de banc devraient couvrir les variations d’angle, les interruptions de faisceau et une lumière ambiante représentative. Une estimation de puissance reçue en conditions idéales ne démontre pas à elle seule la fiabilité d’un lien optique mobile à 1 km.

## 6\. Critères de décision à figer

|Critère|Images avion → sol|Commandes sol → avion|
|-|-|-|
|Débit|20 kB en 1–2 s, overhead inclus à confirmer|Faible ; taille et fréquence des commandes à fixer|
|Latence|Délai maximal acceptable de livraison d’une image|Délai maximal avant application d’une commande|
|Fraîcheur|Âge maximal d’une image exploitable|Âge maximal d’une commande applicable|
|Perte de lien|Détection d’images manquantes ou périmées|Détection de perte de commande et bascule autonome|
|Dégradation acceptable|Pertes temporaires possibles selon l’usage|À définir selon la manœuvre ; aucune commande périmée ne doit agir indéfiniment|
|Validation|Débit, latence, pertes et sensibilité à l’orientation|Essais de banc RF, injection contrôlée de pertes/interférences, validation de la logique de sécurité|

## 7\. Intérêt potentiel pour un projet d’intrapreneuriat

Dans la perspective d’une proposition de sujet au sein d'une entreprise défense, le projet peut être présenté à deux niveaux :

1. **Démonstrateur personnel :** intégration de capteurs, électronique, logiciel embarqué, commande, communications et validation d’un système physique.
2. **Sujet industriel transférable :** autonomie embarquée et résilience des systèmes aériens ou optroniques face à la dégradation des communications et de l’environnement.

L’axe « robustesse aux brouillages » gagne à être formulé en termes d’ingénierie système : **dans quelle enveloppe d’interférences le système conserve-t-il ses fonctions, comment détecte-t-il la dégradation, et quel comportement adopte-t-il lorsque les communications ne sont plus fiables ?**

Cette formulation permet de distinguer :

* les performances mesurées ;
* les résultats issus des modèles ;
* les hypothèses de menace ;
* les fonctions de sécurité ;
* les risques restant à valider.

Avant toute transposition au cadre professionnel, vérifier les règles internes de propriété intellectuelle, de confidentialité et d’utilisation des moyens de l’entreprise.

## 8\. Sources publiques à consulter

1. **Commission européenne — Joint Research Centre (JRC)**, *Characterization of Radio Frequency Emissions of Counter Unmanned Aerial System Jammers: Anonymized results of the tests conducted on seven devices at JRC-Ispra in 2022* (2025). Caractérisation expérimentale de puissance RF, densité spectrale, diagrammes de rayonnement et largeur de faisceau de systèmes anti-UAS.  
https://publications.jrc.ec.europa.eu/repository/bitstream/JRC140656/JRC140656\_01.pdf  
Page de publication : https://op.europa.eu/en/publication-detail/-/publication/d3f4570c-e504-11ef-bc1c-01aa75ed71a1
2. **ANFR**, informations sur les brouilleurs GNSS (GPS, Galileo). Les exemples de perturbation GNSS ne doivent pas être transposés directement à une liaison de commande 2,4 GHz ou LoRa.  
https://www.anfr.fr/controler/sources-possibles/brouilleur-gnss-gps-galileo
3. **EASA**, *Easy Access Rules for Unmanned Aircraft Systems*. Référentiel réglementaire incluant la perte de liaison C2 et les procédures de contingence. Vérifier la version applicable au moment du projet.  
https://www.easa.europa.eu/en/document-library/easy-access-rules/online-publications/easy-access-rules-unmanned-aircraft-systems
4. **Commission européenne — JRC**, *Handbook on UAS protection of Critical Infrastructure and Public Space: A five Phase approach for C-UAS stakeholders* (2023).  
https://publications.jrc.ec.europa.eu/repository/handle/JRC132714
5. **ETSI**, *EN 300 328*, exigences applicables à certains équipements de transmission de données en 2,4 GHz. La conformité et la coexistence radio ne garantissent pas l’immunité au brouillage intentionnel.  
https://www.etsi.org/deliver/etsi\_en/300300\_300399/300328/02.02.02\_60/en\_300328v020202p.pdf

## Conclusion

Le brouillage ciblé doit être pris en compte pour le lien de commande sol → avion, même avec LoRa. Pour la vidéo avion → sol, la directivité des antennes aux deux extrémités et l’éloignement prévu de la station sol par rapport à la cible justifient de classer le brouillage intentionnel du récepteur sol comme un scénario de priorité plus faible dans l’hypothèse d’un perturbateur non directif proche de la cible. Cette appréciation reste conditionnelle : la distance station–cible ne garantit ni la distance au brouilleur ni le niveau d’interférence reçu.

La garantie défendable est donc : **dans une enveloppe de fonctionnement spécifiée, les performances sont caractérisées ; en dehors de cette enveloppe, la perte de communication est détectée et le système adopte un état de vol sûr défini et validé.**

