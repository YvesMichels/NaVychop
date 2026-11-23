\# Proposition d'évolution — Architecture temps réel de la navigation



\## Objectif



Faire évoluer la boucle de navigation vers une architecture \*\*temps réel coopérative\*\*, garantissant une latence minimale et maîtrisée entre l'acquisition inertielle et l'envoi des commandes moteurs, tout en permettant aux autres capteurs et services de fonctionner de manière asynchrone.



L'objectif n'est pas de mettre en place un ordonnanceur ou un système d'interruptions complexe, mais de construire un \*\*ordonnancement déterministe directement dans la boucle principale\*\*.



\---



\## 1. Principe général



La chaîne d'information de navigation est organisée selon le flux suivant :



```text

Capteurs + LoRa

&#x20;     │

&#x20;     ▼

\[ HYBRIDATION ]

&#x20;     │

&#x20;     ▼

&#x20;  CONSIGNES

&#x20;     │

&#x20;     ▼

\[ ASSERVISSEMENT ]

&#x20;     │

&#x20;     ▼

COMMANDES MOTEURS

&#x20;     │

&#x20;     ▼

&#x20;   AILERONS

```



La séparation entre \*\*consignes\*\* et \*\*commandes moteurs\*\* permet de distinguer clairement :



\- les objectifs fournis à l'asservissement ;

\- les commandes effectivement calculées par celui-ci pour les actionneurs.



L'asservissement reste ainsi l'unique élément responsable de la génération des commandes moteurs.



\---



\## 2. Priorité à la chaîne inertielle



La chaîne inertielle constitue la partie critique du système :



```text

Nouveau GRV BNO085

&#x20;       │

&#x20;       ▼

Lecture du quaternion

&#x20;       │

&#x20;       ▼

Calcul de R\_G\_A

&#x20;       │

&#x20;       ▼

Asservissement

&#x20;       │

&#x20;       ▼

Commandes moteurs

```



Cette chaîne doit être exécutée dès qu'un nouveau \*\*GRV\*\* est disponible.



La fréquence nominale de la première version est fixée à \*\*25 Hz\*\*, soit une période de 40 ms.



La boucle `loop()` n'est donc pas elle-même cadencée artificiellement à 40 ms : elle fonctionne librement et exploite l'arrivée d'un nouveau GRV comme événement déclencheur de la chaîne inertielle.



\### Principe



```cpp

loop()

{

&#x20;   lecture BNO085;



&#x20;   if (nouveau\_GRV)

&#x20;   {

&#x20;       calcul R\_G\_A;

&#x20;       asservissement;

&#x20;       envoi commandes moteurs;

&#x20;   }



&#x20;   services secondaires;

}

```



Cela minimise le délai entre :



```text

disponibilité du GRV

&#x20;       ↓

calcul de l'attitude

&#x20;       ↓

calcul de l'asservissement

&#x20;       ↓

commande moteur

```



et évite de faire dépendre cette latence d'une cadence artificielle de la boucle principale.



\---



\## 3. Ordonnancement coopératif à priorités statiques



L'architecture peut être considérée comme un \*\*système d'ordonnancement coopératif implicite\*\*.



Les priorités sont définies par :



1\. l'ordre d'exécution des fonctions ;

2\. la fréquence à laquelle elles sont sollicitées ;

3\. une durée maximale d'exécution pour chaque fonction ;

4\. l'interdiction des attentes bloquantes.



La priorité temporelle peut être représentée ainsi :



```text

P0 — Temps réel critique

│

├── BNO085

├── calcul R\_G\_A

├── asservissement

└── commandes moteurs



P1 — Temps réel souple

│

├── BMP390

├── NEO-N8M

├── réception Vision

├── LoRa

└── Hybridation



P2 — Arrière-plan

│

├── logs

├── statistiques

└── diagnostics

```



Les fonctions P1 et P2 ne doivent jamais empêcher la reprise de la chaîne P0.



\---



\## 4. Services capteurs non bloquants



Les périphériques secondaires sont traités comme des \*\*services opportunistes\*\*.



Une fonction appelée depuis `loop()` doit :



\- vérifier si une donnée est disponible ;

\- traiter la donnée disponible ;

\- retourner immédiatement si aucune donnée n'est disponible ;

\- ne jamais attendre activement l'arrivée d'une donnée ;

\- respecter un budget maximal d'exécution.



Exemple :



```text

GPS disponible ?

&#x20;├── NON → retour immédiat

&#x20;└── OUI → lecture partielle / traitement → retour

```



Une trame UART peut ainsi être reconstruite progressivement sur plusieurs passages dans `loop()`.



Cette approche permet notamment de garantir qu'une lecture GPS ou LoRa ne puisse monopoliser le processeur.



\---



\## 5. Lecture UART avec budget temporel



Les communications UART sont traitées avec une limite temporelle explicite.



Pour une fonction donnée :



```text

entrée dans la fonction

&#x20;       │

&#x20;       ▼

lecture des octets disponibles

&#x20;       │

&#x20;       ▼

reconstruction progressive de la trame

&#x20;       │

&#x20;       ▼

sortie

```



La fonction ne doit pas attendre la réception complète d'une trame.



Une limite d'environ \*\*2 ms par passage\*\* permet de conserver la maîtrise du temps CPU tout en assurant la progression de la réception.



Cette méthode est particulièrement adaptée aux données GPS et aux autres flux asynchrones.



\---



\## 6. Vision déportée



Le traitement d'image ne fait pas partie de la charge CPU de l'ESP32 de navigation.



Il est réalisé sur une \*\*ESP32 dédiée\*\*, qui transmet à la carte de navigation uniquement une information compacte correspondant au mouvement estimé.



```text

Caméra

&#x20;  │

&#x20;  ▼

ESP32 Vision

&#x20;  │

&#x20;  │ traitement image

&#x20;  │

&#x20;  ▼

Trame mouvement

&#x20;  │

&#x20;  │ UART

&#x20;  ▼

ESP32 Navigation

&#x20;  │

&#x20;  ▼

Hybridation

```



La carte de navigation ne réalise donc que :



\- la réception UART ;

\- la reconstruction de la trame ;

\- la validation des données ;

\- l'intégration de l'information dans l'hybridation.



Le coût CPU du traitement d'image est ainsi totalement découplé de la boucle inertielle.



\---



\## 7. Hybridation et consignes



L'hybridation regroupe les informations provenant des différents capteurs :



```text

BNO085

GPS

BMP390

Vision

LoRa

&#x20;  │

&#x20;  ▼

HYBRIDATION

&#x20;  │

&#x20;  ▼

CONSІGNES

```



L'hybridation peut mettre à jour les consignes à n'importe quel moment où de nouvelles informations deviennent disponibles.



Ces nouvelles consignes seront utilisées par le prochain cycle d'asservissement déclenché par le BNO085.



On évite ainsi que les différents capteurs tentent directement de modifier les commandes moteurs.



\---



\## 8. Propriété fondamentale : un seul propriétaire des moteurs



L'architecture impose une séparation claire :



```text

Capteurs + LoRa

&#x20;      ↓

Hybridation

&#x20;      ↓

Consignes

&#x20;      ↓

Asservissement

&#x20;      ↓

Commandes moteurs

```



Les capteurs, l'hybridation et les fonctions de navigation ne modifient pas directement les commandes moteurs.



\*\*L'asservissement est l'unique propriétaire de la sortie moteur.\*\*



Cela facilite notamment :



\- le diagnostic ;

\- la validation ;

\- la gestion des modes de vol ;

\- l'évolution de l'hybridation ;

\- la mesure de la latence.



\---



\## 9. Gestion temporelle et sécurité



Le déclenchement par nouveau GRV ne dispense pas d'une surveillance temporelle.



Le système doit notamment mesurer :



```text

temps depuis le dernier GRV

temps d'exécution de chaque service

temps entre deux cycles inertiels

latence GRV → commande moteur

```



La fréquence nominale de 25 Hz correspond à :



```text

T = 40 ms

```



mais \*\*40 ms ne constitue pas à lui seul le seuil de défaillance\*\*.



Une marge doit être prévue afin de détecter :



\- une perte du BNO085 ;

\- une absence prolongée de nouveaux GRV ;

\- un dépassement anormal du temps d'exécution ;

\- un blocage d'un périphérique ;

\- une dérive de la fréquence réelle.



\---



\## 10. Instrumentation du temps réel



L'évolution de l'architecture doit être accompagnée d'une instrumentation permettant de vérifier expérimentalement les contraintes temporelles.



Les mesures principales sont :



```text

t\_GRV

&#x20; │

&#x20; ├── t\_R\_G\_A

&#x20; ├── t\_asservissement

&#x20; └── t\_moteur

```



et pour les services :



```text

t\_entrée

t\_sortie

durée maximale

nombre d'appels

```



La métrique principale de la chaîne P0 devient alors :



```text

Latence inertielle =

t\_commande\_moteur - t\_disponibilité\_GRV

```



Cette valeur doit être mesurée plutôt qu'estimée.



\---



\## 11. Architecture résultante



La boucle principale peut finalement être structurée ainsi :



```text

┌──────────────────────────────────────────────┐

│                  LOOP                        │

│                                              │

│  Lecture BNO085                              │

│       │                                      │

│       ├── Nouveau GRV ?                      │

│       │       │                              │

│       │      OUI                             │

│       │       ↓                              │

│       │   Calcul R\_G\_A                       │

│       │       ↓                              │

│       │   ASSERVISSEMENT                     │

│       │       ↓                              │

│       │   COMMANDES MOTEURS                  │

│       │                                      │

│       ├── BMP390                             │

│       ├── NEO-N8M                            │

│       ├── UART Vision                        │

│       ├── LoRa                               │

│       ├── Hybridation                        │

│       └── Gestion temps / diagnostics        │

│                                              │

└──────────────────────────────────────────────┘

```



La boucle constitue ainsi un \*\*ordonnanceur coopératif minimal\*\*, dont les priorités sont définies explicitement par l'architecture plutôt que par un système de tâches complexe.



\---



\## 12. Évolution future



Cette architecture permet de faire évoluer indépendamment les différents sous-systèmes.



La fréquence du BNO085 pourra notamment évoluer :



```text

25 Hz → 50 Hz → 100 Hz

```



sans remettre en cause l'organisation générale.



De même, l'ajout d'un nouveau capteur consiste essentiellement à ajouter un nouveau service non bloquant, à condition de respecter les contraintes de temps d'exécution.



L'architecture conserve ainsi une séparation claire entre :



\- \*\*acquisition\*\* ;

\- \*\*estimation / hybridation\*\* ;

\- \*\*consignes\*\* ;

\- \*\*asservissement\*\* ;

\- \*\*commandes moteurs\*\* ;

\- \*\*actionnement\*\*.



Cette séparation constitue la base d'une architecture temps réel simple, mesurable et évolutive pour la navigation autonome de NaVychop.

