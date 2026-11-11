\# LoRa Protocol



\## 1. Objet



Ce document décrit le protocole de communication utilisé entre le système de commande/paramétrage et l'aéronef `navychop`.



Le protocole est utilisé pour :



\* transmettre les commandes de vol issues de la manette PS4 ;

\* transmettre les demandes de sauvegarde ;

\* transmettre les paramètres de configuration de l'aéronef ;

\* assurer l'intégrité des trames par CRC-8.



La transmission physique est réalisée via une liaison série vers le module radio LoRa.



Le protocole applicatif décrit ici est indépendant du contenu physique des paquets LoRa : les octets décrits dans ce document constituent la trame transmise à l'interface radio.



\---



\# 2. Format général d'une trame



Toutes les trames utilisent la même structure :



| Octet | Contenu                         |

| ----: | ------------------------------- |

|   `0` | Synchronisation `SYNC\_1 = 0xAA` |

|   `1` | Synchronisation `SYNC\_2 = 0x55` |

|   `2` | `TYPE`                          |

|   `3` | `DAT0`                          |

|   `4` | `DAT1`                          |

|   `5` | `DAT2`                          |

|   `6` | `CRC8`                          |



Une trame complète contient donc \*\*7 octets\*\*.



```text

AA 55 TYPE DAT0 DAT1 DAT2 CRC

```



Les deux octets de synchronisation ne sont \*\*pas inclus dans le calcul du CRC\*\*.



Le CRC est calculé uniquement sur les quatre octets :



```text

TYPE DAT0 DAT1 DAT2

```



Le calcul est effectué par la fonction `crc8()` du programme.



\---



\# 3. Synchronisation



Les deux premiers octets sont fixes :



```text

SYNC\_1 = 0xAA

SYNC\_2 = 0x55

```



Une trame valide commence donc toujours par :



```text

AA 55

```



Ces deux octets permettent au récepteur de rechercher le début d'une trame dans un flux d'octets.



Ils ne portent aucune information applicative et ne participent pas au CRC.



\---



\# 4. CRC-8



\## 4.1 Polynôme



Le CRC utilise :



```text

POLYNOME = 0x07

```



avec une valeur initiale :



```text

CRC\_INIT = 0x00

```



Le traitement est effectué bit par bit, MSB first.



Équivalent algorithmique :



```text

crc = 0x00



pour chaque octet :

&#x20;   crc ^= octet



&#x20;   répéter 8 fois :

&#x20;       si bit 7 de crc = 1 :

&#x20;           crc = ((crc << 1) ^ 0x07) \& 0xFF

&#x20;       sinon :

&#x20;           crc = (crc << 1) \& 0xFF

```



Le CRC porte exclusivement sur :



```text

TYPE DAT0 DAT1 DAT2

```



et non sur :



```text

AA 55

```



La trame complète est donc :



```text

AA 55 TYPE DAT0 DAT1 DAT2 CRC8(TYPE,DAT0,DAT1,DAT2)

```



\---



\# 5. Types de trames



Deux familles principales sont utilisées pour le pilotage :



| `TYPE`      | Valeur | Fonction               |

| ----------- | -----: | ---------------------- |

| `TYPE\_CMD`  | `0x01` | Commandes de vol       |

| `TYPE\_SAVE` | `0x02` | Commande de sauvegarde |



Les paramètres utilisent un encodage particulier de l'ID dans `TYPE`.



\---



\# 6. Trame de commande de vol



\## 6.1 Structure



Une commande de vol utilise :



```text

TYPE = 0x01

```



et trois octets de données :



```text

DAT0

DAT1

DAT2

```



Ces trois octets contiennent \*\*quatre valeurs de 6 bits\*\* :



\* moteur ;

\* yaw ;

\* pitch ;

\* roll.



Cela représente exactement :



```text

4 × 6 bits = 24 bits = 3 octets

```



Il n'y a donc aucun bit de remplissage.



\---



\# 7. Quantification des commandes



Les commandes issues de la manette sont d'abord représentées sur \*\*8 bits\*\*, dans la plage :



```text

0 ... 255

```



Elles sont ensuite réduites à 6 bits par :



```cpp

value\_6 = (value\_8 >> 2) \& 0x3F;

```



Le décalage `>> 2` supprime donc les \*\*2 bits de poids faible\*\*.



Ainsi :



| Valeur 8 bits | Valeur 6 bits |

| ------------: | ------------: |

|           `0` |           `0` |

|     `1` à `3` |           `0` |

|     `4` à `7` |           `1` |

|           ... |           ... |

| `252` à `255` |          `63` |



La conversion n'est donc pas une mise à l'échelle mathématique suivie d'un arrondi : c'est simplement une \*\*troncature des deux LSB\*\*.



En pratique :



```text

value\_6 = floor(value\_8 / 4)

```



\---



\# 8. Codage des commandes analogiques



\## 8.1 Yaw



La commande yaw interne est comprise entre :



```text

\-1.0 ... +1.0

```



Elle est convertie en 8 bits par :



```cpp

yaw\_byte = int(255 \* (yaw + 1) / 2);

```



La correspondance théorique est donc :



|  Yaw | `yaw\_byte` |

| ---: | ---------: |

| `-1` |        `0` |

|  `0` |      `127` |

| `+1` |      `255` |



Puis :



```cpp

yaw\_6 = (yaw\_byte >> 2) \& 0x3F;

```



\### Subtilité importante : neutre



Le neutre n'est \*\*pas 32\*\*.



Avec le calcul actuel :



```text

yaw = 0

→ yaw\_byte = 127

→ yaw\_6 = 127 >> 2

→ yaw\_6 = 31

```



Le neutre transmis est donc :



```text

31

```



Cette convention est importante pour le décodage embarqué.



\---



\## 8.2 Pitch



Le pitch utilise le même encodage 8 bits puis 6 bits.



Le joystick est d'abord inversé :



```cpp

pitch = -pitch\_in + pitch\_offset;

```



puis limité à :



```text

\-1 ... +1

```



La conversion est :



```cpp

pitch\_byte = int(((pitch + 1) / 2) \* 255);

pitch\_6 = (pitch\_byte >> 2) \& 0x3F;

```



La convention de signe du pitch doit donc être interprétée \*\*avant\*\* l'encodage 6 bits.



\---



\## 8.3 Roll



Le roll est directement issu de l'axe 0 :



```cpp

roll = joystick.get\_axis(0);

```



Après zone morte :



```cpp

roll\_byte = int(255 \* (roll + 1) / 2);

roll\_6 = (roll\_byte >> 2) \& 0x3F;

```



Comme pour yaw et pitch :



```text

roll = -1 → 0

roll =  0 → 31

roll = +1 → 63

```



pour la valeur effectivement transmise sur 6 bits.



\---



\# 9. Encodage du moteur



Le moteur est lui aussi transmis sur 6 bits :



```cpp

motor\_6 = (motor >> 2) \& 0x3F;

```



Contrairement aux commandes yaw/pitch/roll, le moteur n'utilise pas directement une conversion `-1 ... +1`.



\## SAFE



En mode `SAFE` :



```cpp

motor = 0;

```



donc :



```text

motor\_6 = 0

```



\## MANUAL



En mode `MANUAL` :



```cpp

motor = int((0.25 + 0.75 \* r2\_eff) \* 255);

```



La plage normale est donc approximativement :



```text

64 ... 255

```



puis :



```text

motor\_6 = 16 ... 63

```



Le moteur possède ainsi un \*\*niveau minimum de 25 %\*\* lorsque la commande est active.



\## HOLD



En mode `HOLD`, la même conversion est appliquée à `motor\_hold` :



```cpp

motor = int((0.25 + 0.75 \* motor\_hold) \* 255);

```



\---



\# 10. Empaquetage des quatre valeurs 6 bits



Les quatre valeurs sont nommées :



```text

M = motor\_6

Y = yaw\_6

P = pitch\_6

R = roll\_6

```



avec :



```text

M : 6 bits

Y : 6 bits

P : 6 bits

R : 6 bits

```



L'ordre de transmission est :



```text

M Y P R

```



Les 24 bits sont concaténés dans cet ordre.



\## DAT0



```cpp

DAT0 = (M << 2) | (Y >> 4)

```



Structure :



```text

bit : 7 6 5 4 3 2 | 1 0

&#x20;     M M M M M M | Y Y

```



Donc :



\* bits `7..2` = moteur ;

\* bits `1..0` = deux MSB du yaw.



\---



\## DAT1



```cpp

DAT1 = ((Y \& 0x0F) << 4) | (P >> 2)

```



Structure :



```text

bit : 7 6 5 4 | 3 2 1 0

&#x20;     Y Y Y Y | P P P P

```



Donc :



\* bits `7..4` = quatre LSB du yaw ;

\* bits `3..0` = quatre MSB du pitch.



\---



\## DAT2



```cpp

DAT2 = ((P \& 0x03) << 6) | R

```



Structure :



```text

bit : 7 6 | 5 4 3 2 1 0

&#x20;     P P | R R R R R R

```



Donc :



\* bits `7..6` = deux LSB du pitch ;

\* bits `5..0` = roll complet.



\---



\# 11. Vue synthétique de l'empaquetage



Le contenu des trois octets peut être représenté comme suit :



| Octet  | Bits 7 → 0                |

| ------ | ------------------------- |

| `DAT0` | `M5 M4 M3 M2 M1 M0 Y5 Y4` |

| `DAT1` | `Y3 Y2 Y1 Y0 P5 P4 P3 P2` |

| `DAT2` | `P1 P0 R5 R4 R3 R2 R1 R0` |



Le décodage doit donc être exactement l'inverse :



```text

M = DAT0 >> 2



Y = ((DAT0 \& 0x03) << 4) | (DAT1 >> 4)



P = ((DAT1 \& 0x0F) << 2) | (DAT2 >> 6)



R = DAT2 \& 0x3F

```



Les quatre valeurs obtenues sont toujours comprises entre :



```text

0 ... 63

```



\---



\# 12. Exemple : commande neutre



Pour une commande neutre :



```text

yaw   = 0

pitch = 0

roll  = 0

```



les trois axes produisent :



```text

8 bits = 127

6 bits = 31

```



Si le moteur est coupé :



```text

motor\_6 = 0

yaw\_6   = 31

pitch\_6 = 31

roll\_6  = 31

```



L'empaquetage donne :



```text

DAT0 = (0 << 2) | (31 >> 4)

&#x20;    = 0



DAT1 = ((31 \& 0x0F) << 4) | (31 >> 2)

&#x20;    = 0xF7



DAT2 = ((31 \& 0x03) << 6) | 31

&#x20;    = 0xDF

```



La charge utile est donc :



```text

01 00 F7 DF

```



et la trame complète est :



```text

AA 55 01 00 F7 DF CRC

```



où `CRC` est calculé sur :



```text

01 00 F7 DF

```



\---



\# 13. Trame SAVE



La commande de sauvegarde utilise :



```text

TYPE = 0x02

```



et :



```text

DAT0 = save\_request

DAT1 = 0

DAT2 = 0

```



Deux valeurs sont actuellement utilisées :



| Action   | `DAT0` |

| -------- | -----: |

| Log ON   |    `1` |

| Log SAVE |    `2` |



La trame est donc :



```text

AA 55 02 01 00 00 CRC

```



pour `Log ON`, ou :



```text

AA 55 02 02 00 00 CRC

```



pour `Log SAVE`.



Le CRC est toujours calculé sur les quatre octets :



```text

02 DAT0 00 00

```



\---



\# 14. Trames de paramétrage



Le protocole utilise également les mêmes trames de 7 octets pour transmettre les paramètres de configuration.



Contrairement aux commandes de vol, le type de message encode une partie de l'identifiant du paramètre.



Pour un paramètre d'ID :



```text

0xHHLL

```



où :



\* `HH` = octet de poids fort de l'ID ;

\* `LL` = octet de poids faible de l'ID.



Le code construit :



```cpp

id\_lsb = param.id \& 0xFF;

id\_msb = (param.id >> 8) \& 0xFF;



tx\_type = 0x10 + id\_msb;

dat0 = id\_lsb;

```



La structure est donc :



```text

TYPE = 0x10 + ID\_MSB

DAT0 = ID\_LSB

```



Ainsi, l'ID original peut être reconstruit côté réception par :



```text

ID\_MSB = TYPE - 0x10

ID\_LSB = DAT0



ID = (ID\_MSB << 8) | ID\_LSB

```



\### Exemple



Pour :



```text

ID = 0x0123

```



on obtient :



```text

ID\_MSB = 0x01

ID\_LSB = 0x23



TYPE = 0x10 + 0x01

&#x20;    = 0x11



DAT0 = 0x23

```



\---



\# 15. Encodage de la valeur d'un paramètre



La valeur physique est convertie en entier avec un facteur :



```text

scale = 2^binpt

```



Puis :



```cpp

raw = round(value \* scale)

```



La valeur brute est limitée à :



```text

\-32768 ... +32767

```



Le résultat est transmis sur deux octets, \*\*LSB first\*\* :



```cpp

DAT1 = raw \& 0xFF

DAT2 = (raw >> 8) \& 0xFF

```



La structure d'une trame paramètre est donc :



```text

AA 55 TYPE ID\_LSB RAW\_LSB RAW\_MSB CRC

```



avec :



```text

TYPE = 0x10 + ID\_MSB

```



\---



\# 16. Exemple de paramètre



Pour un paramètre :



```text

ID = 0x0123

binpt = 4

valeur physique = 2.5

```



la valeur brute vaut :



```text

raw = round(2.5 × 16)

&#x20;   = 40

&#x20;   = 0x0028

```



La trame contient alors :



```text

TYPE  = 0x11

DAT0  = 0x23

DAT1  = 0x28

DAT2  = 0x00

```



soit :



```text

AA 55 11 23 28 00 CRC

```



\---



\# 17. Point important : représentation signée des paramètres



Le champ `raw` est traité comme un entier signé 16 bits :



```text

int16

```



avec :



```text

INT16\_MIN = -32768

INT16\_MAX = +32767

```



La reconstruction de la valeur physique est :



```text

value = raw / 2^binpt

```



Les deux octets sont cependant transmis comme des octets non signés ; c'est au récepteur de les réassembler puis de les interpréter comme un `int16\_t`.



\---



\# 18. Cadence de transmission



La cadence maximale actuellement utilisée est :



```text

5 Hz

```



soit :



```text

1 trame toutes les 200 ms

```



Cette cadence est utilisée :



\* pour les commandes de vol ;

\* pour l'envoi des paramètres ;

\* comme cadence de rafraîchissement de la commande joystick.



La constante est :



```cpp

LORA\_RATE\_HZ = 5

LORA\_PERIOD\_S = 0.2 s

```



Pour l'envoi d'un ensemble de paramètres, le programme attend également :



```text

200 ms

```



entre deux trames.



\---



\# 19. Modes de pilotage



Les trois modes utilisés par le logiciel de commande sont :



| Mode     | Valeur interne |

| -------- | -------------: |

| `SAFE`   |            `0` |

| `MANUAL` |            `1` |

| `HOLD`   |            `2` |



Ces valeurs ne sont \*\*pas directement transmises dans un champ dédié de la trame de commande\*\*.



Le mode influence la génération des valeurs de commande, notamment du moteur.



Par conséquent, le récepteur ne doit pas chercher un champ `MODE` dans les trois octets de commande.



\---



\# 20. Particularités à respecter côté récepteur



L'implémentation embarquée doit respecter les conventions suivantes.



\### 20.1 Synchronisation



Rechercher :



```text

AA 55

```



avant de traiter une trame.



\### 20.2 Longueur fixe



Une trame applicative complète contient exactement :



```text

7 octets

```



\### 20.3 CRC



Calculer le CRC uniquement sur :



```text

TYPE DAT0 DAT1 DAT2

```



et comparer le résultat avec l'octet `CRC`.



Les octets `AA 55` ne doivent pas être inclus dans le CRC.



\### 20.4 Commandes



Pour `TYPE = 0x01` :



```text

M = DAT0 >> 2

Y = ((DAT0 \& 0x03) << 4) | (DAT1 >> 4)

P = ((DAT1 \& 0x0F) << 2) | (DAT2 >> 6)

R = DAT2 \& 0x3F

```



\### 20.5 Neutre



Pour les axes :



```text

NEUTRAL = 31

```



Le neutre ne doit pas être interprété comme `32`.



\### 20.6 Paramètres



Pour :



```text

TYPE >= 0x10

```



l'ID doit être reconstruit avec :



```text

ID = ((TYPE - 0x10) << 8) | DAT0

```



puis :



```text

raw = int16(DAT1 | (DAT2 << 8))

```



\---



\# 21. Résumé des formats



\## Commande de vol



```text

AA 55 01 DAT0 DAT1 DAT2 CRC

```



avec :



```text

DAT0 = MMMMMMYY

DAT1 = YYYYPPPP

DAT2 = PPRRRRRR

```



où :



```text

M = motor  \[6 bits]

Y = yaw    \[6 bits]

P = pitch  \[6 bits]

R = roll   \[6 bits]

```



\---



\## Sauvegarde



```text

AA 55 02 SAVE 00 00 CRC

```



avec :



```text

SAVE = 0x01 → LOG ON

SAVE = 0x02 → LOG SAVE

```



\---



\## Paramètre



```text

AA 55 (0x10 + ID\_MSB) ID\_LSB RAW\_LSB RAW\_MSB CRC

```



avec :



```text

RAW = int16(value × 2^binpt)

```



\---



\# 22. Points de compatibilité critiques



Les éléments suivants font partie intégrante du protocole et ne doivent pas être modifiés indépendamment côté émetteur ou récepteur :



1\. \*\*Synchronisation `0xAA 0x55`\*\*

2\. \*\*Trame de 7 octets\*\*

3\. \*\*CRC-8 polynomial `0x07`, initialisé à `0x00`\*\*

4\. \*\*CRC calculé uniquement sur `TYPE + DAT0 + DAT1 + DAT2`\*\*

5\. \*\*Commandes analogiques quantifiées de 8 bits vers 6 bits par `>> 2`\*\*

6\. \*\*Neutre des axes transmis = `31`\*\*

7\. \*\*Ordre d'empaquetage : moteur → yaw → pitch → roll\*\*

8\. \*\*Répartition exacte des bits dans `DAT0`, `DAT1`, `DAT2`\*\*

9\. \*\*Paramètres : ID réparti entre `TYPE` et `DAT0`\*\*

10\. \*\*Valeur des paramètres sur 16 bits signés, LSB first\*\*

11\. \*\*Cadence nominale de transmission : 5 Hz\*\*

12\. \*\*Aucune information de mode n'est directement présente dans une trame `TYPE\_CMD`\*\*



Toute modification de l'un de ces éléments constitue une \*\*évolution du protocole\*\* et doit être répercutée simultanément sur les deux extrémités de la liaison.



