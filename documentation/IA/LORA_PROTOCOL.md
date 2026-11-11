# LoRa protocol

Ce document décrit le protocole de communication utilisé sur la liaison LoRa entre le système sol et l'aéronef.

Il définit :

* les paramètres de la liaison radio ;
* la structure des trames ;
* le codage des commandes de vol ;
* le codage et le décodage des trames de paramétrage.

La génération des commandes et leur interface avec la manette sont décrites dans [`COMMANDE_SOL.md`](../COMMANDE_SOL.md).

La liste détaillée des paramètres configurables est générée séparément à partir du fichier de définition des paramètres.

---

## 1. Paramètres radio

Les paramètres actuels de la liaison LoRa sont :

| Paramètre                     |    Valeur | Description                 |
| ----------------------------- | --------: | --------------------------- |
| Fréquence                     | `433 MHz` | Fréquence porteuse actuelle |
| Spreading Factor              |     `SF9` | Compromis portée / débit    |
| Bandwidth                     | `125 kHz` | Bande passante LoRa         |
| Coding Rate                   |     `4/5` | Taux de codage              |
| Période d'envoi des commandes |  `200 ms` | 5 trames/s                  |

La fréquence est définie par :

```cpp
#define LORA_FREQ 433E6
```

Elle peut être adaptée à la bande utilisée par le système :

```cpp
// 433E6 ou 868E6 selon configuration
```

Les paramètres de modulation sont :

```cpp
#define LORA_SF 9
#define LORA_BW 125E3
#define LORA_CR4 5
```

Le système de commande est dimensionné pour une période d'environ `200 ms` entre deux transmissions.

---

## 2. Structure générale des trames

Les trames applicatives utilisent deux octets de synchronisation :

```cpp
#define SYNC_1 0xAA
#define SYNC_2 0x55
```

Une trame complète de commande ou de paramétrage a la structure suivante :

| Octet | Contenu         |
| ----: | --------------- |
|   `0` | `SYNC_1 = 0xAA` |
|   `1` | `SYNC_2 = 0x55` |
|   `2` | `TYPE`          |
|   `3` | `DAT0`          |
|   `4` | `DAT1`          |
|   `5` | `DAT2`          |
|   `6` | `CRC`           |

Soit une longueur fixe de **7 octets**.

Les octets de synchronisation ne font pas partie du contenu applicatif.

Le calcul du CRC est effectué sur :

```text
TYPE DAT0 DAT1 DAT2
```

---

# 3. Trame de commande

## 3.1 Type

Les commandes de vol utilisent :

```text
TYPE_CMD = 0x01
```

La partie utile de la trame est donc :

```text
TYPE DAT0 DAT1 DAT2
```

avec :

```text
TYPE = 0x01
```

---

## 3.2 Valeurs physiques

Les commandes de vol sont représentées par quatre grandeurs normalisées.

| Commande |       Plage |
| -------- | ----------: |
| Moteur   |   `0 ... 1` |
| Yaw      | `-1 ... +1` |
| Pitch    | `-1 ... +1` |
| Roll     | `-1 ... +1` |

Le moteur est donc représenté par une valeur positive normalisée, tandis que les trois commandes de gouvernes sont centrées autour de zéro.

Avant transmission, chaque commande est convertie en une valeur sur **8 bits**.

### Moteur

```text
motor_byte = int(255 × motor)
```

avec :

```text
motor ∈ [0, 1]
```

Ainsi :

| Valeur | `motor_byte` |
| -----: | -----------: |
|    `0` |          `0` |
|    `1` |        `255` |

### Gouvernes

Pour une commande `u` comprise entre `-1` et `+1` :

```text
u_byte = int(255 × (u + 1) / 2)
```

Ainsi :

| Commande | Valeur 8 bits |
| -------: | ------------: |
|     `-1` |           `0` |
|      `0` |         `127` |
|     `+1` |         `255` |

Le neutre est donc codé `127` sur 8 bits.

---

# 4. Réduction sur 6 bits

Afin de transmettre les quatre commandes dans seulement trois octets, chaque valeur 8 bits est réduite à **6 bits**.

La conversion utilisée est :

```cpp
value_6 = value_8 >> 2;
```

Les deux bits de poids faible sont donc supprimés.

Les valeurs transmises sont :

```text
motor_6 : 0 ... 63
yaw_6   : 0 ... 63
pitch_6 : 0 ... 63
roll_6  : 0 ... 63
```

Pour les commandes centrées, le neutre :

```text
value_8 = 127
```

devient :

```text
127 >> 2 = 31
```

Le neutre transmis est donc **31**, et non 32.

La résolution des commandes de gouvernes est ainsi d'environ :

```text
2 / 63 ≈ 0,0317
```

sur la plage normalisée `[-1,+1]`.

---

# 5. Empaquetage des commandes

Les quatre valeurs de 6 bits sont empaquetées dans trois octets.

```text
motor_6 = MMMMMM
yaw_6   = YYYYYY
pitch_6 = PPPPPP
roll_6  = RRRRRR
```

Les octets transmis sont :

```text
DAT0 = MMMMMMYY
DAT1 = YYYYPPPP
DAT2 = PPRRRRRR
```

Le code d'empaquetage est équivalent à :

```cpp
DAT0 = (motor_6 << 2) | (yaw_6 >> 4);

DAT1 = ((yaw_6 & 0x0F) << 4) | (pitch_6 >> 2);

DAT2 = ((pitch_6 & 0x03) << 6) | roll_6;
```

---

## 5.1 Décodage

Le récepteur récupère les quatre valeurs 6 bits avec :

```cpp
motor_6 = DAT0 >> 2;

yaw_6   = ((DAT0 & 0x03) << 4) | (DAT1 >> 4);

pitch_6 = ((DAT1 & 0x0F) << 2) | (DAT2 >> 6);

roll_6  = DAT2 & 0x3F;
```

Les valeurs peuvent ensuite être reconverties dans leur domaine normalisé.

### Moteur

```text
motor ≈ motor_6 / 63
```

### Gouvernes

```text
u ≈ 2 × (u_6 / 63) - 1
```

où `u` représente yaw, pitch ou roll.

Cette conversion est une reconstruction de la valeur quantifiée reçue ; elle ne restitue pas les deux bits de précision supprimés lors de l'émission.

---

# 6. Exemple de trame de commande

Pour une commande neutre :

```text
motor = 0
yaw   = 0
pitch = 0
roll  = 0
```

on obtient :

```text
motor_8 = 0
yaw_8   = 127
pitch_8 = 127
roll_8  = 127
```

puis :

```text
motor_6 = 0
yaw_6   = 31
pitch_6 = 31
roll_6  = 31
```

L'empaquetage donne :

```text
DAT0 = 00000001
DAT1 = 11110111
DAT2 = 11011111
```

soit :

```text
DAT0 = 0x01
DAT1 = 0xF7
DAT2 = 0xDF
```

La partie applicative de la trame est donc :

```text
01 01 F7 DF
```

Le dernier octet est le CRC calculé sur ces quatre octets.

---

# 7. Trames de paramétrage

Les trames de paramétrage utilisent un codage différent des commandes de vol.

Le champ `TYPE` contient une partie de l'identifiant du paramètre.

Pour un identifiant de paramètre sur 16 bits :

```text
ID = 0xHHLL
```

la trame utilise :

```text
TYPE = 0x10 + HH
DAT0 = LL
DAT1 = valeur LSB
DAT2 = valeur MSB
```

Le principe est donc :

```text
+----------------+----------------+----------------+----------------+
|      TYPE      |      DAT0      |      DAT1      |      DAT2      |
|    0x10 + HH   |       LL       |   valeur LSB   |   valeur MSB   |
+----------------+----------------+----------------+----------------+
```

Le paramètre est identifié par :

```text
ID = ((TYPE - 0x10) << 8) | DAT0
```

Les trames dont :

```text
TYPE < 0x10
```

ne sont pas traitées comme des trames de paramétrage.

---

# 8. Codage de la valeur d'un paramètre

La valeur physique d'un paramètre est quantifiée sur un entier signé 16 bits selon son nombre de bits fractionnaires `binpt`.

Le codage est :

```text
raw = round(value × 2^binpt)
```

La valeur `raw` est ensuite transmise sur deux octets, **LSB en premier** :

```text
DAT1 = raw & 0xFF
DAT2 = (raw >> 8) & 0xFF
```

Le récepteur reconstruit l'entier signé :

```cpp
int16_t raw =
    (int16_t)(dat1 | ((uint16_t)dat2 << 8));
```

puis reconstruit la valeur physique :

```text
value = raw / 2^binpt
```

---

## 8.1 Exemple avec `binpt = 4`

Pour un paramètre flottant utilisant `binpt = 4` :

```text
raw = round(value × 16)
```

Au décodage :

```cpp
value = (float)raw / 16.0f;
```

Par exemple, pour :

```text
value = 2.5
```

on obtient :

```text
raw = 2.5 × 16 = 40
```

soit :

```text
raw = 0x0028
```

et donc :

```text
DAT1 = 0x28
DAT2 = 0x00
```

---

## 8.2 Exemple avec `binpt = 8`

Pour `binpt = 8` :

```text
raw = round(value × 256)
```

et :

```text
value = raw / 256
```

Le même principe est utilisé indépendamment du nombre de bits fractionnaires du paramètre.

Le détail de chaque paramètre (`ID`, type, `binpt`, destination dans la structure, etc.) est défini dans le fichier de configuration des paramètres et n'est pas dupliqué dans ce document.

---

# 9. Paramètres entiers

Pour un paramètre entier sans partie fractionnaire :

```text
binpt = 0
```

la valeur transmise est directement l'entier :

```text
raw = value
```

Par exemple, pour un paramètre `uint8_t` :

```cpp
system_param.control_param.ctrl_min_yaw =
    (uint8_t)(dat1 | ((uint16_t)dat2 << 8));
```

Le protocole transporte néanmoins systématiquement la valeur sur **16 bits**, avec :

```text
DAT1 = LSB
DAT2 = MSB
```

Le type du paramètre détermine ensuite la conversion vers le type C/C++ correspondant.

---

# 10. Reconstruction de l'identifiant

Le découpage de l'identifiant permet de transmettre des paramètres répartis sur plusieurs blocs de `0x100` identifiants.

Exemples :

|       ID | `TYPE` | `DAT0` |
| -------: | -----: | -----: |
| `0x0000` | `0x10` | `0x00` |
| `0x0001` | `0x10` | `0x01` |
| `0x00FF` | `0x10` | `0xFF` |
| `0x0100` | `0x11` | `0x00` |
| `0x0101` | `0x11` | `0x01` |

La reconstruction côté aéronef est :

```cpp
uint16_t id =
    ((uint16_t)(tx_type - 0x10) << 8) | dat0;
```

---

# 11. CRC

Le CRC est calculé sur les quatre octets applicatifs :

```text
TYPE DAT0 DAT1 DAT2
```

Les octets de synchronisation :

```text
0xAA 0x55
```

ne sont pas inclus dans le calcul.

L'algorithme utilisé est un **CRC-8 polynomial `0x07`**, avec initialisation à `0x00`.

Principe :

```text
crc = 0x00

pour chaque octet :
    crc ^= octet

    répéter 8 fois :
        si bit 7 de crc = 1 :
            crc = (crc << 1) ^ 0x07
        sinon :
            crc = crc << 1

        conserver uniquement 8 bits
```

La trame complète est donc :

```text
AA 55 TYPE DAT0 DAT1 DAT2 CRC
```

---

# 12. Résumé des types de trames

| Type             |            `TYPE` | DAT0                         | DAT1             | DAT2             |
| ---------------- | ----------------: | ---------------------------- | ---------------- | ---------------- |
| Commande         |            `0x01` | commandes 6 bits             | commandes 6 bits | commandes 6 bits |
| Log / sauvegarde |            `0x02` | `1 = LOG ON`, `2 = LOG SAVE` | `0x00`           | `0x00`           |
| Paramètre        | `0x10 + ID[15:8]` | `ID[7:0]`                    | valeur LSB       | valeur MSB       |


Dans les deux cas, la trame physique/applicative complète est :

```text
AA 55 TYPE DAT0 DAT1 DAT2 CRC
```

Le protocole ne définit pas de numéro de séquence, d'horodatage ou d'accusé de réception dans les trames décrites ici.
