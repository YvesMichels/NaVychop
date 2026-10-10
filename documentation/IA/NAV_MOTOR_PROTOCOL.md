\# Navigation ↔ Motor control protocol



Ce document décrit le protocole de communication entre la \*\*centrale de navigation\*\* et la \*\*carte de contrôle des moteurs et servomoteurs\*\*.



La centrale de navigation transmet périodiquement une trame contenant les commandes de :



\* servo 1 : profondeur ;

\* servo 2 : gouverne d'aile gauche;

\* servo 3 : gouverne d'aile droite;

\* servo 4 : lacet ;

\* ESC : moteur.



La conversion des commandes reçues en angles de servomoteurs et en consigne ESC est effectuée exclusivement par la carte moteurs.



\---



\# 1. Interface physique



La communication utilise une liaison \*\*UART\*\*.



La carte moteurs utilise :



```cpp

Serial.begin(19200);

```



La vitesse de communication est donc :



```text

19200 bauds

```



Les paramètres détaillés de configuration électrique de l'UART ne sont pas définis dans les extraits de code considérés ici.



\---



\# 2. Structure de la trame



Chaque trame contient \*\*6 octets\*\* :



| Octet | Contenu               |

| ----: | --------------------- |

|   `0` | Octet de début `0xFF` |

|   `1` | Données `bits 0..7`   |

|   `2` | Données `bits 8..15`  |

|   `3` | Données `bits 16..23` |

|   `4` | Données `bits 24..31` |

|   `5` | CRC-8                 |



La structure est donc :



```text

FF DATA0 DATA1 DATA2 DATA3 CRC

```



L'octet `0xFF` sert à identifier le début d'une nouvelle trame.



Le CRC est calculé uniquement sur les quatre octets de données :



```text

DATA0 DATA1 DATA2 DATA3

```



L'octet de début `0xFF` n'est pas inclus dans le calcul du CRC.



\---



\# 3. Encodage des commandes



Les cinq commandes sont codées chacune sur \*\*6 bits\*\* :



| Champ    | Taille | Description      |

| -------- | -----: | ---------------- |

| `servo1` | 6 bits | Commande servo 1 |

| `servo2` | 6 bits | Commande servo 2 |

| `servo3` | 6 bits | Commande servo 3 |

| `servo4` | 6 bits | Commande servo 4 |

| `esc`    | 6 bits | Commande moteur  |



Les valeurs sont regroupées dans un entier `uint32\_t` :



```cpp

servo\_msg =  (servo1 \& 0x3F)

&#x20;        | ((servo2 \& 0x3F) << 6)

&#x20;        | ((servo3 \& 0x3F) << 12)

&#x20;        | ((servo4 \& 0x3F) << 18)

&#x20;        | ((esc    \& 0x3F) << 24);

```



La répartition est donc :



|     Bits | Champ      |

| -------: | ---------- |

|   `0..5` | `servo1`   |

|  `6..11` | `servo2`   |

| `12..17` | `servo3`   |

| `18..23` | `servo4`   |

| `24..29` | `esc`      |

| `30..31` | inutilisés |



Les valeurs transmises sont donc comprises entre :



```text

0 ... 63

```



pour chacun des cinq champs.



\---



\# 4. Répartition dans les quatre octets



Le `uint32\_t` est copié directement en mémoire dans les quatre octets de données :



```cpp

memcpy(\&buffer\_msg\[1], \&servo\_msg, 4);

```



Sur l'architecture utilisée, les données sont donc transmises en \*\*little-endian\*\*.



La répartition logique est :



```text

DATA0 = servo1\[5:0] | servo2\[1:0]

DATA1 = servo2\[5:2] | servo3\[3:0]

DATA2 = servo3\[5:4] | servo4\[5:0]

DATA3 = esc\[5:0]    | 00

```



Plus précisément :



| Octet   | Bits 7..6     | Bits 5..4     | Bits 3..2     | Bits 1..0     |

| ------- | ------------- | ------------- | ------------- | ------------- |

| `DATA0` | `servo2\[1:0]` | `servo1\[5:4]` | `servo1\[3:2]` | `servo1\[1:0]` |

| `DATA1` | `servo3\[3:2]` | `servo3\[1:0]` | `servo2\[5:4]` | `servo2\[3:2]` |

| `DATA2` | `servo4\[5:4]` | `servo4\[3:2]` | `servo4\[1:0]` | `servo3\[5:4]` |

| `DATA3` | `00`          | `esc\[5:4]`    | `esc\[3:2]`    | `esc\[1:0]`    |



La représentation la plus simple pour le décodage reste toutefois celle utilisée dans le code :



```cpp

servo1 =  servo\_msg        \& 0x3F;

servo2 = (servo\_msg >> 6)  \& 0x3F;

servo3 = (servo\_msg >> 12) \& 0x3F;

servo4 = (servo\_msg >> 18) \& 0x3F;

esc    = (servo\_msg >> 24) \& 0x3F;

```



\---



\# 5. Décodage côté carte moteurs



Après réception d'une trame valide, les quatre octets de données sont reconstruits en `uint32\_t` :



```cpp

memcpy(\&servo\_msg, buffer\_msg, 4);

```



Les cinq valeurs 6 bits sont ensuite extraites avec :



```cpp

servo1 =  servo\_msg        \& 0x3F;

servo2 = (servo\_msg >> 6)  \& 0x3F;

servo3 = (servo\_msg >> 12) \& 0x3F;

servo4 = (servo\_msg >> 18) \& 0x3F;

esc    = (servo\_msg >> 24) \& 0x3F;

```



\---



\# 6. Conversion des servomoteurs



La carte moteurs convertit les valeurs 6 bits en consignes servo.



La conversion générale est :



```text

servo\_angle = FACT\_SERVO × (commande - CENTRE\_SERVO) + 90°

```



avec :



```cpp

\#define FACT\_SERVO 2

```



La consigne est ensuite limitée par `MIN\_SERVO` et `MAX\_SERVO`.



Les centres actuellement définis sont :



| Canal    | Fonction   | Centre 6 bits | Consigne servo au centre |

| -------- | ---------- | ------------: | -----------------------: |

| `servo1` | profondeur |          `32` |                    `90°` |

| `servo2` | aile       |          `35` |                    `90°` |

| `servo3` | aile       |          `39` |                    `90°` |

| `servo4` | lacet      |          `28` |                    `90°` |



Les limites logicielles sont actuellement :



```text

0° ... 180°

```



pour les quatre servomoteurs.



Exemple pour `servo1` :



```cpp

servo1.write(

&#x20;   min(MAX\_SERVO1,

&#x20;       max(MIN\_SERVO1,

&#x20;           FACT\_SERVO \* (s1 - CENTRE\_SERVO1) + 90))

);

```



Avec `CENTRE\_SERVO1 = 32` et `FACT\_SERVO = 2` :



```text

s1 = 32  → 90°

s1 = 31  → 88°

s1 = 33  → 92°

```



La résolution de commande est donc de \*\*2° par unité 6 bits\*\* avant saturation.



\# 6.1. Convention des servomoteurs et des gouvernes



Les commandes des servomoteurs sont définies selon les conventions suivantes. Les valeurs de consigne servo croissantes correspondent aux mouvements indiqués ci-dessous :



| Servo    | Gouverne          | Consigne minimale (`0°`)       | Consigne maximale (`180°`)     |

| -------- | ----------------- | ------------------------------ | ------------------------------ |

| `servo1` | Profondeur        | Descente                       | Montée                         |

| `servo2` | Aile gauche       | Descente / frein → roll droite | Montée → roll gauche           |

| `servo3` | Aile droite       | Montée → roll droite           | Descente / frein → roll gauche |

| `servo4` | Direction / lacet | Virage gauche                  | Virage droite                  |



Les positions mécaniques de référence indiquées dans le code sont :



\* `servo1` : centre `32`, correspondant à la profondeur alignée avec le fuselage, légèrement orientée vers le haut (`+2°`).

\* `servo2` : centre `35`, correspondant à l'aile gauche alignée avec le plat de l'aile.

\* `servo3` : centre `39`, correspondant à l'aile droite alignée avec le plat de l'aile.

\* `servo4` : centre `28`, correspondant à la gouverne de lacet alignée avec le fuselage.



Ainsi, pour le \*\*roll\*\*, les deux gouvernes d'aile fonctionnent en sens opposés : une commande vers le roll gauche correspond à une montée de la gouverne gauche et à une descente/frein de la gouverne droite ; inversement, une commande vers le roll droite correspond à une descente/frein de la gouverne gauche et à une montée de la gouverne droite.



Pour le \*\*pitch\*\*, une augmentation de la commande `servo1` provoque une montée, tandis qu'une diminution provoque une descente. Pour le \*\*lacet\*\*, une augmentation de `servo4` correspond à un virage à droite et une diminution à un virage à gauche.



Les positions neutres sont écrites dans le code de la carte de contrôle des moteurs et doivent être adaptées aux positions réelles des gouvernes par rapport aux servomoteurs avant téléversement du code dans la carte de contrôle. 



\---



\# 7. Conversion de la commande ESC



La commande moteur est également codée sur 6 bits :



```text

esc = 0 ... 63

```



Elle est convertie en largeur d'impulsion ESC par :



```cpp

servo5.writeMicroseconds(

&#x20;   min(2000, e \* 16 + 1000)

);

```



La relation est donc :



```text

ESC = 1000 + 16 × esc   µs

```



avec une saturation à `2000 µs`.



| `esc` |     Consigne ESC |

| ----: | ---------------: |

|   `0` |        `1000 µs` |

|   `1` |        `1016 µs` |

|  `32` |        `1512 µs` |

|  `63` | `2008 → 2000 µs` |



La valeur maximale réellement appliquée est donc `2000 µs`.



\---



\# 8. CRC



Le CRC utilisé est un \*\*CRC-8 polynomial `0x07`\*\*, initialisé à `0x00`.



L'algorithme est :



```cpp

uint8\_t crc8(const uint8\_t\* data, size\_t len) {

&#x20;   uint8\_t crc = 0x00;



&#x20;   for (size\_t i = 0; i < len; i++) {

&#x20;       crc ^= data\[i];



&#x20;       for (uint8\_t b = 0; b < 8; b++) {

&#x20;           if (crc \& 0x80)

&#x20;               crc = (crc << 1) ^ 0x07;

&#x20;           else

&#x20;               crc <<= 1;

&#x20;       }

&#x20;   }



&#x20;   return crc;

}

```



Pour une trame moteur, le CRC est calculé sur :



```text

DATA0 DATA1 DATA2 DATA3

```



et placé dans le sixième octet :



```text

FF DATA0 DATA1 DATA2 DATA3 CRC

```



Une trame dont le CRC ne correspond pas au CRC calculé est rejetée.



\---



\# 9. Réception et resynchronisation



La carte moteurs utilise une machine d'état à trois états :



```text

WAIT\_START

READ\_DATA

READ\_CRC

```



\### `WAIT\_START`



La carte attend :



```text

0xFF

```



Tout octet différent de `0xFF` est ignoré.



\### `READ\_DATA`



Les quatre octets suivants sont enregistrés comme données.



\### `READ\_CRC`



Le cinquième octet suivant est interprété comme CRC.



Le CRC est vérifié avant toute modification des commandes moteurs.



Après traitement, la machine revient dans :



```text

WAIT\_START

```



\---



\# 10. Failsafe



La carte moteurs dispose d'un watchdog de communication :



```cpp

\#define TIMEOUT\_FAILSAFE 1000

```



Une trame valide doit donc être reçue au moins une fois toutes les \*\*1000 ms\*\*.



Si aucune trame valide n'est reçue pendant plus d'une seconde, la carte passe en failsafe.



Les positions appliquées sont alors :



| Actionneur | Consigne failsafe |

| ---------- | ----------------: |

| Servo 1    |            `106°` |

| Servo 2    |             `84°` |

| Servo 3    |             `76°` |

| Servo 4    |             `90°` |

| ESC        |         `1000 µs` |



La position du servo 1 correspond à une position de profondeur destinée au plané.



L'ESC est ramené à `1000 µs`.



Une erreur CRC provoque également l'activation du drapeau d'erreur de communication et le clignotement de la LED, mais \*\*ne déclenche pas directement le failsafe\*\*. Le failsafe est déclenché par l'absence prolongée de trame valide.



\---



\# 11. Initialisation



Au démarrage, la carte moteurs applique temporairement les positions suivantes :



```text

servo1 = 90°

servo2 = 84°

servo3 = 76°

servo4 = 90°

ESC    = 1000 µs

```



L'ESC reste à `1000 µs` pendant environ :



```text

3 s

```



afin de permettre son initialisation.



Après cette phase, la réception UART est activée à :



```text

19200 bauds

```



\---



\# 12. Matériel



La carte de contrôle est une carte arduino nano 5V munie d'un atmega 168p. 

Les pins sont : 

&#x20; servo1.attach(9);  // queue

&#x20; servo2.attach(5);  // aile gauche

&#x20; servo3.attach(6);  // aile droite

&#x20; servo4.attach(10); // non branché

&#x20; servo5.attach(3);  // ESC (hélice)



Les servomoteurs reçoivent directement leurs positions en degrés sous forme d'entier entre 0 et 180 degrés. 

Ces positions correspondent à l'angle formé par le bras du servo par rapport à sa référence. 

La référence est définie mécaniquement pour que la position neutre soit la plus proche possible de 90 degrés. 

Le biais sur la position neutre est ensuite appliquée dans le code via : 

\#define CENTRE\_SERVO1 32 // alignée fuselage légèrement vers le haut (+2degrés)

\#define CENTRE\_SERVO2 35 // aligné avec le plat de l'aile

\#define CENTRE\_SERVO3 39 // aligné avec le plat de l'aile

\#define CENTRE\_SERVO4 28 // queue lacet aligné avec le fuselage



\# 13. Résumé du protocole



| Élément                   | Valeur         |

| ------------------------- | -------------- |

| Interface                 | UART           |

| Débit                     | `19200 bauds`  |

| Longueur trame            | `6 octets`     |

| Start byte                | `0xFF`         |

| Données                   | `4 octets`     |

| Commandes                 | `5 × 6 bits`   |

| Valeurs de commande       | `0 ... 63`     |

| CRC                       | CRC-8          |

| Polynôme                  | `0x07`         |

| Initialisation CRC        | `0x00`         |

| Données couvertes par CRC | `DATA0..DATA3` |

| Timeout failsafe          | `1000 ms`      |



Format :



```text

FF | DATA0 | DATA1 | DATA2 | DATA3 | CRC

```



avec :



```text

DATA0..DATA3 = servo1\[5:0] + servo2\[5:0] + servo3\[5:0] + servo4\[5:0] + esc\[5:0]

```



Les valeurs 6 bits sont directement utilisées par la carte moteurs pour générer les consignes des quatre servomoteurs et de l'ESC.



