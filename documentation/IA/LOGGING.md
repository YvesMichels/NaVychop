\# Système de logs



\## 1. Objectif et architecture



Le système de logs de la centrale de navigation permet d'enregistrer les principales grandeurs nécessaires à l'analyse du comportement de l'aéronef : mesures inertielles, orientation estimée, altitude, commandes reçues du sol, états internes du contrôleur et commande moteur.



Le système repose sur trois mécanismes complémentaires :



\- \*\*Acquisition et mise en forme\*\* : les données sont converties en valeurs entières quantifiées et regroupées dans une structure `logState`.

\- \*\*Stockage local\*\* : les enregistrements peuvent être sauvegardés dans le système de fichiers LittleFS, dans le fichier `/logs.bin`.

\- \*\*Export CSV\*\* : les enregistrements stockés peuvent être relus et transmis sur l'UART pour analyse sur un ordinateur.



Une fonction distincte permet également de construire une trame binaire contenant deux quaternions et 16 valeurs de log. Dans le code actuel, cette trame est envoyée sur l'interface série ; son émission par LoRa est présente sous forme de code commenté.



\## 2. Structure d'un enregistrement



Chaque enregistrement est représenté par la structure C++ `logState`.



| Champ(s) | Type stocké | Contenu |

|---|---|---|

| `t` | `unsigned long` | Temps associé à l'enregistrement |

| `a\_x`, `a\_y`, `a\_z` | `int16\_t` | Accélérations selon les trois axes |

| `mag\_x`, `mag\_y`, `mag\_z` | `int16\_t` | Composantes du magnétomètre |

| `gqw`, `gqx`, `gqy`, `gqz` | `int16\_t` | Quaternion d'orientation \*Game Rotation\* |

| `altitude` | `int16\_t` | Altitude estimée |

| `cmd\_yaw`, `cmd\_pitch`, `cmd\_roll` | `int8\_t` | Commandes reçues du sol |

| `eta\_yaw`, `eta\_pitch`, `eta\_roll` | `int16\_t` | Intégrateurs du contrôleur |

| `proj\_yaw`, `proj\_pitch`, `proj\_roll` | `int16\_t` | Valeurs précédentes du contrôleur, enregistrées sous forme de projections |

| `ctrl\_yaw`, `ctrl\_pitch`, `ctrl\_roll` | `int8\_t` | Commandes calculées par le contrôleur |

| `esc` | `uint8\_t` | Commande ESC principale |



La structure contient donc 25 champs logiques, dont la représentation en mémoire dépend notamment de la taille de `unsigned long`, de l'alignement et du remplissage éventuel de la structure.



Le quaternion `ROTATION\_VECTOR` n'est pas stocké sous forme de quatre champs dédiés dans `logState`. Il est toutefois présent dans le format binaire de transmission décrit plus loin.



\## 3. Quantification des valeurs



Pour limiter la mémoire utilisée, plusieurs grandeurs flottantes sont converties en entiers signés 16 bits selon une représentation à virgule fixe.



Les facteurs utilisés dans `communication.cpp` sont :



| Format | Facteur d'échelle | Reconstruction |

|---|---:|---|

| Q4 | \\(2^4 = 16\\) | `valeur = raw / 16` |

| Q6 | \\(2^6 = 64\\) | `valeur = raw / 64` |

| Q14 | \\(2^{14} = 16384\\) | `valeur = raw / 16384` |



La quantification est réalisée par arrondi :



```cpp

raw = (int16\_t)roundf(value \* scale);

```



\### Répartition des formats



| Grandeurs | Format effectivement utilisé | Reconstruction |

|---|---|---|

| Accélérations `a\_x`, `a\_y`, `a\_z` | Q4 | `raw / 16` |

| Magnétomètre `mag\_x`, `mag\_y`, `mag\_z` | Q4 | `raw / 16` |

| Quaternion `gqw`, `gqx`, `gqy`, `gqz` | Q14 | `raw / 16384` |

| Altitude | Q6 | `raw / 64` |

| Intégrateurs `eta\_\*` | Q6 | `raw / 64` |

| Projections `proj\_\*` | Q14 | `raw / 16384` |

| Commandes sol `cmd\_\*` | Entier sans fraction | Valeur brute |

| Commandes contrôleur `ctrl\_\*` | Entier sans fraction | Valeur brute |

| ESC | Entier sans fraction | Valeur brute |



Les conversions en `int16\_t` sont réalisées sans saturation explicite préalable, à l'exception de certaines fonctions de quantification distinctes. Il faut donc vérifier que les valeurs restent dans la plage représentable par ce type.



\## 4. Constitution des enregistrements



La fonction `addLog()` remplit un enregistrement à partir :

\- du temps `t` ;

\- des trois composantes d'accélération ;

\- des trois composantes du magnétomètre ;

\- du quaternion `q\_game` ;

\- de l'état de navigation `att` ;

\- des commandes du sol ;

\- de l'état interne du contrôleur `ctrl` ;

\- de la commande ESC.



Les grandeurs flottantes sont quantifiées selon les facteurs décrits précédemment. Les commandes et la valeur ESC sont copiées dans leurs champs respectifs.



L'index du tampon est ensuite incrémenté. Lorsque `logIndex` atteint `LOG\_SIZE`, la fonction le borne à `LOG\_SIZE - 1` et renvoie `1`. Dans les autres cas, elle renvoie `0`.



La fréquence d'appel de `addLog()`, la politique de réutilisation du tampon et le comportement global lorsque celui-ci est plein dépendent du code appelant et ne sont pas déterminables à partir des seules fonctions de communication.



\## 5. Stockage en mémoire flash



La fonction `saveLogsToFlash()` écrit un tampon d'enregistrements dans le système de fichiers LittleFS.



Le fichier utilisé est :



```text

/logs.bin

```



Le fonctionnement est le suivant :



1\. Si `packetIndex == 0`, le fichier existant est supprimé.

2\. Le fichier est ouvert en mode ajout (`FILE\_APPEND`).

3\. La fonction écrit `LOG\_SIZE` enregistrements consécutifs, soit `LOG\_SIZE \* sizeof(logState)` octets.

4\. Le fichier est fermé.

5\. La taille écrite est comparée à la taille attendue ; une erreur est signalée sur l'interface série si l'ouverture ou l'écriture échoue.



Le format stocké est \*\*binaire brut\*\* : la mémoire de la structure `logState` est écrite directement, sans en-tête, version de format ni conversion explicite des champs.



Cette approche est compacte, mais elle implique que la relecture nécessite une structure compatible avec celle utilisée lors de l'écriture : mêmes types, mêmes tailles et même disposition mémoire. Le fichier n'est donc pas un format d'archivage portable garanti entre architectures ou versions différentes du firmware.



La fonction ne vérifie pas le résultat de chaque suppression ou fermeture, et ne met pas en place de mécanisme de reprise après une écriture partielle.



\## 6. Export CSV



La fonction `sendLogsCSVFromFlash()` permet d'exporter les enregistrements sauvegardés.



Elle ouvre `/logs.bin` en lecture, transmet une ligne d'en-tête CSV, puis lit successivement les enregistrements complets de taille `sizeof(logState)`. Chaque enregistrement est converti en texte par `sendLogCSV()`.



Les colonnes sont émises dans l'ordre suivant :



```text

t,

a\_x,a\_y,a\_z,

mag\_x,mag\_y,mag\_z,

gqw,gqx,gqy,gqz,

altitude,

cmd\_yaw,cmd\_pitch,cmd\_roll,

eta\_yaw,eta\_pitch,eta\_roll,

proj\_yaw,proj\_pitch,proj\_roll,

ctrl\_yaw,ctrl\_pitch,ctrl\_roll,

esc

```



L'en-tête réellement transmis est une seule ligne, sans retour à la ligne entre les groupes de colonnes.



Les champs quantifiés sont reconvertis en valeurs numériques selon leur facteur d'échelle. Les commandes sont exportées directement sous forme d'entiers.



Les précisions d'affichage actuelles sont :



| Grandeurs | Nombre de décimales |

|---|---:|

| Accélérations | 4 |

| Magnétomètre | 6 |

| Quaternion `Game Rotation` | 6 |

| Altitude | 3 |

| Intégrateurs | 4 |

| Projections | 6 |

| Commandes et ESC | 0 |



Le séparateur CSV est la virgule. Chaque enregistrement est terminé par un retour à la ligne.



L'export est transmis sur `Serial`. Le débit configuré pour cette interface est de 115200 baud sur le port USB pour permettre la lecture depuis un PC.



\## 7. Déclenchement de l'export CSV



La fonction `checkUartAndTriggerDump()` surveille les octets disponibles sur le port `Serial USB` pour permettre un déclenchement depuis le PC. Elle recherche successivement les quatre octets de synchronisation définis dans la configuration :



\- SYNC\_1 = 0xAA             // octet de synchronisation 1

\- SYNC\_2 = 0x55             // octet de synchronisation 2

\- SYNC\_3 = 0xCC             // octet de synchronisation 3

\- SYNC\_4 = 0x33             // octet de synchronisation 4

Lorsque cette séquence complète est reconnue, la fonction appelle `sendLogsCSVFromFlash()`.



Ce mécanisme constitue une commande de déclenchement par séquence d'octets, et non une commande textuelle. Les valeurs exactes de synchronisation sont définies dans `config.h`.



\*\*Point de vigilance :\*\* l'argument `sync\_state` est passé par valeur. Son état n'est donc pas conservé automatiquement entre deux appels à `checkUartAndTriggerDump()`. La séquence doit être reçue dans un même appel pour être reconnue de manière fiable, sauf si le code appelant assure autrement la persistance de cet état. Pour permettre une réception répartie sur plusieurs appels, il faudrait conserver l'état dans une variable persistante ou le passer par référence.



Le paramètre `buffer` de cette fonction n'est pas utilisé dans son implémentation actuelle.



\## 8. Trame binaire de transmission des quaternions et des logs



La fonction `send\_quaternions\_and\_logs()` construit une trame binaire de \*\*54 octets\*\* et la transmet avec `Serial.write()`.



Le format est le suivant :



| Champ | Taille | Contenu |

|---|---:|---|

| Synchronisation | 4 octets | `SYNC\_1` à `SYNC\_4` |

| Format | 1 octet | `QUATERNION\_FORMAT` |

| Quaternion `GAME\_ROTATION\_VECTOR` | 8 octets | 4 composantes signées sur 16 bits |

| Quaternion `ROTATION\_VECTOR` | 8 octets | 4 composantes signées sur 16 bits |

| Logs | 32 octets | 16 valeurs signées sur 16 bits |

| Checksum | 1 octet | XOR de tous les octets précédents |



Les champs occupent donc 54 octets au total.

Cette fonction est historique et n'est plus utilisée dans les architectures récentes. 



\### 8.1 Codage des quaternions



Chaque composante est convertie en entier signé 16 bits par :



```cpp

int16\_t v = (int16\_t)(x \* 32767.0f);

```



La valeur d'entrée est bornée à l'intervalle :



```text

\[-1.0, 0.999984]

```



Chaque entier est transmis sur deux octets, LSB en premier.



Ce codage est distinct du format Q14 utilisé dans les enregistrements `logState` : il s'agit ici d'une représentation signée normalisée basée sur un facteur de 32767.



\### 8.2 Codage des logs transmis



Le champ de logs contient exactement 16 entiers `int16\_t`, fournis par le tableau `logs`.



Chaque entier est transmis sur deux octets, LSB en premier. Le sens des 16 indices n'est pas défini par `send\_quaternions\_and\_logs()` : il dépend de la convention établie par le code appelant.



Il ne faut donc pas assimiler automatiquement ces 16 valeurs au contenu complet de `logState`.



\### 8.3 Checksum



Le dernier octet est calculé par XOR successif de tous les octets précédents, synchronisation et champ de format compris.



Ce mécanisme est un checksum XOR sur 8 bits ; il est différent du CRC-8 polynomial `0x07` utilisé par les autres protocoles de communication.



\### 8.4 Transport



La transmission active est :



```cpp

Serial.write(buffer, sizeof(buffer));

```



Les appels `LoRa.beginPacket()`, `LoRa.write()` et `LoRa.endPacket()` sont commentés dans cette fonction. Le code fourni ne réalise donc pas actuellement l'envoi de cette trame de 54 octets par LoRa via cette fonction.



\## 9. Autres fonctions de communication



Le fichier contient également `lora\_check\_and\_echo()`, qui vérifie si un paquet LoRa contient exactement deux `uint32\_t`, soit 8 octets. Dans ce cas, le paquet est lu puis réémis immédiatement en mode asynchrone.



Cette fonction constitue un mécanisme d'écho de paquet. Elle ne réalise pas la journalisation des données de vol décrite dans les sections précédentes.

Cette fonction fut utilisée pour valider la communication LoRa maintenant bien établie. Les vérifications actuelles et futurs se font via la vérification des fonctions qui dépendent de la communication LoRa. 

Cette fonction est historique et n'est plus utilisée dans les architectures récentes.



\## 10. Boucle principale et cadence d'enregistrement



L'acquisition des logs est exécutée dans la boucle principale de `main.cpp`, à une cadence cible définie par `d\_t = 40 ms`, soit \*\*25 enregistrements par seconde\*\* lorsque la boucle respecte sa période nominale. Tant que `save\_en == LOG\_RECORDING`, chaque itération appelle `addLog()` pour enregistrer les mesures inertielles, le champ magnétique, le quaternion d'orientation, l'altitude, les commandes reçues du sol, les états internes de l'asservissement et la commande moteur. Le temps associé à chaque enregistrement est le temps écoulé depuis l'initialisation (`t\_cur - t\_0`).



Les enregistrements sont accumulés dans `logBuffer`, dimensionné par `LOG\_SIZE = 50`. Lorsqu'il est plein, le bloc complet est écrit dans `/logs.bin` via `saveLogsToFlash()`, l'index du paquet flash (`flash\_logIndex`) est incrémenté et l'index du buffer est réinitialisé. À la cadence cible, chaque bloc représente environ \*\*2 secondes de données\*\*. Le premier bloc d'une session supprime le fichier existant ; les blocs suivants sont ajoutés à sa suite. L'écriture flash est donc effectuée par blocs de 50 enregistrements, et non à chaque itération de la boucle.



\## 11. Déclenchement de l'enregistrement et de la sauvegarde par LoRa



Le pilotage de l'enregistrement passe par des trames LoRa de type `LORA\_FRAME\_SAVE = 0x02`. Après validation de la trame reçue, `lora\_cmd\_set()` interprète son premier octet de données (`f\[1]`) comme une demande de commande : \*\*`1` déclenche l'enregistrement\*\* (`LOG\_RECORDING`) et \*\*`2` demande son arrêt\*\* (`LOG\_STOPPING`). Seules ces deux valeurs sont acceptées. La demande est mémorisée dans `CmdState`, puis consommée dans la boucle principale via `save\_pending`, qui met à jour `save\_en`.



À l'arrêt, l'enregistrement ne passe à `LOG\_IDLE` que lorsque `logIndex == 0`, c'est-à-dire à la frontière d'un bloc complet. Si la demande d'arrêt intervient au milieu d'un bloc, l'acquisition continue jusqu'à ce que les 50 enregistrements soient collectés et écrits. \*\*Le bloc partiellement rempli n'est donc pas sauvegardé séparément\*\* : l'arrêt peut attendre jusqu'à environ 2 secondes à la cadence nominale. Le premier bloc d'une nouvelle session supprime l'ancien `/logs.bin` lorsque `flash\_logIndex == 0`. Enfin, cette commande LoRa pilote l'acquisition et l'écriture en flash ; l'export CSV est déclenché séparément par la séquence de synchronisation reçue sur le port série `Serial`.

