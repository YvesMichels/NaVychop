\# Estimation de l'altitude (BMP390)



\## 1. Principe



L'altitude est une \*\*altitude relative\*\* au point de départ, calculée à partir de la pression mesurée par le baromètre BMP390 (oversampling pression ×4, température ×2), à la cadence de la boucle principale.



Formule (atmosphère standard) :



```text

altitude\_raw = 44330 × (1 − (P / P0)^0,1903)

altitude     = α\_alt × (altitude\_raw − alt\_bias) + (1 − α\_alt) × altitude\_précédente

```



\- `P` : pression mesurée (Pa)

\- `P0` : pression de référence obtenue à la calibration (Pa)

\- `alt\_bias` : biais de correction, maintenu à 0 par la calibration actuelle

\- `α\_alt` = 0,1 : coefficient du filtre passe-bas



\## 2. Calibration



\### Fonctionnement



| Élément | Comportement |

|---|---|

| Déclenchement | `cal\_en = 1` alors qu'aucune calibration n'est en cours |

| Pendant la calibration | `cal\_en` est ignoré, l'altitude vaut 0 |

| Durée | `tps\_cal / d\_t` échantillons (1000 ms, 25 échantillons si la période est de 40ms) |

| Résultat | `P0` = moyenne exacte des pressions mesurées pendant la calibration |

| Fin | `cal\_active` repasse à 0, le filtre repart de 0 sans saut |

| Avant toute calibration | `P0` = 0, l'altitude reste à 0 |



La calibration est suivie uniquement par les champs de `AttitudeState` : `cal\_active`, `cal\_counter`, `cal\_samples`, `p0`.



\### Calibration automatique



Une calibration est lancée \*\*3 s après le début de la boucle principale\*\* (hors `setup()`), une seule fois (drapeau `alt\_cal\_launched`). La commande par LoRa viendra s'ajouter plus tard.



\### Précautions



\- L'avion doit rester \*\*immobile et au sol\*\* pendant les 1 s de calibration.

\- Une lecture BMP échouée n'avance pas le compteur : la calibration dure alors un peu plus longtemps (elle compte des échantillons, pas du temps).

\- Une pression invalide (≤ 0) est ignorée.



\## 3. Précision théorique



Près du sol, \*\*1 Pa ≈ 8,3 cm\*\* d'altitude.



L'objectif étant la \*\*variation\*\* d'altitude par rapport au départ, l'offset absolu du capteur (de l'ordre de ±50 Pa, soit ±4 m) est éliminé par la calibration.



\### Budget d'erreur (ordres de grandeur, à confirmer avec la datasheet Bosch)



| Source | Ordre de grandeur |

|---|---|

| Bruit du capteur (oversampling ×4) | 0,5 à 1 Pa RMS, soit 4 à 8 cm, réduit par le filtre |

| Erreur de calibration (moyenne sur 25 échantillons) | < 1 cm |

| Précision relative du capteur | environ ±0,08 hPa, soit ±0,7 m au pire |

| Dérive thermique du capteur | environ 0,75 Pa/K, soit 6 cm/K |

| Dérive météo | environ 0,3 hPa/h, soit < 1 m sur un vol de 15 min |

| Arrondis float32 | de l'ordre du millimètre |



\### Erreur du modèle atmosphérique



La formule suppose une atmosphère standard à 15 °C. L'erreur est \*\*proportionnelle à l'amplitude\*\* de la variation d'altitude : environ \*\*0,35 % par °C\*\* d'écart de température.



Exemple à 30 °C au sol : environ 1,5 m d'erreur pour 30 m de montée, environ 5 m pour 100 m.

5% d'erreurs est acceptable pour l'asservissement de l'altitude.



Le paramètre `temperature\_c` est reçu mais non utilisé. Une correction possible est de multiplier l'altitude par `T\_sol\_K / 288,15`.



\### Facteur dominant : la pression dynamique



Le capteur doit mesurer la pression \*\*statique\*\*. À 20 m/s, la pression dynamique atteint environ 240 Pa, soit environ \*\*20 m\*\* d'erreur si le capteur est exposé au vent relatif. Même 10 % de cette valeur donne environ 2 m, variables avec la vitesse, l'incidence et le souffle de l'hélice.



Recommandations de montage :



\- placer le capteur dans une cavité fermée avec un petit évent vers une zone de pression statique ;

\- le mettre à l'abri du souffle de l'hélice ;

\- éviter l'exposition directe à l'air du fuselage.



\### Bilan



\- \*\*Montage correct\*\* : environ ±0,3 m (bruit et dérive), plus 1 à 5 % de l'amplitude due au modèle atmosphérique.

\- \*\*Capteur exposé à l'écoulement\*\* : plusieurs mètres d'erreur, variables avec la vitesse.

