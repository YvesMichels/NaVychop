# Conventions de repères, matrices de rotation et asservissement

## 1. Objectif

Cette documentation définit les conventions utilisées dans la navigation de **NavVychop** pour :

- les repères avion et IMU ;
- les matrices de rotation ;
- le calcul de l'attitude de l'avion à partir du BNO085 ;
- le calcul des erreurs de tangage et de roulis ;
- l'asservissement des axes par régulateur PID ;
- les conventions de signe associées aux commandes.

L'objectif principal est d'éviter les ambiguïtés lors de l'évolution du logiciel ou du remplacement du montage mécanique du BNO085.

---

# 2. Repères utilisés

Deux repères principaux sont utilisés.

## 2.1. Repère avion `A`

Le repère avion est défini par :

| Axe | Direction |
|---|---|
| `X_A` | vers l'avant de l'avion |
| `Y_A` | vers la gauche de l'avion |
| `Z_A` | vers le haut |

Il s'agit donc d'un repère orthonormé lié mécaniquement à l'avion.

Les trois axes vérifient :

```text
X_A : avant
Y_A : gauche
Z_A : haut
```

Le repère est utilisé comme référence pour les commandes de vol.

---

## 2.2. Repère IMU `M`

Le BNO085 est installé dans l'avion avec une orientation différente de celle du repère avion.

Dans le montage actuel :

| Axe IMU | Direction physique |
|---|---|
| `X_M` | vers la gauche de l'avion |
| `Y_M` | vers l'arrière de l'avion |
| `Z_M` | vers le haut |

On a donc directement les correspondances :

```text
X_M = Y_A
Y_M = -X_A
Z_M = Z_A
```

ou, réciproquement :

```text
X_A = -Y_M
Y_A =  X_M
Z_A =  Z_M
```

Cette convention est **dépendante du montage physique du BNO085**.

Toute modification de l'orientation de la carte IMU doit donc entraîner une modification de `R_M_A`.

---

# 3. Matrices de rotation

## 3.1. Convention générale

Les matrices utilisées dans le projet suivent la convention :

> **Les colonnes d'une matrice de rotation contiennent les axes du repère source exprimés dans le repère destination.**

Par exemple :

```text
R_G_M
```

est la rotation du repère IMU `M` vers le repère terrestre `G`.

Ses colonnes sont :

```text
colonne 0 = X_M exprimé dans G
colonne 1 = Y_M exprimé dans G
colonne 2 = Z_M exprimé dans G
```

De manière générale, pour une matrice `R_B_A` :

```text
R_B_A
```

signifie :

> rotation du repère `A` vers le repère `B`.

et :

```text
R_B_A.colonne(i) = axe_i_de_A exprimé dans B
```

---

# 4. Matrice fixe IMU → avion

Le montage mécanique du BNO085 est décrit par :

```cpp
float R_M_A[3][3] = {
    {  0.0f,  1.0f,  0.0f },  // X_M
    { -1.0f,  0.0f,  0.0f },  // Y_M
    {  0.0f,  0.0f,  1.0f }   // Z_M
};
```

Attention : malgré les commentaires présents dans le code, il faut interpréter cette matrice suivant la convention définie ci-dessus.

Les **colonnes** sont :

```text
colonne 0 = ( 0, -1,  0) = X_A exprimé dans M
colonne 1 = ( 1,  0,  0) = Y_A exprimé dans M
colonne 2 = ( 0,  0,  1) = Z_A exprimé dans M
```

On retrouve donc :

```text
X_A = -Y_M
Y_A =  X_M
Z_A =  Z_M
```

ce qui correspond bien au montage physique :

```text
             X_A : avant
                  ↑
                  │
             -Y_M │
                  │
      Y_A ←───────┼──────→ -Y_A
      gauche      │
                  │
                  ↓
             Y_M : arrière
```

avec :

```text
Z_A = Z_M
```

---

# 5. Matrice IMU → terre

Le BNO085 fournit une attitude sous la forme d'un quaternion.

Le code convertit ce quaternion en matrice :

```cpp
computeRotationMatrix(
    q_M_GRV.w,
    q_M_GRV.x,
    q_M_GRV.y,
    q_M_GRV.z,
    R_G_M
);
```

`R_G_M` représente l'orientation de l'IMU dans le repère terrestre.

Ses colonnes représentent :

```text
R_G_M[:,0] = X_M exprimé dans G
R_G_M[:,1] = Y_M exprimé dans G
R_G_M[:,2] = Z_M exprimé dans G
```

Autrement dit, si :

```text
R_G_M =
[ ... ... ... ]
[ ... ... ... ]
[ ... ... ... ]
```

alors :

```text
X_M dans G = (R_G_M[0][0], R_G_M[1][0], R_G_M[2][0])

Y_M dans G = (R_G_M[0][1], R_G_M[1][1], R_G_M[2][1])

Z_M dans G = (R_G_M[0][2], R_G_M[1][2], R_G_M[2][2])
```

---

# 6. Calcul de l'attitude de l'avion

L'attitude recherchée est celle de l'avion dans le repère terrestre :

```text
R_G_A
```

Elle est obtenue par composition :

```cpp
rotMult(R_G_A, R_G_M, R_M_A);
```

mathématiquement :

```text
R_G_A = R_G_M × R_M_A
```

Cette opération effectue :

```text
A → M → G
```

donc :

```text
A --R_M_A--> M --R_G_M--> G
```

La matrice obtenue possède donc les axes de l'avion exprimés dans le repère terrestre :

```text
R_G_A[:,0] = X_A exprimé dans G
R_G_A[:,1] = Y_A exprimé dans G
R_G_A[:,2] = Z_A exprimé dans G
```

C'est cette propriété qui est exploitée directement par l'asservissement.

---

# 7. Lecture physique de `R_G_A`

La première colonne :

```cpp
R_G_A[0][0]
R_G_A[1][0]
R_G_A[2][0]
```

représente l'axe longitudinal de l'avion `X_A` dans le repère terrestre.

On peut donc l'interpréter comme :

```text
X_A = (X_G, Y_G, Z_G)
```

avec :

```text
X_G = R_G_A[0][0]
Y_G = R_G_A[1][0]
Z_G = R_G_A[2][0]
```

De même, la deuxième colonne représente l'axe latéral :

```text
Y_A = (R_G_A[0][1],
       R_G_A[1][1],
       R_G_A[2][1])
```

---

# 8. Calcul du signal d'asservissement

L'asservissement ne travaille pas directement avec des angles d'Euler.

Il utilise les projections des axes de l'avion sur le plan vertical.

Cette approche évite notamment d'avoir à calculer explicitement un `pitch` ou un `roll` avec des fonctions trigonométriques inverses.

Le signal obtenu est une mesure signée de l'écart entre :

```text
attitude mesurée
```

et :

```text
attitude demandée
```

Le signal est ensuite utilisé directement par le régulateur.

---

# 9. Asservissement en tangage

Le tangage est déterminé à partir de l'axe longitudinal :

```text
X_A
```

qui correspond à la première colonne de `R_G_A`.

## 9.1. Projection

Le code calcule :

```cpp
float norm = sqrtf(
    R[0][0]*R[0][0] +
    R[1][0]*R[1][0]
);
```

Cette quantité correspond à la norme de la projection horizontale de `X_A`.

Si `X_A` est unitaire :

```text
norm = cos(pitch)
```

Le code définit ensuite :

```cpp
float y = norm;
float z = -R[2][0];
```

Le signe de `z` est volontairement inversé afin de conserver la convention de signe historique du système :

```text
z = -Z_A
```

---

## 9.2. Consigne

Pour une consigne `pitch_cmd` :

```cpp
float y_target = cosf(pitch_cmd);
float z_target = sinf(pitch_cmd);
```

La mesure et la consigne sont donc représentées comme deux vecteurs 2D :

```text
mesure  = (cos(pitch), -sin(pitch))
cible   = (cos(pitch_cmd), sin(pitch_cmd))
```

Le signal d'erreur est calculé à partir du produit vectoriel scalaire 2D :

```cpp
cross = y * z_target - z * y_target;
```

et du produit scalaire :

```cpp
dot = y * y_target + z * z_target;
```

Le `cross` fournit le signe et l'amplitude locale de l'écart.

Le `dot` permet notamment de gérer le cas où les deux directions sont opposées.

---

## 9.3. Signal de commande

Le résultat :

```cpp
computePitchCommandSignal(R, pitch_cmd)
```

est compris approximativement dans :

```text
[-1 ; +1]
```

dans la zone normale de fonctionnement.

Il représente donc un **signal d'erreur de tangage normalisé**, et non directement une commande moteur.

---

# 10. Asservissement en roulis

Le roulis est déterminé à partir de l'axe latéral :

```text
Y_A
```

c'est-à-dire la deuxième colonne de `R_G_A`.

Le code utilise :

```cpp
float norm = sqrtf(
    R[0][1]*R[0][1] +
    R[1][1]*R[1][1]
);
```

Cette norme correspond à la projection horizontale de `Y_A`.

Pour un axe unitaire :

```text
norm = cos(roll)
```

Le code définit :

```cpp
float x = norm;
float z = R[2][1];
```

Le signal vertical est donc directement :

```text
Z_A appliqué à Y_A
```

---

## 10.1. Consigne de roulis

La consigne est transformée de la même manière :

```cpp
float x_target = cosf(roll_cmd);
float z_target = sinf(roll_cmd);
```

Le signal d'erreur est :

```cpp
cross = x * z_target - z * x_target;
```

avec :

```cpp
dot = x * x_target + z * z_target;
```

Le `cross` donne le sens de correction tandis que le `dot` permet de traiter le cas où la mesure et la consigne sont opposées.

---

# 11. Convention de signe du roulis

Dans le code actuel :

```cpp
z = R[2][1];
```

Or :

```text
R_G_A[:,1] = Y_A exprimé dans G
```

Ainsi :

```text
R_G_A[2][1]
```

est la composante verticale de l'axe gauche de l'avion.

Il faut donc retenir que le signe du roulis est défini par **l'élévation de l'aile gauche**, et non par une convention d'angle d'Euler externe.

Pour éviter toute ambiguïté lors d'une modification du code, il est préférable de parler de :

> **roulis positif selon la convention du signal `computeRollCommandSignal()`**

plutôt que de supposer qu'il correspond automatiquement à la convention mathématique classique de roulis.

---

# 12. Architecture générale de l'asservissement

La chaîne complète est :

```text
              BNO085
                 │
                 │ quaternion
                 ▼
             R_G_M
                 │
                 │ × R_M_A
                 ▼
             R_G_A
                 │
          ┌──────┴──────┐
          │             │
          ▼             ▼
     axe X_A         axe Y_A
          │             │
          ▼             ▼
   erreur pitch     erreur roll
          │             │
          └──────┬──────┘
                 │
                 ▼
              PID
                 │
                 ▼
          commande avion
```

Pour le tangage :

```text
R_G_A[:,0]
    │
    ▼
computePitchCommandSignal()
    │
    ▼
proj_pitch
```

Pour le roulis :

```text
R_G_A[:,1]
    │
    ▼
computeRollCommandSignal()
    │
    ▼
proj_roll
```

Le lacet utilise une fonction séparée :

```cpp
computeYawCommandSignalImproved()
```

qui n'est pas détaillée dans cette documentation car son implémentation n'est pas utilisée.

---

# 13. Régulateur PID

La fonction :

```cpp
control_update_inner()
```

effectue ensuite la régulation des trois axes.

Pour chaque axe, on dispose d'un signal d'erreur :

```text
proj_yaw
proj_pitch
proj_roll
```

---

## 13.1. Intégrale

L'intégrale est mise à jour sous la forme :

```cpp
state.eta_pitch += proj_pitch * d_t;
```

et de manière équivalente pour le lacet et le roulis.

L'état intégral est ensuite limité :

```cpp
bound_integrator(
    state.eta_pitch,
    def_max_eta_pitch
);
```

Il existe donc une protection contre le **wind-up intégral**.

La même logique est appliquée aux trois axes :

```text
eta_yaw
eta_pitch
eta_roll
```

---

# 14. Action proportionnelle et intégrale

La commande est calculée selon :

```cpp
state.ctrl_pitch =
      proj_pitch * gains.Kp_pitch
    + state.eta_pitch * gains.Ki_pitch
    + gains.ctrl_bias_pitch;
```

soit mathématiquement :

```text
u(t) = Kp × e(t)
     + Ki × ∫e(t)dt
     + bias
```

Le terme `ctrl_bias_*` permet de compenser une dissymétrie ou un offset permanent du système.

---

# 15. Terme dérivé

Le terme dérivé est calculé à partir de la variation du signal d'erreur :

```cpp
(proj_pitch - state.old_pitch) / d_t
```

puis multiplié par :

```cpp
gains.Kd_pitch
```

Le régulateur est donc de type :

```text
PID = P + I + D
```

avec :

```text
P = Kp × erreur

I = Ki × intégrale(erreur)

D = Kd × dérivée(erreur)
```

Le terme dérivé porte ici sur le **signal d'erreur projeté**, et non directement sur la commande.

---

# 16. Initialisation du terme dérivé

Le code contient :

```cpp
if(state.init == 0){
    ...
    (proj_pitch - state.old_pitch) * gains.Kd_pitch / d_t;
}
else{
    state.init = 0;
}
```

Cela permet de supprimer le terme dérivé lors du premier calcul, lorsque `old_pitch` n'est pas encore représentatif d'une mesure précédente.

La séquence est donc :

```text
premier appel
    │
    └── pas de dérivée

appels suivants
    │
    └── dérivée active
```

---

# 17. Saturation de la commande

Après calcul du PID, chaque commande est limitée :

```cpp
state.ctrl_pitch =
    min(32.0f, max(-31.0f, state.ctrl_pitch));
```

Même principe pour :

```text
ctrl_yaw
ctrl_pitch
ctrl_roll
```

La plage de sortie est donc :

```text
-31 ≤ commande ≤ +32
```

Cette limitation intervient **après** le calcul du PID.

Elle est distincte de la limitation de l'intégrateur.

---

# 18. Limitation de la consigne de roulis

Avant le calcul de l'erreur de roulis :

```cpp
bound_integrator(cmd_roll, 0.4);
```

la consigne de roulis est limitée à :

```text
|roll_cmd| ≤ 0.4 rad
```

soit environ :

```text
±22,9°
```

Cette limitation est une limitation de **consigne**, et non de sortie du PID.

---

# 19. Résumé des conventions

## Repère avion

```text
X_A → avant
Y_A → gauche
Z_A → haut
```

## Repère IMU

```text
X_M → gauche
Y_M → arrière
Z_M → haut
```

## Correspondance des axes

```text
X_A = -Y_M
Y_A =  X_M
Z_A =  Z_M
```

## Matrices

```text
R_G_M
```

orientation de l'IMU `M` vers le terrestre `G`.

```text
R_M_A
```

orientation de l'avion `A` vers l'IMU `M`, définie par le montage mécanique.

```text
R_G_A = R_G_M × R_M_A
```

orientation de l'avion `A` vers le terrestre `G`.

Les colonnes de `R_G_A` sont :

```text
colonne 0 → X_A dans G
colonne 1 → Y_A dans G
colonne 2 → Z_A dans G
```

---

# 20. Règle importante pour toute modification du montage IMU

Si le BNO085 est physiquement déplacé ou retourné, **ne pas modifier directement les fonctions de calcul du pitch/roll** pour compenser le nouveau montage.

La modification doit normalement être concentrée dans :

```cpp
R_M_A
```

Le reste de la chaîne conserve alors la même convention :

```text
BNO085
  │
  ▼
R_G_M
  │
  × R_M_A
  ▼
R_G_A
  │
  ├── X_A → pitch
  └── Y_A → roll
```

Cela permet de séparer clairement :

1. **l'orientation physique de l'IMU** (`R_M_A`) ;
2. **l'attitude mesurée par l'IMU** (`R_G_M`) ;
3. **l'attitude de l'avion** (`R_G_A`) ;
4. **la logique d'asservissement** (pitch/roll/yaw).

Cette séparation est essentielle pour éviter que des corrections liées au montage mécanique ne contaminent la logique de navigation ou de contrôle.