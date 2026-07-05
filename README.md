# Simulation de Guidage 3D — Navigation Proportionnelle (ProNav) + EKF

Simulation pédagogique d'un système de guidage-poursuite en 3D, combinant une **loi de navigation proportionnelle (ProNav)** découplée azimut/élévation et un **filtre de Kalman étendu (EKF)** à 12 états pour l'estimation de la trajectoire d'une cible mobile.

Implémentation en 🐍 **Python** (`simulation_3d.py`) — NumPy + Matplotlib.

> ⚠️ **Cadre pédagogique.** Ce dépôt a une vocation purement didactique (cours d'automatique, de fusion de données ou de filtrage de Kalman). Le modèle aérodynamique est volontairement simplifié (traînée quadratique, pas de portance, pas de modèle de propulsion réaliste) et ne constitue en aucun cas un outil de conception d'armement.

---

## Aperçu

![Aperçu de la simulation](docs/simulation_3d.png)

Chaque exécution génère un scénario différent (position, vitesse et manœuvres de la cible tirées aléatoirement) et produit :
- une trajectoire 3D interactive (missile, cible réelle, cible estimée par l'EKF) ;
- le profil de vitesse du poursuivant ;
- l'évolution du cap (lacet, format boussole) ;
- l'évolution de la pente (tangage) ;
- le facteur de charge latéral subi, comparé à la limite structurelle.

---

## Fonctionnalités

- **Guidage 3D découplé** : la loi ProNav calcule indépendamment la vitesse de rotation de la ligne de visée en azimut et en élévation, chacune pondérée par un gain `N`.
- **EKF 12 états** : `[xs, ys, zs, ψ, γ, vs, xt, yt, zt, vxt, vyt, vzt]` — position, cap, pente et vitesse du poursuivant, position et vitesse de la cible.
- **Capteur bearing-only 2 axes** : le senseur IR ne fournit que deux angles relatifs (azimut, élévation) bruités, pas de mesure de distance.
- **Contraintes physiques réalistes** :
  - limite de facteur de charge (G-limit) combinant lacet et tangage ;
  - limite de vitesse angulaire (slew rate) des gouvernes ;
  - traînée aérodynamique quadratique et effet de la gravité sur l'axe de vol.
- **Événements de scénario aléatoires** (activés ou non à chaque run) :
  - perte de signal / brouillage temporaire du senseur ;
  - manœuvre d'esquive brutale (rupture horizontale + verticale) ;
  - manœuvre en zigzag périodique.
- **Reproductibilité** : la graine aléatoire (`seed`) utilisée est affichée en console à chaque exécution.

---

## Structure du dépôt

```
.
├── simulation_3d.py     # Implémentation Python (NumPy + Matplotlib)
├── docs/
│   └── simulation_3d.png  # Exemple de sortie graphique
└── README.md
```

---

## Prérequis

- Python ≥ 3.9
- [NumPy](https://numpy.org/)
- [Matplotlib](https://matplotlib.org/)

```bash
pip install numpy matplotlib
```

---

## Utilisation

```bash
python3 simulation_3d.py
```

La figure s'affiche à l'écran et est sauvegardée sous `simulation_3d.png` dans le dossier courant.

---

## Principe du modèle

### Loi de guidage
La navigation proportionnelle commande un taux de virage proportionnel à la vitesse de rotation de la ligne de visée (LOS) :

```
ω_commandé = N × d(LOS)/dt
```

appliquée séparément sur les canaux azimut (lacet) et élévation (tangage), avec un gain `N_gain` (par défaut 4.0).

### Filtre de Kalman étendu
L'EKF prédit l'évolution du système à partir d'un modèle cinématique (accélération axiale + taux de rotation mesurés par IMU bruitée) et recale l'estimation à chaque mesure du senseur IR (azimut et élévation relatifs), sauf pendant les phases de brouillage.

### Contraintes physiques
| Paramètre | Description | Valeur par défaut |
|---|---|---|
| `G_LIMIT` | Facteur de charge structurel maximal | 20 G |
| `MAX_SLEW_RATE_OMEGA` | Vitesse angulaire max des gouvernes | 5 rad/s |
| `Masse`, `Rho`, `S_ref`, `Cx` | Paramètres de traînée aérodynamique | 15 kg, 1.225 kg/m³, 0.015 m², 0.25 |

---

## Personnalisation

Les principaux paramètres modifiables sont regroupés en tête de fonction (`lancer_simulation_3d`) :

- `perte_signal_active`, `manoeuvre_esquive_active`, `manoeuvre_zigzag_active` : activer/désactiver un événement plutôt que de le tirer aléatoirement.
- `N_gain` : agressivité de la loi de guidage.
- `std_accel`, `std_gyro`, `std_ir` : niveaux de bruit des capteurs.
- Bornes des tirages aléatoires (position, vitesse, timing des manœuvres) dans le bloc *« Tirage aléatoire du scénario »*.

---

## Limitations connues

- Modèle aérodynamique simplifié (pas de portance, pas de compressibilité, pas d'effets de roulis).
- Le capteur ne mesure pas le roulis du poursuivant (hypothèse d'un axe senseur aligné avec le cap/la pente du corps).
- Le mouvement de la cible n'intègre pas de contraintes physiques (accélérations libres, non bornées par un facteur de charge).

---

## Licence

À définir selon vos besoins (MIT, GPL-3.0, etc. — libre à vous d'ajouter un fichier `LICENSE`).

## Contribuer

Les suggestions et *pull requests* sont bienvenues : amélioration du modèle physique, ajout d'un vrai modèle 6-DDL, comparaison EKF vs UKF, etc.
