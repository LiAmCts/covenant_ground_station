# Covenant Ground Station

Prototype de station sol pour le projet **COVENANT**.

L’objectif initial est de développer une interface logicielle permettant de lire les données du LiDAR vertical D500 / LDROBOT, de reconstruire progressivement une cartographie simple de l’environnement, puis de préparer une future intégration avec ROS 2 et la Raspberry Pi embarquée sur le drone.

## Objectifs du projet

À court terme, ce dépôt sert à :

* lire les données du LiDAR D500 / LDROBOT branché en USB ;
* décoder les mesures angle-distance issues du LiDAR ;
* afficher une coupe 2D du scan LiDAR ;
* simuler le déplacement du drone avec une variable `x` ;
* empiler les coupes LiDAR pour reconstruire un nuage de points 3D ;
* préparer une future intégration avec ROS 2, les topics LiDAR et l’odométrie du drone.

À terme, la Ground Station devra pouvoir recevoir les données issues de la Raspberry Pi embarquée, afficher une cartographie en temps réel et aider à visualiser l’exploration d’un environnement intérieur.

## Architecture prévue

covenant_ground_station/
├── README.md
├── requirements.txt
├── scripts/
│   ├── 01_raw_serial_test.py
│   ├── 02_decode_lidar_frames.py
│   ├── 03_live_2d_slice.py
│   └── 04_live_3d_mapping_simulated_x.py
├── covenant_gs/
│   ├── __init__.py
│   ├── lidar_reader.py
│   ├── lidar_parser.py
│   ├── mapper.py
│   └── viewer.py
├── data/
├── captures/
└── exports/

Rôle des dossiers principaux :

* `scripts/` : petits scripts de test, développés étape par étape ;
* `covenant_gs/` : futur package Python propre du projet ;
* `data/` : données de test éventuelles ;
* `captures/` : captures brutes LiDAR ;
* `exports/` : exports de nuages de points, cartes ou résultats.

## Prérequis

Le projet est prévu pour Linux.

Prérequis principaux :

* Python 3 ;
* Git ;
* un port série USB disponible, généralement `/dev/ttyUSB0` ;
* le LiDAR D500 / LDROBOT branché en USB.

ROS 2 n’est pas nécessaire pour les premiers tests locaux.
L’intégration ROS 2 viendra dans une étape suivante.

## Initialisation du projet après clone ou fork

Cette section explique comment préparer l’environnement de travail après avoir récupéré le dépôt.

### 1. Cloner le dépôt

Avec HTTPS :

git clone <URL_DU_REPO>
cd covenant_ground_station

Avec SSH :

git clone git@github.com:<username>/covenant_ground_station.git
cd covenant_ground_station

Si le dépôt est déjà présent localement, il suffit de se placer dedans :

cd covenant_ground_station

### 2. Créer l’environnement virtuel Python

python3 -m venv .venv

Cette commande crée un environnement Python isolé dans le dossier `.venv/`.

L’intérêt est d’installer les bibliothèques nécessaires au projet sans modifier le Python global du PC.

### 3. Activer l’environnement virtuel

source .venv/bin/activate

Une fois activé, le terminal doit afficher `(.venv)` au début de la ligne.

Exemple :

(.venv) liam@pc:~/.../covenant_ground_station$

### 4. Installer les dépendances Python

pip install --upgrade pip
pip install -r requirements.txt

Les dépendances principales sont :

* `pyserial` : lecture du LiDAR via port série USB ;
* `numpy` : calculs numériques et manipulation des points ;
* `matplotlib` : premiers affichages 2D simples ;
* `open3d` : affichage 3D du nuage de points.

### 5. Vérifier que le LiDAR est détecté

Brancher le LiDAR en USB, puis vérifier le port :

ls /dev/ttyUSB*

Le port attendu est généralement :

/dev/ttyUSB0

Vérifier les droits du port :

ls -l /dev/ttyUSB0

Si le port n’est pas accessible, il est possible d’ajouter temporairement les droits :

sudo chmod 666 /dev/ttyUSB0

Solution plus propre à long terme :

sudo usermod -aG dialout $USER

Il faut ensuite se déconnecter/reconnecter pour que le changement soit pris en compte.

### 6. Lancer le premier test de lecture série

python scripts/01_raw_serial_test.py

Objectif attendu : vérifier que des octets bruts arrivent bien depuis le LiDAR.

### 7. Lancer le décodage des trames LiDAR

python scripts/02_decode_lidar_frames.py

Objectif attendu : obtenir des mesures exploitables du type :

angle
distance
confidence

### 8. Réactiver l’environnement à chaque nouvelle session

À chaque fois qu’un nouveau terminal est ouvert, il faut revenir dans le dossier du projet et réactiver l’environnement :

cd covenant_ground_station
source .venv/bin/activate

### 9. Quitter l’environnement virtuel

deactivate

## Étapes de développement prévues

### Étape 1 — Lecture série brute

Objectif : vérifier que le LiDAR branché en USB envoie bien des données sur `/dev/ttyUSB0`.

Script prévu :

python scripts/01_raw_serial_test.py

### Étape 2 — Décodage des trames LiDAR

Objectif : transformer les données binaires du LiDAR en mesures exploitables :

angle
distance
intensité / confiance

Script prévu :

python scripts/02_decode_lidar_frames.py

### Étape 3 — Affichage 2D d’une coupe LiDAR

Objectif : afficher en direct une coupe verticale issue du LiDAR.

Script prévu :

python scripts/03_live_2d_slice.py

### Étape 4 — Simulation du déplacement du drone

Objectif : simuler une position `x` qui augmente avec le temps.

Exemple :

x = speed * time

Cela permet d’empiler les coupes LiDAR comme si le drone avançait dans un couloir.

### Étape 5 — Reconstruction 3D simple

Objectif : convertir chaque point LiDAR en point 3D :

x = position simulée du drone
y = distance * cos(angle)
z = distance * sin(angle)

Script prévu :

python scripts/04_live_3d_mapping_simulated_x.py

### Étape 6 — Future intégration ROS 2

À terme, la variable `x` simulée sera remplacée par une vraie odométrie issue de ROS 2.

Architecture cible :

LiDAR vertical
    ↓
Raspberry Pi / ROS 2
    ↓
topic LiDAR + odométrie
    ↓
Ground Station
    ↓
cartographie temps réel

## Commandes Git utiles

Voir l’état du dépôt :

git status

Ajouter des fichiers au prochain commit :

git add <fichier>

Créer un commit :

git commit -m "Message du commit"

Voir l’historique :

git log --oneline

## Notes importantes

* Ne pas versionner l’environnement `.venv/`.
* Ne pas versionner les grosses captures LiDAR.
* Garder les scripts de test simples et progressifs.
* Valider chaque étape avant de passer à la suivante.
* L’intégration ROS 2 ne doit venir qu’après validation de la lecture LiDAR locale.

