# Intelligent Speed Regulation System using ROS2 and CARLA

## Présentation du projet

Ce projet présente la conception et l’implémentation d’un système intelligent de régulation de vitesse développé à travers deux environnements complémentaires :

- une simulation 2D sous ROS2 pour la modélisation et la validation du régulateur ;
- une simulation 3D sous CARLA pour l’évaluation du comportement du véhicule dans un environnement dynamique réaliste.

L’objectif principal est d’assurer un contrôle stable, précis et sécurisé de la vitesse d’un véhicule mobile tout en tenant compte des contraintes physiques et des conditions de circulation.

Le projet combine :

- ROS2 Humble
- Python
- Raspberry Pi 4
- RViz2
- CARLA Simulator
- Régulateur PID
- Communication TCP/IP
- Contrôle temps réel
- Modélisation SysML

---

# Objectifs du projet

## Partie 2D — Régulation PID sous ROS2

La première partie du projet consiste à :

- modéliser la dynamique du véhicule ;
- mesurer expérimentalement les paramètres physiques ;
- développer un régulateur PID ;
- implémenter une architecture ROS2 communicante ;
- visualiser les données en temps réel ;
- valider les performances du système.

---

## Partie 3D — Régulation intelligente sous CARLA

La seconde partie étend le système à un environnement de conduite réaliste.

Le véhicule doit :

- maintenir une vitesse cible ;
- détecter un véhicule à l’avant ;
- adapter automatiquement sa vitesse ;
- éviter les collisions ;
- commuter dynamiquement entre plusieurs modes de conduite.

---

# Architecture générale du système

Le système repose sur une architecture distribuée composée de :

- un PC Linux exécutant ROS2, RViz2 et la simulation ;
- un Raspberry Pi 4 pilotant le moteur réel ;
- une communication TCP/IP temps réel ;
- un environnement CARLA pour la simulation 3D.

---

# Modélisation SysML

Le projet a également été modélisé sous SysML afin de structurer :

- les exigences fonctionnelles ;
- l’architecture du système ;
- les interactions entre les composants ;
- les flux d’informations ;
- les scénarios de fonctionnement.

Les fichiers SysML du projet sont disponibles dans le dossier :

```bash
/sysml
