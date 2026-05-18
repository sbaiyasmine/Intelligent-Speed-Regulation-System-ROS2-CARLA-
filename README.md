# Intelligent Speed Regulation System using ROS2 and CARLA

## Présentation générale

Ce projet présente le développement complet d’un système intelligent de régulation de vitesse combinant :

- une simulation 2D sous ROS2 ;
- une simulation 3D sous CARLA ;
- une architecture temps réel distribuée ;
- un contrôle PID ;
- une communication TCP/IP ;
- une modélisation SysML.

L’objectif principal est de développer un système capable de contrôler automatiquement la vitesse d’un véhicule tout en assurant :

- la stabilité dynamique ;
- la sécurité ;
- la gestion du trafic ;
- l’adaptation à l’environnement ;
- l’évitement des collisions.

Le projet s’inscrit dans une approche Model Based Design (MBD) permettant de passer progressivement :

1. de la modélisation ;
2. à la simulation ;
3. puis à la validation temps réel.

---

# Objectifs du projet

Le système développé doit permettre :

- le suivi d’une vitesse de consigne ;
- la correction dynamique de l’erreur ;
- l’adaptation automatique au trafic ;
- la détection du véhicule précédent ;
- le calcul du Time-To-Collision (TTC) ;
- la gestion de plusieurs modes de conduite ;
- l’intégration ROS2 ↔ Raspberry Pi ;
- la validation sous environnement CARLA.

---

# Architecture globale du système

L’architecture générale repose sur plusieurs composants communicants :

- un PC Linux exécutant ROS2 ;
- un Raspberry Pi 4 ;
- un environnement CARLA ;
- des nœuds ROS2 ;
- un régulateur PID ;
- une communication TCP/IP.

Le système est conçu selon une architecture distribuée temps réel.

---

# Architecture Simulink

L’architecture Simulink développée permet de connecter :

- le modèle dynamique du véhicule ;
- le contrôleur PID ;
- les blocs ROS2 ;
- les modules de communication ;
- les blocs de visualisation.

<p align="center">
  <img src="images/archi_simulink.png" width="950">
</p>

---

# Modélisation SysML

Le projet a également été modélisé sous SysML afin d’assurer une conception système cohérente.

La modélisation SysML permet de définir :

- les exigences du système ;
- l’architecture fonctionnelle ;
- les blocs du système ;
- les flux de données ;
- les interactions entre composants ;
- les scénarios de fonctionnement.

Les diagrammes SysML développés comprennent notamment :

- diagrammes de cas d’utilisation ;
- diagrammes de blocs ;
- diagrammes internes ;
- diagrammes d’exigences.

Les fichiers SysML sont disponibles dans :

```bash
/sysml
