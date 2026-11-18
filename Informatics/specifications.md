# Cahier des charges — Architecture logicielle

> **Document en cours de rédaction.** Les choix d’architecture établis sont
> résumés ci-dessous. Les points indiqués « À détailler » ou « À confirmer »
> restent ouverts.

## Le projet en bref

Le logiciel embarqué du robot sous STM32 lit les capteurs, traite leurs
mesures et commande les actionneurs. La couche **App** porte la logique du
robot et la couche **BSP** prend en charge les accès matériels. Le logiciel
adoptera un fonctionnement **événementiel**, avec **FreeRTOS** et des
**objets actifs**. La répartition des tâches et la distribution des
événements restent à définir.

## Sommaire

- [Architecture](#architecture)
- [Diagrammes](#diagrammes)
- [Organisation App et BSP](#organisation-app-et-bsp)
- [FreeRTOS et objets actifs](#freertos-et-objets-actifs)
- [Composants et interfaces](#composants-et-interfaces)
- [Catalogue des diagrammes](#catalogue-des-diagrammes)

## Architecture

| Élément | Rôle |
| --- | --- |
| **App globale** | Coordonne les composants et porte le comportement du robot. |
| **App par composant** | Traite les données ou les commandes du composant. Sa logique reste portable et indépendante du système d’exploitation temps réel. |
| **BSP global et BSP par composant** | Regroupent les accès au matériel. Chaque plateforme fournit sa propre implémentation des interfaces nécessaires. |
| **FreeRTOS et objets actifs** | Structurent l’exécution événementielle. Les traitements des composants effectuent un travail borné et rendent la main sans attendre les données ni le matériel. |

Consulter l’[organisation des modules](#organisation-app-et-bsp), les
[contraintes d’exécution](#freertos-et-objets-actifs) et le
[périmètre des composants](#composants-et-interfaces).

## Diagrammes

| Vue | Question à laquelle elle répond |
| --- | --- |
| **Système, électrique, couches logicielles et modules** | Quels sont les sous-systèmes, leurs connexions et leurs dépendances ? |
| **Flux de données, machines à états et exemple d’objet actif** | Comment les mesures deviennent-elles des décisions puis des commandes, et comment chaque comportement évolue-t-il ? |
| **Tâches et séquences ciblées** | Où s’exécutent les traitements et comment les participants se coordonnent-ils dans les scénarios qui le nécessitent ? |
| **Chronogrammes** | L’organisation des tâches respecte-t-elle les périodes et les délais de réaction une fois ceux-ci définis ? |

Commencer par la vue système, les couches logicielles et les flux. Les
machines à états décrivent ensuite le comportement, et les séquences
complètent les scénarios dont les échanges demandent une clarification.
Le [catalogue des diagrammes](#catalogue-des-diagrammes) précise le
contenu attendu et l’ordre de travail complet.

## Organisation App et BSP

| Niveau | Application — App | Accès matériel — BSP |
| --- | --- | --- |
| Par composant | `component.c` et `component.h` | `bsp_component.c` et `bsp_component.h` |
| Global | `app.c` et `app.h` | `bsp.c` et `bsp.h` |

Les noms `component` et `bsp_component` sont à adapter au composant concerné.

Chaque composant expose une interface BSP commune. Un BSP peut avoir
plusieurs implémentations selon la plateforme. Chaque compilation en
sélectionne une seule par composant. Le LiDAR utilise déjà le même
analyseur de trames sous Linux et sur STM32, avec un BSP propre à chaque
plateforme. Voir la [structure du composant LiDAR](components/lidar/README.md)
et le [diagramme des couches à produire](#vue-densemble-et-structure).

## FreeRTOS et objets actifs

La logique portable des composants reste indépendante de FreeRTOS. Chaque
appel de traitement effectue un travail borné, rend la main sans attendre
les données ni le matériel et conserve son état si le traitement doit se
poursuivre lors d’un appel ultérieur. Les interruptions, le DMA et la
gestion des tampons propres à la plateforme relèvent du BSP. Le code
embarqué des composants n’utilise pas d’allocation dynamique.

### Décisions à détailler

Les points suivants restent à définir :

- Répartition des responsabilités entre objets actifs et tâches FreeRTOS.
- Événements échangés et mécanisme de distribution.
- Files de messages, propriété des données et synchronisation.
- Priorités, périodes et contraintes de temps de réponse.
- Articulation entre interruptions, traitements différés et logique applicative.

Ces décisions alimenteront les diagrammes des
[flux et contextes d’exécution](#comportement-et-échanges).

## Composants et interfaces

- LiDAR.
- Moteur.
- Encodeur.
- Capteur de distance ToF.
- Capteur de couleur.
- Servomoteur.
- NeoPixel.
- Chargeur.
- Retour du **fuel gauge** : à confirmer.

Pour chaque composant, préciser les données et commandes échangées, leurs
unités, la convention de repère si elle s’applique, le comportement en cas
de donnée indisponible ou de défaut, et au moins une vérification observable.

### LiDAR

- **Sortie actuelle de l’analyseur.** Un point contient un angle corrigé
  dans l’intervalle [0°, 360°[ et une distance en millimètres. Une
  distance de 0 mm indique une absence de retour. Voir
  [l’interface de l’analyseur](components/lidar/STM32_implementation/App/lidar.h).
- **À préciser pour l’intégration.** Définir le zéro et le sens des angles
  dans le repère du robot, ainsi que la manière dont l’App détecte une
  donnée indisponible ou périmée.
- **Vérification proposée.** Fournir une trame en plusieurs fragments.
  Vérifier que chaque appel rend la main, qu’aucune mesure issue de cette
  trame ne sort avant réception complète et que la mesure attendue est
  produite après le dernier fragment.

## Catalogue des diagrammes

Ces vues détaillent l’[architecture](#architecture) et doivent rester
cohérentes entre elles et avec l’implémentation.

### Vue d’ensemble et structure

| Diagramme | Objectif et contenu attendu |
| --- | --- |
| **Système** | Montrer les principaux sous-systèmes, leurs responsabilités et leurs interfaces avec l’environnement. |
| **Électrique** | Décrire les connexions, alimentations et interfaces électriques entre les composants. |
| **Couches logicielles et dépendances** | Situer l’App globale, les App des composants, le BSP global, les BSP des composants, la HAL et FreeRTOS. Préciser les dépendances et les appels autorisés. |
| **Structure des modules et interfaces — OOP** | Décrire les modules, structures de données, interfaces et relations de propriété. La représentation de l’approche objet en C reste à préciser. |

### Comportement et échanges

| Diagramme | Objectif et contenu attendu |
| --- | --- |
| **Flux de données et événements** | Montrer comment les mesures deviennent des décisions puis des commandes. Identifier les producteurs et consommateurs des données et événements. |
| **Machines à états** | Décrire les états, événements déclencheurs, conditions et actions des comportements ou objets actifs concernés. |
| **Exemple d’objet actif** | Illustrer le traitement d’un événement, l’état interne et les interactions d’un objet actif avec le reste de l’application. |
| **Tâches et contextes d’exécution** | Montrer la répartition entre tâches FreeRTOS et interruptions, les files de messages et la propriété des données partagées. |
| **Séquences ciblées** | Décrire l’ordre des échanges entre composants lorsque leur coordination nécessite une explication : démarrage, détection d’obstacle ou défaillance d’un capteur, par exemple. |

**Machines à états et séquences sont complémentaires.** Une machine à états
décrit l’évolution d’un comportement. Une séquence décrit les interactions
entre plusieurs participants au cours d’un scénario. Ajouter une séquence
uniquement lorsque les échanges ou leur ordre nécessitent une clarification,
sans en produire systématiquement une pour chaque machine à états.

### Contraintes temporelles — Dans un second temps

| Diagramme | Objectif et contenu attendu |
| --- | --- |
| **Chronogramme** | Représenter les périodes d’acquisition, les périodes de contrôle et les délais de réaction une fois ces contraintes définies. Vérifier leur compatibilité avec l’organisation des tâches. |

### Ordre de travail conseillé

1. Établir la vue système et les couches logicielles.
2. Définir les flux de données et les événements.
3. Décrire les machines à états et un exemple d’objet actif.
4. Définir les tâches et les contextes d’exécution à partir de ces besoins.
5. Ajouter les séquences utiles à la compréhension des interactions.
6. Compléter les chronogrammes lorsque les contraintes temporelles sont connues.

La vue électrique et la structure des modules sont à maintenir en cohérence
avec ces choix au fil de la conception.
