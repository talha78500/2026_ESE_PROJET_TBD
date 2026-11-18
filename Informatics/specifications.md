# Cahier des charges — Architecture logicielle

> **Document en cours de rédaction.** Les exigences ci-dessous constituent
> la base actuelle du projet. Les points indiqués « À détailler » ou
> « À confirmer » restent ouverts.

## 1. Principes d’architecture

| Exigence | État |
| --- | --- |
| Séparer la logique applicative (**App**) des accès matériels (**BSP**). | Structure définie ci-dessous |
| Implémenter **FreeRTOS**. | À détailler |
| Implémenter une architecture à **objets actifs**. | À détailler |
| Documenter l’architecture à l’aide de diagrammes complémentaires. | Vues attendues définies ci-dessous |

### Séparation App / BSP

| Niveau | Application — App | Accès matériel — BSP |
| --- | --- | --- |
| Par composant | `component.c` et `component.h` | `bsp_component.c` et `bsp_component.h` |
| Global | `app.c` et `app.h` | `bsp.c` et `bsp.h` |

Les noms `component` et `bsp_component` sont à adapter au composant concerné.

### FreeRTOS et objets actifs — À détailler

Les points suivants restent à définir :

- Répartition des responsabilités entre objets actifs et tâches FreeRTOS.
- Événements échangés et mécanisme de distribution.
- Files de messages, propriété des données et synchronisation.
- Priorités, périodes et contraintes de temps de réponse.
- Articulation entre interruptions, traitements différés et logique applicative.

## 2. Composants à intégrer

- LiDAR.
- Moteur.
- Encodeur.
- Capteur de distance ToF.
- Capteur de couleur.
- Servomoteur.
- NeoPixel.
- Chargeur.
- Retour du **fuel gauge** : à confirmer.

## 3. Diagrammes à produire

Chaque diagramme doit répondre à une question précise et rester cohérent
avec les autres vues ainsi qu’avec l’implémentation.

### Vue d’ensemble et structure

| Diagramme | Objectif et contenu attendu |
| --- | --- |
| **Système** | Montrer les principaux sous-systèmes, leurs responsabilités et leurs interfaces avec l’environnement. |
| **Électrique** | Décrire les connexions, alimentations et interfaces électriques entre les composants. |
| **Couches logicielles et dépendances** | Situer l’App globale, les App des composants, le BSP global, les BSP des composants, la HAL et FreeRTOS ; préciser les dépendances et les appels autorisés. |
| **Structure des modules et interfaces — OOP** | Décrire les modules, structures de données, interfaces et relations de propriété. La représentation de l’approche objet en C reste à préciser. |

### Comportement et échanges

| Diagramme | Objectif et contenu attendu |
| --- | --- |
| **Flux de données et événements** | Montrer comment les mesures deviennent des décisions puis des commandes ; identifier les producteurs et consommateurs des données et événements. |
| **Machines à états** | Décrire les états, événements déclencheurs, conditions et actions des comportements ou objets actifs concernés. |
| **Exemple d’objet actif** | Illustrer le traitement d’un événement, l’état interne et les interactions d’un objet actif avec le reste de l’application. |
| **Tâches et contextes d’exécution** | Montrer la répartition entre tâches FreeRTOS et interruptions, les files de messages et la propriété des données partagées. |
| **Séquences ciblées** | Décrire l’ordre des échanges entre composants lorsque leur coordination nécessite une explication : démarrage, détection d’obstacle ou défaillance d’un capteur, par exemple. |

**Machines à états et séquences sont complémentaires.** Une machine à états
décrit l’évolution d’un comportement ; une séquence décrit les interactions
entre plusieurs participants au cours d’un scénario. Ajouter une séquence
uniquement lorsque les échanges ou leur ordre nécessitent une clarification,
sans en produire systématiquement une pour chaque machine à états.

### Contraintes temporelles — Dans un second temps

| Diagramme | Objectif et contenu attendu |
| --- | --- |
| **Chronogramme** | Représenter les périodes d’acquisition, les périodes de contrôle et les délais de réaction une fois ces contraintes définies ; vérifier leur compatibilité avec l’organisation des tâches. |

### Ordre de travail conseillé

1. Établir la vue système et les couches logicielles.
2. Définir les flux de données et les événements.
3. Décrire les machines à états et un exemple d’objet actif.
4. Définir les tâches et les contextes d’exécution à partir de ces besoins.
5. Ajouter les séquences utiles à la compréhension des interactions.
6. Compléter les chronogrammes lorsque les contraintes temporelles sont connues.

La vue électrique et la structure des modules sont à maintenir en cohérence
avec ces choix au fil de la conception.
