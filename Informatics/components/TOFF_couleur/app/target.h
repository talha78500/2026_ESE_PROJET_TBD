#ifndef TARGET_H
#define TARGET_H

#include <stdbool.h>
#include <stdint.h>

#include "obstacle.h"
#include "color.h"

/* Machine à états : SEARCH -> APPROACH -> MEASURE_COLOR -> DONE.
 * Logique pure : aucun appel HAL. Les sorties sont target_t.centering et
 * target_t.obstacle_color ; les moteurs sont gérés ailleurs. */

#define TARGET_X_COLOR_MM        50U   /* x_color : la mesure de couleur démarre à <= 50 mm */
#define TARGET_X_COLOR_HYST_MM   10U   /* on abandonne la mesure si x > 60 mm */
#define TARGET_COLOR_SKIP        1U    /* mesures jetées à l'entrée (intégration en cours pendant l'arrivée) */
#define TARGET_COLOR_SAMPLES     5U    /* votes R/G valides nécessaires */
#define TARGET_COLOR_MAX_TRIES   15U   /* sinon on conclut '?' */

typedef enum {
    TARGET_SEARCH,         /* aucune canette dans la portée */
    TARGET_APPROACH,       /* canette vue : avancer en corrigeant centering */
    TARGET_MEASURE_COLOR,  /* à x_color : le robot doit être ARRÊTÉ pendant la mesure */
    TARGET_DONE            /* obstacle_color figée jusqu'à target_reset() */
} target_state_t;

typedef struct {
    target_state_t state;
    obstacle_t     obs;
    color_vote_t   votes;
    uint8_t        skip;
    int16_t        centering;       /* sortie : demi-colonnes, <0 gauche, >0 droite */
    char           obstacle_color;  /* sortie : '\0' pas encore mesurée, 'R', 'G' ou '?' */
} target_t;

void target_init(target_t * const me);
void target_reset(target_t * const me);   /* à appeler une fois la canette traitée */

/* À appeler à chaque nouveau profil de distances. */
void target_on_profile(target_t * const me, uint16_t const d_mm[], uint8_t n);

/* À appeler à chaque nouvelle mesure couleur (inutile hors MEASURE_COLOR, ignorée). */
void target_on_color(target_t * const me, uint16_t clear, uint16_t red, uint16_t green);

#endif /* TARGET_H */
