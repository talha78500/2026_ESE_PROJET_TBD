#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <stdbool.h>
#include <stdint.h>

/* Logique pure : aucun appel HAL, compile aussi sur PC.
 *
 * Entrée : un profil de distances d_mm[0..n-1], colonne 0 = gauche, n-1 = droite
 *          (ex. 8 colonnes d'un capteur ToF multizone). Une colonne sans cible
 *          valide doit valoir OBS_INVALID_MM (ou 0).
 *
 * Principe : la canette est un cylindre. Son point le plus proche (x) est au
 * centre, ses bords (x_loc gauche/droit) sont plus loin. On part du minimum x et
 * on s'éloigne de chaque côté tant que la distance ne fait que croître. On
 * s'arrête (1) au fond (saut > OBS_DEPTH_MAX_MM), (2) quand la distance redécroît
 * (creux entre deux canettes côte à côte), ou (3) à la largeur physique maximale
 * d'une canette à cette distance (filet de sécurité quand le capteur est trop
 * grossier pour voir le creux). La 2e canette n'entre donc jamais dans le calcul.
 * centering vise le milieu [bord gauche, bord droit] d'UNE seule canette. */

#define OBS_INVALID_MM    0xFFFFU

/* À régler */
#define OBS_DETECT_MM       400U   /* portée de détection : obstacle si x <= 400 mm */
#define OBS_NOISE_MM        10U    /* bruit de mesure toléré entre deux colonnes */
#define OBS_DEPTH_MAX_MM    45U    /* profondeur max d'une canette : rayon ~33 mm + marge */
#define OBS_CAN_RADIUS_MM   33U    /* canette 33 cl : diamètre 66 mm */
#define OBS_COL_MRAD        98U    /* angle d'UNE colonne en milliradians :
                                      45 deg / 8 colonnes = 5,6 deg = 98 mrad (VL53L5CX 8x8) */

typedef struct {
    bool     found;
    bool     clipped;         /* la canette touche le bord du champ de vue */
    uint16_t x_mm;            /* distance minimale (centre du cylindre) */
    uint16_t x_loc_left_mm;   /* distance au bord gauche de la canette */
    uint16_t x_loc_right_mm;  /* distance au bord droit */
    uint8_t  idx_min;
    uint8_t  idx_left;
    uint8_t  idx_right;
    int16_t  centering;       /* en DEMI-colonnes : <0 canette à gauche, >0 à droite, 0 centrée */
} obstacle_t;

/* Cible la canette la plus proche. Renvoie me->found. */
bool obstacle_find(obstacle_t * const me, uint16_t const d_mm[], uint8_t n);

#endif /* OBSTACLE_H */
