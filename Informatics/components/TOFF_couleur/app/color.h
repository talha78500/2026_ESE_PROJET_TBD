#ifndef COLOR_H
#define COLOR_H

#include <stdint.h>

/* Logique pure : aucun appel HAL, compile aussi sur PC. */

#define COLOR_RED      'R'
#define COLOR_GREEN    'G'
#define COLOR_UNKNOWN  '?'

/* Seuils à calibrer sur tes canettes (loguer clear/red/green, voir tests). */
#define COLOR_CLEAR_MIN   50U      /* en dessous : trop sombre, mesure inutilisable */
#define COLOR_CLEAR_MAX   37000U   /* au dessus : saturation (max 37888 avec ATIME=0xDB) */
#define COLOR_RATIO_PCT   20U      /* R doit dépasser G d'au moins 20 % (et inversement) */

/* Classe UNE mesure : 'R', 'G' ou '?' (sombre, saturée ou ambiguë). */
char color_classify(uint16_t clear, uint16_t red, uint16_t green);

/* Vote majoritaire sur plusieurs mesures. */
typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t total;   /* mesures ajoutées, '?' compris */
} color_vote_t;

void    color_vote_reset(color_vote_t * const me);
void    color_vote_add(color_vote_t * const me, char color);
uint8_t color_vote_valid(color_vote_t const * const me);    /* nb de votes R + G */
char    color_vote_result(color_vote_t const * const me);   /* 'R', 'G', ou '?' si égalité */

#endif /* COLOR_H */
