#include "obstacle.h"

static bool is_valid(uint16_t d)
{
    return (d != 0U) && (d != OBS_INVALID_MM);
}

bool obstacle_find(obstacle_t * const me, uint16_t const d_mm[], uint8_t n)
{
    uint8_t  i_min = 0U;
    uint16_t x     = OBS_INVALID_MM;

    me->found          = false;
    me->clipped        = false;
    me->x_mm           = 0U;
    me->x_loc_left_mm  = 0U;
    me->x_loc_right_mm = 0U;
    me->idx_min        = 0U;
    me->idx_left       = 0U;
    me->idx_right      = 0U;
    me->centering      = 0;

    /* 1. Point le plus proche = centre du cylindre */
    for (uint8_t i = 0U; i < n; i++) {
        if (is_valid(d_mm[i]) && (d_mm[i] < x)) {
            x     = d_mm[i];
            i_min = i;
        }
    }
    if (x > OBS_DETECT_MM) {    /* rien de valide, ou trop loin */
        return false;
    }

    /* 2. Bords : on s'éloigne du minimum tant que la distance croît */
    uint32_t const limit = (uint32_t)x + OBS_DEPTH_MAX_MM;

    /* Demi-largeur max d'une canette en colonnes, arrondie au-dessus :
     * R / (D * angle_colonne), D = distance au centre du cylindre = x + R. */
    uint32_t const denom = ((uint32_t)x + OBS_CAN_RADIUS_MM) * OBS_COL_MRAD;
    uint32_t const half  = ((OBS_CAN_RADIUS_MM * 1000U) + denom - 1U) / denom;

    uint8_t  l    = i_min;
    uint8_t  r    = i_min;
    uint16_t peak = x;

    while ((l > 0U) && ((uint32_t)(i_min - l) < half)) {
        uint16_t const nd = d_mm[l - 1U];
        if (!is_valid(nd) || ((uint32_t)nd > limit)) {
            break;                                  /* fond ou trou */
        }
        if ((int32_t)nd < ((int32_t)peak - (int32_t)OBS_NOISE_MM)) {
            break;                                  /* creux : autre objet */
        }
        if (nd > peak) {
            peak = nd;
        }
        l--;
    }

    peak = x;
    while (((r + 1U) < n) && ((uint32_t)(r - i_min) < half)) {
        uint16_t const nd = d_mm[r + 1U];
        if (!is_valid(nd) || ((uint32_t)nd > limit)) {
            break;
        }
        if ((int32_t)nd < ((int32_t)peak - (int32_t)OBS_NOISE_MM)) {
            break;
        }
        if (nd > peak) {
            peak = nd;
        }
        r++;
    }

    /* 3. Centrage : milieu des bords, ou position du minimum si un bord est
     *    hors du champ de vue (la canette est tronquée). */
    bool const clipped = (l == 0U) || (r == (n - 1U));
    int32_t const center2 = clipped ? (2 * (int32_t)i_min)
                                    : ((int32_t)l + (int32_t)r);

    me->found          = true;
    me->clipped        = clipped;
    me->x_mm           = x;
    me->x_loc_left_mm  = d_mm[l];
    me->x_loc_right_mm = d_mm[r];
    me->idx_min        = i_min;
    me->idx_left       = l;
    me->idx_right      = r;
    me->centering      = (int16_t)(center2 - ((int32_t)n - 1));
    return true;
}
