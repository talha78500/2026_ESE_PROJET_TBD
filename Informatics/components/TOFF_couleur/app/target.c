#include "target.h"
#include <string.h>

void target_init(target_t * const me)
{
    memset(me, 0, sizeof *me);          /* state = TARGET_SEARCH, obstacle_color = '\0' */
    color_vote_reset(&me->votes);
}

void target_reset(target_t * const me)
{
    target_init(me);
}

void target_on_profile(target_t * const me, uint16_t const d_mm[], uint8_t n)
{
    if (me->state == TARGET_DONE) {
        return;                         /* couleur figée jusqu'à target_reset */
    }

    if (!obstacle_find(&me->obs, d_mm, n)) {
        me->state     = TARGET_SEARCH;
        me->centering = 0;
        return;
    }

    me->centering = me->obs.centering;

    if (me->state == TARGET_SEARCH) {
        me->state = TARGET_APPROACH;
    }

    if ((me->state == TARGET_APPROACH) && (me->obs.x_mm <= TARGET_X_COLOR_MM)) {
        color_vote_reset(&me->votes);
        me->skip  = TARGET_COLOR_SKIP;
        me->state = TARGET_MEASURE_COLOR;
    } else if ((me->state == TARGET_MEASURE_COLOR) &&
               (me->obs.x_mm > (TARGET_X_COLOR_MM + TARGET_X_COLOR_HYST_MM))) {
        me->state = TARGET_APPROACH;    /* la canette s'est éloignée : on abandonne ce vote */
    }
}

void target_on_color(target_t * const me, uint16_t clear, uint16_t red, uint16_t green)
{
    if (me->state != TARGET_MEASURE_COLOR) {
        return;
    }
    if (me->skip > 0U) {
        me->skip--;
        return;
    }

    color_vote_add(&me->votes, color_classify(clear, red, green));

    if ((color_vote_valid(&me->votes) >= TARGET_COLOR_SAMPLES) ||
        (me->votes.total >= TARGET_COLOR_MAX_TRIES)) {
        me->obstacle_color = color_vote_result(&me->votes);
        me->state          = TARGET_DONE;
    }
}
