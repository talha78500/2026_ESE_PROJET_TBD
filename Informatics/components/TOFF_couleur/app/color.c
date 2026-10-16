#include "color.h"

char color_classify(uint16_t clear, uint16_t red, uint16_t green)
{
    if ((clear < COLOR_CLEAR_MIN) || (clear >= COLOR_CLEAR_MAX)) {
        return COLOR_UNKNOWN;
    }
    if ((uint32_t)red * 100U > (uint32_t)green * (100U + COLOR_RATIO_PCT)) {
        return COLOR_RED;
    }
    if ((uint32_t)green * 100U > (uint32_t)red * (100U + COLOR_RATIO_PCT)) {
        return COLOR_GREEN;
    }
    return COLOR_UNKNOWN;
}

void color_vote_reset(color_vote_t * const me)
{
    me->red   = 0U;
    me->green = 0U;
    me->total = 0U;
}

void color_vote_add(color_vote_t * const me, char color)
{
    if (me->total == UINT8_MAX) {
        return;
    }
    me->total++;
    if (color == COLOR_RED) {
        me->red++;
    } else if (color == COLOR_GREEN) {
        me->green++;
    }
}

uint8_t color_vote_valid(color_vote_t const * const me)
{
    return (uint8_t)(me->red + me->green);
}

char color_vote_result(color_vote_t const * const me)
{
    if (me->red > me->green) {
        return COLOR_RED;
    }
    if (me->green > me->red) {
        return COLOR_GREEN;
    }
    return COLOR_UNKNOWN;
}
