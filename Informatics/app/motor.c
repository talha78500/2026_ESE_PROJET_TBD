#include "motor.h"
#include "motor_drv.h"
#include <stdio.h>

#define MOVE_PI_F                    3.14159265f
#define MOVE_ENC_SIGN_LOCK_TICKS     10      // Seuil (ticks) pour fixer le sens de comptage reel
#define MOVE_ACCEL_MS                150     // Rampe de demarrage doux (evite patinage/ecart initial)
#define MOVE_RAMP_TICKS              200     // Rampe de deceleration (~19 mm avant cible)
#define MOVE_CREEP_SPEED             280     // Vitesse d'approche finale (evite le calage sous ~675/1000 PWM)
#define MOVE_SYNC_KP                 8       // Gain P de synchronisation gauche/droite (permille / tick)
#define MOVE_SYNC_KI_DIV             40      // Diviseur I (correction += integral(tick*ms) / 40)
#define MOVE_SYNC_INT_MAX            16000   // Anti-windup integral (tick*ms -> max +-400 permille)
#define MOVE_SYNC_MAX_CORR           800     // Correction differentielle max (pour-mille)
#define MOVE_TORQUE_STALL_MS         50      // Delai sans avance avant renfort automatique de couple
#define MOVE_TORQUE_BOOST_STEP       80      // Increment de couple anti-calage (pour-mille)
#define MOVE_TORQUE_BOOST_MAX        700     // Plafond du renfort de couple
#define MOVE_BRAKE_SETTLE_MS         120     // Duree de freinage actif en fin de course
#define MOVE_STALL_TIMEOUT_MS        1500    // Securite si aucun mouvement detecte
#define MOVE_SINGLE_DONE_TIMEOUT_MS  800     // Securite si une seule roue termine et l'autre bloque
#define SPEED_AVG_WINDOW_MS          50      // Fenetre de calcul de la vitesse (evite quantification a 5 ms)

typedef enum {
    MOVE_IDLE = 0,
    MOVE_RUN,
    MOVE_BRAKE_SETTLE
} move_state_t;

typedef struct {
    move_state_t state;
    bool         is_turn;                    // false = translation (mm), true = rotation sur place (deg)
    int8_t       dir;                        // Signe de la consigne utilisateur (+1 ou -1)
    int8_t       wheel_sign[MOTOR_COUNT];    // Signe applique a chaque moteur (+1 ou -1)
    int32_t      target_ticks;               // Cible positive en ticks
    int16_t      base_speed;                 // Consigne nominale (1..1000)
    int32_t      start_pos[MOTOR_COUNT];
    int8_t       enc_sign[MOTOR_COUNT];      // 0 = auto, sinon +1/-1
    int32_t      progress[MOTOR_COUNT];
    bool         wheel_done[MOTOR_COUNT];
    uint32_t     start_tick;
    uint32_t     last_step_tick;
    int32_t      sync_err_int;
    uint32_t     last_progress_tick;
    int32_t      last_total_progress;
    int32_t      boost_last_prog;
    uint32_t     boost_stall_tick;
    int16_t      torque_boost;
    bool         first_done_seen;
    uint32_t     first_done_tick;
    uint32_t     brake_start_tick;
} move_ctrl_t;

static motor_drv_t  s_drv[MOTOR_COUNT];
static motor_info_t s_info[MOTOR_COUNT];
static uint32_t     s_last_update_ms;
static uint32_t     s_last_cmd_ms[MOTOR_COUNT];
static int32_t      s_speed_ticks_acc[MOTOR_COUNT];
static uint32_t     s_speed_dt_acc;
static bool         s_enabled;
static move_ctrl_t  s_move;

static bool id_valid(motor_id_t id){
    return ((unsigned)id < MOTOR_COUNT) && (s_info[id].state != MOTOR_STATE_UNINIT);
}

static void move_cancel(void){
    s_move.state = MOVE_IDLE;
}

static bool apply_speed(motor_id_t id, int16_t permille){
    if (!id_valid(id) || !s_enabled) return false;

    if (permille > MOTOR_DRV_DUTY_MAX)  permille = MOTOR_DRV_DUTY_MAX;
    if (permille < -MOTOR_DRV_DUTY_MAX) permille = -MOTOR_DRV_DUTY_MAX;

#if MOTOR_MIN_DUTY > 0
    // Compensation de zone morte : remappe 1..1000 vers MIN..1000 (0 reste 0)
    if (permille > 0) {
        permille = (int16_t)(MOTOR_MIN_DUTY +
                   ((int32_t)permille * (MOTOR_DRV_DUTY_MAX - MOTOR_MIN_DUTY)) / MOTOR_DRV_DUTY_MAX);
    } else if (permille < 0) {
        permille = (int16_t)(-MOTOR_MIN_DUTY +
                   ((int32_t)permille * (MOTOR_DRV_DUTY_MAX - MOTOR_MIN_DUTY)) / MOTOR_DRV_DUTY_MAX);
    }
#endif

    motor_drv_set_duty(&s_drv[id], permille);
    s_info[id].cmd_permille = permille;
    s_info[id].state = (permille != 0) ? MOTOR_STATE_RUNNING : MOTOR_STATE_STOPPED;
    s_last_cmd_ms[id] = HAL_GetTick();
    return true;
}

static void apply_brake(motor_id_t id){
    if (!id_valid(id)) return;
    motor_drv_brake(&s_drv[id]);
    s_info[id].cmd_permille = 0;
    s_info[id].state = MOTOR_STATE_BRAKING;
}

static int32_t clamp_i32(int32_t val, int32_t lo, int32_t hi){
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

static void refresh_encoders_and_speeds(uint32_t now){
    uint32_t dt = now - s_last_update_ms;
    s_last_update_ms = now;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (!id_valid((motor_id_t)i)) continue;

        int32_t delta = motor_drv_read_delta(&s_drv[i]);
        s_info[i].position_ticks = motor_drv_get_position(&s_drv[i]);
        s_speed_ticks_acc[i] += delta;
    }

    s_speed_dt_acc += dt;
    if (s_speed_dt_acc >= SPEED_AVG_WINDOW_MS) {
        for (int i = 0; i < MOTOR_COUNT; i++) {
            if (!id_valid((motor_id_t)i)) continue;
            s_info[i].speed_tps = (int32_t)(((int64_t)s_speed_ticks_acc[i] * 1000) / (int32_t)s_speed_dt_acc);
            s_info[i].speed_rpm = (int32_t)(((int64_t)s_info[i].speed_tps * 60) / MOTOR_ENC_TICKS_PER_REV);
            s_speed_ticks_acc[i] = 0;
        }
        s_speed_dt_acc = 0;
    }
}

static void move_update_progress(void){
    for (int i = 0; i < MOTOR_COUNT; i++) {
        int32_t raw_diff = s_info[i].position_ticks - s_move.start_pos[i];
        // Auto-detecte le signe reel de comptage de l'encodeur apres un court deplacement,
        // afin de fonctionner meme si invert_enc n'a pas encore ete calibre par moteur.
        if (s_move.enc_sign[i] == 0) {
            if (raw_diff >= MOVE_ENC_SIGN_LOCK_TICKS)       s_move.enc_sign[i] = 1;
            else if (raw_diff <= -MOVE_ENC_SIGN_LOCK_TICKS) s_move.enc_sign[i] = -1;
            s_move.progress[i] = (raw_diff >= 0) ? raw_diff : -raw_diff;
        } else {
            s_move.progress[i] = (int32_t)s_move.enc_sign[i] * raw_diff;
        }
    }
}

static int16_t move_compute_profile_speed(int32_t remaining, uint32_t elapsed_ms){
    if (remaining <= 0) return 0;

    int16_t creep = (s_move.base_speed < MOVE_CREEP_SPEED) ? s_move.base_speed : MOVE_CREEP_SPEED;
    int32_t span  = (int32_t)s_move.base_speed - (int32_t)creep;

    int32_t speed = s_move.base_speed;

    // Rampe de demarrage doux pour eviter qu'un moteur prenne une grosse avance au depart
    if (elapsed_ms < MOVE_ACCEL_MS) {
        int32_t accel_speed = creep + ((int32_t)elapsed_ms * span) / MOVE_ACCEL_MS;
        if (accel_speed < speed) speed = accel_speed;
    }

    // Rampe de deceleration en approche de la cible
    if (remaining < MOVE_RAMP_TICKS) {
        int32_t decel_speed = creep + (remaining * span) / MOVE_RAMP_TICKS;
        if (decel_speed < speed) speed = decel_speed;
    }

    return (int16_t)speed;
}

static void move_apply_sync_diff(int32_t *cmd0, int32_t *cmd1, int32_t corr){
    int32_t c0 = *cmd0 - corr / 2;
    int32_t c1 = *cmd1 + (corr - corr / 2);

    // Si une roue sature au plafond, reporter la correction sur le ralentissement de l'autre
    if (c0 > MOTOR_DRV_DUTY_MAX) {
        c1 -= (c0 - MOTOR_DRV_DUTY_MAX);
        c0 = MOTOR_DRV_DUTY_MAX;
    }
    if (c1 > MOTOR_DRV_DUTY_MAX) {
        c0 -= (c1 - MOTOR_DRV_DUTY_MAX);
        c1 = MOTOR_DRV_DUTY_MAX;
    }

    // Autorise la mise a 0 (roue libre) de la roue trop en avance pour laisser l'autre rattraper
    *cmd0 = clamp_i32(c0, 0, MOTOR_DRV_DUTY_MAX);
    *cmd1 = clamp_i32(c1, 0, MOTOR_DRV_DUTY_MAX);
}

static void move_step(uint32_t now){
    if (s_move.state == MOVE_IDLE) return;

    uint32_t dt_ms = now - s_move.last_step_tick;
    s_move.last_step_tick = now;
    if (dt_ms > 50U) dt_ms = 50U;

    move_update_progress();

    if (s_move.state == MOVE_BRAKE_SETTLE) {
        if ((now - s_move.brake_start_tick) >= MOVE_BRAKE_SETTLE_MS) {
            int32_t p0      = s_move.progress[MOTOR_1];
            int32_t p1      = s_move.progress[MOTOR_2];
            int8_t  dir     = s_move.dir;
            bool    is_turn = s_move.is_turn;
            int32_t avg_t   = (p0 + p1) / 2;

            motor_stop_all();
            if (is_turn) {
                float avg_deg = (float)dir * motor_ticks_to_deg(avg_t);
                printf("\r\n[MOTOR] Rotation terminee : %.1f deg (M1: %ld ticks, M2: %ld ticks)\r\n",
                       (double)avg_deg, (long)p0, (long)p1);
            } else {
                float avg_mm = (float)dir * motor_ticks_to_mm(avg_t);
                printf("\r\n[MOTOR] Trajet termine : %.1f mm (M1: %ld ticks, M2: %ld ticks)\r\n",
                       (double)avg_mm, (long)p0, (long)p1);
            }
        }
        return;
    }

    // Etat MOVE_RUN
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (!s_move.wheel_done[i] && s_move.progress[i] >= s_move.target_ticks) {
            s_move.wheel_done[i] = true;
            apply_brake((motor_id_t)i);
        }
    }

    if (s_move.wheel_done[MOTOR_1] && s_move.wheel_done[MOTOR_2]) {
        s_move.state            = MOVE_BRAKE_SETTLE;
        s_move.brake_start_tick = now;
        return;
    }

    // Securite si une roue a fini mais que l'autre ne suit pas dans le delai imparti
    if (s_move.wheel_done[MOTOR_1] || s_move.wheel_done[MOTOR_2]) {
        if (!s_move.first_done_seen) {
            s_move.first_done_seen = true;
            s_move.first_done_tick = now;
        } else if ((now - s_move.first_done_tick) >= MOVE_SINGLE_DONE_TIMEOUT_MS) {
            apply_brake(MOTOR_1);
            apply_brake(MOTOR_2);
            s_move.state            = MOVE_BRAKE_SETTLE;
            s_move.brake_start_tick = now;
            printf("\r\n[MOTOR] Attention : ecart entre roues en fin de trajet (M1=%ld, M2=%ld / %ld)\r\n",
                   (long)s_move.progress[MOTOR_1], (long)s_move.progress[MOTOR_2], (long)s_move.target_ticks);
            return;
        }
    }

    // Renfort automatique de couple anti-calage + securite blocage total
    int32_t total_prog = s_move.progress[MOTOR_1] + s_move.progress[MOTOR_2];
    if (total_prog > s_move.boost_last_prog + 1) {
        s_move.boost_last_prog  = total_prog;
        s_move.boost_stall_tick = now;
    } else if ((now - s_move.boost_stall_tick) >= MOVE_TORQUE_STALL_MS) {
        s_move.boost_stall_tick = now;
        s_move.torque_boost = (int16_t)clamp_i32(s_move.torque_boost + MOVE_TORQUE_BOOST_STEP,
                                                 0, MOVE_TORQUE_BOOST_MAX);
    }

    if (total_prog > s_move.last_total_progress + 2) {
        s_move.last_total_progress = total_prog;
        s_move.last_progress_tick  = now;
    } else if ((now - s_move.last_progress_tick) >= MOVE_STALL_TIMEOUT_MS) {
        printf("\r\n[MOTOR] Arret securise (blocage ou encodeur inactif : M1=%ld/%ld, M2=%ld/%ld ticks)\r\n",
               (long)s_move.progress[MOTOR_1], (long)s_move.target_ticks,
               (long)s_move.progress[MOTOR_2], (long)s_move.target_ticks);
        motor_stop_all();
        return;
    }

    uint32_t elapsed = now - s_move.start_tick;
    int32_t cmd0 = clamp_i32(move_compute_profile_speed(s_move.target_ticks - s_move.progress[MOTOR_1], elapsed) + s_move.torque_boost,
                             0, MOTOR_DRV_DUTY_MAX);
    int32_t cmd1 = clamp_i32(move_compute_profile_speed(s_move.target_ticks - s_move.progress[MOTOR_2], elapsed) + s_move.torque_boost,
                             0, MOTOR_DRV_DUTY_MAX);

    // Synchronisation PI gauche/droite pour garder la trajectoire droite
    if (!s_move.wheel_done[MOTOR_1] && !s_move.wheel_done[MOTOR_2]) {
        int32_t sync_err = s_move.progress[MOTOR_1] - s_move.progress[MOTOR_2];
        s_move.sync_err_int = clamp_i32(s_move.sync_err_int + sync_err * (int32_t)dt_ms,
                                        -MOVE_SYNC_INT_MAX, MOVE_SYNC_INT_MAX);

        int32_t corr = (sync_err * MOVE_SYNC_KP) + (s_move.sync_err_int / MOVE_SYNC_KI_DIV);
        corr = clamp_i32(corr, -MOVE_SYNC_MAX_CORR, MOVE_SYNC_MAX_CORR);
        move_apply_sync_diff(&cmd0, &cmd1, corr);
    }

    if (!s_move.wheel_done[MOTOR_1]) {
        apply_speed(MOTOR_1, (int16_t)(s_move.wheel_sign[MOTOR_1] * cmd0));
    }
    if (!s_move.wheel_done[MOTOR_2]) {
        apply_speed(MOTOR_2, (int16_t)(s_move.wheel_sign[MOTOR_2] * cmd1));
    }
}

bool motor_init(TIM_HandleTypeDef *htim_pwm,
                TIM_HandleTypeDef *htim_enc1,
                TIM_HandleTypeDef *htim_enc2){
    // Moteur 1 : TIM1 CH1/CH2 (AIN1/AIN2), encodeur TIM2 (32 bits)
    const motor_drv_cfg_t cfg1 = {
        .htim_pwm   = htim_pwm,
        .ch_in1     = TIM_CHANNEL_1,
        .ch_in2     = TIM_CHANNEL_2,
        .htim_enc   = htim_enc1,
        .enc_32bit  = true,
        .invert_dir = false,
        .invert_enc = false
    };

    // Moteur 2 : TIM1 CH3/CH4 (BIN1/BIN2), encodeur TIM3 (16 bits)
    const motor_drv_cfg_t cfg2 = {
        .htim_pwm   = htim_pwm,
        .ch_in1     = TIM_CHANNEL_3,
        .ch_in2     = TIM_CHANNEL_4,
        .htim_enc   = htim_enc2,
        .enc_32bit  = false,
        .invert_dir = false,
        .invert_enc = false
    };

    // Driver en veille pendant la configuration
    motor_drv_sleep(false);

    if (!motor_drv_init(&s_drv[MOTOR_1], &cfg1)) return false;
    if (!motor_drv_init(&s_drv[MOTOR_2], &cfg2)) return false;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        s_info[i] = (motor_info_t){ .state = MOTOR_STATE_STOPPED };
        s_last_cmd_ms[i] = HAL_GetTick();
        s_speed_ticks_acc[i] = 0;
    }
    s_speed_dt_acc   = 0;
    s_last_update_ms = HAL_GetTick();
    move_cancel();

    motor_enable(true);
    return true;
}

void motor_enable(bool enable){
    s_enabled = enable;
    if (!enable) {
        motor_stop_all();
    }
    motor_drv_sleep(enable);
}

bool motor_set_speed(motor_id_t id, int16_t permille){
    move_cancel();
    return apply_speed(id, permille);
}

void motor_stop(motor_id_t id){
    move_cancel();
    if (!id_valid(id)) return;
    motor_drv_coast(&s_drv[id]);
    s_info[id].cmd_permille = 0;
    s_info[id].state = MOTOR_STATE_STOPPED;
}

void motor_brake(motor_id_t id){
    move_cancel();
    apply_brake(id);
}

void motor_stop_all(void){
    move_cancel();
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_stop((motor_id_t)i);
    }
}

void motor_reset_position(motor_id_t id){
    if (!id_valid(id)) return;
    if (s_move.state != MOVE_IDLE) {
        motor_stop_all();
    }
    motor_drv_reset_position(&s_drv[id]);
    s_info[id].position_ticks = 0;
    s_speed_ticks_acc[id] = 0;
}

void motor_update(void){
    uint32_t now = HAL_GetTick();
    refresh_encoders_and_speeds(now);
    move_step(now);

#if MOTOR_CMD_TIMEOUT_MS > 0
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (!id_valid((motor_id_t)i)) continue;
        if (s_move.state == MOVE_IDLE &&
            s_info[i].state == MOTOR_STATE_RUNNING &&
            (now - s_last_cmd_ms[i]) > MOTOR_CMD_TIMEOUT_MS) {
            motor_stop((motor_id_t)i);
        }
    }
#endif
}

void motor_print_diagnostics(void){
    motor_update();

    static const char *state_str[] = {"UNINIT", "ARRET", "FREIN", "EN MARCHE"};

    printf("\r\n========================================\r\n");
    printf("       [APP_MOTOR] DIAGNOSTIC MOTEURS   \r\n");
    printf("========================================\r\n");
    printf("Driver            : %s\r\n", s_enabled ? "ACTIF" : "EN VEILLE");

    for (int i = 0; i < MOTOR_COUNT; i++) {
        const motor_info_t *m = &s_info[i];
        printf("Moteur %d : %s\r\n", i + 1, state_str[m->state]);
        printf("  Commande : %d/1000\r\n", m->cmd_permille);
        printf("  Position : %ld ticks (%.1f mm)\r\n",
               (long)m->position_ticks, (double)motor_ticks_to_mm(m->position_ticks));
        printf("  Vitesse  : %ld ticks/s (%ld tr/min)\r\n", (long)m->speed_tps, (long)m->speed_rpm);
    }
    printf("========================================\r\n\r\n");
}

const motor_info_t* motor_get_info(motor_id_t id){
    if ((unsigned)id >= MOTOR_COUNT) return NULL;
    return &s_info[id];
}

int32_t motor_mm_to_ticks(float mm){
    float circumference = MOVE_PI_F * MOTOR_WHEEL_DIAMETER_MM;
    float ticks = (mm * (float)MOTOR_ENC_TICKS_PER_REV) / circumference;
    return (int32_t)((ticks >= 0.0f) ? (ticks + 0.5f) : (ticks - 0.5f));
}

float motor_ticks_to_mm(int32_t ticks){
    float circumference = MOVE_PI_F * MOTOR_WHEEL_DIAMETER_MM;
    return ((float)ticks * circumference) / (float)MOTOR_ENC_TICKS_PER_REV;
}

int32_t motor_deg_to_ticks(float deg){
    float arc_mm = (MOVE_PI_F * MOTOR_TRACK_WIDTH_MM * deg) / 360.0f;
    return motor_mm_to_ticks(arc_mm);
}

float motor_ticks_to_deg(int32_t ticks){
    float mm = motor_ticks_to_mm(ticks);
    return (mm * 360.0f) / (MOVE_PI_F * MOTOR_TRACK_WIDTH_MM);
}

static bool start_trajectory(bool is_turn,
                             int8_t dir,
                             int8_t m1_sign,
                             int8_t m2_sign,
                             int32_t target_ticks,
                             uint16_t speed_permille){
    if (!id_valid(MOTOR_1) || !id_valid(MOTOR_2) || !s_enabled) return false;

    uint32_t now = HAL_GetTick();
    refresh_encoders_and_speeds(now);

    if (target_ticks <= 0) target_ticks = 1;

    uint16_t speed = (speed_permille == 0) ? MOTOR_MOVE_DEFAULT_SPEED : speed_permille;
    if (speed > MOTOR_DRV_DUTY_MAX) speed = MOTOR_DRV_DUTY_MAX;

    s_move = (move_ctrl_t){
        .state               = MOVE_RUN,
        .is_turn             = is_turn,
        .dir                 = dir,
        .wheel_sign          = { m1_sign, m2_sign },
        .target_ticks        = target_ticks,
        .base_speed          = (int16_t)speed,
        .start_pos           = { s_info[MOTOR_1].position_ticks, s_info[MOTOR_2].position_ticks },
        .start_tick          = now,
        .last_step_tick      = now,
        .last_progress_tick  = now,
        .last_total_progress = 0,
        .boost_stall_tick    = now
    };

    move_step(now);
    return true;
}

bool motor_move_distance_mm(int32_t distance_mm, uint16_t speed_permille){
    if (!id_valid(MOTOR_1) || !id_valid(MOTOR_2) || !s_enabled) return false;

    if (distance_mm == 0) {
        motor_stop_all();
        return true;
    }

    int8_t  dir          = (distance_mm > 0) ? 1 : -1;
    int32_t abs_dist_mm  = (distance_mm > 0) ? distance_mm : -distance_mm;
    int32_t target_ticks = motor_mm_to_ticks((float)abs_dist_mm);

    return start_trajectory(false, dir,
                            dir, (int8_t)(dir * MOTOR_CHASSIS_M2_SIGN),
                            target_ticks, speed_permille);
}

bool motor_turn_angle_deg(int32_t angle_deg, uint16_t speed_permille){
    if (!id_valid(MOTOR_1) || !id_valid(MOTOR_2) || !s_enabled) return false;

    if (angle_deg == 0) {
        motor_stop_all();
        return true;
    }

    int8_t  dir          = (angle_deg > 0) ? 1 : -1;
    int32_t abs_deg      = (angle_deg > 0) ? angle_deg : -angle_deg;
    int32_t target_ticks = motor_deg_to_ticks((float)abs_deg);
    int8_t  spin         = (int8_t)(dir * MOTOR_TURN_DIR_SIGN);

    // En rotation sur place, les 2 roues roulent en sens lineaire oppose :
    // M1 recoit +spin et M2 recoit l'oppose de la marche avant (-spin * MOTOR_CHASSIS_M2_SIGN).
    return start_trajectory(true, dir,
                            spin, (int8_t)(-spin * MOTOR_CHASSIS_M2_SIGN),
                            target_ticks, speed_permille);
}

bool motor_is_moving(void){
    return (s_move.state != MOVE_IDLE);
}
