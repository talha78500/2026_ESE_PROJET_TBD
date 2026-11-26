#include "shell.h"
#include "uart_console.h"
#include "motor.h"
#include "power.h"
#include "battery.h"
#include "ota.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SHELL_PROMPT  "> "

typedef int (*shell_handler_t)(int argc, char **argv);

typedef struct {
    const char     *name;
    const char     *usage;
    const char     *help;
    shell_handler_t fn;
} shell_cmd_t;

static char    s_line[SHELL_LINE_MAX];
static uint8_t s_len;
static char    s_prev;

/* ------------------------------------------------------------------------- */
/* Utilitaires de parsing                                                     */
/* ------------------------------------------------------------------------- */

static bool parse_long(const char *s, long *out){
    char *end = NULL;
    long  v   = strtol(s, &end, 10);
    if (end == s || *end != '\0') return false;
    *out = v;
    return true;
}

/* "1" -> MOTOR_1, "2" -> MOTOR_2 */
static bool parse_motor_id(const char *s, motor_id_t *id){
    long v;
    if (!parse_long(s, &v) || v < 1 || v > (long)MOTOR_COUNT) return false;
    *id = (motor_id_t)(v - 1);
    return true;
}

/* ------------------------------------------------------------------------- */
/* Commandes                                                                  */
/* ------------------------------------------------------------------------- */

static int cmd_help(int argc, char **argv);

static int cmd_motor(int argc, char **argv){
    long speed;

    if (argc != 3 || !parse_long(argv[2], &speed)) {
        printf("Usage : motor <1|2|all> <-1000..1000>\r\n");
        return -1;
    }
    if (speed < -1000 || speed > 1000) {
        printf("Erreur : vitesse hors limites (-1000..1000)\r\n");
        return -1;
    }

    if (strcmp(argv[1], "all") == 0) {
        // Commande simultanee des 2 moteurs.
        // Les moteurs etant montes face a face (en miroir) sur le chassis,
        // la consigne du Moteur 2 est inversee (-speed) pour que les deux roues
        // tournent dans le meme sens de marche (avance / recul).
        int16_t speed1 = (int16_t)speed;
        int16_t speed2 = (int16_t)(-speed);

        bool ok1 = motor_set_speed(MOTOR_1, speed1);
        bool ok2 = motor_set_speed(MOTOR_2, speed2);

        if (!ok1 || !ok2) {
            printf("Erreur : moteurs non initialises ou driver en veille\r\n");
            return -1;
        }
        printf("Moteurs all -> M1: %d, M2: %d\r\n", speed1, speed2);
        return 0;
    }

    motor_id_t id;
    if (!parse_motor_id(argv[1], &id)) {
        printf("Usage : motor <1|2|all> <-1000..1000>\r\n");
        return -1;
    }
    if (!motor_set_speed(id, (int16_t)speed)) {
        printf("Erreur : moteur non initialise ou driver en veille\r\n");
        return -1;
    }
    printf("Moteur %d -> %ld\r\n", (int)id + 1, speed);
    return 0;
}

static int cmd_stop(int argc, char **argv){
    motor_id_t id;

    if (argc == 1 || (argc == 2 && strcmp(argv[1], "all") == 0)) {
        motor_stop_all();
        printf("Tous les moteurs arretes\r\n");
        return 0;
    }
    if (argc == 2 && parse_motor_id(argv[1], &id)) {
        motor_stop(id);
        printf("Moteur %d arrete\r\n", (int)id + 1);
        return 0;
    }
    printf("Usage : stop [1|2|all]\r\n");
    return -1;
}

static int cmd_brake(int argc, char **argv){
    motor_id_t id;

    if (argc == 2 && strcmp(argv[1], "all") == 0) {
        for (int i = 0; i < MOTOR_COUNT; i++) motor_brake((motor_id_t)i);
        printf("Frein sur tous les moteurs\r\n");
        return 0;
    }
    if (argc == 2 && parse_motor_id(argv[1], &id)) {
        motor_brake(id);
        printf("Frein moteur %d\r\n", (int)id + 1);
        return 0;
    }
    printf("Usage : brake <1|2|all>\r\n");
    return -1;
}

static int cmd_status(int argc, char **argv){
    (void)argc; (void)argv;
    motor_print_diagnostics();
    return 0;
}

static int cmd_reset(int argc, char **argv){
    motor_id_t id;

    if (argc == 2 && strcmp(argv[1], "all") == 0) {
        for (int i = 0; i < MOTOR_COUNT; i++) motor_reset_position((motor_id_t)i);
        printf("Positions remises a zero\r\n");
        return 0;
    }
    if (argc == 2 && parse_motor_id(argv[1], &id)) {
        motor_reset_position(id);
        printf("Position moteur %d remise a zero\r\n", (int)id + 1);
        return 0;
    }
    printf("Usage : reset <1|2|all>\r\n");
    return -1;
}

static int cmd_power(int argc, char **argv){
    (void)argc; (void)argv;
    power_print_diagnostics();
    return 0;
}

static int cmd_battery(int argc, char **argv){
    (void)argc; (void)argv;
    battery_print_diagnostics();
    return 0;
}

static int cmd_shutdown(int argc, char **argv){
    (void)argc; (void)argv;
    power_shutdown();
    return 0;
}

static int cmd_ota(int argc, char **argv){
    (void)argc; (void)argv;
    ota_enter_bootloader();
    return 0;
}

static bool parse_opt_speed(int argc, char **argv, int idx, uint16_t *speed_out){
    long speed = 0;
    if (argc > idx) {
        if (!parse_long(argv[idx], &speed) || speed < 1 || speed > 1000) {
            printf("Erreur : vitesse hors limites (1..1000)\r\n");
            return false;
        }
    }
    *speed_out = (uint16_t)speed;
    return true;
}

static int cmd_move(int argc, char **argv){
    long     dist_mm;
    uint16_t speed = 0;

    if ((argc != 2 && argc != 3) || !parse_long(argv[1], &dist_mm)) {
        printf("Usage : move <distance_mm> [vitesse 1..1000]\r\n");
        return -1;
    }
    if (!parse_opt_speed(argc, argv, 2, &speed)) return -1;

    if (!motor_move_distance_mm((int32_t)dist_mm, speed)) {
        printf("Erreur : moteurs non initialises ou driver en veille\r\n");
        return -1;
    }

    if (dist_mm == 0) {
        printf("Distance nulle : moteurs arretes\r\n");
        return 0;
    }

    int32_t  abs_mm    = (dist_mm > 0) ? (int32_t)dist_mm : (int32_t)(-dist_mm);
    int32_t  ticks     = motor_mm_to_ticks((float)abs_mm);
    uint16_t eff_speed = (speed > 0) ? speed : MOTOR_MOVE_DEFAULT_SPEED;

    printf("Deplacement : %ld mm (%ld ticks), vitesse %u/1000 (tapez 'stop' pour annuler)\r\n",
           dist_mm, (long)ticks, eff_speed);
    return 0;
}

static int cmd_turn(int argc, char **argv){
    long     angle_deg;
    uint16_t speed = 0;

    if ((argc != 2 && argc != 3) || !parse_long(argv[1], &angle_deg)) {
        printf("Usage : turn <angle_deg> [vitesse 1..1000]\r\n");
        return -1;
    }
    if (!parse_opt_speed(argc, argv, 2, &speed)) return -1;

    if (!motor_turn_angle_deg((int32_t)angle_deg, speed)) {
        printf("Erreur : moteurs non initialises ou driver en veille\r\n");
        return -1;
    }

    if (angle_deg == 0) {
        printf("Angle nul : moteurs arretes\r\n");
        return 0;
    }

    int32_t  abs_deg   = (angle_deg > 0) ? (int32_t)angle_deg : (int32_t)(-angle_deg);
    int32_t  ticks     = motor_deg_to_ticks((float)abs_deg);
    uint16_t eff_speed = (speed > 0) ? speed : MOTOR_MOVE_DEFAULT_SPEED;

    printf("Rotation : %ld deg (%ld ticks/roue), vitesse %u/1000 (tapez 'stop' pour annuler)\r\n",
           angle_deg, (long)ticks, eff_speed);
    return 0;
}

static const shell_cmd_t s_cmds[] = {
    { "help",     "help",                        "Liste des commandes",               cmd_help     },
    { "motor",    "motor <1|2|all> <-1000..1000>", "Commande un moteur (pour-mille)", cmd_motor    },
    { "move",     "move <dist_mm> [vit 1..1000]", "Avance/recule d'une distance (mm)", cmd_move     },
    { "turn",     "turn <angle_deg> [vit 1..1000]", "Tourne sur place d'un angle (deg)", cmd_turn   },
    { "stop",     "stop [1|2|all]",              "Arret en roue libre",               cmd_stop     },
    { "brake",    "brake <1|2|all>",             "Arret avec frein",                  cmd_brake    },
    { "status",   "status",                      "Etat des moteurs (vitesse, pos)",   cmd_status   },
    { "reset",    "reset <1|2|all>",             "Remise a zero de la position",      cmd_reset    },
    { "power",    "power",                       "Diagnostic alimentation",           cmd_power    },
    { "battery",  "battery",                     "Diagnostic jauge de batterie",      cmd_battery  },
    { "ota",      "ota",                         "Redemarre en mode OTA (UART3)",     cmd_ota      },
    { "shutdown", "shutdown",                    "Eteint le robot (Ship Mode)",       cmd_shutdown },
};
#define SHELL_CMD_COUNT  (sizeof(s_cmds) / sizeof(s_cmds[0]))

static int cmd_help(int argc, char **argv){
    (void)argc; (void)argv;
    printf("Commandes :\r\n");
    for (size_t i = 0; i < SHELL_CMD_COUNT; i++) {
        printf("  %-30s %s\r\n", s_cmds[i].usage, s_cmds[i].help);
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
/* Execution d'une ligne                                                      */
/* ------------------------------------------------------------------------- */

static void shell_execute(char *line){
    char *argv[SHELL_ARGV_MAX];
    int   argc = 0;
    char *p    = line;

    // Decoupage en place : les espaces deviennent des '\0'
    while (*p != '\0' && argc < SHELL_ARGV_MAX) {
        while (*p == ' ' || *p == '\t') *p++ = '\0';
        if (*p == '\0') break;
        argv[argc++] = p;
        while (*p != '\0' && *p != ' ' && *p != '\t') p++;
    }
    if (argc == 0) return;

    for (size_t i = 0; i < SHELL_CMD_COUNT; i++) {
        if (strcmp(argv[0], s_cmds[i].name) == 0) {
            s_cmds[i].fn(argc, argv);
            return;
        }
    }
    printf("Commande inconnue : %s (tapez 'help')\r\n", argv[0]);
}

/* ------------------------------------------------------------------------- */
/* API                                                                        */
/* ------------------------------------------------------------------------- */

void shell_init(void){
    // stdout non bufferise : l'echo et le prompt (sans '\n') partent immediatement
    setvbuf(stdout, NULL, _IONBF, 0);

    s_len  = 0;
    s_prev = 0;

    printf("\r\n=== Shell robot === (tapez 'help')\r\n");
    printf(SHELL_PROMPT);
}

void shell_process(void){
    uint8_t c;

    while (uart_console_read_byte(&c)) {
        if (c == '\r' || c == '\n') {
            // Gere les terminaux qui envoient \r\n : le \n apres un \r est ignore
            if (c == '\n' && s_prev == '\r') {
                s_prev = (char)c;
                continue;
            }
            s_prev = (char)c;

            printf("\r\n");
            s_line[s_len] = '\0';
            if (s_len > 0) {
                shell_execute(s_line);
            }
            s_len = 0;
            printf(SHELL_PROMPT);
        }
        else if (c == 0x08 || c == 0x7F) {          // Backspace / Delete
            s_prev = (char)c;
            if (s_len > 0) {
                s_len--;
                printf("\b \b");
            }
        }
        else if (c >= 32 && c < 127) {              // Caractere imprimable
            s_prev = (char)c;
            if (s_len < SHELL_LINE_MAX - 1) {
                s_line[s_len++] = (char)c;
                putchar(c);                         // Echo
            }
        }
    }
}
