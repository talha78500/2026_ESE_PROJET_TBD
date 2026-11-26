#ifndef APP_SHELL_H
#define APP_SHELL_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @file shell.h
 * @brief Couche Applicative (APP) : mini shell sur la console UART.
 *        Edition de ligne (echo, backspace), decoupage en arguments et table de commandes.
 *
 *        Commandes disponibles : help, motor, move, turn, stop, brake, status, reset, power, battery, ota, shutdown.
 *        Pour ajouter une commande : ecrire un handler "int cmd_xxx(int argc, char **argv)"
 *        dans shell.c et l'ajouter dans la table s_cmds[].
 */

#define SHELL_LINE_MAX   64     // Taille max d'une ligne
#define SHELL_ARGV_MAX   6      // Nombre max d'arguments (commande comprise)

/** @brief Initialise le shell (stdout non bufferise, banniere, prompt). Appeler apres uart_console_init(). */
void shell_init(void);

/** @brief Traite les octets recus. A appeler le plus souvent possible dans la boucle principale. */
void shell_process(void);

#endif /* APP_SHELL_H */
