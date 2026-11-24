/*
 * functions.h
 *
 *  Created on: 5 oct. 2026
 *      Author: Talha
 */

#ifndef INC_FUNCTIONS_H_
#define INC_FUNCTIONS_H_

// Teste la présence du capteur (adresse 0x52) et lit son ID (0xEE attendu).
// Retourne un message à afficher sur l'UART.
const char *I2C_getBack(void);

#endif /* INC_FUNCTIONS_H_ */
