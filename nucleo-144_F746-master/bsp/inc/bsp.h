/*
 * bsp.h
 *
 *  Created on: 5 août 2017
 *      Author: Laurent
 */

#ifndef BSP_INC_BSP_H_
#define BSP_INC_BSP_H_

#include "stm32f7xx.h"

/*
 * LED driver functions
 */

#define	GREEN	0
#define BLUE	1
#define	RED		2

void	BSP_LED_Init	(void);
void	BSP_LED_On		(uint8_t id);
void	BSP_LED_Off		(uint8_t id);
void	BSP_LED_Toggle	(uint8_t id);


/*
 * Push-Button driver functions
 */

void	BSP_PB_Init		(void);
uint8_t	BSP_PB_GetState	(void);


/*
 * Debug Console driver functions
 */

void	BSP_Console_Init	(void);



#endif /* BSP_INC_BSP_H_ */
