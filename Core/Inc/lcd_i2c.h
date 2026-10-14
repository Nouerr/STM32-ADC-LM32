#ifndef LCD_I2C_H
#define LCD_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Initialise l'ecran. Retourne HAL_OK si un module repond sur le bus I2C
   (adresse 0x27 ou 0x3F detectee automatiquement), sinon HAL_ERROR. */
HAL_StatusTypeDef LCD_Init(void);

void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);   /* row : 0..1, col : 0..15 */
void LCD_SendString(const char *str);

#ifdef __cplusplus
}
#endif

#endif /* LCD_I2C_H */
