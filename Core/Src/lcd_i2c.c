#include "lcd_i2c.h"
#include "i2c.h"

/* Adresses 7 bits possibles du module PCF8574 / PCF8574A */
#define LCD_ADDR_1     0x27
#define LCD_ADDR_2     0x3F

/* Broches du convertisseur I2C-PCF8574 */
#define LCD_BACKLIGHT  0x08
#define LCD_ENABLE     0x04
#define LCD_RS         0x01

/* Adresse utilisee (decalee d'un bit pour HAL), trouvee dans LCD_Init() */
static uint16_t lcd_addr = (LCD_ADDR_1 << 1);

/* Visible en debug : HAL_OK si la derniere transmission I2C a reussi */
volatile HAL_StatusTypeDef lcd_i2c_status = HAL_OK;

static void LCD_WriteNibble(uint8_t nibble, uint8_t rs);
static void LCD_SendCommand(uint8_t cmd);
static void LCD_SendData(uint8_t data);

/* Envoie un demi-octet au LCD (3 octets en une seule transaction I2C) */
static void LCD_WriteNibble(uint8_t nibble, uint8_t rs)
{
    uint8_t data = (nibble & 0xF0)
                 | LCD_BACKLIGHT
                 | (rs ? LCD_RS : 0);

    uint8_t buf[3];
    buf[0] = data;                              /* E = 0, donnees presentes */
    buf[1] = data | LCD_ENABLE;                 /* E = 1 : le LCD lit       */
    buf[2] = data & (uint8_t)(~LCD_ENABLE);     /* E = 0                    */

    lcd_i2c_status = HAL_I2C_Master_Transmit(&hi2c1, lcd_addr, buf, 3, 100);
}

/* Envoie une commande au LCD */
static void LCD_SendCommand(uint8_t cmd)
{
    LCD_WriteNibble(cmd & 0xF0, 0);
    LCD_WriteNibble((uint8_t)(cmd << 4) & 0xF0, 0);
}

/* Envoie un caractere au LCD */
static void LCD_SendData(uint8_t data)
{
    LCD_WriteNibble(data & 0xF0, 1);
    LCD_WriteNibble((uint8_t)(data << 4) & 0xF0, 1);
}

/* Initialisation de l'ecran LCD 16x2 */
HAL_StatusTypeDef LCD_Init(void)
{
    /* Detection automatique de l'adresse */
    if (HAL_I2C_IsDeviceReady(&hi2c1, (LCD_ADDR_1 << 1), 3, 50) == HAL_OK)
    {
        lcd_addr = (LCD_ADDR_1 << 1);
    }
    else if (HAL_I2C_IsDeviceReady(&hi2c1, (LCD_ADDR_2 << 1), 3, 50) == HAL_OK)
    {
        lcd_addr = (LCD_ADDR_2 << 1);
    }
    else
    {
        lcd_i2c_status = HAL_ERROR;
        return HAL_ERROR;   /* Aucun module : verifier SCL/SDA, 5V, GND */
    }

    HAL_Delay(50);

    /* Sequence d'init du HD44780 en mode 4 bits */
    LCD_WriteNibble(0x30, 0);
    HAL_Delay(5);

    LCD_WriteNibble(0x30, 0);
    HAL_Delay(1);

    LCD_WriteNibble(0x30, 0);
    HAL_Delay(1);

    LCD_WriteNibble(0x20, 0);   /* Passage en mode 4 bits */
    HAL_Delay(1);

    LCD_SendCommand(0x28);      /* 4 bits, 2 lignes, police 5x8 */
    LCD_SendCommand(0x0C);      /* Affichage ON, curseur masque */
    LCD_SendCommand(0x06);      /* Curseur auto-incremente      */
    LCD_SendCommand(0x01);      /* Effacer l'ecran              */
    HAL_Delay(2);

    return lcd_i2c_status;
}

/* Efface l'ecran */
void LCD_Clear(void)
{
    LCD_SendCommand(0x01);
    HAL_Delay(2);
}

/* Positionne le curseur : row = 0 ou 1, col = 0 a 15 */
void LCD_SetCursor(uint8_t row, uint8_t col)
{
    uint8_t address = (row == 0) ? (0x00 + col) : (0x40 + col);
    LCD_SendCommand(0x80 | address);
}

/* Affiche une chaine de caracteres */
void LCD_SendString(const char *str)
{
    while (*str != '\0')
    {
        LCD_SendData((uint8_t)*str);
        str++;
    }
}
