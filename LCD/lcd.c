/******************************************************************************
 *
 * Module: LCD
 *
 * File Name: LCD.c
 *
 * Description: source file for The LCD Driver.
 *
 * Author: Sherin Sameh Abd El-Samad Ali
 *         Esraa Amr Abdelmoneam Hassan Kandil
 *******************************************************************************/

#include "lcd.h"

/*******************************************************************************
 *                           Functions Definitions                             *
 *******************************************************************************/

/*
 * Description :
 * Initialization of the LCD.
 * Register(s) : GPIO_DIR , GPIO_DATA
 */


void LCD_INIT(void)
{
    GPIO_Init(PORTA_ID);
    GPIO_Init(PORTB_ID);
    GPIO_setPortDirection(DATA_PORT, 0xFF);
    GPIO_setPinDirection(CTRL_PORT, Enable, PIN_OUTPUT);
    GPIO_setPinDirection(CTRL_PORT, RS, PIN_OUTPUT);
    lcd_command(Function_set_8bit);
    Generic_delay_m_sec(1);
    lcd_command(moveCursorRight); // set to entry mode
    Generic_delay_m_sec(1);
    lcd_command(cursorBlink); //  display is on and cursor blinking
    Generic_delay_m_sec(1);
    lcd_command(clear_display); // clear screen
    Generic_delay_m_sec(2)
    lcd_command(returnHome); // return home
    Generic_delay_m_sec(2);
}
/*
 * Description :
 * send commands to LCD.
 * Register(s) : GPIO_DATA.
 */
void lcd_command(unsigned char cmd)
{
    GPIO_writePort(DATA_PORT, cmd);
    GPIO_writePort(CTRL_PORT, 0x08);
    Generic_delay_m_sec(1);
    GPIO_writePort(CTRL_PORT, 0x00); 
    Generic_delay_m_sec(50);

    return;
}
/*
 * Description :
 * Sending Char.
 * Register(s) : CTRL_PORT,DATA_PORT.
 */
void send_char(unsigned char data) //-----> recive only one char
{
    GPIO_writePort(DATA_PORT, data);
    // GPIO_PORTB_DATA_R=data;
    GPIO_writePort(CTRL_PORT, 0x0C);
    // GPIO_PORTE_DATA_R=0X05;
    Generic_delay_m_sec(1);
    GPIO_writePort(CTRL_PORT, 0x04);
    // GPIO_PORTE_DATA_R=0X01;
    Generic_delay_m_sec(50);

    return;
}
/*
 * Description :
 * Sending String.
 * Register(s) : CTRL_PORT,DATA_PORT.
 */
void send_String(unsigned char *data)
{
    while (*data)
    {
        send_char(*data);
        data++;
    }
}
/*
 * Description :
 * move cursor to write data on preffered row and column
 * Register(s) : CTRL_PORT,DATA_PORT.
 */
void LCD_Move_Curser(unsigned char row, unsigned char col)
{
    unsigned char position = 0;

    if (row == 1)
    {
        position = (FirstRow) + col - 1;
    }
    else if (row == 2)
    {
        position = (SecondRow) + col - 1;
    }
    else
    {
        position = FirstRow;
    }
    lcd_command(position);
    Generic_delay_m_sec(1);
}

/*
 * Description :
 * countdown time on LCD.
 * Register(s) : CTRL_PORT,DATA_PORT.
 */


void LCD_COUNT_DOWN(float TotalTime, uint8_t key)
{
    uint8_t pause_flag = 0;
    uint8_t SW2_flag = 0;
    uint8_t state = key;
    float Time_At_Pause = 0.0f;
    int Minutes1 = 0, Minutes2 = 0;
    int Seconds1 = 0, Seconds2 = 0;
    float Seconds;
    int Minutes = (int)TotalTime;
    Seconds = TotalTime - Minutes;
    

    if ((key == 'A') || (key == 'B') || (key == 'C'))
    {
        Seconds = Seconds * 0.6;
        Seconds2 = (int)(Seconds * 10);
        Seconds1 = (int)(Seconds * 100 - Seconds2 * 10);
        Minutes2 = 0;
        Minutes1 = Minutes;
    }
    else if ((key = 'D'))
    {
        Seconds = Seconds * 0.6;
        Seconds2 = (int)(Seconds * 10);
        Seconds1 = (int)(Seconds * 100 - Seconds2 * 10);
        Minutes2 = (int)(Minutes / 10);
        Minutes1 = (int)(Minutes % 10);
    }

    while (1)
    {
        RBG_OUTPUT(PF123_mask);
        lcd_command(clear_display);
        send_char(Minutes2 + 48);
        send_char(Minutes1 + 48);
        send_char(':');
        send_char(Seconds2 + 48);
        send_char(Seconds1 + 48);
        Generic_delay_sec(1);

        if ((SW1_INPUT() == 0 && (pause_flag == 0)))
        {
            lcd_command(clear_display);
            pause_flag = 1;
            send_String("Cooking Paused");
            Generic_delay_sec(2);
            lcd_command(clear_display);
            Minutes = Minutes1 + (Minutes2 * 10);
            Seconds = Seconds1 + (Seconds2 * 10);
            Time_At_Pause = Minutes + Seconds / 60.0;

            while (!(SW1_INPUT() == 0))
            {
                LED_Blinking();
                if (SW2_INPUT() == 0)
                {
                    lcd_command(clear_display);
                    Generic_delay_sec(1);
                    LCD_COUNT_DOWN(Time_At_Pause, state);
                    SW2_flag++;
                    break;
                }
            }
            break;
        }

        if ((BUTTON_READ() == 0))
        {
            lcd_command(clear_display);
            send_String("DOOR IS OPEN");
            
            
            Minutes = Minutes1 + (Minutes2 * 10);
            Seconds = Seconds1 + (Seconds2 * 10);
            Time_At_Pause = Minutes + Seconds / 60.0;

            while ((BUTTON_READ() == 0))
            {
                LED_Blinking();
            }

            lcd_command(clear_display);
            Generic_delay_sec(1);
            LCD_COUNT_DOWN(Time_At_Pause, state);
            SW2_flag++;
            break;
        }

        if (Seconds1 != 0)
        {
            Seconds1--;
            continue;
        }
        else if (Seconds1 == 0 && Seconds2 != 0)
        {
            Seconds1 = 9;
            Seconds2--;
            continue;
        }
        else if (Seconds2 == 0 && Seconds1 == 0 && Minutes1 != 0)
        {
            Seconds2 = 5;
            Seconds1 = 9;
            Minutes1--;
            continue;
        }
        else if (Seconds2 == 0 && Seconds1 == 0 && Minutes1 == 0 && Minutes2 != 0)
        {
            Seconds2 = 5;
            Seconds1 = 9;
            Minutes2--;
            Minutes1 = 9;
            continue;
        }
        else
        {
            break;
        }
    }

    if (SW2_flag == 0)
    {
        lcd_command(clear_display);
				LED_BUZZER();
        send_String(" Cooking Done ");
        
        RBG_OUTPUT(0x00);
    }
}
