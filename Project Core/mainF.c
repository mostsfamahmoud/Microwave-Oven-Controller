/******************************************************************************
 *
 * Module: Main Function
 *
 * File Name: mainF.c
 *
 * Description: Source file for Main Functions used in our project.
 *
 *******************************************************************************/

#include "mainF.h"

/*******************************************************************************
 *                           Functions Definitions                             *
 *******************************************************************************/

/*
 * Description:
 *   1) This Function implements State D in our projects.
 *   2) It supports taking numeric inputs from the keypad and do some processing on the taken data.
 *   3) It also supports displaying "ERR" for any unexpected event.
 *   4) if everything is correct then it starts displaying cooking time as [??:??] on the LCD 
 *      according to the states of Switches
 */

void state_D()
{
    uint8_t min1 = 0, min2 = 0;
    uint8_t sec1 = 0, sec2 = 0;

    float MIN2 = 0.0f, MIN1 = 0.0f;
    float SEC2 = 0.0f, SEC1 = 0.0f;

    float TotalSeconds = 0.0f, TotalTimeInMinutes = 0.0f;

    do
    {
        send_String("Cooking Time?");
        Generic_delay_sec(2);
        lcd_command(clear_display);

        send_String("00:0");
        min2 = KEYPAD_READ();
        Generic_delay_m_sec(200);

        if ((min2 < '3') && (min2 >= '0'))
            break;
        else
        {
            lcd_command(clear_display);
            send_String("Err");
            Generic_delay_sec(2);
            lcd_command(clear_display);
            continue;
        }

    } while (!((min2 < '3') && (min2 >= '0')));

    // Displaying the second digit of Minutes     MAX(29:59)

    send_char(min2);
    Generic_delay_m_sec(900);
    lcd_command(clear_display);

    // Taking Input from user as the first digit of Minutes and display it

    send_String("00:");
    min1 = KEYPAD_READ();
    Generic_delay_m_sec(200);
    send_char(min2);
    send_char(min1);
    Generic_delay_m_sec(900);
    lcd_command(clear_display);

    // To check on the Second Digit of Seconds won't exceed 5
    do
    {
        send_char('0');
        sec2 = KEYPAD_READ();
        Generic_delay_m_sec(200);
        send_char(min2);
        send_char(':');
        send_char(min1);
        if ((sec2 < '6') && (sec2 >= '0'))
            break;
        else
        {
            lcd_command(clear_display);
            send_String("Err");
            Generic_delay_sec(2);
            lcd_command(clear_display);
            continue;
        }
    } while (!((sec2 < '6') && (sec2 >= '0')));

    // Displaying the second digit of Seconds     MAX(29:59)

    send_char(sec2);
    Generic_delay_m_sec(900);
    lcd_command(clear_display);

    // Taking Input from user as the first digit of Seconds and display it

    sec1 = KEYPAD_READ();
    Generic_delay_m_sec(200);
    send_char(min2);
    send_char(min1);
    send_char(':');
    send_char(sec2);
    send_char(sec1);
    Generic_delay_sec(1);

    // CASE D COUNTING DOWN

    MIN2 = (min2 - '0') * 10.0f;
    MIN1 = min1 - '0';
    SEC2 = (sec2 - '0') * 10.0f;
    SEC1 = sec1 - '0';

    TotalSeconds = SEC1 + SEC2;
    TotalTimeInMinutes = (MIN1 + MIN2) + ((TotalSeconds) / 60);

    while (1)
    {
        if (SW_INPUT() == Switch2)
        {
            while (BUTTON_READ() == 0)
            {
                lcd_command(clear_display);
                send_String(" DOOR IS OPEN");
                Generic_delay_m_sec(500);
                lcd_command(clear_display);
            }
            LCD_COUNT_DOWN(TotalTimeInMinutes, 'D');
        }

        else if (SW_INPUT() == Switch1)
        {
            lcd_command(clear_display);
            state_D();
        }
    }
}
