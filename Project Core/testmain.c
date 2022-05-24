#include "mainF.h"

int main()
{
	uint8_t keyy;
	LCD_INIT();
	RBGLED_init();
	Buzzer_init();
	KEYPAD_INIT();
	BUTTON_INIT();
	SW1_Init();
	SW2_Init();

	keyy = KEYPAD_READ();
	Generic_delay_m_sec(200);
	if (keyy == 'A')
		state_A();
	else if (keyy == 'B')
		state_B();

	else if (keyy == 'C')
		state_C();

	else if (keyy == 'D')
		state_D();

	else{
		lcd_command(clear_display);
		send_String(" Invalid Input ");
		Generic_delay_sec(2);
		lcd_command(clear_display);
	}
}
