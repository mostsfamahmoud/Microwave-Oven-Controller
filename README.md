# Microwave Oven Controller Project

## `Aim:`
This project simulates the embedded controller in a microwave oven.

## `Hardware Used:`
• LCD

• Keypad

• One External Push Buttons

• TM4C123G Launch Pad ( ***Tiva C*** )

## `Procedure Details:`

### While not cooking (stopped), enter a choice to begin cooking:

#### ***`Case A:`***  
If A is pushed on the keypad (for popcorn), 
> * LCD shows `Popcorn`
> * Cooking for **1 Minute** while the remaining cook time counts down (in seconds) on the LCD, 
> * LCD is cleared after cooking completes.

#### ***`Case B & C:`***  
If B (for Beef) or C (for chicken) is pushed on the keypad, 
> * `Beef weight?` or `Chicken weight?` (respectively) appeares on the LCD. 
> * Then the user must enter an integer value between 1 and 9 on the keypad to indicate how many kilos are there to be defrosted.

     NOTE:  Only whole kilo values are to be entered. 
> * After a valid number is entered, LCD display is cleared and the value of the weight is displayed for 2 seconds.
> * Cooking starts while the remaining cook time counts down (in seconds) on the LCD 

   >> **Beef** is defrosted at a rate of `0.5 minutes per kilo`.
   >>
   >> **Chicken** is defrosted at a rate of `0.2 minutes per kilo`.
   >>
   >> **If an illegal number is entered** , LCD shows `“Err” for 2 seconds`, then shows previous message.
   
#### ***`Case D:`*** 
If D is pushed on the keypad.
> * `Cooking Time?` appeares on the LCD. 
>* After that the user can enter a value between 1 and 30 to indicate the cooking time required in minutes and seconds. 
>* This value is displayed on the LCD as it is entered, right to left. 

***For example :*** 
>Pressing 1 displays `00:01` , Then pressing 2 displays `00:12` , Then pressing 4 displays `01:24` , Then pressing 5 displays `12:45`. 
> * Press _`button1 (SW1)`_ to clear the LCD display.
> * Press _`button2 (SW2)`_ to start cooking and counting down (in seconds) the cooking time on the LCD.

### Start/Stop/Pause cooking conditions:

***Switch SW3*** (external push button) simulates the `microwave oven door` latch where:
1. Switch being down would be simulating the open-door situation 
2. Switch being up would be simulating the door closed situation. 

> Only when the latch is closed should the oven be able to be started.
>> * When **SW1** is pressed for **first time**, `the oven operation pauses` (keeping remaining time on the display).
>> * When **SW1** is pushed for **second time** after pressing it for first time,  `time is cleared` and `the cooking stops`.
>> * When **SW2** is pushed after the _oven is paused_ and the _door is closed_, then `cooking resumes from the time it was paused`.

> While cooking, **Opening the door** pauses the cooking and keep the remaining time on display.

> When the microwave is running, ***The array of LEDs is ON***. When it is stopped, they go off.

>If paused, ***The array of LEDs blinks*** (wait time on and wait time off) till the cooking is resumed or stopped.

> When the microwave completes its function and timer has counted down to zero (regular timed cooking or defrosting), 
>> * The array of LEDs should flash 3 times (wait time on and wait time off), 
>> * The buzzer should produce an audible tone/alarm during these 3 second times periods.
