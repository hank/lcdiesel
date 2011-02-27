#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "lcd.h"

int main(void)
{
    /* initialize display, cursor off */
    lcd_init(LCD_DISP_ON);
    lcd_command(LCD_FUNCTION_4BIT_2LINES );
    lcd_clrscr();

    lcd_puts("Test");
    lcd_puts_p(PSTR("Test"));

    for(;;);
}
