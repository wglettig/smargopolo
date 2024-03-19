#include "prigoLCD_lib.h"

int redraw(LCDClass lcd) {
    lcd.open_port(4);
    lcd.cls();
    lcd.drawLogo(61,0);
    lcd.drawbox();
    lcd.close_port();
    return 0;
}


int main()
{
    LCDClass lcd;
    
    redraw(lcd);

    lcd.open_port(4); 
    lcd.line1    ("Testing, 123 ...");
    char msg [256];
    sprintf (msg, "123456789012345678");
    lcd.line2(msg);
    lcd.line3("BUTTON1   BUTTON2    ");
    lcd.close_port();

    int i = 0;
    while (1) {
        lcd.open_port(4); 
        lcd.toggle();
        lcd.close_port();
        printf("Look:%d\n",i);
        i++;
    }


}

