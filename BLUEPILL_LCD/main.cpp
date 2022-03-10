#include "mbed.h"
#include "TextLCD.h"
#include <cstdlib>

#define BLINKING_RATE     1000

I2C lcd_i2c(PB_7,PB_6);

CAN can(PB_8,PB_9);

SPISlave spi(PA_7,PA_6,PA_5,NC);

DigitalOut led(PC_13);

TextLCD_I2C lcd(&lcd_i2c,0x4E,TextLCD::LCD16x2);

int main()
{   
    spi.format(16,0);
    spi.frequency(5000000);

    long podatak;
    float d;
    int id;
    short temp,lenght,podatak_spi,control_word;
    CANMessage poruka;

    lcd.cls();
    lcd.setBacklight(TextLCD::LightOn);
    lcd.cls();
    lcd.locate(0,0);
    lcd.printf("D:");
    lcd.locate(8,0);
    lcd.printf("T:");
    led=1;

    while(1){
        if(spi.receive()){                   //provjera jeli spi prima podatke
            control_word = spi.read();       //očitavanje kontolne rijeci
            switch (control_word) {
            case 0x8A:                       //kontrolna rijec za podatak duljine 
            while(!spi.receive()){}          //beskonacni while loop dok spi ne prima podatke tj. ceka se da se pošalje podatak
            if(spi.receive()){
                lenght = spi.read();         //očitavanje podatka duljine
                lcd.locate(2,0);
                lcd.printf("      ");
                lcd.locate(2,0);
                lcd.printf("%dcm",lenght);
                led=!led;
            }
            break;
            case 0x8D:                              //kontrolna rijec za podatak temperature
            while(!spi.receive()){}                 //beskonacni while loop dok spi ne prima podatke tj. ceka se da se pošalje podata
            if(spi.receive()){
                temp = spi.read();                  //očitavanje podatka temperature
                lcd.locate(10,0);
                lcd.printf("      ");
                lcd.locate(10,0);
                lcd.printf("%.2fC",float(temp)/100);//ispis temperature tako da se prije ispisa podatak podjeli sa 100
                led=!led;
            }
            break;
            }          
        }
        if(can.read(poruka)){                       //provjera jeli can bus prima podatke 
            id = poruka.id;                         //spremanje can bus id podatka
            switch (id) {
            case 1:                                 //id=1 --> podatak je x
            podatak = poruka.data[1] << 8;          //spremanje podatka velicine 2 bajta 
            podatak = podatak | poruka.data[0];
            d = float(short(podatak))/100;          //pretvorba podatka u float vrijednost tako da se podjeli sa 100 
            lcd.locate(0,1);
            lcd.printf("     ");
            lcd.locate(0,1);
            lcd.printf("%+.2f",d);
            break;
            case 2:                                 //id=2 --> podatak je y
            podatak = poruka.data[1] << 8;          //spremanje podatka velicine 2 bajta
            podatak = podatak | poruka.data[0];
            d = float(short(podatak))/100;          //pretvorba podatka u float vrijednost tako da se podjeli sa 100
            lcd.locate(5,1);
            lcd.printf("      ");
            lcd.locate(5,1);
            lcd.printf("%+.2f",d);
            break;
            case 3:                               //id=3 --> podatak je z
            podatak = poruka.data[1] << 8;        //spremanje podatka velicine 2 bajta
            podatak = podatak | poruka.data[0];
            d = float(short(podatak))/100;        //pretvorba podatka u float vrijednost tako da se podjeli sa 100
            lcd.locate(10,1);
            lcd.printf("     ");
            lcd.locate(10,1);
            lcd.printf("%+.2f",d);
            }
        }
    }
}
