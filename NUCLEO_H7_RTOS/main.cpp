#include "mbed.h"
#include <BufferedSerial.h>
#include <cmath>
#include "TextLCD.h"
#include "math.h"

#define adxl_w 0xA6 // registar adrese za pisanje
#define adxl_r 0xA7 // registar adrese za čitanje
#define adxl_df 0x31 // registar za format podataka
#define adxl_dr 0x0F // registar za brzinu prijenosa podataka
#define adxl_pc 0x2D // registar za kontrolu snage

#define MBED_CONF_APP_CAN1_RD PD_0
#define MBED_CONF_APP_CAN1_TD PD_1

Queue<int, 1> queue1; //udaljenost
Queue<float, 3> queue2; //akceleracija
Queue<double, 1> queue3; //temeratura

Mutex mutex;

I2C i2c(PF_0,PF_1);
I2C lcd_i2c(PB_9,PB_8);

SPI spi(PC_12,PC_11,PC_10);

CAN can(MBED_CONF_APP_CAN1_RD, MBED_CONF_APP_CAN1_TD);

DigitalOut trig(PG_2);
InterruptIn echo(PG_3);

DigitalOut led(LED1);

Timer timer;

Thread task1(osPriorityNormal,500,NULL,NULL);
Thread task2(osPriorityAboveNormal,1024,NULL,NULL);
Thread task3(osPriorityNormal,1024,NULL,NULL);
Thread task4(osPriorityNormal,3000,NULL,NULL);

int  adresa=0x1F;

int16_t g_force_raw_x,g_force_raw_y,g_force_raw_z;

float  a = 0.0039;

char data_format[2] = {adxl_df,0x0B}, data_rate[2] = {adxl_dr,0x0F}, 
     power_control[2] = {adxl_pc,0x08},data[6],adxl_x0[1] = {0x32},
     adxl_x1[1] = {0x33}, adxl_y0[1] = {0x34}, adxl_y1[1] = {0x35},
     adxl_z0[1] = {0x36}, adxl_z1[1] = {0x37},msb ,lsb,clr_fl[1] = {0x1F},
     Ta_pointer[1] = {0x05},msb_lsb_bajtovi[2], Con_pointer_msb_lsb[3] = {0x01,0x00,0x19},
     Tu_pointer_msb_lsb[3] = {0x02,0x03,0xC0}, Tl_pointer_msb_lsb[3] = {0x03,0x00,0xA0}, 
     Tc_pointer_msb_lsb[3] = {0x04,0x06,0x40}, rezolucija[2] = {0x08,0x01};

void task_echo()               //void task za trigger input za modul od 10us
{
    while(1){
        trig=1;
        wait_us(10);
        trig=0;
        thread_sleep_for(100);
    }  
}

void ec()                                             //void funkcija koja se aktivira interrupt ulazom echo
{
  timer.reset();                                      //resetiranje tajmera
  timer.start();                                      //startanje tajmera
  while(echo == 1){}                                  //beskonacni while loop dok je interrupt pin u stanju 1 
  timer.stop();                                       //stopiranje tajmera
  if(queue1.empty()){                                 //provjera jeli je queue prazan
            int d = (timer.elapsed_time()).count()/58;//racunanje udaljenosti u cm sa podatkom vremena koji je tajmer izmjerio
            queue1.try_put((int *)d);                 //spremanje podatka udaljenosti u queue
        }
}

void task_adxl()                                      //void task za akcelerometar
{
    while(1){
        i2c.start();                                  //startanje I2C komunikacije
        i2c.write(adxl_w,adxl_x0,1);                  //slanje registra da akcelerometar pošalje podatke 
        i2c.read(adxl_r,data,6,true);                 //očitavanje 3 podatka, svaki veličine 2 bajta
 
        g_force_raw_x = (data[1]<<8)|data[0];         //spremanje nultog i prvog podatka u g_force_raw_x
        g_force_raw_y = (data[3]<<8)|data[2];         //spremanje drugog i treceg podatka u g_force_raw_y
        g_force_raw_z = (data[5]<<8)|data[4];         //spremanje cetvrtog i petog podatka u g_force_raw_z
        //množenje očitanih podatka sa varijablom a = 0.0039
        float g_force_x = float(g_force_raw_x*a);     
        float g_force_y = float(g_force_raw_y*a);
        float g_force_z = float(g_force_raw_z*a);

        if(queue2.empty()){                           //provjera jeli je queue prazan
            queue2.try_put(&g_force_x);               //spremanje podatka u queue
            queue2.try_put(&g_force_y);
            queue2.try_put(&g_force_z);
        }
        thread_sleep_for(100);
    }
}

void task_mcp()                                            //void task za temperaturni senzor
{
    while(1){
        double temp;
        i2c.start();                                       //startanje I2C komunikacije
        i2c.write((adresa)<<1 & 0xFE,Ta_pointer,1,false);  //slanje registra da temperaturni senzor pošalje podatak
        thread_sleep_for(70);                              //potrebno vrijeme pretvorbe za rezoluciju od +/-0.25°C je 65ms, 
                                                           //ali ponekad vrati grešku pa je postavljeno na 70ms
        i2c.read((adresa)<<1 | 0x01,msb_lsb_bajtovi,2,true);
        msb_lsb_bajtovi[0] = msb_lsb_bajtovi[0] & 0x1F;    //Clear flag bits
        if ((msb_lsb_bajtovi[0] & 0x10) == 0x10){          //TA < 0°C, provjera jeli je Ta manji od 0
        msb_lsb_bajtovi[0] = msb_lsb_bajtovi[0] & 0x0F;    //Clear SIGN
        temp = 256 - (float(msb_lsb_bajtovi[0]) * 16 + float(msb_lsb_bajtovi[1] / 16));
        }
        else { 
            temp = (float(msb_lsb_bajtovi[0]) * 16) + (float(msb_lsb_bajtovi[1]) / 16);
        } 
        if(queue3.empty()){                                //provjera jeli queue prazan
            queue3.try_put(&temp);                         //spremanje podatka temperature u queue
        }
        thread_sleep_for(100);
    }
}

void task_send()                                //void task za slanje podatka
{   
    while(1){  
        int *a;
        float *force_x;
        float *force_y;
        float *force_z;
        
        double *te;
        int control_d = 0x8A;                   //kontrolna rijec za udaljenost
        if(queue1.full()){                      //provjera jeli queue pun
            queue1.try_get(&a);                 //pokušaj očitanja queue-a i spremanje podataka u a pointer
            mutex.lock();
            spi.write(control_d);               //slanje kontrolne rijeci
            thread_sleep_for(1);                //delay kako bi slave procesirao podatak
            spi.write(short(int(a)));           //slanje podatka koji se pretvori u 2-bitni pomoću short 
            led = !led;                         //led promjeni stanje ako je spi uspješna
            printf("Udaljenost: %dcm\n",int(a));//ispis podatka udaljenosti na terminal         
            mutex.unlock();
        }
        if(queue2.full()){                      //provjera jeli queue pun
            queue2.try_get(&force_x);           //pokušaj očitanja queue-a i spremanje podataka u force_x float pointer
            queue2.try_get(&force_y);           //pokušaj očitanja queue-a i spremanje podataka u force_y float pointer
            queue2.try_get(&force_z);           //pokušaj očitanja queue-a i spremanje podataka u force_z float pointer

            char podatak_force_x[2], podatak_force_y[2], podatak_force_z[2];
            
            long data_x,data_y,data_z;
            
            // množenje podatka sa 100 te korištenje floor kako bi se dobio integer
            data_x = floor(double(*(float*)force_x)*100);
            data_y = floor(double(*(float*)force_y)*100);
            data_z = floor(double(*(float*)force_z)*100);
            
            //postavljanje integer vrijednosti u 2-bitni char 
            podatak_force_x[0] = data_x;
            podatak_force_x[1] = data_x >> 8;

            podatak_force_y[0] = data_y;
            podatak_force_y[1] = data_y >> 8;

            podatak_force_z[0] = data_z;
            podatak_force_z[1] = data_z >> 8;
            
            //slanje podataka na preko CAN-bus-a
            //Ako je slanje bilo uspješno ispiše se tekst na terminal
            mutex.lock();
            if(can.write(CANMessage(1,podatak_force_x,2))){
                printf("Podatak o X akceleraciji je poslan\n");         
            }
            else{
                can.reset();
            }
            if(can.write(CANMessage(2,podatak_force_y,2))){
                printf("Podatak o Y akceleraciji je poslan\n");         
            }
            else{
                can.reset();
            }
            if(can.write(CANMessage(3,podatak_force_z,2))){
                printf("Podatak o Z akceleraciji je poslan\n");         
            }
            else{
                can.reset();
            }
            //ispis izvornih posatka akceleracije na terminal
            printf("Akceleracija %+.2f,%+.2f,%+.2f\n",*(float*)force_x,*(float *)force_y,*(float *)force_z);
            mutex.unlock();
        }
        if(queue3.full()){
            queue3.try_get(&te);                        //provjera jeli queue pun
            short t;
            int control_t = 0x8D;                       //kontrola rijec za temperaturu
            t = (float)*(double*)te * 100;              //množenje podataka sa 100 kako bi se dobio integer
            mutex.lock();
            spi.write(control_t);                       //slanje kontolne rijeci
            thread_sleep_for(1);                        //delay kako bi slave procesirao podatak
            spi.write(t);                               //slanje podatka
            led = !led;
            printf("Temperatura puta sto: %d\n",t);     //ispis poslanog podatka 
            printf("Temperatura: %.2fC\n",*(double*)te);//ispis izvornog podataka temperature
            mutex.unlock();
        }
        thread_sleep_for(100);
    }
}

int main()
{
    i2c.frequency(400000);                        //postavljanje i2c frekvencije na 400kHz  

    i2c.start();                                  //pocetak komunikacije za akcelerometar
    i2c.write(adxl_w,data_format,2);              //postavljanje format podatka
    i2c.write(adxl_w,data_rate,2);                //postavljanje brzinu prijenosa podatka
    i2c.write(adxl_w,power_control,2,true);       //postavljanje kontrolu snage na mjerenje

    i2c.start();                                  //pocetak komunikacije za temperaturni senzor
    i2c.write((adresa)<<1,Con_pointer_msb_lsb,3); //configuracijski registar 
    i2c.write((adresa)<<1,Tu_pointer_msb_lsb,3);  //postavljanje gornji limita temperature na 44C 
    i2c.write((adresa)<<1,Tl_pointer_msb_lsb,3);  //postavljanje donjeg limita za temperaturu na 10C
    i2c.write((adresa)<<1,Tc_pointer_msb_lsb,3);  //postavljanje kritičnog limita na 100C
                                                  //gornji, donji i kritični limit postavljaju 
                                                  //alert pin na 1 te se može koristiti kao interapt
    i2c.write((adresa)<<1,rezolucija,2,true);     //postavljanje rezolucije na 0.25

    spi.format(16,0);                             //postavljanje spi komunikacije da šalje 16 bitni podatak i da radi u formatu 0
    spi.frequency(5000000);                       //postavljanje spi komunikacije na frekvenciju od 5MHz 

    trig=0;
    echo.rise(&ec);                               //postavljanje interrupt pin echo da se u slučaju prelaska pina iz 0 u 1 aktivira funkcija ec  
    task1.start(task_echo);
    task2.start(task_mcp);
    task3.start(task_adxl);  
    task4.start(task_send);
}
