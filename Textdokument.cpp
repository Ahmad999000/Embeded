/******************************************************************************/
/*                                                                            */
/*              Ostfalia Hochschule für angewandte Wissenschaften             */
/*                        Fakultät für Fahrzeugtechnik                        */
/*           Institut für Fahrzeuginformatik und Fahrzeugelektronik           */
/*                             Fahrzeuginformatik                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/
/* Paket:        ES_LAB                                                       */
/* Modul:        ES_LAB_3                                                     */
/* Projekt:      ES_LAB_3                                                     */
/* Beschreibung: Labor 3 "Grundsoftware für ein Autonomes Modellfahrzeug      */
/*               mit mbed"                                                    */
/*----------------------------------------------------------------------------*/
/* Datei:        'ES_Labor_3.h'                                               */
/*                                                                            */
/* Autoren:      Ahmed Youssef                                                */
/*               Ala Ben Ticha                                                */
/*               Aldar Youssef                                                */
/* Datum:        23.12.22                                                     */
/* Version:      WS22/23                                                      */
/******************************************************************************/
/* Gruppe         : B                                                         */
/* Name           : Ala Ben Ticha                                             */
/* Matrikelnummer : 70466868                                                  */
/* Name           : Ahmed Youssef                                             */
/* Matrikelnummer : 70465553                                                  */
/* Name           : Aldar Youssef                                             */
/* Matrikelnummer : 70466646                                                  */
/******************************************************************************/

//------------------------------------------------------------------------------
//  Headerdateien:
//------------------------------------------------------------------------------







#include "mbed_stats.h"
#include "rtos.h"
#include "Thread.h"
#include "mbed_mem_trace.h"
#include "Timer.h"

#include "ES_Labor_3.h"
#include "mbed.h"
#include "LCD_Touch_ILI9328.h"
#include "SRF08.h"
#include "platform/mbed_assert.h"
#include "platform/mbed_debug.h"
#include "platform/mbed_error.h"
#include "platform/mbed_stats.h"
#if !defined(MBED_THREAD_STATS_ENABLED)

#endif

#define MAX_THREAD_STATS    0x8
#define MBED_ALL_STATS_ENABLED
#define MAX_THREAD_INFO 10


//----------------------------------------------------------------------------------------------------------------------------------------
//  Makro-/Konstantendefinitionen:
//----------------------------------------------------------------------------------------------------------------------------------------

// Verbindung zum Display
BusOut dataBus( p26, p25, p24, p23, p22, p21, P2_6, P2_7); // 8 pins
ILI9328_LCD lcd( P0_22, p15, P0_24, P0_21, P0_19, P0_20, &dataBus, P0_25 );// control pins and data bus  CS, RS, WR, DIR, EN, LE, DATA_PORT, RD
TouchScreenADS7843 touch(p5, p6, p7, p8, P2_13, &lcd);     // ADS7843 -> mosi, miso, sclk, cs, irq, SPI_TFT


// Definition der Infrarot-Sensoren
AnalogIn sensorFront(P1_31);
AnalogIn sensorRight(P1_30);
AnalogIn sensorLeft(P0_26);
AnalogIn sensorback(P0_2);

// Definition der Ultraschallsensoren                                            //-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-
SRF08 US_L(P0_27, P0_28, 0xFE); // hier ggf. Adressen der US-Sensoren ändern (Datenblatt)
SRF08 US_R(P0_27, P0_28, 0xE4);
//-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-

// Definition der Aktoren
PwmOut lenko(P1_20); // Lenko
PwmOut motor(P1_21); // Motor

int zurueckAusFunktion = 0; // Variable zum Neuzeichnen der Buttons, sobald aus einer Funktion zurückgesprungen wird

typedef struct {
    // Definition des Structs, in dem die Sensordaten gespeichert werden
    uint16_t ir_front;
    uint16_t ir_left;
    uint16_t ir_right;
    uint16_t ir_back;
    int us_left;
    int us_right;
} sensorData;

MemoryPool<sensorData, 16> mpool; // Shared Memory !!!!!!!!!!!!!!!!!!!
//Mutex verhindert nebenläufige Prozesse  bzw. Threads.
// Mutex-Verfahren koordinieren den zeitlichen Ablauf nebenläufiger Prozesse/Threads derart, dass andere Prozesse/Threads von der Ausführung kritischer Abschnitte ausgeschlossen sind, wenn sich bereits ein Prozess/Thread im kritischen Abschnitt befindet
Mutex ADC_Mutex; // Mutex für Wandler Analog-Digital (Sensoren)
Mutex Mem_Mutex; // Mutex für Shared Memory
Mutex LCD_Mutex; // Mutex für Display

sensorData *s = mpool.alloc(); // Speicher Reservieren für die Sensordaten

// Timer
Timer t;
Timer t1;
Timer t3;
Timer t4;
//----------------------------------------------------------------------------------------------------------------------------------------
//  Main:
//----------------------------------------------------------------------------------------------------------------------------------------

int main()
{

    displayInit(); // Funktion Display initialisieren

    // Button zum Starten des Testprogramms

    Button test; // Erstellung Test Button
//Das Konzept lässt sich mittels Read-Write-Locks so verfeinern, dass sich Prozesse, die nur Daten lesen, gegenseitig nicht behindern – das ist besonders für den Zugriff auf Dateien und Datenbanken verbreitet.
    LCD_Mutex.lock();
    test.Create("Testlauf", 20, 10, &lcd, &touch, 219, 60);
    test.Draw(COLOR_BLACK, COLOR_RED);
    LCD_Mutex.unlock(); // Release

    // Button zum Starten des Fahrtprogramms
    Button fahrt; // Erstellung fahrt Button
    LCD_Mutex.lock();
    fahrt.Create("Folgefahrt", 20, 80, &lcd, &touch, 219, 130);
    fahrt.Draw(COLOR_BLACK, COLOR_RED);

    //Speicherauslastung ausgeben
    mbed_stats_heap_t heap_stats;
    mbed_stats_heap_get(&heap_stats);

    t.start();// dauer diesen abschnitt

    char print[50];
    float current = heap_stats.current_size;
    float reserved = heap_stats.reserved_size;

    sprintf(print, "Speicherauslastung:%.2f Pzt", (current/64000)*100);
    lcd.Print(print,5,160,-2,-1,0);

    t.stop();// hier ist der dauer zu ende

    float tread = t.read() * 1000000;

    int countertime = 0;

    char cyctime[50];

    sprintf(cyctime, "aktuelle Zykluszeit: %f mk-seconds", tread);
    lcd.Print(cyctime, 10, 230, -2, -1, 0);

    if(t.read() >= 250000 || t.read() <= 250000) {
        countertime++;
    }
    char countertime_char[50];
    sprintf(countertime_char, "HurtCounter: %i times", countertime);
    lcd.Print(countertime_char, 10, 240, -2, -1, 0);

    LCD_Mutex.unlock();

    while (1) {

        if (zurueckAusFunktion == 1) {

            LCD_Mutex.lock();
            lcd.ClearScreen();                  // Löschen
            test.Draw(COLOR_BLACK, COLOR_RED);  // Buttons zum Starten der Funktionen werden neu erstellt,
            fahrt.Draw(COLOR_BLACK, COLOR_RED); // sobald ein Programm beendet wurde
            LCD_Mutex.unlock();

            zurueckAusFunktion = 0;
            t1.start();

            //Speicherauslastung
            mbed_stats_heap_t heap_stats;
            mbed_stats_heap_get(&heap_stats);

            char print[50];
            float current = heap_stats.current_size;
            float reserved = heap_stats.reserved_size;

            sprintf(print, "Speicherauslastung:%.2fPzt", (current/64000)*100);
            lcd.Print(print,5,160,-2,-1,0);

            //die grade benutzte Threads 
            mbed_stats_thread_t *stats = new mbed_stats_thread_t[MAX_THREAD_STATS];
            int count = mbed_stats_thread_get_each(stats, MAX_THREAD_STATS);


            lcd.Print("Threads:",5,170,-2,-1,0);

            for(int i = 0; i < count; i++) {
                sprintf(print,"%s", stats[i].name);
                lcd.Print(print,10,180+i*12,-2,-1,0);
            }
            //free(stats);

            float tread = t.read() * 1000000;

            t1.stop();


            sprintf(cyctime, "aktuelle Zykluszeit: %f seconds", tread);
            lcd.Print(cyctime, 10, 230, -2, -1, 0);

            if(t.read() >= 250000) {
                countertime++;
            }
            char countertime_char[50];
            sprintf(countertime_char, "HurtCounter: %i times", countertime);
            lcd.Print(countertime_char, 10, 240, -2, -1, 0);

            t1.reset();

        }

        if (test.GetButton_touched() == 1) {
            // bei Klick auf Button wird Testprogramm gestartet

            testprogramm();
        }

        if (fahrt.GetButton_touched() == 1) {
            // bei Klick auf Button wird Fahrtprogramm gestartet

            fahrtprogramm();
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------------
//  Display initialisieren:
//----------------------------------------------------------------------------------------------------------------------------------------

void displayInit()
{

    lcd.Initialize(PORTRAIT_REV); // Display initialisieren
    touch.TouchPanel_Calibrate(); // Display kalibirieren
    lcd.FillScreen(COLOR_BLACK);
    lcd.SetBackground(COLOR_BLACK);
    lcd.SetFont(&TerminusFont);
}

//----------------------------------------------------------------------------------------------------------------------------------------
//  Testprogramm:
//----------------------------------------------------------------------------------------------------------------------------------------

void testprogramm()
{

    LCD_Mutex.lock();
    lcd.ClearScreen();
    LCD_Mutex.unlock();

    // Button zum Beenden des Testprogramms
    Button ende_Test;
    LCD_Mutex.lock();
    mbed_stats_heap_t heap_stats;
    mbed_stats_heap_get(&heap_stats);

    char print[50];
    float current = heap_stats.current_size;
    float reserved = heap_stats.reserved_size;
    sprintf(print, "Speicherauslastung:%.2fPzt", (current/64000)*100);
    lcd.Print(print,5,140,-2,-1,0);
// Erstellen der Threads zum Auslesen der Sensoren und Schreiben der Werte auf das Display
    Thread thread_messungSensoren(osPriorityNormal, OS_STACK_SIZE/2, NULL, "messungSensoren");
    Thread thread_displayBeschreiben(osPriorityNormal, OS_STACK_SIZE/2, NULL, "displayBeschreiben");




    // Starten der Threads
    thread_messungSensoren.start(callback(messungSensoren));
    thread_displayBeschreiben.start(callback(displayBeschreiben));


    //Threads
    mbed_stats_thread_t *stats = new mbed_stats_thread_t[MAX_THREAD_STATS];
    int count = mbed_stats_thread_get_each(stats, MAX_THREAD_STATS);


    lcd.Print("Threads:",5,150,-2,-1,0);

    for(int i = 0; i < count; i++) {
        sprintf(print,"%s", stats[i].name);
        lcd.Print(print,10,160+i*12,-2,-1,0);

    }
    free(stats);


    ende_Test.Create("Beenden", 20, 250, &lcd, &touch, 219, 310);
    ende_Test.Draw(COLOR_BLACK, COLOR_RED);
    LCD_Mutex.unlock();



    // Testen der beiden Servomotoren
    LCD_Mutex.lock();
    lcd.Print("Aktor-Test startet", CENTER, 110, COLOR_WHITE, COLOR_BLACK);
    LCD_Mutex.unlock();

    aktorTest(); // Aktor-Test wird ausgeführt

    LCD_Mutex.lock();
    lcd.Print("Aktor-Test abgeschlossen", CENTER, 130, COLOR_WHITE, COLOR_BLACK);
    LCD_Mutex.unlock();

    while (1) {
        // Beim Beenden des Programms werden die Threads terminiert
        if (ende_Test.GetButton_touched() == 1) {

            zurueckAusFunktion = 1;
            thread_messungSensoren.terminate();    // Beenden
            thread_displayBeschreiben.terminate(); // Beenden
            mpool.free(s);                         // Speicherplatz für s wird freigegeben
            return;                                // Zurückspringen in die Main
        }
    }
}

//------------------------------------------------------------------------------
//  Fahrtprogramm:
//------------------------------------------------------------------------------

void fahrtprogramm()
{

    int sensor = 0;

    LCD_Mutex.lock();
    lcd.ClearScreen();
    LCD_Mutex.unlock();

    // Knopf zum Beenden des Fahrtprogramms
    Button ende_Fahrt;
    LCD_Mutex.lock();
    mbed_stats_heap_t heap_stats;
    mbed_stats_heap_get(&heap_stats);

    char print[50];
    float current = heap_stats.current_size;
    float reserved = heap_stats.reserved_size;
    sprintf(print, "Speicherauslastung:%.2f Pzt", (current/64000)*100);
    lcd.Print(print,5,140,-2,-1,0);

// Erstellen der Threads zum Auslesen der Sensoren für die Simulationsfahrt
    Thread thread_messungSensoren(osPriorityNormal, OS_STACK_SIZE/2, NULL,"messungSensoren");
    Thread thread_simFahrt(osPriorityNormal, OS_STACK_SIZE/2, NULL,"simFahrt");
    Thread thread_displayBeschreiben(osPriorityNormal, OS_STACK_SIZE/2, NULL,"displayBeschreiben");

    // Starten der Threads zum Auslesen der Sensoren für die Simulationsfahrt
    thread_messungSensoren.start(callback(messungSensoren));
    thread_displayBeschreiben.start(callback(displayBeschreiben));
    //Threads
    mbed_stats_thread_t *stats = new mbed_stats_thread_t[MAX_THREAD_STATS];
    int count = mbed_stats_thread_get_each(stats, MAX_THREAD_STATS);


    lcd.Print("Threads:",5,150,-2,-1,0);

    for(int i = 0; i < count; i++) {
        sprintf(print,"%s", stats[i].name);
        lcd.Print(print,10,160+i*12,-2,-1,0);
    }
    free(stats);
    ende_Fahrt.Create("Beenden", 20, 250, &lcd, &touch, 219, 310);
    ende_Fahrt.Draw(COLOR_BLACK, COLOR_RED);
    LCD_Mutex.unlock();



    while (1) {

        Mem_Mutex.lock();
        sensor = s->us_left; // Pfeiloperator
        Mem_Mutex.unlock();

        // Simulationsfahrt wird gestartet, sobald Objekt in 1 m Entfernung auftaucht
        if (sensor < 105 && sensor > 95) {

            thread_simFahrt.start(callback(simFahrt));
            LCD_Mutex.lock();
            lcd.Print("simFahrt",10,220,-2,-1,0);
            LCD_Mutex.unlock();
        }




        // Beim Beenden des Programms werden die Threads terminiert
        if (ende_Fahrt.GetButton_touched() == 1) {

            motor.pulsewidth(0.0015);
            zurueckAusFunktion = 1;
            thread_messungSensoren.terminate();
            thread_displayBeschreiben.terminate();
            thread_simFahrt.terminate();
            mpool.free(s); // Speicherplatz für s wird freigegeben
            return;        // Zurückspringen in die Main
        }
    }

}
//------------------------------------------------------------------------------
//  Sensoren auslesen:
//------------------------------------------------------------------------------

void messungSensoren(void)
{

    // Variablen für IR- und US-Sensoren
    uint16_t IR_range_front;
    uint16_t IR_range_left;
    uint16_t IR_range_right;
    uint16_t IR_range_back;
    uint16_t US_range_left;
    uint16_t US_range_right;

    // Register für US-Sensoren
    US_L.setRangeRegister(30);
    US_L.setMaxGainRegister(5);
    US_R.setRangeRegister(30);
    US_R.setMaxGainRegister(5);

    while (1) {

        // Werte der IR-Sensoren in Variablen speichern
        ADC_Mutex.lock();
        IR_range_front = sensorFront.read_u16();
        IR_range_left = sensorLeft.read_u16();
        IR_range_right = sensorRight.read_u16();
        IR_range_back = sensorback.read_u16();
        ADC_Mutex.unlock();

        // Werte der US-Sensoren einlesen
        ADC_Mutex.lock();
        US_L.startRanging();
        US_R.startRanging();
        ADC_Mutex.unlock();

        while ((!US_L.rangingFinished()) && (!US_R.rangingFinished())) {
            // Einlesen beider Sensoren noch nicht abgeschlossn

            ThisThread::sleep_for(100); // Sensoren werden alle 100 ms angesteuert
        }

        // Werte der US-Sensoren in Variablen speichern
        Mem_Mutex.lock();
        US_range_left = US_L.getRange();
        US_range_right = US_R.getRange();
        Mem_Mutex.unlock();

        // Werte der IR-Sensoren werden in Shared Memory geschrieben
        Mem_Mutex.lock();
        s->ir_front = IR_range_front;
        s->ir_left = IR_range_left;
        s->ir_right = IR_range_right;
        s->ir_back = IR_range_back;
        s->us_left = US_range_left;
        s->us_right = US_range_right;
        Mem_Mutex.unlock();
    }
}

//------------------------------------------------------------------------------
//  Display beschreiben:
//------------------------------------------------------------------------------

void displayBeschreiben(void)
{
    // Arrays für die Zeilen auf dem Display
    char row1[100];
    char row2[100];
    char row3[100];
    char row4[100];
    char row5[100];
    char row6[100];

    char cyctime4[50];

    char countertime_char[50];

    int countertime = 0;

    uint16_t ir1 = 0;
    uint16_t ir2 = 0;
    uint16_t ir3 = 0;
    uint16_t ir4 = 0;

    int us1 = 0;
    int us2 = 0;



    while (1) {

        ThisThread::sleep_for(1000);
        t4.start();

        // Werte aus der Sensor-Task in Variablen speichern
        Mem_Mutex.lock();
        ir1 = s->ir_front;
        ir2 = s->ir_left;
        ir3 = s->ir_right;
        ir4 = s->ir_back;
        us1 = s->us_left;
        us2 = s->us_right;
        Mem_Mutex.unlock();

        // Werte in Strings speichern und auf Dispkay ausgeben
        LCD_Mutex.lock();
        sprintf(row1, "IR Sensor front: %6d", ir1);
        lcd.Print(row1, CENTER, 20, COLOR_WHITE, COLOR_BLACK);
        sprintf(row2, "IR Sensor left: %6d", ir2);
        lcd.Print(row2, CENTER, 35, COLOR_WHITE, COLOR_BLACK);
        sprintf(row3, "IR Sensor right: %6d", ir3);
        lcd.Print(row3, CENTER, 50, COLOR_WHITE, COLOR_BLACK);
        sprintf(row4, "IR Sensor rear: %6d", ir4);
        lcd.Print(row4, CENTER, 65, COLOR_WHITE, COLOR_BLACK);
        sprintf(row5, "Usensor links: %6d", us1);
        lcd.Print(row5, CENTER, 80, COLOR_WHITE, COLOR_BLACK);
        sprintf(row6, "Usensor rechts: %6d", us2);
        lcd.Print(row6, CENTER, 95, COLOR_WHITE, COLOR_BLACK);
        LCD_Mutex.unlock();

        t4.stop();

        int tread4 = t4.read() * 1000000;


        if(tread4 >= 250000 || tread4 <= 250000) {
            countertime++;
        }

        LCD_Mutex.lock();
        sprintf(cyctime4, "Zykluszeit: %i sec", tread4);
        lcd.Print(cyctime4, 10, 230, -2, -1, 0);
        LCD_Mutex.unlock();
        t4.reset();



        LCD_Mutex.lock();
        sprintf(countertime_char, "HurtCounter: %i times", countertime);
        lcd.Print(countertime_char, 10, 240, -2, -1, 0);
        LCD_Mutex.unlock();




    }
}

//------------------------------------------------------------------------------
//  Aktor-Test:
//------------------------------------------------------------------------------

void aktorTest(void)
{


    lenko.period(0.022);        // Lenkung Grundfrequenz
    float lenk_pos = 0.0015;    // Lenkung Grundposition (Neutral) Pulsweite
    lenko.pulsewidth(lenk_pos); // Lenkung mit Grundfrequenz ansteuern !!!!!!!

    // Lenkeinschlag vollständig nach links
    for (int i = 0; i < 10; i++) {

        lenk_pos = lenk_pos - 0.00005; //?????
        lenko.pulsewidth(lenk_pos);
        ThisThread::sleep_for(200);

    }

    // Lenkeinschlag vollständig nach rechts
    for (int i = 0; i < 20; i++)

    {

        lenk_pos += 0.00005; //??????
        lenko.pulsewidth(lenk_pos);
        ThisThread::sleep_for(100);

    }

    // Lenkeinschlag bis Grundeinstellung
    for (int i = 0; i < 10; i++) {

        lenk_pos -= 0.00005;
        lenko.pulsewidth(lenk_pos);
        ThisThread::sleep_for(100);
    }

    motor.period(0.022);         // Motor Grundfrequenz
    float motor_pos = 0.0015;    // Motor Grundposition
    motor.pulsewidth(motor_pos); // Motor mit Grundfrequenz ansteuern

    ThisThread::sleep_for(3000);

    // Beschleunigung vorwärts
    for (int i = 0; i < 5; i++) {

        motor_pos += 0.0001;
        motor.pulsewidth(motor_pos);
        ThisThread::sleep_for(200);
    }

    // Verzögerung bis Stillstand
    for (int i = 0; i < 5; i++) {

        motor_pos -= 0.0001;
        motor.pulsewidth(motor_pos);
        ThisThread::sleep_for(200);
    }

    ThisThread::sleep_for(2100); // Pause zwischen Vorwärts- und Rückwärtsgang

    // Beschleunigung rückwärts
    for (int i = 0; i < 5; i++) {

        motor_pos -= 0.0001;
        motor.pulsewidth(motor_pos);
        ThisThread::sleep_for(200);
    }

    // Verzögerung bis Stillstand
    for (int i = 0; i < 5; i++) {

        motor_pos += 0.0001;
        motor.pulsewidth(motor_pos);
        ThisThread::sleep_for(200);
    }
}


//------------------------------------------------------------------------------
//  Simulationsfahrt:
//------------------------------------------------------------------------------

void simFahrt(void)
{

    // Deklaration der Variablen
    char buf[100];
    int iVal_1 = 0;
    int iVal_2 = 0;
    float fMean = 0;
    float pw = 0;

    LCD_Mutex.lock();
    lcd.Print("Verfolgung aufgenommen", CENTER, 20, COLOR_WHITE, COLOR_BLACK);
    LCD_Mutex.unlock();

    while (1) {

        ThisThread::sleep_for(20); // Aktoren alle 20 ms ansteuern

        // Werte der US-Sensoren aus Sensor-Task in Variablen speichern
        Mem_Mutex.lock();
        iVal_1 = s->us_left;
        iVal_2 = s->us_right;
        Mem_Mutex.unlock();

        fMean = (iVal_1 + iVal_2) / 2; // Mittelwert der US-Sensorwerte bestimmen

        // volle Geschwindigkeit bei Objekt in über 1 m Entfernung
        if ((fMean >= 100) || (fMean < 3)) {
            // (fMean < 3) -> Ist das Objekt außer Reichweite, wird iVal = 0

            motor.pulsewidth(0.002);
        }

        // reduzierte/angepasste Geschwindigkeit bei Objekt zwischen 20 cm und 1 m Entfernung
        if ((fMean < 100) && (fMean >= 20)) {

            pw = (0.0015 + 0.0005 * (fMean / 100));
            motor.pulsewidth(pw);
        }

        // Stillstand bei Objekt in unter 20 cm Entfernung
        if ((fMean < 20) && (fMean > 3)) {
            // (fMean > 3)-> ansonsten wird Geschwindigkeit getoggelt (ist in zwei if-Schleifen vorhanden)

            motor.pulsewidth(0.0015);
        }
    }
}