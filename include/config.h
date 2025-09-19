#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include "MAX31856.h"
#include <mypid.h>
#include "driver/adc.h"
#include "NextionHandler.h"
#include <Wire.h>
#include <I2C_eeprom.h>
#include <PZEM004Tv30.h>
#define EE24LC01MAXBYTES 2048
#define DEVICEADDRESS (0x50)

#define ADDR_SUHU_SET 0
#define ADDR_SUHU_PREHEAT 8
#define ADDR_SUHU_ALARM 16
#define ADDR_SET_TIMER 24
#define ADDR_KP 32
#define ADDR_KI 40
#define ADDR_KD 48
#define ADDR_DRAN 56
#define ADDR_ADJ_SUHU 64
#define ADDR_PRESS 72
#define ADDR_REMINDER 80
#define ADDR_MULAI_PID 88
#define ADDR_ADJ_PRESS 96
#define ADDR_TIMERRUN 108
#define ADDR_HOURMETER 116

I2C_eeprom eeprom(DEVICEADDRESS, 2048);
template<class T> int EEPROM_writeAnything(int ee, const T& value) ;
template<class T> int EEPROM_readAnything(int ee, T& value) ;

double csetpoint, setSuhuAlarm, suhupreheat;
unsigned long  csetTimer;
double cKp,cKi,cKd;
double dran,adjustmentSuhu,setpressure,timereminder,adjpress,cmulaipid;
unsigned long timerrun,hourmeter,milishourmeter,cekelarmmilis ;
bool cekalarmTriggered = false;
bool ledState = false;
unsigned long lastBlink = 0;
const unsigned long blinkInterval = 500; // LED kedip tiap 500ms

// === Pin Config ===
const int START_PIN = 16;
const int STOP_PIN = 17;
const int HEATER_PIN = 40;
const int BUZZER_PIN = 37;
const int LED_PIN = 34;
enum Button
{
  BTN_NONE,
  BTN_MINUS,
  BTN_PLUS,
  BTN_OK,
  BTN_MENU,
  BTN_START,
  BTN_STOP,
  BTN
};
Button lastButton = BTN_NONE;
Button stableButton = BTN_NONE;
unsigned long lastChangeTime = 0;
int buttonCounter;
// === Status Sistem ===
enum State
{
  STANDBY,
  PREHEATING,
  STANDBY2,
  PRERUNNING,
  RUNNING,
  FINISHED,
  STOPPED
};


State currentState = STANDBY;
State LastState = STANDBY;

// === Konfigurasi Suhu & Timer ===

//float preheatTemp = 50.0;        // Suhu preheat
//float setpointTemp = 80.0;       // Suhu target
//unsigned long settime = 10 * 60; // 10 menit (ms)
float arus = 0, pressure = 0;

// === Tombol (pakai Bounce2) ===
bool timesup = false;
bool Heaterpin = false;
unsigned long lastDisplayUpdate = 0;
unsigned long pbmilis;
void updatedisplay(float temperature);
const char *getStatusText();
void saveConfig() ;
void loadConfig() ;
Button readButton();
Button getButtonDebounced(unsigned long t);
void sendconfigdisplay();
void savetimer();
float readWithRetry(std::function<float()> reader) ;



MAX31856Sensor maxthermo = MAX31856Sensor(8, 11, 9, 7);
PZEM004Tv30 pzem(Serial0, 2, 3);
#define cektc 38

#endif
