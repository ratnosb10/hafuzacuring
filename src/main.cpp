#include <Arduino.h>
#include "config.h"

#define OPENING 0
#define MAIN 1
#define CONFIG 2
#define ADVANCE 3
#define TEST 4

byte currentpage;

void setup()
{
  Serial.begin(115200);
  delay(1000);
  dspSerial.begin(9600, SERIAL_8N1, 13, 12);
  Serial0.begin(9600, SERIAL_8N1, 2, 3);
  pinMode(START_PIN, INPUT);
  pinMode(STOP_PIN, INPUT);
  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW);

  Wire.begin();
  eeprom.begin();
  analogSetPinAttenuation(1, ADC_11db);
  // analogSetAttenuation(ADC_11db);
  analogReadResolution(6);
  Serial.println("==============mulai");
  maxthermo.begin();
  currentpage = OPENING;
}

void loop()
{
  nextionReceiveHandler();
  hourmetertask();
  if (currentpage == CONFIG || currentpage == ADVANCE)
  {
    Button btn = getButtonDebounced(50);
    if (btn == BTN_MENU)
    {
      dspSerial.print("cursor.val++");
      sendNextionEnd();
      dspSerial.print("var.val=0");
      sendNextionEnd();
      dspSerial.print("cmd.val=1");
      sendNextionEnd();
      delay(400);
    }
    if (btn == BTN_MINUS)
    {
      if (buttonCounter < 10)
      {
        dspSerial.print("var.val=-1");
        sendNextionEnd();
        dspSerial.print("cmd.val=1");
        sendNextionEnd();
      }
      else
      {
        dspSerial.print("var.val=-10");
        sendNextionEnd();
        dspSerial.print("cmd.val=1");
        sendNextionEnd();
      }
      buttonCounter++;
      delay(400);
    }
    if (btn == BTN_PLUS)
    {
      if (buttonCounter < 10)
      {
        dspSerial.print("var.val=1");
        sendNextionEnd();
        dspSerial.print("cmd.val=1");
        sendNextionEnd();
      }
      else
      {
        dspSerial.print("var.val=10");
        sendNextionEnd();
        dspSerial.print("cmd.val=1");
        sendNextionEnd();
      }
      buttonCounter++;
      delay(400);
    }
    if (btn == BTN_OK)
    {
      dspSerial.print("cmd.val=2");
      sendNextionEnd();
      delay(400);
    }
  }
  if (currentpage == MAIN)
  {
    float voltage = readWithRetry([&]()
                                  { return pzem.voltage(); });
    arus = readWithRetry([&]()
                         { return pzem.current(); });
    // Serial.println(arus);
    float suhuraw = maxthermo.readRawTemperature();
    float suhukalman = maxthermo.applyKalman(suhuraw);
    float suhuavg = maxthermo.applyMovingAverage(suhukalman) + adjustmentSuhu;

    updatedisplay(suhuavg); // <- update tampilan via Serial
                            // temperature =sensor.getFilteredTemperature();

    bool startpress = false, stoppress = false, resetpress = false;

    if (digitalRead(START_PIN) == 1 && digitalRead(STOP_PIN) == 1)
      pbmilis = millis();
    else
    {
      if (millis() - pbmilis > 50)
      {
        if (digitalRead(START_PIN) == 0)
          startpress = true;
        if (digitalRead(STOP_PIN) == 0)
          stoppress = true;
      }
      if (millis() - pbmilis > 2000)
        if (digitalRead(STOP_PIN) == 0)
          resetpress = true;
    }

    switch (currentState)
    {
    case STANDBY:
      Heteroff();
      Heaterpin = 0;

      if (getButtonDebounced(3000) == BTN_MENU)
      {
        dspSerial.print("page config");
        sendNextionEnd();
        currentpage = CONFIG;
        delay(2000);
      }
      else if (startpress)
      {
        LastState = STANDBY;
        currentState = PREHEATING;
        setupPID(HEATER_PIN, cKp / 3, cKi / 3, cKd, cmulaipid);
      }
      break;

    case PREHEATING:
      Heaterpin = runPID(suhuavg, suhupreheat);
      if (stoppress)
      {
        LastState = PREHEATING;
        currentState = STOPPED;
      }
      if (suhuavg >= suhupreheat)
        currentState = STANDBY2;
      break;
    case STANDBY2:
      Heaterpin = runPID(suhuavg, suhupreheat);
      if (startpress)
      {
        LastState = STANDBY2;
        currentState = PRERUNNING;
        setupPID(HEATER_PIN, cKp, cKi, cKd, cmulaipid);
        reset_pid();
      }
      if (stoppress)
      {
        LastState = STANDBY2;
        currentState = STOPPED;
      }
      break;
    case PRERUNNING:
      Heaterpin = runPID(suhuavg, csetpoint);
      if (stoppress)
      {
        LastState = PRERUNNING;
        currentState = STOPPED;
      }
      if (suhuavg >= csetpoint)
      {
        currentState = RUNNING;
        dspSerial.print("btn.txt=\"start\"");
        sendNextionEnd();
      }
      break;
    case RUNNING:
      Heaterpin = runPID(suhuavg, csetpoint);
      if (stoppress)
      {
        LastState = RUNNING;
        currentState = STOPPED;
        dspSerial.print("btn.txt=\"stop\"");
        sendNextionEnd();
      }
      if (timesup)
      {
        LastState = RUNNING;
        timesup = false;
        currentState = FINISHED;
      }
      break;
    case FINISHED:
      if (stoppress)
      {
        LastState = FINISHED;
        currentState = STANDBY;
      }
      break;
    case STOPPED:
      Heaterpin = 0;
      Heteroff();
      if (startpress)
      {
        reset_pid();
        if (LastState == RUNNING)
        {
          currentState = RUNNING;
        }
        if (LastState == PREHEATING)
          currentState = PREHEATING;
        if (LastState == PRERUNNING)
          currentState = PRERUNNING;
      }
      if (resetpress)
      {
        LastState = STOPPED;
        currentState = STANDBY;
        dspSerial.print("btn.txt=\"reset\"");  
        sendNextionEnd();
      }
      break;
    }
  }
  if (currentpage == OPENING)
  {
    dspSerial.print("page opening");
    sendNextionEnd();
    delay(2000);
    loadConfig();
    sendconfigdisplay();
    dspSerial.print("page main");
    sendNextionEnd();
    currentpage = MAIN;
  }
  delay(1);
}

void hourmetertask()
{
  unsigned long currentMillis = millis();
  if (currentMillis - milishourmeter >= 360000)
  {
    milishourmeter = currentMillis;
    hourmeter++;
    EEPROM_writeAnything(ADDR_HOURMETER, hourmeter);
    char buffer[10];
    sprintf(buffer, "%06ld", hourmeter);
    for (int i = 0; i < 6; i++)
    {
      dspSerial.printf("hourmeter.t%d.txt=\"%c\"", i + 10, buffer[i]);
      sendNextionEnd();
    }
  }
}
void sendconfigdisplay()
{
  int jamSet = csetTimer / 3600;
  int menitSet = (csetTimer % 3600) / 60;

  dspSerial.printf("main.t7.txt=\"%d\"", (int)suhupreheat);
  sendNextionEnd();
  dspSerial.printf("main.t8.txt=\"%d\"", (int)csetpoint);
  sendNextionEnd();
  dspSerial.printf("main.t2.txt=\"%02d:%02d\"", jamSet, menitSet);
  sendNextionEnd();
  dspSerial.printf("main.t3.txt=\"%d\"", (int)setpressure);
  sendNextionEnd();
  unsigned long hours = timerrun / 3600;
  unsigned long minutes = (timerrun % 3600) / 60;
  unsigned long seconds = timerrun % 60;

  char buffer[10];
  sprintf(buffer, "%06ld", hourmeter);
  for (int i = 0; i < 6; i++)
  {
    dspSerial.printf("hourmeter.t%d.txt=\"%c\"", i + 10, buffer[i]);
    sendNextionEnd();
  }

  dspSerial.printf("main.tjam.val=%d", hours);
  sendNextionEnd();
  dspSerial.printf("main.tmenit.val=%d", minutes);
  sendNextionEnd();
  dspSerial.printf("main.tdetik.val=%d", seconds);
  sendNextionEnd();

  dspSerial.printf("config.n1.val=%d", (int)csetpoint);
  sendNextionEnd();
  dspSerial.printf("config.n2.val=%d", (int)suhupreheat);
  sendNextionEnd();
  dspSerial.printf("config.n3.val=%d", (int)setpressure);
  sendNextionEnd();

  dspSerial.printf("config.n4.val=%d", (int)jamSet);
  sendNextionEnd();
  dspSerial.printf("config.n5.val=%d", (int)menitSet);
  sendNextionEnd();
  dspSerial.printf("config.n6.val=%d", (int)setSuhuAlarm);
  sendNextionEnd();
  dspSerial.printf("config.n7.val=%d", (int)timereminder);
  sendNextionEnd();

  dspSerial.printf("advance.n1.val=%1.0f", adjustmentSuhu * 10);
  sendNextionEnd();
  dspSerial.printf("advance.n2.val=%1.0f", adjpress * 10);
  sendNextionEnd();
  dspSerial.printf("advance.x0.val=%1.0f", cKp * 10);
  sendNextionEnd();
  dspSerial.printf("advance.x1.val=%1.0f", cKi * 10);
  sendNextionEnd();
  dspSerial.printf("advance.x2.val=%1.0f", cKd * 10);
  sendNextionEnd();
  dspSerial.printf("advance.x3.val=%1.0f", cmulaipid * 10);
  sendNextionEnd();
}

void updatedisplay(float temperature)
{
  float suhudsp = temperature;
  if (millis() - lastDisplayUpdate >= 500)
  {
    // temperature += 0.5;
    lastDisplayUpdate = millis();

    if (suhudsp >= 100.0)
      dspSerial.printf("tsuhu.txt=\"%d\"", (int)suhudsp);
    else
      dspSerial.printf("tsuhu.txt=\"%.1f\"", suhudsp);
    sendNextionEnd();
    unsigned int warna;
    if (suhudsp < setpoint)
      warna = 65535;
    if (suhudsp >= setpoint)
      warna = 34784;
    if (suhudsp >= setpoint + 3 && currentState != STANDBY2)
      warna = 55782;
    dspSerial.printf("tsuhu.pco=%d", warna);
    sendNextionEnd();

    // arus = random(50, 60);
    // arus /= 10.0;
    int arusint = arus * 10;
    dspSerial.printf("xarus.val=%d", arusint);
    sendNextionEnd();

    //  pressure = random(30, 33);
    dspSerial.printf("tpress.txt=\"%.0f\"", pressure);
    sendNextionEnd();

    dspSerial.printf("tstat.txt=\"%s\"", getStatusText());
    sendNextionEnd();

    dspSerial.printf("p0.pic=%d", Heaterpin ? 3 : 2);
    sendNextionEnd();
  }
}
void sendNextionEnd()
{
  dspSerial.write(0xFF);
  dspSerial.write(0xFF);
  dspSerial.write(0xFF);
}

const char *getStatusText()
{
  switch (currentState)
  {
  case STANDBY:
    return "STANDBY";
  case PREHEATING:
    return "PREHEATING";
  case STANDBY2:
    return "STANDBY2";
  case PRERUNNING:
    return "PRERUNNING";
  case RUNNING:
    return "RUNNING";
  case FINISHED:
    return "FINISHED";
  case STOPPED:
    return "STOPPED";
  default:
    return "UNKNOWN";
  }
}

Button readButton()
{
  long sum = 0;
  for (int i = 0; i < 30; i++)
  {
    sum += analogRead(1);
    delayMicroseconds(200);
  }
  int adcValue = sum / 30;
  // Serial.println(adcValue);
  if (adcValue >= 41 && adcValue <= 50)
    return BTN_MINUS; // ~63
  if (adcValue >= 31 && adcValue <= 40)
    return BTN_PLUS; // ~19-22
  if (adcValue >= 21 && adcValue <= 30)
    return BTN_OK; // ~5-10
  if (adcValue >= 13 && adcValue <= 18)
    return BTN_MENU; // ~1-5
  if (adcValue >= 51 && adcValue <= 63)
    return BTN_NONE; // no push ~17
  if (digitalRead(START_PIN == 0))
    return BTN_START;
  if (digitalRead(STOP_PIN == 0))
    return BTN_STOP;
  return BTN_NONE; // default
}

Button getButtonDebounced(unsigned long t)
{
  Button currentButton = readButton();
  unsigned long now = millis();
  if (currentButton == BTN_NONE)
  {
    buttonCounter = 0;
    stableButton = currentButton;
  }
  if (currentButton != lastButton)
  {
    lastChangeTime = now;
    lastButton = currentButton;
  }

  if ((now - lastChangeTime) > t)
  {
    stableButton = currentButton;
  }
  return stableButton;
}

void saveConfig()
{

  EEPROM_writeAnything(ADDR_SUHU_SET, csetpoint);
  EEPROM_writeAnything(ADDR_SUHU_PREHEAT, suhupreheat);
  EEPROM_writeAnything(ADDR_SUHU_ALARM, setSuhuAlarm);
  EEPROM_writeAnything(ADDR_SET_TIMER, csetTimer);
  EEPROM_writeAnything(ADDR_KP, cKp);
  EEPROM_writeAnything(ADDR_KI, cKi);
  EEPROM_writeAnything(ADDR_KD, cKd);
  EEPROM_writeAnything(ADDR_DRAN, dran);
  EEPROM_writeAnything(ADDR_ADJ_SUHU, adjustmentSuhu);
  EEPROM_writeAnything(ADDR_PRESS, setpressure);
  EEPROM_writeAnything(ADDR_REMINDER, timereminder);
  EEPROM_writeAnything(ADDR_ADJ_PRESS, adjpress);
  EEPROM_writeAnything(ADDR_MULAI_PID, cmulaipid);
  EEPROM_writeAnything(ADDR_TIMERRUN, timerrun);
  EEPROM_writeAnything(ADDR_HOURMETER, hourmeter);
  Serial.println("savecomplit");
}

void loadConfig()
{
  EEPROM_readAnything(ADDR_SUHU_SET, csetpoint);
  EEPROM_readAnything(ADDR_SUHU_PREHEAT, suhupreheat);
  EEPROM_readAnything(ADDR_SUHU_ALARM, setSuhuAlarm);
  EEPROM_readAnything(ADDR_SET_TIMER, csetTimer);
  EEPROM_readAnything(ADDR_KP, cKp);
  EEPROM_readAnything(ADDR_KI, cKi);
  EEPROM_readAnything(ADDR_KD, cKd);
  EEPROM_readAnything(ADDR_DRAN, dran);
  EEPROM_readAnything(ADDR_ADJ_SUHU, adjustmentSuhu);
  EEPROM_readAnything(ADDR_PRESS, setpressure);
  EEPROM_readAnything(ADDR_REMINDER, timereminder);
  EEPROM_readAnything(ADDR_ADJ_PRESS, adjpress);
  EEPROM_readAnything(ADDR_MULAI_PID, cmulaipid);

  EEPROM_readAnything(ADDR_TIMERRUN, timerrun);
  EEPROM_readAnything(ADDR_HOURMETER, hourmeter);

  // Cek NaN atau nilai tidak wajar
  if (isnan(csetpoint) || csetpoint < 24.0 || csetpoint > 300.0)
    csetpoint = 140.0;

  if (isnan(setSuhuAlarm) || setSuhuAlarm < csetpoint || setSuhuAlarm > 350.0)
    setSuhuAlarm = csetpoint + 5;

  if (isnan(suhupreheat) || suhupreheat < 0 || suhupreheat >= csetpoint)
    suhupreheat = 50;

  if (csetTimer < 60 || csetTimer > 86400)
    csetTimer = 3600;

  if (isnan(cKp) || cKp < 0.00 || cKp > 100.0)
    cKp = 6.0;
  if (isnan(cKi) || cKi < 0.00 || cKi > 100.0)
    cKi = 0.3;
  if (isnan(cKd) || cKd < 0.00 || cKd > 100.0)
    cKd = 50;

  dran = 2.0;

  if (isnan(adjustmentSuhu) || adjustmentSuhu < -20.0 || adjustmentSuhu > 20.0)
    adjustmentSuhu = 0.0;
  if (isnan(setpressure) || setpressure < 0 || adjustmentSuhu > 50)
    setpressure = 33.0;

  if (timereminder < 0 || csetTimer > 86400)
    csetTimer = 900;

  if (isnan(cmulaipid) || cmulaipid < 0.00 || cmulaipid > 50.0)
    cmulaipid = 15;

  if (isnan(hourmeter) || hourmeter < 0 || hourmeter > 999999)
    hourmeter = 0;

  if (isnan(timerrun) || timerrun < 0 || timerrun > 999999)
    timerrun = 0;

  Serial.println(csetpoint);
  Serial.println(suhupreheat);
  Serial.println(setSuhuAlarm);
  Serial.println(csetTimer);
  Serial.println(cKp);
  Serial.println(cKi);
  Serial.println(cKd);
  Serial.println(dran);
  Serial.println(adjustmentSuhu);
  Serial.println(setpressure);
  Serial.println(timereminder);
  Serial.println(adjpress);
  Serial.println(cmulaipid);
}

float readWithRetry(std::function<float()> reader)
{
  float val = NAN;
  for (int i = 0; i <= 2; i++)
  {
    val = reader();
    if (!isnan(val))
    {
      return val; // sukses, kembalikan nilai baru
    }
  }
  delay(3000);
  return 0.5; // kalau gagal semua, pakai nilai lama
}

template <class T>
int EEPROM_writeAnything(int ee, const T &value)
{
  const byte *p = (const byte *)(const void *)&value;
  unsigned int i;
  for (i = 0; i < sizeof(value); i++)
  {
    eeprom.writeByte(ee++, *p++);
    delay(10);
  }

  return i;
}

template <class T>
int EEPROM_readAnything(int ee, T &value)
{
  byte *p = (byte *)(void *)&value;
  unsigned int i;
  for (i = 0; i < sizeof(value); i++)
    *p++ = eeprom.readByte(ee++);
  return i;
}