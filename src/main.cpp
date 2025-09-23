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
  dspSerial.begin(9600, SERIAL_8N1, 12, 13);
  Serial0.begin(9600, SERIAL_8N1, 2, 3);
  pinMode(START_PIN, INPUT);
  pinMode(STOP_PIN, INPUT);
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(CEKTC_PIN, OUTPUT);
  pinMode(EMER_PIN, INPUT);
  pinMode(BIGRELAY_PIN, OUTPUT);
  pinMode(INF_PIN, OUTPUT);
  pinMode(RIL_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(CEKTC_PIN, LOW);
  digitalWrite(RIL_PIN, LOW);
  digitalWrite(INF_PIN, LOW);
  Wire.begin();
  ADS.begin();
  ADS.setGain(0);
  eeprom.begin();
  analogSetPinAttenuation(1, ADC_11db);
  // analogSetAttenuation(ADC_11db);
  analogReadResolution(6);

  maxthermo.begin();

  currentState = STANDBY;
  currentpage = OPENING;
}

void loop()
{

  if (digitalRead(EMER_PIN) == 1)
  {
    digitalWrite(BIGRELAY_PIN, HIGH);
    emergency = true;
    currentState = EMERGENCY;
  }
  nextionReceiveHandler();

  if (currentpage == CONFIG || currentpage == ADVANCE)
  {
    if (pidautotune)
    {
      if (!initauto)
      {
        initauto = true;
        initAutoTune(maxthermo.getFilteredTemperature() + adjustmentSuhu, HEATER_PIN);
      }
      finishauto = autotune(maxthermo.getFilteredTemperature() + adjustmentSuhu);
      if (finishauto)
      {
        Serial.println("Autotune selesai, kirim hasil ke Nextion");
        pidautotune = false;
        initauto = false;
        finishauto = false;
      }
    }
    else if (tccal)
    {

      if (digitalRead(START_PIN) == 0)
      {
        shortval = 0;
        shortcjval = 0;
        suhunormal = 0;
        dspSerial.print("t8.txt=\"MULAI KALIBRASI ...\"");
        sendNextionEnd();
        int j1val = 0;
        for (byte i = 0; i < 10; i++)
        {
          suhunormal += maxthermo.readRawTemperature();
          delay(10);
          digitalWrite(CEKTC_PIN, 1);
          delay(10);
          shortval += maxthermo.readshortTemperature();
          delay(10);
          digitalWrite(CEKTC_PIN, 0);
          shortcjval += maxthermo.readshortCJ();
          delay(500);
          j1val += 9;
          dspSerial.printf("j1.val=%d", j1val);
          sendNextionEnd();
        }
        dspSerial.printf("j1.val=%d", 100);
        sendNextionEnd();
        dspSerial.print("t8.txt=\"--SELESAI--\"");
        sendNextionEnd();
        shortvalset = shortval / 10;
        shortcjvalset = shortcjval / 10;
        suhunormalset = suhunormal / 10;

        EEPROM_writeAnything(ADDR_SHORT_VAL_SET, shortvalset);
        EEPROM_writeAnything(ADDR_SUHU_NORMAL_SET, suhunormalset);
        EEPROM_writeAnything(ADDR_SHORT_CJ_VAL_SET, shortcjvalset);
        tccal = false;
        delay(2000);
        dspSerial.print("page main");
        sendNextionEnd();
        currentpage = MAIN;
      }
    }
    else
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
  }
  if (currentpage == MAIN)
  {

    float voltage = readWithRetry([&]()
                                  { return pzem.voltage(); });
    arus = readWithRetry([&]()
                         { return pzem.current(); });

    // Serial.print("voltage ");
    // Serial.print(voltage);
    // Serial.print("   arus ");
    // Serial.println(arus);

    float suhuavg = maxthermo.getFilteredTemperature() + adjustmentSuhu;
    cekshorttc(suhuavg);
    isiairbag();
    updatedisplay(suhuavg); // <- update tampilan via Serial
                            // temperature =sensor.getFilteredTemperature();

    bool startpress = false, stoppress = false, resetpress = false, infpress = false;

    if (digitalRead(START_PIN) == 1 && digitalRead(STOP_PIN) == 0 && digitalRead(RESET_PIN) == 0)
      pbmilis = millis();
    else
    {
      if (millis() - pbmilis > 50)
      {
        if (digitalRead(START_PIN) == 0)
          startpress = true;
        if (digitalRead(STOP_PIN) == 1)
          stoppress = true;
        if (digitalRead(RESET_PIN) == 1)
          infpress = true;
      }
      if (millis() - pbmilis > 2000)
        if (digitalRead(STOP_PIN) == 1)
          resetpress = true;
    }

    switch (currentState)
    {
    case STANDBY:
    stepinf = 10;
      Heteroff();
      Heaterpin = 0;
      lampStatus = OFF;
      if (getButtonDebounced(3000) == BTN_MENU)
      {
        dspSerial.print("page config");
        sendNextionEnd();
        currentpage = CONFIG;
      }
      else if (startpress)
      {
        LastState = STANDBY;
        currentState = PREHEATING;
        setupPID(HEATER_PIN, cKp / 5, cKi / 5, cKd, cmulaipid);
      }
      if (resetpress)
      {
        dspSerial.print("btn.txt=\"reset\"");
        sendNextionEnd();
      }
      break;

    case PREHEATING:
    stepinf = 10;
      Heaterpin = runPID(suhuavg, suhupreheat);
      lampStatus = ON;
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
      lampStatus = OFF;
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
      if (infpress)
      {
        isiangin = true;
        stepinf = 0;
      }

      break;
    case PRERUNNING:
    stepinf = 2;
      lampStatus = ON;
      Heaterpin = runPID(suhuavg, csetpoint);
      if (stoppress)
      {
        LastState = PRERUNNING;
        currentState = STOPPED;
      }
      if (suhuavg >= csetpoint && suhuavg <200)
      {
        currentState = RUNNING;
        dspSerial.print("btn.txt=\"start\"");
        sendNextionEnd();
      }
      break;
    case RUNNING:
    stepinf = 2;
      lampStatus = ON;
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
    stepinf = 10;
      if (stoppress)
      {
        LastState = FINISHED;
        currentState = STANDBY;
        dspSerial.print("btn.txt=\"reset\"");
        sendNextionEnd();
      }
      break;
    case STOPPED:
    stepinf = 10;
      lampStatus = OFF;
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
    case EMERGENCY:
    stepinf = 10;
      lampStatus = OFF;
      Heaterpin = 0;
      Heteroff();
      break;
    }

    buzzeron = false;
    if (tcshort)
      buzzeron = true;
    if (suhuavg >= setSuhuAlarm && setSuhuAlarm > 0)
      buzzeron = true;
    if (emergency)
      buzzeron = true;
    if (buzzeron)
      digitalWrite(BUZZER_PIN, HIGH);
    else
      digitalWrite(BUZZER_PIN, LOW);
    updateLamp(lampStatus);
  }
  if (currentpage == OPENING)
  {
    dspSerial.print("page opening");
    sendNextionEnd();
    delay(3000);
    loadConfig();
    sendconfigdisplay();
    dspSerial.print("page main");
    sendNextionEnd();
    currentpage = MAIN;
    Serial.println("==============siap");
    updatehourmeter();
  }
  delay(1);
}

void savetimer()
{
  EEPROM_writeAnything(ADDR_TIMERRUN, timerrun);
  EEPROM_writeAnything(ADDR_HOURMETER, hourmeter);
  updatehourmeter();
}
void updatehourmeter()
{
  unsigned long hm = hourmeter / 6;
  char buffer[11]; // cukup besar, unsigned long max 10 digit + null
  sprintf(buffer, "%06lu", hm);

  // kirim per karakter
  for (int i = 0; i < 6; i++)
  {
    dspSerial.printf("t%d.txt=\"%c\"", i + 10, buffer[i]);
    sendNextionEnd();
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

  dspSerial.printf("advance.n1.val=%1.0f", adjustmentSuhu);
  sendNextionEnd();
  dspSerial.printf("advance.n2.val=%1.0f", adjpress);
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
  unsigned long now = millis();
  float suhudsp;

  if (now - lastDisplayUpdate >= 500)
  {
    // Serial.println("==============update");
    //  temperature += 0.5;
    lastDisplayUpdate = now;

    if (currentState == RUNNING || currentState == PREHEATING)
    {
      if (temperature >= setpoint)
        reachedSetpoint = true;
      else if (fabs(temperature - setpoint) >= 2.0)
        reachedSetpoint = false;

      if (reachedSetpoint)
        suhudsp = setpoint;
      else
        suhudsp = temperature;
    }
    else
    {
      suhudsp = temperature;
    }

    if (tcshort)
    {
      dspSerial.printf("tsuhu.txt=\"%s\"", "SRT");
      suhudsp = 500;
    }
    else if (temperature == 888)
      dspSerial.printf("tsuhu.txt=\"%s\"", "OPN");
    else if (suhudsp >= 100.0)
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

    dspSerial.printf("t0.txt=\"%s\"", getStatusText());
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
void kirimautotune(float Kp, float Ki, float Kd, float mulaipid)
{
  dspSerial.printf("advance.x0.val=%1.0f", Kp * 10);
  sendNextionEnd();
  dspSerial.printf("advance.x1.val=%1.0f", Ki * 10);
  sendNextionEnd();
  dspSerial.printf("advance.x2.val=%1.0f", Kd * 10);
  sendNextionEnd();
  dspSerial.printf("advance.x3.val=%1.0f", mulaipid * 10);
  sendNextionEnd();
  dspSerial.print("vis advance.j0,0");
  sendNextionEnd();
}
void cekshorttc(float suhu)
{
  if (millis() - milisshort > timecek)
  {
    if (suhu < 50)
    {
      suhunormal = maxthermo.readRawTemperature();
      delay(10);
      digitalWrite(CEKTC_PIN, 1);
      delay(10);
      shortval = maxthermo.readshortTemperature();
      delay(10);
      digitalWrite(CEKTC_PIN, 0);
      shortcjval = maxthermo.readshortCJ();
      float calshortvalset = shortvalset - (shortcjvalset - 29);
      shortval = shortval - (shortcjval - 29);
      calshortvalset = calshortvalset * 1.1;

      Serial.print("  short val =");
      Serial.print(shortval, 1);
      Serial.print(" ");
      Serial.print(calshortvalset, 1);
      Serial.print(" ");
      Serial.println(calshortvalset - suhunormal, 1);
      if (shortval - 20 < calshortvalset - suhunormal)
      {
        tcshort = true;
        timecek = 3000;
      }
      else
      {
        tcshort = false;
        timecek = 6000;
      }
    }
    else
    {
      tcshort = false;
      timecek = 6000;
    }
    milisshort = millis();
    if (tcshort)
      Serial.println("  TC SHORT!");
    else
      Serial.println("  TC OK!");
  }
}
float readpressure()
{
  unsigned int val_0 = 0;
  int p = 10;
  for (int i = 0; i < p; i++)
  {
    val_0 += ADS.readADC(0);
  }
  val_0 /= (p * 10);
  int presint = map(val_0, 300, 674, 30, 330);
  return ((float)presint / 10) + 0.9;
}
float readrowsensor()
{
  unsigned int val_0 = 0;
  int p = 10;
  for (int i = 0; i < p; i++)
  {
    val_0 += ADS.readADC(0);
  }
  val_0 /= (p * 10);

  return val_0;
}
void isiairbag()
{
  unsigned long now = millis();
  // press = readpressure();

  if (stepinf == 0)
  {

    if (now - pressmilis > windowpress)
    {
      counter++;
      pressmilis = now;
      digitalWrite(INF_PIN, LOW);
      delay(200);
      pressure = readpressure();
      selisihpress = setpressure - pressure;
      digitalWrite(INF_PIN, HIGH);
      if (selisihpress > 30)
        windowpress = 15000;
      else if (selisihpress > 20)
        windowpress = 8000;
      else if (selisihpress > 10)
        windowpress = 4000;
      else if (selisihpress > 5)
        windowpress = 2000;
      else if (selisihpress > 2)
        windowpress = 1000;
      else if (selisihpress < -1)
      {
        delay(500);
        stepinf = 1;
        digitalWrite(INF_PIN, LOW);
        windowpress = 2000;
      }
      Serial.print("ISI ");
      Serial.print(pressure, 1);
      Serial.print(" ");
      Serial.print(selisihpress, 1);
      Serial.print(" ");
      Serial.print(counter);
      Serial.print(" ");
      Serial.print(stepinf);
      Serial.println(" ");
    }
  }
  if (stepinf == 1)
  {

    if (now - pressmilis > windowpress)
    {
      counter++;
      pressmilis = now;
      digitalWrite(RIL_PIN, LOW);
      delay(200);
      pressure = readpressure();
      selisihpress = setpressure - pressure;
      if (fabs(selisihpress) <= 0.2)
      {
        digitalWrite(RIL_PIN, LOW);
        stepinf = 2;
        windowpress = 1000;
      }
      else
      {
        digitalWrite(RIL_PIN, HIGH);
      }
      Serial.print("RILIS ");
      Serial.print(pressure, 1);
      Serial.print(" ");
      Serial.print(selisihpress, 1);
      Serial.print(" ");
      Serial.print(counter);
      Serial.print(" ");
      Serial.print(stepinf);
      Serial.println(" ");
    }
  }
  if (stepinf == 2)
  {
    if (now - pressmilis > windowpress)
    {
      pressmilis = now;
      digitalWrite(RIL_PIN, LOW);
      digitalWrite(INF_PIN, LOW);
      delay(200);
      pressure = readpressure();
      selisihpress = setpressure - pressure;
      String str;
      if (selisihpress > 0.5)
      {
        digitalWrite(RIL_PIN, LOW);
        digitalWrite(INF_PIN, HIGH);
        windowpress = 1000;
        str = "kurang";
      }
      else if (selisihpress < -0.5)
      {
        digitalWrite(RIL_PIN, HIGH);
        digitalWrite(INF_PIN, LOW);
        windowpress = 1000;
        str = "lebih";
      }
      else
      {
        digitalWrite(RIL_PIN, LOW);
        digitalWrite(INF_PIN, LOW);
        windowpress = 2000;
        str = "stabil";
      }
      Serial.print("maintain ");
      Serial.print(pressure, 1);
      Serial.print(" ");
      Serial.print(selisihpress, 1);
      Serial.print(" ");
      Serial.print(counter);
      Serial.print(" ");
      Serial.print(stepinf);
      Serial.print(" ");
      Serial.print(str);
      Serial.println(" ");
    }
  }
  if (stepinf == 10)
  {
    digitalWrite(RIL_PIN, LOW);
    digitalWrite(INF_PIN, LOW);
    pressure= readpressure();
  }
}

void updateLamp(LampMode mode)
{
  switch (mode)
  {
  case OFF:
    digitalWrite(LED_PIN, LOW);
    break;

  case ON:
    digitalWrite(LED_PIN, HIGH);
    break;

  case BLINKING:
    digitalWrite(LED_PIN, (millis() / 500) % 2); // kedip 0.5 detik
    break;
  }
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
  case EMERGENCY:
    return "EMERGENCY STOP !!";
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
  if (digitalRead(STOP_PIN == 1))
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
  EEPROM_writeAnything(ADDR_SHORT_VAL_SET, shortvalset);
  EEPROM_writeAnything(ADDR_SUHU_NORMAL_SET, suhunormalset);
  EEPROM_writeAnything(ADDR_SHORT_CJ_VAL_SET, shortcjvalset);
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

  EEPROM_readAnything(ADDR_SHORT_VAL_SET, shortvalset);
  EEPROM_readAnything(ADDR_SUHU_NORMAL_SET, suhunormalset);
  EEPROM_readAnything(ADDR_SHORT_CJ_VAL_SET, shortcjvalset);

  // Cek NaN atau nilai tidak wajar
  if (isnan(csetpoint) || csetpoint < 24.0 || csetpoint > 300.0)
    csetpoint = 140.0;

  if (isnan(setSuhuAlarm) || setSuhuAlarm < csetpoint || setSuhuAlarm > 350.0)
    setSuhuAlarm = csetpoint + 5;

  if (isnan(suhupreheat) || suhupreheat < 0 || suhupreheat >= csetpoint)
    suhupreheat = 50;
  if (isnan(timereminder) || timereminder < 0 || timereminder > 120)
    timereminder = 15;
  if (isnan(csetTimer) || csetTimer < 60 || csetTimer > 86400)
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

  if (isnan(cmulaipid) || cmulaipid < 0.00 || cmulaipid > 50.0)
    cmulaipid = 15;

  if (isnan(hourmeter) || hourmeter < 0 || hourmeter > 999999)
    hourmeter = 0;

  if (isnan(timerrun) || timerrun < 0 || timerrun > 999999)
    timerrun = 0;

  /* Serial.println(csetpoint);
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
 Serial.println(timerrun);
 Serial.println(hourmeter); */
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
  return 0; // kalau gagal semua, pakai nilai lama
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
