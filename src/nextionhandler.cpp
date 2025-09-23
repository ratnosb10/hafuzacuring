#include "NextionHandler.h"
#define OPENING 0
#define MAIN 1
#define CONFIG 2
#define ADVANCE 3
#define TEST 4

extern double csetpoint, setSuhuAlarm, suhupreheat;
extern unsigned long csetTimer;
extern double cKp, cKi, cKd;
extern double dran, adjustmentSuhu, setpressure, timereminder, adjpress, cmulaipid;
extern byte currentpage;
extern unsigned long timerrun, hourmeter;
extern void saveConfig();
extern void loadConfig();
extern void sendconfigdisplay();
extern void savetimer();
extern bool timesup;
extern bool tccal;
extern bool pidautotune;
String cmd;
void nextionReceiveHandler()
{
  while (dspSerial.available())
  {
    String incomingString = dspSerial.readStringUntil('$');
    cmd = getValue(incomingString, ' ', 1);
    if (cmd == "tomain")
    {
      sendconfigdisplay();
      currentpage = MAIN;
      dspSerial.print("page main");
      sendNextionEnd();
    }
    if (cmd == "toadvance")
    {
      sendconfigdisplay();
      currentpage = ADVANCE;
      dspSerial.print("page advance");
      sendNextionEnd();
    }
    if (cmd == "save")
    {
      csetpoint = getValue(incomingString, ' ', 2).toDouble();
      suhupreheat = getValue(incomingString, ' ', 3).toDouble();
      setpressure = getValue(incomingString, ' ', 4).toDouble();
      setSuhuAlarm = getValue(incomingString, ' ', 7).toDouble();
      timereminder = getValue(incomingString, ' ', 8).toDouble();
      unsigned long jam = getValue(incomingString, ' ', 5).toInt();
      unsigned long menit = getValue(incomingString, ' ', 6).toInt();
      csetTimer = (jam * 3600UL) + (menit * 60UL);
      saveConfig();
      dspSerial.print("page main");
      sendNextionEnd();
      loadConfig();
      sendconfigdisplay();
      currentpage = MAIN;
    }
    if (cmd == "saveadvan")
    {
      adjustmentSuhu = getValue(incomingString, ' ', 2).toDouble();
      adjpress = getValue(incomingString, ' ', 3).toDouble();
      cKp = getValue(incomingString, ' ', 4).toDouble() / 10;
      cKi = getValue(incomingString, ' ', 5).toDouble() / 10;
      cKd = getValue(incomingString, ' ', 6).toDouble() / 10;
      cmulaipid = getValue(incomingString, ' ', 7).toDouble() / 10;
      saveConfig();
      sendconfigdisplay();

      /*  Serial.println(adjustmentSuhu);
       Serial.println(adjpress);
      Serial.println(cKp);
  Serial.println(cKi);
  Serial.println(cKd);
Serial.println(cmulaipid); */

      dspSerial.print("page main");
      sendNextionEnd();
      currentpage = MAIN;
    }
    if (cmd == "timerrun")
    {
      String strtimer;
      strtimer = getValue(incomingString, ' ', 2);
      int j, m, d;
      j = getValue(strtimer, ':', 0).toInt();
      m = getValue(strtimer, ':', 1).toInt();
      d = getValue(strtimer, ':', 2).toInt();
      timerrun = (j * 3600UL) + (m * 60UL) + d;
      hourmeter++;
      savetimer();
    }
    if (cmd == "finished")
    {
      timesup = true;
    }
    if (cmd == "resettimer")
    {
      timerrun = 0;
    }
    if (cmd == "tccal")
    {
      tccal = true;
      currentpage = ADVANCE;
    }
   if (cmd == "pidtune")
    {
      pidautotune = true;
      currentpage = ADVANCE;
    }

  }
}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++)
  {
    if (data.charAt(i) == separator || i == maxIndex)
    {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
