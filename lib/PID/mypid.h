#ifndef MYPID_H
#define MYPID_H

#include <Arduino.h>
#include <PID_v1.h>

// === Variabel Global PID Autotune ===
float suhuAwal = 0;
float suhuOff = 0;
float suhuTertinggi = 0;
float suhuTurun = 0;
float delta = 0;

// Internal tracking
static double lastinputsuhu = 0;
static unsigned long lastSlopeCheck = 0;
static unsigned long stagnanStartTime = 0;
static double stagnanStartTemp = 0;

bool prosesNaik = false;
bool heaterNyala = false;
bool heaterDimatikan = false;
bool mulaiTurun = false;

unsigned long lastSendTime = 0;

double Kp = 0, Ki = 0, Kd = 0;
double setpoint = 150;
double output = 0;
double input = 0;

double mulaipid;
double slope = 0;
String lain;
unsigned long stagnanTime = 0;
bool boostMode = false;

unsigned long windowSize = 3000;
unsigned long windowStartTime = 0;
unsigned long now = 0;
int pinheater;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
extern void kirimautotune(float Kp, float Ki, float Kd,float mulaipid);
// === Inisialisasi Autotune ===
void initAutoTune(float suhuSekarang, int HEATER_PIN)
{
  pinheater = HEATER_PIN;
  suhuAwal = suhuSekarang;
  prosesNaik = true;
  heaterNyala = true;
  heaterDimatikan = false;
  mulaiTurun = false;
  suhuTertinggi = 0;

  pinMode(pinheater, OUTPUT);
  digitalWrite(pinheater, HIGH); // nyalakan heater
  Serial.println("Autotune dimulai...");
}

// === Proses Autotune PID ===
bool autotune(float suhuSekarang)
{
  unsigned long now = millis();

  // Debug suhu berkala
  if (now - lastSendTime >= 1000)
  {
    lastSendTime = now;
    Serial.print("Awal: ");
    Serial.print(suhuAwal, 2);
    Serial.print(" | Temp: ");
    Serial.print(suhuSekarang, 2);
    Serial.print(" | Off: ");
    Serial.print(suhuOff, 2);
    Serial.print(" | Tertinggi: ");
    Serial.println(suhuTertinggi, 2);
  }

  // Tahap Naik 10 derajat
  if (prosesNaik && suhuSekarang >= suhuAwal + 15.0)
  {
    digitalWrite(pinheater, LOW);
    heaterNyala = false;
    prosesNaik = false;
    heaterDimatikan = true;
    suhuTertinggi = suhuSekarang;
    suhuOff = suhuSekarang;
    Serial.print("Heater dimatikan pada suhu: ");
    Serial.println(suhuOff);
    Serial.println("Menunggu suhu mulai turun...");
    delay(5000);
  }

  // Deteksi suhu mulai turun
  if (heaterDimatikan)
  {
    if (suhuSekarang > suhuTertinggi)
    {
      suhuTertinggi = suhuSekarang;
      mulaiTurun = false;
      delay(2000);
    }

    if (!mulaiTurun && suhuSekarang < suhuTertinggi - 0.3)
    {
      suhuTurun = suhuSekarang;
      delta = suhuTurun - suhuOff;
      mulaipid = delta * 2;
      mulaiTurun = true;

      if (delta != 0)
      {
        Kp = 75.0 / delta;
        Ki = Kp / (2 * delta);
        Kd = Kp * (0.5 * delta);
      }
      else
      {
        Kp = Ki = Kd = 0;
      }

      Kp = constrain(Kp, 0, 10);
      Ki = constrain(Ki, 0, 0.5);
      Kd = constrain(Kd, 0, 200);

       kirimautotune( Kp,  Ki,  Kd, mulaipid);
      Serial.println("=== AUTOTUNE SELESAI ===");
      Serial.print("suhuOff: ");
      Serial.println(suhuOff);
      Serial.print("suhuTurun: ");
      Serial.println(suhuTurun);
      Serial.print("delta: ");
      Serial.println(delta);
      Serial.print("mulaipid: ");
      Serial.println(mulaipid);
      Serial.print("Kp: ");
      Serial.print(Kp, 2);
      Serial.print(" | Ki: ");
      Serial.print(Ki, 2);
      Serial.print(" | Kd: ");
      Serial.println(Kd, 2);

      // Kirim ke Nextion
      return true;
    }
  }
  return false;
}

void getTuneresult(double &p, double &i, double &d, double &m)
{
  p = Kp;
  i = Ki;
  d = Kd;
  m = mulaipid;
}

void setupPID(int HEATER_PIN, double p, double i, double d, double m)
{
  Kp = p;
  Ki = i;
  Kd = d;
  mulaipid = m;
  pinheater = HEATER_PIN;
  pinMode(HEATER_PIN, OUTPUT);
  myPID.SetOutputLimits(0, 100); // Jika perlu diatur ulang
  myPID.SetMode(AUTOMATIC);
  myPID.SetTunings(Kp, Ki, Kd);
  Serial.print(" ");
  Serial.print(Kp);
  Serial.print(" ");
  Serial.print(Ki);
  Serial.print(" ");
  Serial.print(Kd);
  Serial.print(" ");
  Serial.print(mulaipid);
}
void reset_pid()
{

  myPID.SetMode(MANUAL);
  output = 0;

  myPID.SetTunings(Kp, Ki, Kd); // Jika mau set ulang PID
  myPID.SetMode(AUTOMATIC);
}

void Heteroff()
{
  digitalWrite(pinheater, LOW);
}

bool runPID(float suhusekarang, float set)
{
  setpoint = set;
  bool heaterpin;
  unsigned long now = millis();
  input = suhusekarang;
  // Hitung slope setiap 2 detik
  if (now - lastSlopeCheck >= 2000)
  {
    slope = input - lastinputsuhu;
    lastSlopeCheck = now;
    lastinputsuhu = input;
  }

  if (input >= (setpoint - mulaipid))
  {
    // Deteksi stagnan
    if (input < setpoint && !boostMode && abs(slope) < 0.05)
    {
      if (stagnanStartTime == 0)
      {
        stagnanStartTime = now;
        stagnanStartTemp = input;
      }
      else if (now - stagnanStartTime >= 60000)
      {
        boostMode = true;
        Serial.println("⚠️ Stagnan terlalu lama. BOOST mode ON (heater penuh)");
      }
      stagnanTime = (now - stagnanStartTime) / 1000;
    }
    else
    {
      stagnanStartTime = 0;
    }

    // BOOST MODE aktif
    if (boostMode)
    {
      lain = "boostMode";
      digitalWrite(pinheater, HIGH);
      heaterpin = HIGH;
      if (input >= stagnanStartTemp + 0.3 || input > setpoint)
      {
        boostMode = false;
        stagnanStartTime = 0;
        Serial.println("✅ BOOST selesai. PID aktif kembali");
      }
    }
    // Overheat proteksi
    else if (input > setpoint && slope > 0.01)
    {
      lain = "Overheat";
      digitalWrite(pinheater, LOW);
      heaterpin = LOW;
    }
    // Suhu turun → Heater ON penuh
    else if (input < setpoint && slope < 0.01)
    {
      lain = "Suhu turun ";
      digitalWrite(pinheater, HIGH);
      heaterpin = HIGH;
    }
    // PID aktif normal
    else
    {
      myPID.Compute();
      if (output > 0.5 && output < 33)
        output = 33;

      if (now - windowStartTime > windowSize)
      {
        windowStartTime = now;
      }
      else
      {
        double scaledOutput = (output / 100.0) * windowSize;
        if (now - windowStartTime < scaledOutput)
        {
          digitalWrite(pinheater, HIGH);
          heaterpin = HIGH;
        }
        else
        {
          digitalWrite(pinheater, LOW);
          heaterpin = LOW;
        }
      }

      lain = "PID aktif normal";
    }
  }
  else
  {
    digitalWrite(pinheater, HIGH); // Heater ON penuh saat suhu masih jauh
    heaterpin = HIGH;
    reset_pid();
    lain = "belum";
  }

  // Serial debug
  if (now - lastSendTime >= 2000)
  {
    lastSendTime = now;

    Serial.print(input);
    Serial.print("\tSet:");
    Serial.print(setpoint, 0);
    Serial.print(" ");
    Serial.print(Kp);
    Serial.print(" ");
    Serial.print(Ki);
    Serial.print(" ");
    Serial.print(Kd);
    Serial.print(" ");
    Serial.print("\tOut:");
    Serial.print(output);
    Serial.print("\tMulai:");
    Serial.print(mulaipid, 0);
    Serial.print("\tSlope:");
    Serial.print(slope, 3);
    Serial.print("\tLain:");
    Serial.print(lain);
    Serial.print("\tStag(s):");
    Serial.println(stagnanTime);
  }
  return heaterpin;
}

bool deteksiNoHeating(bool heaterOn, float suhuSekarang, unsigned long intervalMs = 60000, float ambangSlope = 0.3)
{
  static unsigned long lastCheckTime = 0;
  static float lastSuhu = 0;
  static unsigned long heaterStartTime = 0;

  unsigned long now = millis();

  // Jalankan hanya jika suhu masih di bawah 50
  if (suhuSekarang < 50.0)
  {
    // Jika heater ON
    if (heaterOn)
    {
      if (heaterStartTime == 0)
      {
        heaterStartTime = now;
        lastCheckTime = now;
        lastSuhu = suhuSekarang;
      }

      // Cek tiap interval
      if (now - lastCheckTime >= intervalMs)
      {
        float slope = suhuSekarang - lastSuhu;
        lastCheckTime = now;
        lastSuhu = suhuSekarang;

        if (slope < ambangSlope)
        {
          Serial.println("❌ No Heating Detected! (Suhu tidak naik saat heater ON)");
          return true;
        }
      }
    }
    else
    {
      // Heater OFF → reset semua
      heaterStartTime = 0;
      lastCheckTime = now;
      lastSuhu = suhuSekarang;
    }
  }
  else
  {
    // Jika suhu >= 50, reset timer
    heaterStartTime = 0;
    lastCheckTime = now;
    lastSuhu = suhuSekarang;
  }

  return false;
}

#endif
