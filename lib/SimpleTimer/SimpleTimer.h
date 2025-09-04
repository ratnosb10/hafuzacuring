#ifndef SIMPLETIMER_H
#define SIMPLETIMER_H

#include <Arduino.h>

bool timerRunning = false;
unsigned long elapsedMillis = 0;
unsigned long previousMillis = 0;

void startTimer() {
  if (!timerRunning) {
    timerRunning = true;
    previousMillis = millis();
  }
}

void stopTimer() {
  if (timerRunning) {
    timerRunning = false;
    elapsedMillis += millis() - previousMillis;
  }
}

void updateTimer() {
  if (timerRunning) {
    unsigned long now = millis();
    elapsedMillis += now - previousMillis;
    previousMillis = now;
  } else {
    previousMillis = millis();  // Hindari lompatan waktu
  }
}

void resetTimer() {
  timerRunning = false;
  elapsedMillis = 0;
  previousMillis = millis();
}

unsigned long getElapsedTime() {
  return elapsedMillis / 1000;
}

String getElapsedTimeString() {
  unsigned long totalSeconds = getElapsedTime();
  int hours = totalSeconds / 3600;
  int minutes = (totalSeconds % 3600) / 60;
  int seconds = totalSeconds % 60;

  char buffer[9];
  sprintf(buffer, "%02d:%02d:%02d", hours, minutes, seconds);
  return String(buffer);
}

void setTimer(unsigned long startSeconds) {
  elapsedMillis = startSeconds * 1000;
}












#endif
