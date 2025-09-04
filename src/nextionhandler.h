#ifndef NEXTION_HANDLER_H
#define NEXTION_HANDLER_H
#define dspSerial Serial1
#include <Arduino.h>



void nextionReceiveHandler();
void sendNextionEnd();
String getValue(String data, char separator, int index) ;
#endif
