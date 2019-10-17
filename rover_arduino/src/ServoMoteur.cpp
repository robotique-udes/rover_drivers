#include "ServoMoteur.h"
#include <Arduino.h>


ServoMoteur::ServoMoteur(int _pin,int _periode = 20000,int deg0 = 1500, int deg90 =2000,int _angleMax = 90, int _angleMin = -90)
  {
    pin = _pin;
    temps = micros();
    haut = 1500;
    periode = _periode;
    pinMode(pin,OUTPUT);
    microDeg = (deg90-deg0)/90;
    angleMax = _angleMax;
    angleMin = _angleMin;
  }
  void ServoMoteur::setAngle(int angle)//degrés
  {
    if (angle > angleMax)angle = angleMax;
    else if (angle < angleMin)angle = angleMin;
    
    haut = 1500 + angle * microDeg;
  }
  void ServoMoteur::update()
  {


    unsigned long delta = micros() - temps;
    if (delta<0) delta = 4294967295 - temps + micros();
    if (delta <= haut)
digitalWrite(pin,HIGH);
    
    else digitalWrite(pin,LOW);
    if (delta >= periode)temps = micros();
  }