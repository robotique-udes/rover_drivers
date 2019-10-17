#ifndef SERVOMOTEUR
#define SERVOMOTEUR
 
class ServoMoteur
{
  protected:
  int pin;
  unsigned long temps;//microsecondes
  unsigned int periode;//microsecondes
  unsigned int haut;
  int microDeg;
  int angleMax;
  int angleMin;

  public:
  ServoMoteur(int _pin,int _periode = 20000,int deg0 = 1500, int deg90 =2000,int _angleMax = 90, int _angleMin = -90);

  void setAngle(int angle);//degrés
  void update();
};
 
#endif