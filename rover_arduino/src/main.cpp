#include <ros.h>
#include "rover_udes/CamCommand.h"
#include "Encodeur.cpp" //utilise le timer 2
#include <Servo.cpp>

//servo-moteurs
/*le servo vertical utilise la pin 10
  et le servo Horizontal la pin 9
  L'encodeur doit être brancher dans la pin A0 */
Servo servoCamVertical; 
Servo servoCamHorizontal;
Encodeur controlServoH('A0');
int objectifServoHorizontal = 0;


//node
ros::NodeHandle nh;

// callback ros
void servoCam_cb (const rover_udes::CamCommand &angles)
{
  objectifServoHorizontal = angles.cam_vertical;
  servoCamVertical.writeMicroseconds((int)1500+angles.cam_horizontal*500/90);
}

//ajustement servoHorizontal
void updateServoCamH()
{
  const float tourEncParTourServo = 4;
  float position = controlServoH.ReadAngle() / tourEncParTourServo;
  int delta = objectifServoHorizontal - position;
  while (delta >= 180)delta -= 360;
  while (delta < -180)delta += 360;
  int vitesse = 1500;
  if (delta > 5) vitesse = 1700;
  else if (delta > 0) vitesse = 1600;
  else if (delta < -5) vitesse = 1300;
  else if (delta < 0) vitesse = 1400;

  servoCamHorizontal.writeMicroseconds(vitesse);
}


//subcriber utilisant le callback servo_cb
ros::Subscriber<rover_udes::CamCommand> ear ("cam_cmd",servoCam_cb);



void setup() {

  servoCamHorizontal.attach(9);
  servoCamVertical.attach(10);
  gestionaireTimer2::ajouter(updateServoCamH);

  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(ear);
}

void loop() {
  //mise à jour de la node
  nh.spinOnce();
}