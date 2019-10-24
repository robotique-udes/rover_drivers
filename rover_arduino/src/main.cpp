#include <ros.h>
#include "rover_control/CamCommand.h"
#include "ServoMoteur.h"

//servo-moteurs
/*le servo vertical utilise la pin 13
  et le servo Horizontal la pin 12 */
ServoMoteur servoCamVertical(13,20000,1500,2500,90,-90); 
ServoMoteur servoCamHorizontal(12,20000,1500,2500,90,-90);
//servo-moteurs pour bras untilisation des pin 2 à 7
ServoMoteur servoPince(2); 
ServoMoteur servoPoignet(3); 
ServoMoteur servoJ2(4); 
ServoMoteur servoJ1(5); 
ServoMoteur servoJ0(6);
ServoMoteur servoBase(7); 

//node
ros::NodeHandle nh;

// callback ros
void servoCam_cb (const rover_control::CamCommand &angles)
{
  servoCamHorizontal.setAngle(angles.cam_horizontal);
  servoCamVertical.setAngle(angles.cam_vertical);
}

//subcriber utilisant le callback servo_cb
ros::Subscriber<rover_control::CamCommand> ear ("cam_cmd",servoCam_cb);



void setup() {
  Serial.begin(57600);
  //initialisation de la node et du subcriber
  nh.initNode();
  nh.subscribe(ear);

  servoJ2.setAngle(90);
  servoJ1.setAngle(45);
  servoJ0.setAngle(0);
}

void loop() {
  //mise à jour des servo-moteurs
  servoCamVertical.update();
  servoCamHorizontal.update();
  servoPince.update();
  servoPoignet.update();
  servoJ2.update();
  servoJ1.update();
  servoJ0.update();
  servoBase.update();
  //mise à jour de la node
  nh.spinOnce();
}