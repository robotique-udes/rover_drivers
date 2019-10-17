#include <Arduino.h>
#include <ros.h>
#include "rover_control/CamCommand.h"
#include "ServoMoteur.h"

//servo-moteurs
/*le servo vertical utilise la pin 13
  et le servo Horizontal la pin 12 */
ServoMoteur servoCamVertical(13,20000,1500,2500,90,-90); 
ServoMoteur servoCamHorizontal(12,20000,1500,2500,90,-90);

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
}

void loop() {
  //mise à jour des servo-moteurs
  servoCamVertical.update();
  servoCamHorizontal.update();
  //mise à jour de la node
  nh.spinOnce();
}