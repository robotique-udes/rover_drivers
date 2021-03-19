#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include "rover_udes/CamCommand.h"
#include "kg_encoder.h" //uses timer2
#include <Servo.h>

// Servomotors
/*The vertical servo (up-down) uses Arduino pin 7
  The horizontal servo (left-right) uses Arduino pin 6
  The encoder for the horizontal servo has to be plugged in Arduino pin A0
  The encoder for the vertical servo has to be plugged in Arduino pin A1 */
Servo verticalCamServo; 
Servo horizontalCamServo;

Encoder horizontalServoControl(A0);
Encoder verticalServoControl(A1);

float horizontalServoGoal = 0;
float verticalServoGoal = 0;

int mode = 0;

bool currentlyTakingPanorama = false;
const float encoderRotationsForEachServoRotation = 48.0/16.0;
float INFO = 100.0;

// ROS node
ros::NodeHandle nh;

// function prototypes
void callback_camServo (const rover_udes::CamCommand &msg);
void updateHorizontalCamServo();
void updateVerticalCamServo();

// Subcriber using callback callback_camServo
geometry_msgs::Twist infoEncoder;
// ros::Subscriber<rover_udes::CamCommand> sub_cam_cmd ("mux_cmd_ptu", callback_camServo);
ros::Subscriber<rover_udes::CamCommand> sub_cam_cmd ("/cmd_ptu", callback_camServo);
ros::Publisher test_output("/encoder_status", &infoEncoder);

// ROS callback
void callback_camServo (const rover_udes::CamCommand &msg)
{
	/* This will be used for implementing constant intervals picture taking for panoramas */

	// if (msg.is_pano and !currentlyTakingPanorama)
	// {
	// 	currentlyTakingPanorama = true;
	// }

	// if (currentlyTakingPanorama)
	// {
	// 	horizontalServoGoal = 0;
	// 	verticalCamServo.writeMicroseconds((int)1500);
	// }
	// else
	// {
	// }
	mode = msg.mode;
	horizontalServoGoal = msg.cam_horizontal;
	verticalServoGoal = msg.cam_vertical;
}

// servoHorizontal
void updateHorizontalCamServo()
{
	const float encoderRotationsForEachServoRotation = 48.0/16.0;
	float position = horizontalServoControl.ReadAngle() / encoderRotationsForEachServoRotation;

	/* This will be used for implementing constant intervals picture taking for panoramas */

	// static bool panoramaBeginning = true;

	// if (currentlyTakingPanorama)
	// {
	// 	float initialPosition;
	// 	if (panoramaBeginning)
	// 	{
	// 		initialPosition = position;
	// 		panoramaBeginning = false;
	// 	}

	// 	if (position < (initialPosition + 360))
	// 	{
	// 		horizontalCamServo.writeMicroseconds(1600);
	// 	}
	// 	else
	// 	{
	// 		currentlyTakingPanorama = false;
	// 		panoramaBeginning = true;
	// 	}
	// }
	// else
	// {
	int speed = 1500;

	if(mode == 0)  // position mode
	{
		int delta = horizontalServoGoal - position;

	/*  These two statements can be enabled for the camera to always use the shortest path to reach its goal,
		but this could cause problems with the wires */
				// while (delta >= 180)delta -= 360;
				// while (delta < -180)delta += 360;
		if (delta > 5) speed = 1700;
		else if (delta > 0) speed = 1600;
		else if (delta < -5) speed = 1300;
		else if (delta < 0) speed = 1400;
	}
	else if(mode == 1)  // velocity mode
	{
		speed = (200.0 * horizontalServoGoal) + 1500;
	}

	horizontalCamServo.writeMicroseconds(speed);
	// }
}


void updateVerticalCamServo()
{
	const float encoderRotationsForEachServoRotation = 48.0/16.0;
	float position = verticalServoControl.ReadAngle() / encoderRotationsForEachServoRotation;

	int speed = 1500;

	if(mode == 0)  // position mode
	{
		int delta = verticalServoGoal - position;

	/*  These two statements can be enabled for the camera to always use the shortest path to reach its goal,
		but this could cause problems with the wires */
				// while (delta >= 180)delta -= 360;
				// while (delta < -180)delta += 360;
		if (delta > 5) speed = 1700;
		else if (delta > 0) speed = 1600;
		else if (delta < -5) speed = 1300;
		else if (delta < 0) speed = 1400;
	}
	else if(mode == 1)  // velocity mode
	{
		speed = (200.0 * verticalServoGoal) + 1500;
	}

	verticalCamServo.writeMicroseconds(speed);
	// }
}

void setup()
{
	horizontalCamServo.attach(6);
	verticalCamServo.attach(7);
	Timer2Manager::add(updateHorizontalCamServo);
	Timer2Manager::add(updateVerticalCamServo);

	Serial.begin(57600);
	nh.initNode();
	nh.subscribe(sub_cam_cmd);
	nh.advertise(test_output);
}

void loop()
{
	// Continuously updating the node
	
    
	infoEncoder.linear.x = horizontalServoControl.ReadAngle(); // Will act as horizontal display of encoder
	infoEncoder.linear.y = horizontalCamServo.read(); // Will act as horizontal display of Servo position (angle between 0 and 180)
	infoEncoder.linear.z = horizontalCamServo.readMicroseconds(); //Should give actual pulse
	infoEncoder.angular.x = INFO;
	test_output.publish(&infoEncoder);

	nh.spinOnce();
	
}