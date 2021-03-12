#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include "rover_udes/CamCommand.h"
#include "kg_encoder.h" //uses timer2
#include <Servo.h>

// Servomotors
/*The vertical servo (up-down) uses Arduino pin 7
  The horizontal servo (left-right) uses Arduino pin 6
  The horizontal encoder has to be plugged into the A0 pin of the Arduino 
  The vertical encoder has to be plugged into the A1 pin of the Arduino*/
  
Servo verticalCamServo; 
Servo horizontalCamServo;

Encoder horizontalServoControl(A0);
Encoder verticalServoControl(A1);

int horizontalServoGoal = 0;
int verticalServoGoal = 0;

int mode = 0;

bool currentlyTakingPanorama = false;
const float encoderRotationsForEachServoRotation = 48.0/16.0;

// ROS node
ros::NodeHandle nh;

// function prototypes
void callback_camServo(const rover_udes::CamCommand &msg);
float velocityLimitation(float vel, float pos);
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

float velocityLimitation(float vel, float pos) // Limit velocity command to -1, 1 and 0
{
	if(vel > 1.0)
	{
		return 1.0;
	}

	else if(vel < -1.0)
	{
		return -1.0;
	}

	if((pos > 175 && vel > 0) || (pos < -175 && vel < 0))
	{
		return 0.0;
	}
	
	return 0.0; // in case of error, 0.0 is returned
}

// servoHorizontal
void updateHorizontalCamServo()
{
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
		float velocityCmd = velocityLimitation(horizontalServoGoal, position);
		speed = (200.0 * velocityCmd) + 1500;
	}

	horizontalCamServo.writeMicroseconds(speed);
	// }
}

void updateVerticalCamServo()
{
	float position = verticalServoControl.ReadAngle() / encoderRotationsForEachServoRotation;

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
	// 		verticalCamServo.writeMicroseconds(1600);
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
		float velocityCmd = velocityLimitation(verticalServoGoal, position);
		speed = (200.0 * velocityCmd) + 1500;
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
	nh.spinOnce();
	test_output.publish(&infoEncoder);
	infoEncoder.linear.x = horizontalServoControl.ReadAngle(); // Will act as horizontal display of encoder
	infoEncoder.linear.y = verticalServoControl.ReadAngle(); // Will act as vertical display of encoder
    
}
