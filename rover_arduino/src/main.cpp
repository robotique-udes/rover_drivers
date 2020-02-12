#include <ros.h>
#include "rover_udes/CamCommand.h"
#include "Encodeur.h" //utilise le timer 2
#include <Servo.h>

//servo-moteurs
/*le servo vertical utilise la pin 10
  et le servo horizontal la pin 9
  L'encodeur doit être branché dans la pin A0 */
Servo servoCamVertical; 
Servo servoCamHorizontal;
Encodeur controlServoH(A0);
int objectifServoHorizontal = 0;
bool panoEnCours = false;

//node
ros::NodeHandle nh;

// callback ros
void servoCam_cb (const rover_udes::CamCommand &angles)
{
	if (angles.is_pano and !panoEnCours)
	{
		panoEnCours = true;
	}

	if (panoEnCours)
	{
		objectifServoHorizontal = 0;
		servoCamVertical.writeMicroseconds((int)1500);
	}
	else
	{
		objectifServoHorizontal = angles.cam_horizontal;
		servoCamVertical.writeMicroseconds((int)1500+angles.cam_vertical*500/90);
	}
}

//ajustement servoHorizontal
void updateServoCamH()
{
	const float tourEncParTourServo = 4;
	float position = controlServoH.ReadAngle() / tourEncParTourServo;
	static bool debutPano = true;

	if (panoEnCours)
	{
		float posInit;
		if (debutPano)
		{
			posInit = position;
			debutPano = false;
		}

		if (position < (posInit + 360))
		{
			servoCamHorizontal.writeMicroseconds(1600);
		}
		else
		{
			panoEnCours = false;
			debutPano = true;
		}
	}
	else
	{	
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
}

//subcriber utilisant le callback servo_cb
ros::Subscriber<rover_udes::CamCommand> ear ("cam_cmd",servoCam_cb);

void setup()
{
	servoCamHorizontal.attach(9);
	servoCamVertical.attach(10);
	gestionaireTimer2::ajouter(updateServoCamH);

	Serial.begin(57600);
	nh.initNode();
	nh.subscribe(ear);
}

void loop()
{
	//mise à jour de la node
	nh.spinOnce();
}