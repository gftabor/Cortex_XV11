

task lidarMotorSpeed()
{
	motor[lidarMotor] = 65;
	short speed = 60;

	while(true)
	{
		lidarRPM = lidarRPM > 500? 500 : lidarRPM;

		if(lidarRPM < goalRPM)
			speed = speed > 75?75:speed+1;
		if(lidarRPM > goalRPM)
			speed = speed < 40?40:speed-1;
		motor[lidarMotor] = speed;
		delay(125);
	}
}

void clearComms()
{
	while(true)
	{
		char crap = getChar(CORTEXA);
		writeDebugStreamLine("HERE'S THE CRAP IN THE COMMS %X *******",crap);
		if(crap == 0xFF)
		{
			writeDebugStreamLine("CLEARED THE BUFFER FOR YOU BOSS");
			break;
		}
	}
}


bool calculateBallPos(float xPos, float yPos, float theta, short mag, short angle, float* x, float* y, short* newMag, short* newAngle)
{
	const float chassisLength=292.5;
	int dist = mag;
	short newTheta=180-abs(104-(angle));

	float newDist= sqrt((dist)*(dist)+chassisLength*chassisLength-2*(dist)*chassisLength*cosDegrees(newTheta));
	*newMag=newDist;
	*newAngle =((angle<104)?-1:1)*((angle>284)?-1:1)*(180-radiansToDegrees(acos(((dist)*(dist)-newDist*newDist-chassisLength*chassisLength)/(-2*chassisLength*newDist))));

	*x = xPos+newDist*cosDegrees(theta+(*newAngle));
	*y = yPos+newDist*sinDegrees(theta+(*newAngle));

	return true;
}
