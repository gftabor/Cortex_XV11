/**********************************************************************************************
* XV-11 lidar data parser
* Data packets are 22 bytes long and are packaged as follows <start> <index> <speed_L> <speed_H> [Data 0] [Data 1] [Data 2] [Data 3] <checksum_L> <checksum_H>
* Data arrays are packaged as follows:
* `byte 0 : <distance 7:0>`
* `byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8>`
* `byte 2 : <signal strength 7:0>`
* `byte 3 : <signal strength 15:8>`
* by Nick Sorensen
* Created 3/19/2016
*
**********************************************************************************************/
#pragma systemFile

#define timeOut 25

float R  = 1.42;//*25.4;

/**
 * Gets data for 360 degrees of data. Array index corresponds to angle of the reading.
 * Data is magnitude of sensor value and is in units of milimeters.
 *
 * 360 degrees of sensors readings [0-359]
 *
 * Returns true
 */
bool getLidarData(const TUARTs uart, short* lidarDistData)
{
	short data[5];
	short count = 0;

	for (int i = 0; i < 360; i++)
	{
		lidarDistData[i] = -1;
	}

	readLidarUART(uart, data);

	//clearTimer(T4);
	/*while (data[0] != 0)
	{
	readLidarUART(uart, data);
	}

	for (int x = 0; x < 4; x++)
	{
	lidarDistData[x] = data[x + 1];
	}*/
	//writeDebugStreamLine("Time: %d",time1[T4]);

	for (int i = 0; i < 90; i++)
	{
		if (readLidarUART(uart, data))
		{

			short angle = data[0];

			for (int x = 0; x < 4; x++)
			{
				lidarDistData[angle + x] = data[x + 1];
			}
			if (count == 10)
			{
				sleep(1);
				count = 0;
			}
			count++;
		}
	}
	return true;
}

/**
* Reads the uart port and extracts data for only one packet from the lidar.
* Data in array format as follows. {Start Angle, dist1, dist2, dist3, dist4}
* Returns true
*/
bool readLidarUART(const TUARTs uart, short* data)
{
	char lidarRawData[22];
	short dist[4];
	short angle;

	if (readLidarRaw(uart, lidarRawData))
	{
		lidarRPM = ((float)((lidarRawData[2] ) | (lidarRawData[3] << 8))) / 64.0;

		for (int i = 0; i < 4; i++) //read every first and second byte from the data to get the distance
		{
			char distPacket[4];
			for (int j = 0; j < 4; j++)
			{
				distPacket[j] = lidarRawData[((i + 1) * 4) + j]; //capture the data part of the lidar packet, four per lidar packet.
			}
			dist[i] = readDistDataPacket(distPacket); //determine the distance for each data packet.
			//writeDebugStreamLine("%d",dist[i]);
		}

		angle = (lidarRawData[1] - 0xA0) * 4;
		data[0] = angle; //assign the angle to the first element in the distData array

		for (int i = 0; i < 4; i++)
		{
			data[i + 1] = dist[i];
		}

		return true;
	}
	return false;
}

/**
* Reads the raw data from the lidar
*
* TODO: make the function less cpu time expensive?
*
* Returns true
*/
bool readLidarRaw(const TUARTs uart, char* data)
{
	byte bytes = 1;
	//clearTimer(timer1);
	long lidarTime = nSysTime;
	while (getChar(uart) != 0xFA)// && (nSysTime - lidarTime) < timeOut)
		getChar(uart);
	data[0] = 0xFA;
	while (true)//&& (nSysTime - lidarTime) < timeOut)
	{
		char dataByte = 0;
		dataByte = getChar(uart);
		if (dataByte != 0xFF)
		{
			data[bytes] = dataByte;
			bytes++;
			if (bytes == 22)
				break;
		}
	}

	if(RELAYRAWDATA)
	{
		for(int i = 0; i < 22; i++)
		{
			sendChar(CORTEXA,data[i]);
		}
	}

	//writeDebugStreamLine("it took %d",time1[T1]);
	if (bytes != 22)
	{
		//writeDebugStreamLine("timed out reading the raw lidar data!");
		return false;
	}
	else
		lidarRPM = ((float)((data[2] ) | (data[3] << 8))) / 64.0;
	return true;
}

/**
* Reads the current lidar rpm
*
* Returns the current rpm of the lidar
*/
float getLidarRPM(const TUARTs uart)
{
	char data[22];
	float rpm;
	bool goodData = readLidarRaw(uart, data);

	if (goodData)
		return ((float)((data[2] ) | (data[3] << 8))) / 64.0;
	return 0;
}

task fuckYouBuffer()
{
	while (true)
	{
		lidarRPM = getLidarRPM(UART1);
		abortTimeslice();
	}
}


/**
* Converts dist data packet to an integer representing distance in milimeters
* Data packet should contain 4 bytes:
* `byte 0 : <distance 7:0>`
* `byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8>`
* `byte 2 : <signal strength 7:0>`
* `byte 3 : <signal strength 15:8>`
*
* Returns -1 if dist data is not valid
*/
short readDistDataPacket(char* data)
{
	char *distData;
	distData = data;
	short dist = -1;
	if (distData[1] & 0x80) //returns -1 if the invalid data flag is 1.
		return dist;
	if (distData[1] & 0x40) //returns -1 if the invalid data flag is 1.
		return dist;
	dist = (distData[0] | ((distData[1] & 0x3F) << 8)); //shift second byte over and ignore the flags.
	return dist;
}

/**
* Reads the lidar and determines the distance reading for a specified angle.
* Angles range from 0 to 359 degrees in the counter-clockwise direction.
*
* Returns the distance at the specified angle. Returns -1 if angle is out of range or distance is invalid.
*/
short getLidarDistAtAngle(const TUARTs uart, short angle)
{
	short distPacket[5];
	short index;
	short dataPos;

	if (angle > 359)  //angle wraps around
	{
		angle = angle % 359;
	}

	if (angle < 0) //angle wraps around
	{
		int x = abs(angle) % 359;
		if (x == 0)
			x++;
		angle = 360 - x;
	}

	dataPos = angle % 4;
	index = angle - dataPos;

	readLidarUART(uart, distPacket);
	while (distPacket[0] != index)
	{
		readLidarUART(uart, distPacket);
	}

	return distPacket[dataPos + 1];
}

/**
* Converts the given angle and magnitude in polar form to cartesian.
*
* Data is x and y values. {x,y}
*
* Returns true
*/
bool convertToCartesian(short angle, short mag, float* xy)
{
	float x = 0;
	float y = 0;
	xy[0] = 0;
	xy[1] = 0;

	x = mag * cosDegrees(angle);
	y = mag * sinDegrees(angle);

	xy[0] = x;
	xy[1] = y;

	return true;
}

bool triangleFilter(short* data, triFilterData* balls, short size, short deltaD)
{
	short count = 0;
	short index = 0;

	for (int i = 0; i < size; i++)
	{
		balls[i].middleIndex = -1;
		balls[i].hasTwoLeft = false;
		balls[i].hasTwoRight = false;
	}

	for (int i = 0; i < 360; i++) //filter out data that wouldn't every be a ball. Allows and data that fits between a line and a sharp absolute value function.
	{

		if (index > 15 - 1)
			break;
		if (data[i] != -1) //filter out bad data readings from lidar data.
		{
			if (data[i - 1 >= 0 ? i - 1 : 359] > data[i] && data[i + 1 < 359 ? i + 1 : 0] >= data[i])
			{
				if (abs(data[i - 1 >= 0 ? i - 1 : 359] - data[i]) < deltaD && abs(data[i + 1 <= 359 ? i + 1 : 0] - data[i]) < deltaD)
				{
					if (data[i - 2 >= 0 ? i - 2 : 358] >  data[i - 1 >= 0 ? i - 1 : 359] && data[i + 2 <= 359 ? i + 2 : 1] >  data[i + 1 <= 359 ? i + 1 : 0]) //next two points are further than prev two points or invalid
					{
						if (abs(data[i + 2 <= 359 ? i + 2 : 1] - data[i + 1 <= 359 ? i + 1 : 0]) < deltaD - 25 && (abs(data[i - 2 >= 0 ? i - 2 : 358] - data[i - 1 >= 0 ? i - 1 : 359]) < deltaD - 25)) //five points are found to be within range!
						{
							balls[index].middleIndex = i;
							balls[index].hasTwoRight = true;
							balls[index].hasTwoLeft = true;
							index++;
							balls[index].middleIndex = i;
							balls[index].hasTwoRight = false;
							balls[index].hasTwoLeft = false;
							index++;
						}
						else //bad result
						{}
					}
					else if (data[i - 2 >= 0 ? i - 2 : 358] >  data[i - 1 >= 0 ? i - 1 : 359] && data[i + 2 <= 359 ? i + 2 : 1] == -1) //only point to left is seen, right point is invalid
					{
						if (abs(data[i - 2 >= 0 ? i - 2 : 358] - data[i - 1 >= 0 ? i - 1 : 359]) < deltaD - 25) //left point fits data and ball is described by four points
						{
							balls[index].middleIndex = i;
							balls[index].hasTwoRight = false;
							balls[index].hasTwoLeft = true;
							index++;
							balls[index].middleIndex = i;
							balls[index].hasTwoRight = false;
							balls[index].hasTwoLeft = false;
							index++;
						}
						else //bad result
						{}
					}
					else if (data[i + 2 <= 359 ? i + 2 : 1] > data[i + 1 <= 359 ? i + 1 : 0] && data[i - 2 >= 0 ? i - 2 : 358] == -1) //only point to right is seen, left point is invalid
					{
						if (abs(data[i + 2 <= 359 ? i + 2 : 1] - data[i + 1 <= 359 ? i + 1 : 0]) < deltaD - 25) //right point fits data and ball is described by four points
						{
							balls[index].middleIndex = i;
							balls[index].hasTwoRight = true;
							balls[index].hasTwoLeft = false;
							index++;
							balls[index].middleIndex = i;
							balls[index].hasTwoRight = false;
							balls[index].hasTwoLeft = false;
							index++;
						}
						else //bad result
						{}
					}
					else if (data[i - 2 >= 0 ? i - 2 : 358] == -1 && data[i + 2 <= 359 ? i + 2 : 1] == -1) //only three points have been found to be within profile of ball
					{
						balls[index].middleIndex = i;
						balls[index].hasTwoRight = false;
						balls[index].hasTwoLeft = false;
						index++;
					}
					else //bad result
					{}
				}
				else //left and right were > center point but were way bigger than the center point ie all three points don't really make an object
				{}
			}
			else //left and right points didn't fit characteristic curve of a semicircle with left and right > center point.
			{}
		}
		else
		{}
		count++;
		if (count == 30)
		{
			count = 0;
			sleep(1);
		}
	}
	//writeDebugStreamLine("Nick takes %d time",time1[T1]);
	short numOfFound = 0;
	bool foundData = false;
	for (int i = 0; i < size; i++)
	{
		if (balls[i].middleIndex != -1)
		{
			foundData = true;
			numOfFound++;
			//writeDebugStreamLine("THE FUCKING ANGLE %d", balls[i].middleIndex);
		}
	}


	//writeDebugStreamLine("Found %d Points that fit triangle filter Master ***********",numOfFound);
	return foundData;
}

enum TObjectType detectObjectsNear(short* data, struct triFilterData ball, short range, short* distAway)
{
	short leftBound;
	short rightBound;
	short pointsAround[20];
	short index;


	for (int i = 0; i < 20; i++)
	{
		pointsAround[i] = -1;  //null the array just to be safe
	}

	if (ball.middleIndex != -1)
	{
		if (ball.hasTwoRight) //determine the bounds of the data to check around
		{
			rightBound = ball.middleIndex <= 357 ? ball.middleIndex + 2 : ball.middleIndex + 2 - 360;
		} else {
			rightBound = ball.middleIndex <= 358 ? ball.middleIndex + 1 : ball.middleIndex + 1 - 360;
		}
		if (ball.hasTwoLeft)
		{
			leftBound = ball.middleIndex >= 2 ? ball.middleIndex - 2 : ball.middleIndex - 2  + 360;
		} else {
			leftBound = ball.middleIndex >= 1 ? ball.middleIndex - 1 : ball.middleIndex - 1  + 360;
		}

		index = 0;
		for (int i = 0; i < range; i++) //left bound of values
		{
			pointsAround[index] = leftBound - (range - i) >= 0 ? leftBound - (range - i) : 360 - (range - i);
			index++;
		}


		for (int i = 0; i < range; i++) //right bound of values
		{
			pointsAround[index] = rightBound + i + 1 <= 359 ? rightBound + i + 1 : rightBound + i + 1 - 360;
			index++;
		}//array is now in accending order to keep things simple!

		//lets check for a line! array is range*2 long!
		float x[10 * 2];
		float y[10 * 2];

		short index2 = 0;
		for (int i = 0; i < index; i++)
		{
			if (data[pointsAround[i]] > 0)
			{
				writeDebugStreamLine("Dist: **** %d  %d", data[pointsAround[i]], pointsAround[i]);
				float xy[2];
				convertToCartesian(pointsAround[i], data[pointsAround[i]], xy);
				x[index2] = xy[0];
				y[index2] = xy[1];
				index2++;
			}
		}
		float fit = calcCorrelationCoefficent(index, x, y);

		writeDebugStreamLine("correlation: %1.5f index: %d", fit, index);
		if (fit > 0.95)
		{
			short pointT = index / 2;
			*distAway = (short)sqrt(x[pointT] * x[pointT] + y[pointT] * y[pointT]);
			//writeDebugStreamLine("HERHE****** Dist: %d",*distAway);
			return wall;
		}


	}
	return undetermined;
}

/**
* Finds the balls on the field given lidar data. will find the middle angle values of a set of data points that has been
* identified as a balls. deltaD is the change is the max distance expected to see between data points on a ball.
* Balls that have only two data points will not be identified as a ball.
*
* Returns the angle of the closest ball found
*/
short findBalls(short* data, triFilterData* balls, short* posBalls, short size, float circleTolerance, float* certainty)
{
	short ball = -1;
	short min;
	ballData circle;
	short index = 0;

	for (int i = 0; i < 20; i++)
	{
		posBalls[i] = -1;
	}

	//writeDebugStreamLine("Doing Work Bitch...Balls don't find themselves ------> bitchin");
	for (int z = 0; z < size; z++)
	{
		min = 9999; //find smallest
		short minIndex = 0;
		short minAngle = -1;
		for (int i = 0; i < size; i++)
		{
			if (balls[i].middleIndex != -1) //if the element is a valid reading
			{
				if (data[balls[i].middleIndex] < min)
				{
					min = data[balls[i].middleIndex];
					minIndex = i;
				}
			}
			else
			{
			}
		}
		minAngle = balls[minIndex].middleIndex;
		balls[minIndex].middleIndex = -1; //"Remove" the min element in the array

		if (minAngle == -1)
			break;
		// calculate if the data points fit the circle fit!

		short leftAngle;
		short rightAngle;
		short rightOff;
		short leftOff;
		if (balls[minIndex].hasTwoRight)
		{
			rightAngle = minAngle <= 357 ? minAngle + 2 : minAngle + 2 - 360;
			rightOff = 2;
		} else {
			rightAngle = minAngle <= 358 ? minAngle + 1 : minAngle + 1 - 360;
			rightOff = 1;
		}
		if (balls[minIndex].hasTwoLeft)
		{
			leftAngle = minAngle >= 2 ? minAngle - 2 : minAngle - 2  + 360;
			leftOff = 2;
		} else {
			leftAngle = minAngle >= 1 ? minAngle - 1 : minAngle - 1  + 360;
			leftOff = 1;
		}

		circle.a = ((float)data[leftAngle]) * cosDegrees(leftOff) / 25.4;
		circle.b = ((float)data[leftAngle]) * sinDegrees(leftOff) / 25.4;
		circle.c = ((float)data[rightAngle]) * cosDegrees(-1 * rightOff) / 25.4;
		circle.d = ((float)data[rightAngle]) * sinDegrees(-1 * rightOff) / 25.4;
		circle.x = ((float)data[minAngle]) / 25.4;

		float a2 = circle.a * circle.a;
		float b2 = circle.b * circle.b;
		float c2 = circle.c * circle.c;
		float d2 = circle.b * circle.d;
		float bd2 = (circle.b - circle.d) * (circle.b - circle.d);
		float R2 = R * R;

		circle.k = (-circle.a * sqrt(-bd2 * (a2 - 2 * circle.a * circle.c + b2 - 2 * circle.b * circle.d
		                                     + c2 + d2) * (a2 - 2 * circle.a * circle.c + b2 - 2 * circle.b * circle.d + c2
		                                             + d2 - 4 * R2)) + circle.c * sqrt(-bd2 * (a2 - 2 * circle.a * circle.c + b2
		                                                     - 2 * circle.b * circle.d + c2 + d2) * (a2 - 2 * circle.a * circle.c + b2 - 2 * circle.b * circle.d
		                                                             + c2 + d2 - 4 * R2)) + a2 * b2 - a2 * d2 - 2 * circle.a * b2 * circle.c
		            + 2 * circle.a * circle.c * d2 + pow(circle.b, 4) - 2 * pow(circle.b, 3) * circle.d + b2 * c2 + 2 * circle.b * pow(circle.d, 3)
		            - c2 * d2 - pow(circle.d, 4)) / (2 * (circle.b - circle.d) * (a2 - 2 * circle.a * circle.c + b2 - 2 * circle.b * circle.d + c2 + d2));


		circle.h = (pow(circle.a, 3) + sqrt(-bd2 * (a2 - 2 * circle.a * circle.c + b2
		                                    - 2 * circle.b * circle.d + c2 + d2) * (a2 - 2 * circle.a * circle.c + b2
		                                            - 2 * circle.b * circle.d + c2 + d2 - 4 * R2)) - a2 * circle.c + circle.a * b2
		            - 2 * circle.a * circle.b * circle.d - circle.a * c2 + circle.a * d2 + b2 * circle.c - 2 * circle.b * circle.c * circle.d
		            + pow(circle.c, 3) + circle.c * d2) / (2 * (a2 - 2 * circle.a * circle.c + b2 - 2 * circle.b * circle.d + c2 + d2));

		//calculate the new r
		float newR = (circle.x - circle.h) * (circle.x - circle.h) + circle.k * circle.k;
		if (newR > 0)
		{
			newR = sqrt(newR); // take the pos value from the square root
			float errorR = calcPercentError(2.0, newR);
			//writeDebugStreamLine("newR: %1.5f Error: %1.5f Angle: %d h: %1.5f k%1.5f",newR,errorR,minAngle,circle.h,circle.k);
			if (abs(errorR) < circleTolerance)
			{
				ball = minAngle; //the angle to return for the center of the ball that was found


				/*triFilterData test;
				test.middleIndex = minAngle;
				writeDebugStreamLine("%d Dist: %d",test.middleIndex,data[test.middleIndex]);
				test.hasTwoRight = balls[minIndex].hasTwoRight;
				test.hasTwoLeft = balls[minIndex].hasTwoLeft;
				short *distAway;
				sleep(1);

				if(detectObjectsNear(data,test,5,distAway) == wall)
				writeDebugStreamLine("******WALL NEAR BALL************Dist Away: %d",distAway);
				*/

				//writeDebugStreamLine("Found closest ball Angle: %d Dist: mm:%d inch:%1.5f R: %1.5f",ball,data[ball],((float)data[ball])/25.4,newR);
				//writeDebugStreamLine("I'm %1.5f percent Certain its a ball!",(1.0-abs(errorR))*100.0);
				*certainty = (1 - abs(errorR)) * 100.0;
				posBalls[index] = ball;
				posBalls[index + 1] = data[ball];
				index += 2;
			}
		}
		else //shit!
		{}
		sleep(1); //give the cpu some time lol
	}

	return ball;
}

bool notBlacklisted(float xPos, float yPos, float theta, short mag, short angle)
{
	float x, y;
	short newMag, newAngle;
	float b = 2338.4;


	if (abs(xPos) > 1828.8) //sanity check the values being received by the cortex over uart
		return false; //just give up

	if (abs(yPos) > 1828.8)
		return false;

	if (abs(theta) > 180 )
		return false;

	calculateBallPos(xPos, yPos, theta, mag, angle, &x, &y, &newMag, &newAngle);

	writeDebugStreamLine("Y: %1.5f  X: %1.5f", y, x);
	if (b < (abs(x) + abs(y)))
		return false;
	if (abs(x) > 1750 || abs(y) > 1750)
		return false;
	return true;
}

/**
*	Finds the closest ball using three loops to help insure we don't find crap
*
*
*/
bool findClosestBall(Ball* theBall)
{
	triFilterData potBalls[15];
	short possibleBalls[20];
	short allPossibleBalls[60];
	short confirmedBalls[80];

	short count = 0;
	short totalBallCount = 0;
	short confirmedBallCount = 0;
	bool ballFound = false;
	float ballC;

	for (int i = 0; i < 60; i++) {
		allPossibleBalls[i] = -1; //why not
	}
	for (int i = 0; i < 80; i++)
	{
		confirmedBalls[i] = -1;
	}
	for (int i = 0; i < 20; i++) {
		possibleBalls[i] = -1;	//why not
	}

	for (int i = 0; i < 3; i++)
	{
		stopTask(fuckYouBuffer);
		getLidarData(UART1, dist);
		sleep(1);
		filterChassisLidar(dist);
		sleep(1);


		if (triangleFilter(dist, potBalls, 15, 50))
		{
			findBalls(dist, potBalls, possibleBalls, 15, 0.08, &ballC);

			for (int x = 0; x < 20; x += 2)
			{
				//writeDebugStreamLine("Possible Balls angle %d",possibleBalls[x]);
				if (possibleBalls[x] != -1)
				{
					allPossibleBalls[totalBallCount] = possibleBalls[x];
					allPossibleBalls[totalBallCount + 1] = possibleBalls[x + 1];
					totalBallCount += 2;
					//writeDebugStreamLine("Possible Ball # %d  Angle: %d Dist: %d",totalBallCount/2,possibleBalls[x],possibleBalls[x + 1]);
				}
			}

			//if(ball != -1 && ball != 0 && dist[ball != -1? ball: 0] != -1)
			//	goodDataToSend = true;
		}
		sleep(1);
		for (int i = 0; i < 20; i++)
		{
			possibleBalls[i] = -1;	//why not
		}
	}

	confirmedBallCount = 0;
	for (int i = 0; i < totalBallCount - 2; i += 2)
	{
		for (int x = i + 2; x < totalBallCount - 1; x += 2)
		{
			//writeDebugStreamLine("i: %d x: %d Angle1: %d Angle2: %d Diff: %d",i,x,allPossibleBalls[i],allPossibleBalls[x],abs(allPossibleBalls[i] - allPossibleBalls[x]));
			if (abs(allPossibleBalls[i] - allPossibleBalls[x]) < 3)
			{
				//writeDebugStreamLine("confirmed %d  i %d",confirmedBallCount,i);
				confirmedBalls[confirmedBallCount] = allPossibleBalls[i];
				confirmedBalls[confirmedBallCount + 1] = allPossibleBalls[i + 1];
				allPossibleBalls[i] = -1;
				confirmedBallCount += 2;
			}
		}
	}
	sleep(1);
	writeDebugStreamLine("Found legit confirmed %d", confirmedBallCount / 2);

	short minDist = 9999;
	short minAngle = 0;
	for (int i = 0; i < 20; i += 2)
	{
		if (confirmedBalls[i] > 0)
		{
			writeDebugStreamLine("THE GOOD ONESS: %d Dist: %d", confirmedBalls[i], confirmedBalls[i + 1]);
			if (confirmedBalls[i + 1] < minDist)
			{
				if (notBlacklisted(xPos.f, yPos.f, theta.f, confirmedBalls[i + 1], confirmedBalls[i]))
				{
					float x, y;
					short newMag, newAngle;

					//writeDebugStreamLine("Saving Angle: %d Mag: %d", minAngle, minDist);
					ballFound = true;
					calculateBallPos(xPos.f, yPos.f, theta.f, confirmedBalls[i + 1], confirmedBalls[i], &x, &y, &newMag, &newAngle);
					minDist = newMag;
					minAngle = newAngle;
				}
				else {
					writeDebugStreamLine("You've been denied!");
				}
			}
		}
	}

	sleep(1);
	writeDebugStreamLine("FOUND BALL: %d,   Angle: %d   Mag: %d", ballFound, minAngle, minDist);
	if (ballFound)
	{
		//writeDebugStreamLine("Sending Angle: %d Mag: %d", minAngle, minDist);
		theBall->angle = minAngle;
		theBall->mag = minDist;
	}
	else
	{
		theBall->angle = 0;
		theBall->mag = 0;
	}

	startTask(fuckYouBuffer);
	return ballFound;
}

/**
* Filters out data that is from the chassis blocking the line of slight of the lidar
*
* Returns true if all data is filtered correctly
*/
bool filterChassisLidar(short* data)				//needs to be updated to hardcode ignored angles
{


	for (int i = 0; i < 360; i++)
	{
		if (data[i] < 160)
			data[i] = -1;
		if (i >= 0 && i <= 24)
			data[i] = -1;
		if (i > 180 && i <= 265)
			data[i] = -1;
		if (i > 265 && i <= 300)
		{
			if (data[i] > 1500 || data[i] < 650)
				data[i] = -1;
		}
		if (i > 300 && i <= 359)
			data[i] = -1;

	}
	return true;
}

/**
* Checks if a ball is within the field given the current location from the odometry
* uses xPos yPos and theta.
*/
bool checkBallWithinField(short angle, short mag)
{
	float x = xPos.f; //convert position information into something easier to use.
	float y = yPos.f;
	float alpha = theta.f;

	if (abs(x) > 1828.8) //sanity check the values being received by the cortex over uart
		return false; //just give up

	if (abs(y) > 1828.8)
		return false;

	if (abs(alpha) > 180)
		return false;



	return true;
}



/**
* Calculates the pearson correlation coefficent, given number of samples, and x,y values in a array of floats.
*
* Returns the correlation of all of the sample points.
*/
float calcCorrelationCoefficent(short numSamples, float* x, float* y)
{
	float *xSamples;
	float *ySamples;

	float xMean = 0;
	float yMean = 0;
	float xDev = 0;
	float yDev = 0;
	float correlation = 0;

	xSamples = x;
	ySamples = y;

	if (numSamples < 2)
		return 0;

	for (int i = 0; i < numSamples; i++)
	{
		if (xSamples[i] == -1)
			return 0;
	}

	for (int i = 0; i < numSamples; i++)
	{
		if (ySamples[i] == -1)
			return 0;
	}

	for (int i = 0; i < numSamples; i++)
	{
		xMean = xMean + xSamples[i];
	}
	xMean = xMean / ((float)numSamples);

	for (int i = 0; i < numSamples; i++)
	{
		yMean = yMean + ySamples[i];
	}
	yMean = yMean / ((float)numSamples);

	float xTemp = 0;
	for (int i = 0; i < numSamples; i++)
	{
		xTemp = xTemp + (pow((xSamples[i] - xMean), 2));
	}

	xDev = sqrt(xTemp / ((float)(numSamples - 1)));

	float yTemp = 0;
	for (int i = 0; i < numSamples; i++)
	{
		yTemp = yTemp + (pow((ySamples[i] - yMean), 2));
	}

	yDev = sqrt(yTemp / ((float)(numSamples - 1)));

	float xyTemp = 0;
	for (int i = 0; i < numSamples; i++)
	{
		xyTemp = xyTemp + ((xSamples[i] - xMean) * (ySamples[i] - yMean));
	}

	correlation = (xyTemp) / (((float)(numSamples - 1)) * xDev * yDev);

	return abs(correlation);
}

/**
* Calculates a linear regression given a set of x and y points.
* Linear equation is in form y = mx + b.
*
* mb is a float array containing the m and b. ex) y = mx + b : {a,b}
*
* Returns true if the given data is valid
*/
bool calcLinearFit(short num, float* x, float* y, float* mb)
{
	mb[0] = 0;
	mb[1] = 0;

	float numSamples = (float)num;
	float xSum = 0;
	float ySum = 0;
	float xySum = 0;
	float xSqrdSum = 0;
	float ySqrdSum = 0;

	float m = 0;
	float b = 0;

	float *xSamples;
	float *ySamples;

	xSamples = x;
	ySamples = y;

	if (numSamples < 2)
		return false;

	for (int i = 0; i < numSamples; i++)
	{
		if (xSamples[i] == -1)
			return false;
	}

	for (int i = 0; i < numSamples; i++)
	{
		if (ySamples[i] == -1)
			return false;
	}

	for (int i = 0; i < numSamples; i++)
	{
		xSum = xSum + xSamples[i];
	}

	for (int i = 0; i < numSamples; i++)
	{
		ySum = ySum + ySamples[i];
	}

	for (int i = 0; i < numSamples; i++)
	{
		xySum = xySum + (xSamples[i] * ySamples[i]);
	}

	for (int i = 0; i < numSamples; i++)
	{
		xSqrdSum = xSqrdSum + pow(xSamples[i], 2);
	}

	for (int i = 0; i < numSamples; i++)
	{
		ySqrdSum = ySqrdSum + pow(ySamples[i], 2);
	}

	if (((numSamples * xSqrdSum) - (xSum * xSum)) == 0)
		return false;

	m = ((numSamples * xySum) - (xSum * ySum)) / ((numSamples * xSqrdSum) - (xSum * xSum));
	b = ((xSqrdSum * ySum) - (xSum * xySum)) / ((numSamples * xSqrdSum) - (xSum * xSum));

	mb[0] = m;
	mb[1] = b;

	return true;
}

/**
* Searches through the given lidar data to find a defined number of consecutive points that have the best linear fit
* num is the number of consecutive points to look for. threshold is the required correlation needed to be considered as a line.
* num can go up to 15 data points.
*
* Returns false if nothing if no data is within tolerance.   ///NOT DONE AT ALLLL DONT USE YET!!!!!!!!!!!!!!!!!!!!!!!!!!!!!***********************************
*/
bool findBestLine(short num, short* data, short* points, float threshold) //WIP
{
	float bestCorrelation = 0;
	float xy[2];
	short startAngle = -1;
	float xValues[15];
	float yValues[15];
	short bestIndex = 0;

	if (num > 15)
		return false;

	for (int i = 0; i < 360; i++)
	{
		if (i + num > 360)
			break;
		for (int x = 0; x < num; x++) //grab n number of data points and test for linearity
		{
			convertToCartesian(i, data[i], xy);
			xValues[x] = xy[0];
			yValues[x] = xy[1];
			i++;
		}
		float testCorrelation = calcCorrelationCoefficent(num, xValues, yValues);
		if (testCorrelation > bestCorrelation)
		{
			bestCorrelation =  testCorrelation;
			bestIndex = i - num;
		}

	}
	return true;
}

/**
* Determines points that are on a given functions within a set threshold.
* Lidar data is expected to be a full 360 degree reading of points.
* Points subject to evaluation are testing by calculating pearson correlation with n orig points and one additional point.
*
* origPoints are n (up to 10) points that have been determined to be linear and to tests other points against for linearaity.
* origPoints are formatted as follows {start angle, dist1, dist2, dist3,..., distn}
*
* Data is an array of angles that were found to be on the given line. {angle1,angle2,...,angle n} -1 in place of missing data.
*
* Returns false if given data is not within range of initial requirements.
*/
bool findIncludedPoints(short* data, short num, short* orig, float threshold, short* fitData)
{
	short *lidarData;
	short *origPoints;
	float xy[2];
	float origmb[2];
	float newmb[2];

	float xOrig[10];
	float yOrig[10];


	if (sizeof(orig) / 2 > 11) //dont use this brahhhh sizeof #hurtyourfeelsrealgood
		return false;

	short startAngle = 0;
	float origCorrelation = 0;

	lidarData = data;
	origPoints = orig;
	startAngle = orig[0];

	if (startAngle == -1)
		return false;

	for (int i = 0; i < num; i++) //calculate all the x and y values of the orig data set.
	{
		convertToCartesian(startAngle + i, origPoints[1 + i], xy);
		xOrig[i] = xy[0];
		yOrig[i] = xy[1];
		//writeDebugStreamLine("X= %1.5f  Y= %1.5f",xy[0],xy[1]);
	}

	origCorrelation = calcCorrelationCoefficent(num, xOrig, yOrig); //calculate the orignal correlation for reference
	calcLinearFit(num, xOrig, yOrig, origmb);

	//writeDebugStreamLine("R= %1.5f", origCorrelation);
	for (int i = 0; i < 360; i++)
	{
		fitData[i] = -1;
	}

	if (origCorrelation < 0.95)
		return false;
	if (origmb[0] == 0 || origmb[1] == 0)
		return false;

	float xValues[11];
	float yValues[11];

	for (int i = 0; i < num; i++) //calculate all the x and y values of the orig data set.
	{
		convertToCartesian(startAngle + i, origPoints[1 + i], xy);
		xValues[i] = xy[0];
		yValues[i] = xy[1];
	}
	writeDebugStreamLine("Working.....");
	float newCorrelation = 0;
	for (int i = 0; i < 360; i++) //check individual points with orig data for correlation.
	{
		//writeDebugStreamLine("%d",lidarData[i]);
		if (lidarData[i] != -1)
		{
			convertToCartesian(i, lidarData[i], xy);
			xValues[num] = xy[0];
			yValues[num] = xy[1];

			//writeDebugStreamLine("X= %1.5f  Y= %1.5f",xValues[num],yValues[num]);

			newCorrelation = calcCorrelationCoefficent(num + 1, xValues, yValues); //calculate new correlation with four orignal points and one new point.
			calcLinearFit(num + 1, xValues, yValues, newmb);
			//writeDebugStreamLine("%1.5f",newCorrelation);

			float correlationError = calcPercentError(origCorrelation, newCorrelation);

			float slopeError = calcPercentError(origmb[0], newmb[0]);

			//writeDebugStreamLine("Orig = %1.5f  New = %1.5f  Error = %1.5f",origmb[0],newmb[0],slopeError);
			if (correlationError < threshold && slopeError < threshold * 2) //new point is found to be within the defined percent error of linearaity.
			{
				fitData[i] = i; // just give the angles of the points that fit
				//writeDebugStreamLine("%d : Y=%1.5fx+%1.5f R=%1.5f cError: %1.5f mError = %1.5f",i,newmb[0],newmb[1], newCorrelation,correlationError, slopeError);
			}
		}
	}
	return true;
}

/**
* Calculates the width of a line given an array of points that are assumed to be corresponding to a linear function.
* Data is expected in form: {angle1,dist1,angle2,dist2,...,angle n,dist n}.
* Points are converted to cartisian form and line boundries are used to determine the length of a line with distance formula.
* length = sqrt((x2-x1)^2 + (y2-y1)^2). where x1,x2,y1,y2 are bouding points.
* Bounding points are found by determining the points at which extreme values of y are found.
*
* Data is expected as {angle1,angle2,...,angle n}
*
* Returns the width of the wall in milimeters.
*/
float calcLineWidth(short* data, short* fit)
{
	short *fitData;
	short *lidarData;
	fitData = fit;
	lidarData = data;
	float yMin = 2097200.0;
	float yMax = -2097200.0;
	float xMin = 2097200.0;
	float xMax = -2097200.0;
	float width = 0;
	short minAngle;
	short maxAngle;

	float xy[2];
	for (int i = 0; i < 360; i++)
	{
		if (fitData[i] != -1)
		{
			convertToCartesian(fitData[i], data[fitData[i]], xy);
			if (xy[1] > yMax)
			{
				yMax = xy[0];
				maxAngle = fitData[i];
			}
			if (xy[1] < yMin)
			{
				yMin = xy[1];
				minAngle = fitData[i];
			}
		}
	}

	float xyMin[2];
	float xyMax[2];

	convertToCartesian(maxAngle, data[maxAngle], xyMax);
	convertToCartesian(minAngle, data[minAngle], xyMin);

	xMax = xyMax[0];
	yMax = xyMax[1];
	xMin = xyMin[0];
	yMin = xyMin[1];

	width = sqrt(pow((xMax - xMin), 2) + pow((yMax - yMin), 2));

	return width;
}

/**
* Calculates the percent error given a expected and mesured value. 85% error --> 0.85000
*
* Returns the percent error of the measured result.
*/
float calcPercentError(float expected, float measured)
{
	float percent;
	if (expected > 1 && measured < 1)
		return 1;
	if (expected < 1 && measured > 1)
		return 1;
	if (expected == 0)
		return 1;
	expected = abs(expected);
	measured = abs(measured);
	percent = abs((expected - measured) / expected);
	return percent;
}
