/**********************************************************************************************
* Cortex to Cortex communication protocol
*
*
* by Nick Sorensen
* Created 4/10/2016
*
**********************************************************************************************/


enum T_PacketType {
	T_ball = 0xFB,
	T_pos = 0xFC,
	T_debug = 0xFD,
	T_GoodRecieved = 0xF
};

short debugShort1 = 0;
short debugShort2 = 0;
short debugShort3 = 0;
short debugShort4 = 0;
short debugShort5 = 0;
short debugShort6 = 0;
short debugShort7 = 0;
short debugShort8 = 0;
float2bytes debugFloat1;
float2bytes debugFloat2;


bool nothingToSend = true;
bool goodDataToSend = false;
short angleToSend = 0;
short magToSend = 0;
char packetRequestType = 0;

float2bytes xPos;
float2bytes yPos;
float2bytes theta;

task cortexCommunication()
{
	debugFloat1.f = 0.0;
	debugFloat2.f = 0.0;
	char packet[14];
	char data;

	long lidarRequestTimeout;
	long packetTime;

	nothingToSend = true;
	goodDataToSend = false;
	short countDebug = 0;

	short count = 0;

	while (true)
	{
		//writeDebugStreamLine("I'M GOING TO SEND MASTER Angle: %d Dist: %d ******", angleToSend, magToSend);

		if (nothingToSend)
		{
			for (int i = 0; i < 14; i++)
			{
				packet[i] = 0;
			}

			packetTime = nSysTime;
			data = getChar(CORTEXA); //at this point all data has already been calculated and wait to send per request

			while (data != 0xFA && count < 15)
			{
				data = getChar(CORTEXA);
				count++;
			}

			count = 0;
			if (data == 0xFA)
			{
				sleep(2);
				packet[0] = 0xFA;
				data = getChar(CORTEXA);
				if (data != 0xFF && data != 0xFA && packet[0] == 0xFA)
				{
					packet[1] = data;
					count = 2;
				}
				sleep(1);
				while (count != 14)
				{
					data = getChar(CORTEXA);
					if (data != 0xFF && data != 0xFB)
					{
						packet[count] = data;
						count++;
					}
				}
				if ((nSysTime - packetTime > 250))
					writeDebugStreamLine("timed");
				if (packet[0] == 0xFA && packet[1] != 0xFF && count == 14)
				{

					packetRequestType = packet[1];

					lidarRequestTimeout = nSysTime;
					writeDebugStreamLine("MASTER WANTS The D **********************");

					xPos.b[0] = packet[2];
					xPos.b[1] = packet[3];
					xPos.b[2] = packet[4];
					xPos.b[3] = packet[5];

					yPos.b[0] = packet[6];
					yPos.b[1] = packet[7];
					yPos.b[2] = packet[8];
					yPos.b[3] = packet[9];

					theta.b[0] = packet[10];
					theta.b[1] = packet[11];
					theta.b[2] = packet[12];
					theta.b[3] = packet[13];
					nothingToSend = false;
				}
			}
		}

		if ((nSysTime - lidarRequestTimeout > lidarTimeout) && !goodDataToSend && !nothingToSend)
		{
			goodDataToSend = true;
			angleToSend = 0;
			magToSend = 0;
			ballCertainty.f = 0.0;
			writeDebugStreamLine("Timed OUT BITCH");
		}

		if (!nothingToSend && goodDataToSend)
		{
			switch (packet[1])
			{

			case T_ball:  //ball request

				char packetBall[10]; //packet will be 10 bytes long

				packetBall[0] = 0xFA; //start
				packetBall[1] = 0xFB; //type of response
				angleToSend = angleToSend + 360;
				packetBall[2] = (angleToSend >> 8) & 0xFF;
				packetBall[3] = angleToSend & 0xFF;

				packetBall[4] = (magToSend >> 8) & 0xFF;
				packetBall[5] = magToSend & 0xFF;

				packetBall[6] = ballCertainty.b[0];
				packetBall[7] = ballCertainty.b[1];
				packetBall[8] = ballCertainty.b[2];
				packetBall[9] = ballCertainty.b[3];

				for (int i = 0; i < 10; i++)
				{
					sendChar(CORTEXA, packetBall[i]);
					writeDebugStreamLine("%X", packetBall[i]);
				}
				nothingToSend = true;
				goodDataToSend = false;
				packetRequestType = 0;
				writeDebugStreamLine("Done Sending*******  ANGLE: %d  Dist: %d", angleToSend, magToSend);
				break;

			case T_pos: //position request

				char packetPos[15];

				packetPos[0] = 0xFA;
				packetPos[1] = 0xFC;

				packetPos[2] = xPos.b[0];
				packetPos[3] = xPos.b[1];
				packetPos[4] = xPos.b[2];
				packetPos[5] = xPos.b[3];

				packetPos[6] = yPos.b[0];
				packetPos[7] = yPos.b[1];
				packetPos[8] = yPos.b[2];
				packetPos[9] = yPos.b[3];

				packetPos[10] = theta.b[0];
				packetPos[11] = theta.b[1];
				packetPos[12] = theta.b[2];
				packetPos[13] = theta.b[3];

				packetPos[14] = 0xFE; // needs to be define based off of how the position of the robot was calculated

				for (int i = 0; i < 15; i++)
				{
					sendChar(CORTEXA, packetPos[i]);
				}
				nothingToSend = true;
				goodDataToSend = false;
				packetRequestType = 0;
				break;

			default:
				//nothing to send! whoops
				//we didn't receive the packet right!
				nothingToSend = true;
				goodDataToSend = false;
				packetRequestType = 0;
				break;
			}
		}

		sleep(1);
		if (DEBUGTOCORTEXA) //holy shit hella debug
		{
			count = 0;
			char debugPacket[26];
			debugPacket[0] = 0xFA;
			debugPacket[1] = T_debug;

			debugPacket[2] = (debugShort1 >> 8) & 0xFF;
			debugPacket[3] = debugShort1 & 0xFF;

			debugPacket[4] = (debugShort2 >> 8) & 0xFF;
			debugPacket[5] = debugShort2 & 0xFF;

			debugPacket[6] = (debugShort3 >> 8) & 0xFF;
			debugPacket[7] = debugShort3 & 0xFF;

			debugPacket[8] = (debugShort4 >> 8) & 0xFF;
			debugPacket[9] = debugShort4 & 0xFF;

			debugPacket[10] = (debugShort5 >> 8) & 0xFF;
			debugPacket[11] = debugShort5 & 0xFF;

			debugPacket[12] = (debugShort6 >> 8) & 0xFF;
			debugPacket[13] = debugShort6 & 0xFF;

			debugPacket[14] = (debugShort7 >> 8) & 0xFF;
			debugPacket[15] = debugShort7 & 0xFF;

			debugPacket[16] = (debugShort8 >> 8) & 0xFF;
			debugPacket[17] = debugShort8 & 0xFF;

			debugPacket[18] = debugFloat1.b[0];
			debugPacket[19] = debugFloat1.b[1];
			debugPacket[20] = debugFloat1.b[2];
			debugPacket[21] = debugFloat1.b[3];

			debugPacket[22] = debugFloat2.b[0];
			debugPacket[23] = debugFloat2.b[1];
			debugPacket[24] = debugFloat2.b[2];
			debugPacket[25] = debugFloat2.b[3];

			for (int i = 0; i < 26; i++)
			{
				sendChar(CORTEXA, debugPacket[i]);
			}
		}
		countDebug++;

	}
	sleep(45);
}

/*
*	Converts angles from 180 through -179 to 0-360
*
*
*/
void sendBallToPython(short num, short angle, short mag)
{
	writeDebugStreamLine("Sending Angle: %d",angle);
	angle += 180;

	char packet[7];
	packet[0] = 0xFA;
	packet[1] = 0xFD;
	packet[2] = num;
	packet[3] = angle >> 8 & 0xFF;
	packet[4] = angle & 0xFF;
	packet[5] = mag >> 8 & 0xFF;
	packet[6] = mag & 0xFF;

	for (int i = 0; i < 7; i++)
	{
		sendChar(CORTEXA, packet[i]);
		writeDebugStreamLine("%X",packet[i]);
	}
}
