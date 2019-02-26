
#include <Pixy2.h>
#include <Pixy2Line.h>
#include <Pixy2UART.h>

#include <Pixy2I2C.h>
#include <Pixy2SPI_SS.h>

/**
 * Communication code from the pixys to the arduino and from the duino to the rio
 * The highPixy is the pixy placed high on the robot, which will be used for pathing to the white line
 * The lowPixy is the pixy placed low on the robot used for shorter range tracking of the reflective tape
 */

//  Declares the Pixys
//Pixy2I2C highPixy;
Pixy2SPI_SS lowPixy;

//  Declares variables for the low pixy
int leftX;
int rightX;
int leftWidth;
int rightWidth;
float aL;
float aR;
float arrL[3] = {0,0,0};
float arrR[3] = {0,0,0};
float avgL;
float avgR;
float dA;
float maxDa = 0.15;
int xLOne;
int xROne;
int xLTwo;
int xRTwo;
int centerPoint;
int absoluteCenter = 158;
int adjAbsCenter;
int distToCenter;
double angleToCenter;
double tempDistCenter;
double selectedDistCenter = 39;
double targetIndex;

//  Value of pi for calculations
double pi = 3.1415926535;

//  Height and angle of the pixy (in)
double cameraHeight = 47.375;
double cameraAngle = 30;

//  Coordinates of the origin point (p)
double originX = 39;
double originY = 51;

//  Constants for the pixy
double thirtyDegInRad = ((pi / 180) * 30);
double degPerVertPix = (40.0 / 51.0);
double degPerHorizPix = (60.0 / 78.0);

//  Values for calculating position later [RECOMMENT]
//  Degrees from vertical to the end of the blindspot
double blindspotDeg = (cameraAngle - 20);

//  Pixel distance from the base to the target point
double xDist;
double yDist;

//  Diagonal distance from the camera to the target point
double hypToCamera;
double yDistDeg;
double xDistDeg;

//  Y Distance in inches from the robot to the target
double distRobotToTarget;

double xDistRobotToTarget;

//  Width of the projection at the given y value
double xWidth;

//  Inches per pixel in the x direction
double xInPerPix;

//  Distance from midline to the target on the floor in Inches
double xDistIn;

//  Degrees from the base of the robot to the target point
double degToTarget;
double distToTarget;

//  Command char recieved from the rio, where 2 is degToTarget, 1 is distToTarget, 3 is angleToCenter, and 4 is lowPosition
char incCommand = '0';

//	Commands we are comparing incCommand to
const char GET_DEG_TO_TARGET = '2';
const char GET_DIST_TO_TARGET = '1';
const char GET_ANGLE_TO_CENTER = '3';
const char GET_LOW_POSITION = '4';
const char GET_AVG_AREA = '5';

//  This is the return for the position according to the lowPixy, where 1 is left, 2 is center, and 3 is right. -1 is no blocks
int lowPosition = -1;

//  Convert degrees to radians
double degToRad(double degInput)
{
	double radOutput = degInput * (pi / 180);
	return radOutput;
}

////  Calculate the inches per x pixel on a given y value in order to calulate angle to the robot
//void calcInPerPix(double height, double angle, double tailX, double tailY)
//{
//	xDist = (tailX - originX);
//	yDist = (originY - tailY);
//	yDistDeg = (blindspotDeg + (yDist * degPerVertPix));
//	distRobotToTarget = 11+(height * tan(degToRad(yDistDeg)));
//	hypToCamera = sqrt(sq(height) + sq(distRobotToTarget));
//  xDistDeg = (xDist * degPerHorizPix);
//  xDistRobotToTarget = hypToCamera * tan(degToRad(abs(xDistDeg)));
//}

void calcDistToCenterLow()
{
	if (lowPixy.ccc.blocks[0].m_x < lowPixy.ccc.blocks[1].m_x)
	{
		leftX = lowPixy.ccc.blocks[0].m_x;
		rightX = lowPixy.ccc.blocks[1].m_x;
		leftWidth = lowPixy.ccc.blocks[0].m_width;
		rightWidth = lowPixy.ccc.blocks[1].m_width;
    aL = lowPixy.ccc.blocks[0].m_width * lowPixy.ccc.blocks[0].m_height;
    aR = lowPixy.ccc.blocks[1].m_width * lowPixy.ccc.blocks[1].m_height;
	}
	else if (lowPixy.ccc.blocks[0].m_x > lowPixy.ccc.blocks[1].m_x)
	{
		leftX = lowPixy.ccc.blocks[1].m_x;
		rightX = lowPixy.ccc.blocks[0].m_x;
		leftWidth = lowPixy.ccc.blocks[1].m_width;
		rightWidth = lowPixy.ccc.blocks[0].m_width;
    aL = lowPixy.ccc.blocks[1].m_width * lowPixy.ccc.blocks[1].m_height;
    aR = lowPixy.ccc.blocks[0].m_width * lowPixy.ccc.blocks[0].m_height;
	}
  arrL[2] = arrL[1];
  arrL[1] = arrL[0];
  arrL[0] = aL;
  int iL;
  float lSum = 0;
  for(iL = 0; iL < 3; iL++) {
    lSum += arrL[iL];
  }
  avgL = lSum / 3;
  
  arrR[2] = arrR[1];
  arrR[1] = arrR[0];
  arrR[0] = aR;
  int iR;
  float rSum = 0;
  for(iR = 0; iR < 3; iR++) {
    rSum += arrR[iR];
  }
  avgR = rSum / 3;
  
  dA = ( 1 - ( avgR / avgL ) );

  if ((abs(dA)) > maxDa)
  {
    adjAbsCenter = (int)(10 * dA) + absoluteCenter;
  }
  else
  {
    adjAbsCenter = absoluteCenter;
  }

	xLOne = leftX - (leftWidth / 2);
	xROne = rightX + (leftWidth / 2);
	xLTwo = leftX - (rightWidth / 2);
	xRTwo = rightX + (rightWidth / 2);
	centerPoint = ((xLTwo - xROne) / 2) + xROne;
	distToCenter = adjAbsCenter - centerPoint;
	angleToCenter = distToCenter * 0.189873;
}

void setup()
{
	// put your setup code here, to run once:
	Serial.begin(9600);

	//  Initializes the Pixy
	//  Hexadecimal values passed in correspond to address set on the pixy
//	highPixy.init(0x54);
	lowPixy.init(0x53);
//	highPixy.changeProg("line");
//  highPixy.line.setMode(LINE_MODE_MANUAL_SELECT_VECTOR);
}

//  Reads command off of the wire and converts it to a usable char
void receiveCommand()
{
	incCommand = (char)Serial.read();
}

//  Flushes excess data off after we read the command
void serialFlush()
{
	while (Serial.available() > 0)
	{
		char t = Serial.read();
	}
}

//  Writes data down the wire based on command passed in
void sendData(char command)
{
//	if (command == GET_DEG_TO_TARGET)
//	{
//		if (String(degToTarget) == ("-34.36"))
//		{
//			degToTarget = -180;
//		}
//		Serial.println(degToTarget);
//	}
//	else if (command == GET_DIST_TO_TARGET)
//	{
//		Serial.println(distToTarget);
//	}
	if (command == GET_ANGLE_TO_CENTER)
	{
		Serial.println(angleToCenter);
	}
	else if (command == GET_LOW_POSITION)
	{
		Serial.println(lowPosition);
    //Serial.println(adjAbsCenter);
   // Serial.print("Left Block Area: ");
    //Serial.print(aL);
    //Serial.print("\t");
    //Serial.print("Right Block Area: ");
    //Serial.println(aR);
	}
 else if (command == GET_AVG_AREA) {
  double avgArea = (avgL + avgR) / 2;
  Serial.println(avgArea);
 }
}

void loop()
{
//    Serial.print("dA\t");
  //  Serial.println(dA);
//  int indexindex = 0;
//  targetIndex = -500;
//	//  Gets data from the highPixy
//	highPixy.line.getAllFeatures(1, false);
// 
//  selectedDistCenter = 39;
//  tempDistCenter = 39;
//  
//  for(indexindex = 0; indexindex < highPixy.line.numVectors; indexindex++) 
//  {
//    tempDistCenter = abs(originX - highPixy.line.vectors[indexindex].m_x0);
//    if (tempDistCenter < selectedDistCenter)
//    {
//      selectedDistCenter = tempDistCenter;
//      targetIndex = highPixy.line.vectors[indexindex].m_index;
//    }
//  }
//  highPixy.line.setVector(targetIndex);
//  highPixy.line.getMainFeatures();

	//  Calculates return values for the highPixy
//	calcInPerPix(cameraHeight, cameraAngle, highPixy.line.vectors->m_x0, highPixy.line.vectors->m_y0);
//	degToTarget = (atan(xDistRobotToTarget / distRobotToTarget) * (180 / pi)) * (abs(xDist)/xDist);
//	distToTarget = sqrt(sq(distRobotToTarget) + sq(xDistRobotToTarget));

	//  Gets data from the lowPixy
	lowPixy.ccc.getBlocks(true, 255, 2);

	//  Calculates return values for the lowPixy
	if (lowPixy.ccc.numBlocks)
	{
		calcDistToCenterLow();

		if (absoluteCenter - centerPoint < 2)
		{
			lowPosition = 1;
		}
		else if (absoluteCenter - centerPoint > 2)
		{
			lowPosition = 3;
		}
		else
		{
			lowPosition = 2;
		}
	}
	else
	{
		lowPosition = -1;
	}

	//  Runs the communication code if a command is available
	if (Serial.available() > 0)
	{
		receiveCommand();
		serialFlush();
		sendData(incCommand);
		Serial.flush();
		incCommand = 0;
	}
}
