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
Pixy2SPI_SS lowPixy;

// The x values of the center of blocks
int leftX;
int rightX;

// The width of the blocks
int leftWidth;
int rightWidth;

// Area of the blocks
float aL;
float aR;

// Position of the edges of each block
int xLOne;
int xROne;
int xLTwo;
int xRTwo;

// Center of the two blocks
int centerPoint;

// The x value of the center of the lowPixy's FOV
const int absoluteCenter = 158;

// Degrees per x pixel, used to convert 'distance to center' to 'angle to center'
const double xPixToDeg = 60.0/316.0;

// The center adjusted based on compared area
int adjAbsCenter;

// The pixel distance from the target to the center
double distToCenter;

// The final value for the degrees to target passed to the rio
double angleToCenter;

//  Value of pi for calculations
double pi = 3.1415926535;

//  Command char recieved from the rio, where 2 is degToTarget, 1 is distToTarget, 3 is angleToCenter, and 4 is lowPosition
char incCommand = '0';

//	Commands we are comparing incCommand to
//  These command constants must be the same as the constants in RobotMap
const char GET_DEG_TO_TARGET = '2';
const char GET_DIST_TO_TARGET = '1';
const char GET_ANGLE_TO_CENTER = '3';
const char GET_LOW_POSITION = '4';
const char GET_AVG_AREA = '5';

int callCounter = 0;
boolean callToggle = false;

boolean badFlag = false;

//  This is the return for the position according to the lowPixy, where 1 is left, 2 is center, and 3 is right. -1 is no blocks
int lowPosition = -1;

//  Convert degrees to radians
double degToRad(double degInput)
{
	double radOutput = degInput * (pi / 180);
	return radOutput;
}

void calcDistToCenterLow()
{
  if (lowPixy.ccc.getBlocks() == 2) {
    badFlag = false;
  }
  else {
    badFlag = true;
  }
  
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

    adjAbsCenter = absoluteCenter;

	xLOne = leftX + (leftWidth / 2);
	xROne = rightX + (rightWidth / 2);
	xLTwo = leftX - (leftWidth / 2);
	xRTwo = rightX - (rightWidth / 2);
	centerPoint = ((xLTwo - xROne) / 2) + xROne;
	distToCenter = absoluteCenter - centerPoint;
	angleToCenter = distToCenter * xPixToDeg;
}

void setup()
{
	// put your setup code here, to run once:
	Serial.begin(9600);

	//  Initializes the Pixy
	//  Hexadecimal values passed in correspond to address set on the pixy
	lowPixy.init(0x53);
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
	if (command == GET_ANGLE_TO_CENTER)
	{
    	if (badFlag) {
      		Serial.println("NO");
    	}
    	else {
			Serial.println(angleToCenter);
    	}
	}
	else if (command == GET_LOW_POSITION)
	{
		Serial.println(lowPosition);
	}
}

void loop()
{
	//  Gets data from the lowPixy
	lowPixy.ccc.getBlocks(true, 255, 2); 
 	calcDistToCenterLow();
	
	//  Calculates return values for the lowPixy
	if (lowPixy.ccc.numBlocks)
	{
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
