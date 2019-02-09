#include <Pixy2I2C.h>
#include <Pixy2SPI_SS.h>


/**
 * Communication code from the pixys to the arduino and from the duino to the rio
 * The highPixy is the pixy placed high on the robot, which will be used for pathing to the white line
 * The lowPixy is the pixy placed low on the robot used for shorter range tracking of the reflective tape
 */
 
//  Declares the Pixys
Pixy2SPI_SS highPixy;
Pixy2I2C lowPixy;

//  Declares variables for the low pixy
int leftX;
int rightX;
int leftWidth;
int rightWidth;
int xLOne;
int xROne;
int xLTwo;
int xRTwo;
int centerPoint;
int absoluteCenter = 158;
int distToCenter;
double angleToCenter;

//  Value of pi for calculations
double pi = 3.1415926535;

//  Height and angle of the pixy (in)
double cameraHeight = 46;
double cameraAngle = 25;

//  Coordinates of the origin point (p)
double originX = 39;
double originY = 51;

//  Constants for the pixy
double thirtyDegInRad = ((pi/180)*30);
double degPerVertPix = (40/51);

//  Values for calculating position later [RECOMMENT]
//  Degrees from vertical to the end of the blindspot
double blindspotDeg = (cameraAngle - 20);

//  Pixel distance from the base to the target point
double xDist; double yDist;

//  Diagonal distance from the camera to the target point
double hypToCamera;
double yDistDeg;

//  Y Distance in inches from the robot to the target
double distRobotToTarget;

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

//  This is the return for the position according to the lowPixy, where 1 is left, 2 is center, and 3 is right. -1 is no blocks
int lowPosition = -1;

//  Convert degrees to radians
double degToRad (double degInput) {
  double radOutput = degInput * (pi/180);
  return radOutput;
}

//  Calculate the inches per x pixel on a given y value in order to calulate angle to the robot
void calcInPerPix (double height, double angle, double tailX, double tailY) {
  xDist = (tailX - originX);
  yDist = (originY - tailY);
  yDistDeg = (blindspotDeg + (yDist * degPerVertPix));
  distRobotToTarget = height * tan(degToRad(yDistDeg));
  hypToCamera = sqrt( sq(height) + sq(distRobotToTarget) );
  xWidth = 2 * (hypToCamera * tan(thirtyDegInRad));
  xInPerPix = xWidth/78;

  //  Calc for xDistIn for distToTarget
  xDistIn = xDist*xInPerPix;
}

void calcDistToCenterLow() {
  if(lowPixy.ccc.blocks[0].m_x < lowPixy.ccc.blocks[1].m_x){
    leftX = lowPixy.ccc.blocks[0].m_x;
    rightX = lowPixy.ccc.blocks[1].m_x;
    leftWidth = lowPixy.ccc.blocks[0].m_width;
    rightWidth = lowPixy.ccc.blocks[1].m_width;
  }
  else if (lowPixy.ccc.blocks[0].m_x > lowPixy.ccc.blocks[1].m_x){
    leftX = lowPixy.ccc.blocks[1].m_x;
    rightX = lowPixy.ccc.blocks[0].m_x;
    leftWidth = lowPixy.ccc.blocks[1].m_width;
    rightWidth = lowPixy.ccc.blocks[0].m_width;
  }
    
  xLOne = leftX - ( leftWidth / 2 );
  xROne = rightX + ( leftWidth / 2 );
  xLTwo = leftX - ( rightWidth / 2 );
  xRTwo = rightX + ( rightWidth / 2 );
  centerPoint = ( ( xLTwo - xROne) / 2 ) + xROne;
  distToCenter = absoluteCenter - centerPoint;
  angleToCenter = distToCenter*0.189873;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //  Initializes the Pixy
  //  Hexadecimal values passed in correspond to address set on the pixy 
  highPixy.init(0x54);
  lowPixy.init(0x53);
  highPixy.changeProg("line");
}

//  Reads command off of the wire and converts it to a usable char
void receiveCommand () {
   incCommand = (char)Serial.read();
}

//  Flushes excess data off after we read the command
void serialFlush() {
  while (Serial.available() > 0) {
    char t = Serial.read();
  }
}

//  Writes data down the wire based on command passed in
void sendData (char command) {
  if (command == '2') {
    Serial.println(degToTarget);
  }
  else if (command == '1') {
    Serial.println(distToTarget);
  }
  else if (command == '3') {
    Serial.println(angleToCenter);
  }
  else if (command == '4') {
    Serial.println(lowPosition);
  }
}

void loop() {
  //  Gets data from the highPixy
  highPixy.line.getMainFeatures();

  //  Calculates return values for the highPixy
  calcInPerPix(cameraHeight, cameraAngle, highPixy.line.vectors->m_x0, highPixy.line.vectors->m_y0);
  degToTarget = atan((xDist*xInPerPix)/distRobotToTarget) * (180/pi);
  distToTarget = sqrt(sq(distRobotToTarget) + sq(xDistIn));
 
  //  Gets data from the lowPixy
  lowPixy.ccc.getBlocks(true, 255, 2);

  //  Calculates return values for the lowPixy
  if (lowPixy.ccc.numBlocks){
    calcDistToCenterLow();
    
    if( absoluteCenter - centerPoint < 2 ){
      lowPosition = 1;
      }
    else if( absoluteCenter - centerPoint > 2 ){
      lowPosition = 3;
    }
    else{
      lowPosition = 2;
    }
    
  }
  else {
    lowPosition = -1;
  }

  //  Runs the communication code if a command is available
  if (Serial.available() > 0) {
    receiveCommand();
    serialFlush();
    sendData(incCommand);
    Serial.flush();
    incCommand = 0;
  }
  
  
}
