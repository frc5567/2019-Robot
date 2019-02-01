#include <PIDLoop.h>
#include <Pixy2I2C.h>
#include <Pixy2SPI_SS.h>


/**
 * 
 */
 
//  Declares the Pixy
Pixy2SPI_SS pixy;
// Pixy2I2C lowPixy;

//  Value of pi for calculations
double pi = 3.1415926535;

//  Height and angle of the pixy
double cameraHeight = 46;
double cameraAngle = 25;

//  Coordinates of the origin point
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

//  Total degrees to the point from the camera relative to zero

//  Diagonal distance from the camera to the target point
double hypToCamera;
double yDistDeg;

//  Y Distance in inches from the robot to the target
double distRobotToTarget;

//  Width of the projection at the given y value
double xWidth;

//  Inches per pixel in the x direction
double xInPerPix;

//  Degrees from the base of the robot to the target point
double degToTarget;

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
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //Serial.print("Starting");

  //  Initializes the Pixy
  pixy.init(0x54);
//  lowPixy.init(0x53);
  pixy.changeProg("line");
}

void loop() {
  // put your main code here, to run repeatedly:
  int i;
  char buf[128];
  pixy.line.getMainFeatures();
//  lowPixy.ccc.getBlocks();
  //  Serial.print("enter");
  // If there are detect blocks, print them!
//  if (lowPixy.ccc.numBlocks){
//    Serial.print("Detected ");
//    Serial.println(lowPixy.ccc.numBlocks);
//    for (i=0; i<lowPixy.ccc.numBlocks; i++) {
//      Serial.print("  block ");
//      Serial.print(i);
//      Serial.print(": ");
//      lowPixy.ccc.blocks[i].print();
//    }
//  }
//  else {
//    Serial.println("No Blocks");
//  }
  
//  Serial.print("exit");
  calcInPerPix(cameraHeight, cameraAngle, pixy.line.vectors->m_x0, pixy.line.vectors->m_y0);
  degToTarget = atan((xDist*xInPerPix)/distRobotToTarget) * (180/pi);
  //char *cDegToTarget = new char[16];
  //sprintf(cDegToTarget, "%.2f", degToTarget);
  //Serial.print(cDegToTarget);
  Serial.print(degToTarget);
  Serial.print('\n');
//  Serial.print("X: ");
//  Serial.print(pixy.line.vectors->m_x0);
//  Serial.print("Y: ");
//  Serial.print(pixy.line.vectors->m_y0);
//  Serial.print("Theta: ");
//  Serial.println(degToTarget);
}
