#include <PIDLoop.h>
#include <Pixy2I2C.h>

// Declares the Pixy
Pixy2I2C lowPixy;

// Creates the variables we need
int blockAreaOne = 0;
int blockAreaTwo = 0;
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


void setup() {
Serial.begin(9600);
Serial.print("Starting");

// Initializes the Pixy
lowPixy.init(0x53);
}

void loop() {
int i;
lowPixy.ccc.getBlocks(true, 255, 2);
if (lowPixy.ccc.numBlocks){
// blockAreaOne = (lowPixy.ccc.blocks[0].m_width)(lowPixy.ccc.blocks[0].m_height);
// blockAreaTwo = (lowPixy.ccc.blocks[1].m_width)(lowPixy.ccc.blocks[1].m_height);

// 
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

if (leftX >= 158){
 // Serial.print("Right");
}
else if (rightX <= 157){
//  Serial.print("Left");
}
// if( (blockAreaOne-blockAreaTwo) < 100 && (blockAreaOne-blockAreaTwo) > -100 ) {
// Serial.println("Center");
// }

int xLOne = leftX - ( leftWidth / 2 );
int xROne = rightX + ( leftWidth / 2 );
int xLTwo = leftX - ( rightWidth / 2 );
int xRTwo = rightX + ( rightWidth / 2 );
int centerPoint = ( ( xLTwo - xROne) / 2 ) + xROne;
int distToCenter = absoluteCenter - centerPoint;
double angleToCenter = distToCenter * 0.189873;


if( absoluteCenter - centerPoint < 2 ) {
   Serial.print(angleToCenter);
   Serial.print("\t");
   Serial.print("degrees left of center");
}

else if( absoluteCenter - centerPoint > 2 ) {
    Serial.print(angleToCenter);
    Serial.print("\t");
    Serial.print("degrees right of center");
}

else{
  Serial.println("Centered");
}

if( centerPoint > 160 ){
  Serial.print(centerPoint);
  Serial.print("/t");
  Serial.println("center to right");
}

else if( centerPoint < 156 ){
  Serial.print(centerPoint);
  Serial.print("/t");
  Serial.println("center to left");
}



Serial.print("Detected ");
// Serial.println(lowPixy.ccc.numBlocks);
// for (i=0; i<lowPixy.ccc.numBlocks; i++) {
//  Serial.print("  block ");
//  Serial.print(i);
//  Serial.print(": ");
//  lowPixy.ccc.blocks[i].print();
// }
}
else {
Serial.println("No Blocks");
}
}
