#include <PIDLoop.h>
#include <Pixy2I2C.h>

// Declares the Pixy
Pixy2I2C lowPixy;

//
int blockAreaOne = 0;
int blockAreaTwo = 0;
int leftX;
int rightX;

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
if(lowPixy.ccc.blocks[0].m_x < lowPixy.ccc.blocks[1].m_x){
leftX = lowPixy.ccc.blocks[0].m_x;
rightX = lowPixy.ccc.blocks[1].m_x;
}
else if (lowPixy.ccc.blocks[0].m_x > lowPixy.ccc.blocks[1].m_x){
leftX = lowPixy.ccc.blocks[1].m_x;
rightX = lowPixy.ccc.blocks[0].m_x;
}

if (leftX >= 158){
  Serial.print("Right");
}
else if (rightX <= 157){
  Serial.print("Left");
}
// if( (blockAreaOne-blockAreaTwo) < 100 && (blockAreaOne-blockAreaTwo) > -100 ) {
// Serial.println("Center");
// }

Serial.print("Detected ");
Serial.println(lowPixy.ccc.numBlocks);
for (i=0; i<lowPixy.ccc.numBlocks; i++) {
  Serial.print("  block ");
  Serial.print(i);
  Serial.print(": ");
  lowPixy.ccc.blocks[i].print();
}
}
else {
Serial.println("No Blocks");
}
}
