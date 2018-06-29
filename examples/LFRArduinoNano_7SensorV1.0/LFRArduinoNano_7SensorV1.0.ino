#include <LFRArduinoNano7Sesnor.h>
LFR lfr;
void setup() {

  //------------------------------------------
  // put your move strategy here, to run robot:
  //------------------------------------------
  lfr.Init();
  lfr.turnRight(900);

}

void loop() {
  // put your main code here, to run repeatedly:
   lfr.turnRight(900);
}
