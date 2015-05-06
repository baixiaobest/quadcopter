#include <Esplora.h>

boolean throttleControlEnable = true;
int lastValue = 0;
void setup()
{
  Serial.begin(115200);
} 

void loop()
{
  //get slider value
  int value = Esplora.readSlider();
  value = map(value,0, 1023, 800, 1500);
  //get button value
  int resetButton = Esplora.readButton(SWITCH_1);
  
  //enable throttleControl if certain safty precedure is executed
  if(!throttleControlEnable && value < 805 && Esplora.readButton(SWITCH_2)==0){
    throttleControlEnable = true;
  }
  //reset quadcopter
  if (resetButton == 0){
    Serial.println("0");
    throttleControlEnable = false;
  }
  //donnot send data when throttle is disabled and there is not much change in value
  if (throttleControlEnable && abs(lastValue-value)>5){
     Serial.println(value);
  }
  if (abs(lastValue-value)>5)
    lastValue = value;
}
