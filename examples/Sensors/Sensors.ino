#include <CapacitiveSensor.h>

CapacitiveSensor   cs1 = CapacitiveSensor(33,25);  //pin 25 is the receiver
CapacitiveSensor   cs2 = CapacitiveSensor(35,32);  //pin 32 is the receiver

void setup() {


}

void loop() {
  analogRead(A17); //Thermistor
  analogRead(A19); //Battery

}
