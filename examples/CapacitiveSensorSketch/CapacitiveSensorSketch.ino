#include <CapacitiveSensor.h>

/*
 * CapitiveSense Library Demo Sketch
 * Paul Badger 2008
 * Uses a high value resistor e.g. 10M between send pin and receive pin
 * Resistor effects sensitivity, experiment with values, 50K - 50M. Larger resistor values yield larger sensor values.
 * Receive pin is the sensor pin - try different amounts of foil/metal on this pin
 */


CapacitiveSensor   cs_4_2 = CapacitiveSensor(14,27);        // 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired
//CapacitiveSensor   cs_4_6 = CapacitiveSensor(14,6);        // 10M resistor between pins 4 & 6, pin 6 is sensor pin, add a wire and or foil

void setup()                    
{
   //cs_4_2.set_CS_AutocaL_Millis(0xFFFFFFFF);     // turn off autocalibrate on channel 1 - just as an example
   Serial.begin(115200); 
   pinMode(13, OUTPUT);
   digitalWrite(13, LOW);
}

void loop()                    
{
    long total1 =  cs_4_2.capacitiveSensorRaw(30);

    Serial.print("Test:");        // check on performance in milliseconds
    Serial.print("\t");                    // tab character for debug windown spacing
    //Serial.println(touchRead(7));
    Serial.println(map(total1, 3800, 25900, 0, 100));
    /*
    Serial.print(total1);                  // print sensor output 1
    Serial.print("\t");
    Serial.print(total2);                  // print sensor output 2
    Serial.print("\t");
    Serial.println(total3);                // print sensor output 3
    */

    delay(500);                             // arbitrary delay to limit data to serial port 
}
