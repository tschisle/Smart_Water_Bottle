#include <CapacitiveSensor.h>

#define pls_samples 2
#define DUMMY_SENSORS 0 // 1 - uses the test data set, 0 - normal operation
const float bottle_radius = 1.5; //inches
const float bottle_height = 6; //inches
const long cs_full = 25900;
const long cs_empty = 3800;
const float cs_dump_slope = -0.75;
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define update_time 1000 //milliseconds between updates
const float settle_slope_tolerance = 0.25;  //will update water level after the change of water settles
const float intermediate_ounce_tolerance = 0.33; //this limits the ounce updates to only occur when the level increases or decreases beyond this tolerance, this was done because the cases where the amount of water was inbetween ounce measurements would bounce between ounces and count as drinks

int sensor_data[6] = {0, 0, 0, 0, 0, 0}; // Battery voltage (mV), Temperature (C), Water Level (fl-oz), Battery Low?, Battery Charged?, Dumped?

long cs1_raw;
long cs2_raw;
long cs_temp;
float cs_floz;
float prev_cs_floz;
unsigned long max_time_between_drinks = 5000; //ms
int analog_battery = 0;
int analog_thermistor = 0;
bool dump_flag = false; //used for checking for if dump slope condition was met
/*
   CapitiveSense Library Demo Sketch
   Paul Badger 2008
   Uses a high value resistor e.g. 10M between send pin and receive pin
   Resistor effects sensitivity, experiment with values, 50K - 50M. Larger resistor values yield larger sensor values.
   Receive pin is the sensor pin - try different amounts of foil/metal on this pin
*/

class PLSF_Filter
{
    //Variables/Matrices
    float least_square_mat[4][pls_samples]; //used to find the slope of the averaged sample clusters   1: sami - samavg 2: magi-magavg 3: 1*2 4: 1*1
    float least_square_avg; //used to find the slope of the averaged sample clusters
    float least_square_sum_comp[2]; //used to find the slope of the averaged sample clusters
    float least_square_slope_inter[2]; //holds slope and intercept values to approximate current value
    float sammat[pls_samples]; //holds the averaged samples in a matrix
    float approximation; //holds final approximated value
  public:
    void PLSF_Initialization(void) { //Initializes variables, matrices and does some early calculations to simplify the math for each update call
      //Precalculating static least square values HAS REDUNDANT ELEMENTS
      for (int _x = 0; _x < pls_samples; _x++) {
        least_square_mat[0][_x] = _x - ((pls_samples + 1) / 2);
        least_square_mat[3][_x] = least_square_mat[0][_x] * least_square_mat[0][_x];
        sammat[_x] = 0;
      }
      least_square_sum_comp[1] = 0;
      for (int _x = 0; _x < pls_samples; _x++) {
        least_square_sum_comp[1] = least_square_sum_comp[1] + least_square_mat[3][_x];
      }
    }
    float PLSF_Update(float _input) { //updates the buffer with the new input and outputs the filtered output
      //clearing previous values and shifting data
      for (int x = 0; x < pls_samples; x++) {
        if (x < (pls_samples - 1)) {
          sammat[x] = sammat[x + 1];
        }
        least_square_avg = 0;
        least_square_sum_comp[0] = 0;
      }
      sammat[pls_samples - 1] = _input;
      //finding average mag values
      for (int x = 0; x < pls_samples; x++) {
        least_square_avg = least_square_avg + sammat[x];
      }
      least_square_avg = least_square_avg / pls_samples;
      //calculating least square values
      for (int x = 0; x < pls_samples; x++) {
        least_square_mat[1][x] = sammat[x] - least_square_avg;
        least_square_mat[2][x] = least_square_mat[0][x] * least_square_mat[1][x];
      }
      for (int x = 0; x < pls_samples; x++) {
        least_square_sum_comp[0] = least_square_sum_comp[0] + least_square_mat[2][x];
      }
      least_square_slope_inter[0] = least_square_sum_comp[0] / least_square_sum_comp[1]; //slopes
      least_square_slope_inter[1] = least_square_avg - (least_square_slope_inter[0] * ((pls_samples + 1) / 2)); //intercept
      //approximating current mag data from best fit LINE and reporting movement percentage
      approximation = (least_square_slope_inter[0] * pls_samples) + least_square_slope_inter[1];
      //this step provides the slope
      return (least_square_slope_inter[0]);
    }
    void PLSF_Clear(void) { //clears the variables, matrices  (this function should be called before the first calibration step each time it's called
      //Precalculating static least square values HAS REDUNDANT ELEMENTS
      for (int _x = 0; _x < pls_samples; _x++) {
        least_square_mat[1][_x] = 0;
        least_square_mat[2][_x] = 0;
        sammat[_x] = 0;
      }
      least_square_sum_comp[0] = 0;
      least_square_sum_comp[1] = 0;
      least_square_avg = 0;
      least_square_sum_comp[0] = 0;
      least_square_sum_comp[1] = 0;
      least_square_slope_inter[0] = 0;
      least_square_slope_inter[1] = 0;
    }
};

PLSF_Filter cap;
CapacitiveSensor   cs1 = CapacitiveSensor(33, 25);        // 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired
//CapacitiveSensor   cs_4_6 = CapacitiveSensor(14,6);        // 10M resistor between pins 4 & 6, pin 6 is sensor pin, add a wire and or foil

void setup()
{
  esp_sleep_enable_timer_wakeup(10000000);
  //cs_4_2.set_CS_AutocaL_Millis(0xFFFFFFFF);     // turn off autocalibrate on channel 1 - just as an example
  Serial.begin(115200);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
}

void loop()
{
  esp_light_sleep_start();
  long total1 =  cs1.capacitiveSensorRaw(30);
  cs1_raw = cs1.capacitiveSensorRaw(30);
  //cs2_raw = cs2.capacitiveSensorRaw(30);
  cs_temp = map((cs1_raw + cs1_raw) / 2, cs_empty, cs_full, 0, 100); // converts to percentage full
  //cs_temp = map((cs1_raw + cs2_raw) / 2, cs_empty, cs_full, 0, 100); // converts to percentage full
  if (cs_temp < 0) {
    cs_temp = 0;
  } else if (cs_temp > 100) {
    cs_temp = 100;
  }
  cs_floz = ((float)cs_temp / 100.0) * 3.14159 * bottle_radius * bottle_radius * bottle_height * 0.5541; //converts from percentage to fl-oz

  float test = cap.PLSF_Update(cs_floz);
  if (test < cs_dump_slope) {
    dump_flag = true;
  }
  Serial.print("slope: ");
  Serial.print(test);
  Serial.print("   df = ");
  Serial.println(dump_flag);
  if (abs(test) < abs(settle_slope_tolerance)) { //only updates water level when rate of change is near 0 (this also means it'll update if the person drinks slowly and consistently)
    if (dump_flag && ((int)cs_floz == 0)) {
      sensor_data[5] = 1;
      dump_flag = false;
    }
    if (abs(prev_cs_floz - cs_floz) > intermediate_ounce_tolerance) { //if the change in ounces is greater than the tolerance
      prev_cs_floz = cs_floz;
      sensor_data[2] = (int)cs_floz;
    }
  }
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
