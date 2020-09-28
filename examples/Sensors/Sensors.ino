#include <CapacitiveSensor.h>

//-=-=-=-=-=-  Constants
#define pls_samples 20
#define DUMMY_SENSORS 1 // 1 - uses the test data set, 0 - normal operation
const float bottle_radius = 1.5; //inches
const float bottle_height = 6; //inches

//-=-=-=-=-=-  Global Variables & Arrays
int sensor_data[6] = {0, 0, 0, 0, 0, 0}; // Battery voltage (mV), Temperature (C), Water Level (fl-oz), Battery Low?, Battery Charged?, Dumped?
/* "sensor_data" NOTES:
   Battery Voltage - reported in milliVolts to avoid having to use float, which takes more memory
   Temperature - reported in Celcius
   Water Level - reported in fluid ounces (this requires the use of bottle radius and height
   Battery Low? - boolean, 1 when battery is low & 0 when battery is charged
   Battery Charged? - boolean, 1 when battery is charged & 0 when battery wasn't just charged
   Dumped? - boolean, 1 when bottle was likely dumped & 0 when bottle wasn't dumped

   Boolean variables currently will not be cleared by the sensor code to ensure they are used properly
*/
unsigned long max_time_between_drinks = 5000; //ms

//-=-=-=-=-=-  DEBUGGING Global Variables & Arrays
//                                full, normal                one drink                 two drinks                dumped                 refilled          full, cold, charged        hot, one drink            two drinks              three drinks               dead battery
int sensor_data_test[10][6] = {{3700, 8, 32, 0, 0, 0}, {3700, 12, 24, 0, 0, 0}, {3700, 15, 18, 0, 0, 0}, {3700, 25, 0, 0, 0, 1}, {3700, 7, 32, 0, 0, 0}, {4200, 2, 32, 0, 1, 0}, {3800, 80, 24, 0, 0, 0}, {3700, 12, 16, 0, 0, 0}, {3700, 13, 12, 0, 0, 0}, {3400, 13, 12, 1, 0, 0}};
int sensor_data_test_stepper = 0; //steps through the matrix - micro should get stuck on the last step
unsigned long time_between_arrary_updates = 1500; //ms
unsigned long time_update; //required to send out array updates


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-  Filter
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
      //this step provides the unitless value
      return (approximation);
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

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-  Object Creation
CapacitiveSensor   cs1 = CapacitiveSensor(33, 25); //pin 25 is the receiver
CapacitiveSensor   cs2 = CapacitiveSensor(35, 32); //pin 32 is the receiver
PLSF_Filter cap1;
PLSF_Filter cap2;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-  Setup
void setup() {
  sensor_initialization();
  Serial.begin(115200);
  delay(500);
  Serial.println("Start up");
  time_update = millis();
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-  Loop
void loop() {
  sensor_update();
  Serial.println(sensor_data[0]);
  delay(250);
}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-  Sensor Initialization
int sensor_initialization(void) {
  if (!DUMMY_SENSORS) {
    cap1.PLSF_Initialization();
    cap2.PLSF_Initialization();
  }
  return (sensor_update());
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-  Sensor Update
int sensor_update(void) {
  if (!DUMMY_SENSORS) {
    sensor_data[0] = (int) (ADC_correction(analogRead(A19)) * 1.612); //Battery ideal conversion factor = 1.612
    if (sensor_data[0] < 3400) {
      sensor_data[3] = 1; //sets battery low flag
      return (3); //returns location of set flag - is an error state so function ends early
    } else if (sensor_data[0] >= 4200) {
      sensor_data[4] = 1; //sets battery charged flag
    }
    cap1.PLSF_Update(cs1.capacitiveSensorRaw(10));
    cap2.PLSF_Update(cs2.capacitiveSensorRaw(10));
    ADC_correction(analogRead(A17)); //Thermistor
  } else {
    if (millis() >= time_update) {
      if ((sensor_data_test_stepper == 1) || (sensor_data_test_stepper == 6)) {
        time_update = millis() + max_time_between_drinks + time_between_arrary_updates; //to trigger the Drink Reminder LED
      } else {
        time_update = millis() + time_between_arrary_updates;
      }
      if (sensor_data_test_stepper < 9) {
        sensor_data_test_stepper++;
      }
      for (int x = 0; x < 6; x++) {
        sensor_data[x] = sensor_data_test[sensor_data_test_stepper][x];
      }
    }
  }
  return (0);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-  ADC Correction
int ADC_correction(int bad_analog) {
  float temp_analog = (float)bad_analog; //done to avoid unintentional data trimming
  temp_analog = 171 + (0.961 * temp_analog) + (0.0000793 * temp_analog * temp_analog) - (0.0000000227 * temp_analog * temp_analog * temp_analog); //derived from experimental data
  if (temp_analog < 0) {
    temp_analog = 0;
  } else if (temp_analog > 4095) {
    temp_analog = 4095;
  }
  return ((int)temp_analog);
}
