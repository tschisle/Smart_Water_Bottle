#include <CapacitiveSensor.h>
#include <driver/adc.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//-=-=-=-=-=-  Constants
#define pls_samples 3
#define DUMMY_SENSORS 1 // 1 - uses the test data set, 0 - normal operation
const float bottle_radius = 1.5; //inches
const float bottle_height = 6; //inches
const long cs_full = 25900;
const long cs_empty = 3800;
const float cs_dump_slope = -12;
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define update_time 1000 //milliseconds between updates
const float settle_slope_tolerance = 0.25;  //will update water level after the change of water settles


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
long cs1_raw;
long cs2_raw;
long cs_temp;
float cs_floz;
unsigned long max_time_between_drinks = 5000; //ms
int analog_battery = 0;
int analog_thermistor = 0;
bool dump_flag = false; //used for checking for if dump slope condition was met

//-=-=-=-=-=-  DEBUGGING Global Variables & Arrays
//                                full, normal                one drink                 two drinks                dumped                 refilled          full, cold, charged        hot, one drink            two drinks              three drinks               dead battery
int sensor_data_test[10][6] = {{3700, 8, 32, 0, 0, 0}, {3700, 12, 24, 0, 0, 0}, {3700, 15, 18, 0, 0, 0}, {3700, 25, 0, 0, 0, 1}, {3700, 7, 32, 0, 0, 0}, {4200, 2, 32, 0, 1, 0}, {3800, 80, 24, 0, 0, 0}, {3700, 12, 16, 0, 0, 0}, {3700, 13, 12, 0, 0, 0}, {3400, 13, 12, 1, 0, 0}};
int sensor_data_test_stepper = 0; //steps through the matrix - micro should get stuck on the last step
unsigned long time_between_arrary_updates = 1500; //ms
unsigned long time_update; //required to send out array updates
int ledPin = 2;// Setup GPIO2 for LED output
bool led_on = false;// LED on flag
int total_volume = 0;// Total amount of water inside the bottle
unsigned long drink_time = 0;// Amount of time since user drank
unsigned long hour_time = 0;// Running timer to track hourly water consumption
int drank_last_hour = 0;// Amount of water drank in the last hour
int drank_last_day_arr[23] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};// Array to keep track of amount of water drank in the 23 hours before


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-  Filter
/***************************************************************************************
     Title: Predicitve Least Squares Filter
     Author: Trey Schisler
     Date: 5/15/2020
     Code version: 0.3
     Availability: https://github.com/tschisle/ESP-32_MPU9250_VR-Glove

***************************************************************************************/
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

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-  Object Creation
CapacitiveSensor   cs1 = CapacitiveSensor(33, 25); //pin 25 is the receiver
CapacitiveSensor   cs2 = CapacitiveSensor(35, 32); //pin 32 is the receiver
PLSF_Filter cap;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-  Setup
void setup() {
  sensor_initialization();
  Serial.begin(115200);
  adc2_config_channel_atten( ADC2_CHANNEL_7, ADC_ATTEN_11db );
  adc2_config_channel_atten( ADC2_CHANNEL_9, ADC_ATTEN_11db );
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  delay(500);
  Serial.println("Start up");
  time_update = millis();
  // Initial measurements
  check_low_battery();
  total_volume = sensor_data[2];
  update_display();
  // Initiate time since last drink
  //drink_time = millis();
  hour_time = millis();
  delay(2000);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-  Loop
void loop() {
  sensor_update();
  unsigned long cur_time = millis();
  check_low_battery();
  // Reset hourly statistics if an hour has passed
  if ((cur_time - hour_time) > 3600000) {
    hour_time = cur_time;
    update_last_day_array(drank_last_hour);
    drank_last_hour = 0;
  }
  // If water level increaes update water statistics
  if (sensor_data[2] > total_volume) {
    update_display();
    total_volume = sensor_data[2];
    // If water level decreases, update water and user statistics depending on if they drank or dumped water
  } else if (sensor_data[2] < total_volume) {
    int decrease = total_volume - sensor_data[2];
    total_volume = sensor_data[2];
    if (!sensor_data[5]) {
      update_statistics(decrease);
    }
    update_display();
  }
  update_display();
  // Replace with flip flop for final product
  if ((cur_time - drink_time) > 3600000) {
    if (led_on) {
      digitalWrite(ledPin, LOW);
      delay(1000);
      led_on = false;
    } else {
      digitalWrite(ledPin, HIGH);
      delay(1000);
      led_on = true;
    }
  }
  //↓ REPLACE WITH LIGHT SLEEP ↓
  delay(update_time);
}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-  Sensor Initialization
int sensor_initialization(void) {
  if (!DUMMY_SENSORS) {
    cap.PLSF_Initialization();
  }
  return (sensor_update());
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-  Sensor Update
int sensor_update(void) {
  if (!DUMMY_SENSORS) {
    //-=-=-=-=-=-=-=-=-=-=-     Battery     -=-=-=-=-=-=-=-=-=-=-
    adc2_get_raw(ADC2_CHANNEL_9, ADC_WIDTH_12Bit, &analog_battery);
    sensor_data[0] = (int) (ADC_correction(analog_battery) * 1.612); //Battery ideal conversion factor = 1.612
    if (sensor_data[0] < 3400) {
      sensor_data[3] = 1; //sets battery low flag
      return (3); //returns location of set flag - is an error state so function ends early
    } else if (sensor_data[0] >= 4200) {
      sensor_data[4] = 1; //sets battery charged flag
    }
    //-=-=-=-=-=-=-=-=-=-=-     Water Level     -=-=-=-=-=-=-=-=-=-=-
    cs1_raw = cs1.capacitiveSensorRaw(30);
    cs2_raw = cs2.capacitiveSensorRaw(30);
    cs_temp = map((cs1_raw + cs2_raw) / 2, cs_empty, cs_full, 0, 100); // converts to percentage full
    if (cs_temp < 0) {
      cs_temp = 0;
    } else if (cs_temp > 100) {
      cs_temp = 100;
    }
    cs_floz = ((float)cs_temp / 100.0) * 3.14159 * bottle_radius * bottle_radius * bottle_height * 0.5541; //converts from percentage to fl-oz
    if (cap.PLSF_Update(cs_floz) < cs_dump_slope) {
      dump_flag = true;
    }
    if (abs(cap.PLSF_Update(cs_floz)) < settle_slope_tolerance) { //only updates water level when rate of change is near 0 (this also means it'll update if the person drinks slowly and consistently)
      if (dump_flag && (cs_floz == 0)) {
        sensor_data[5] = 1;
        dump_flag = false;
      }
      sensor_data[2] = (int)cs_floz;
    }
    //-=-=-=-=-=-=-=-=-=-=-     Thermistor     -=-=-=-=-=-=-=-=-=-=-
    adc2_get_raw(ADC2_CHANNEL_7, ADC_WIDTH_12Bit, &analog_thermistor);
    sensor_data[1] = thermistor_conversion(ADC_correction(analog_thermistor));
  } else {
    //-=-=-=-=-=-=-=-=-=-=-     Dummy Sensors     -=-=-=-=-=-=-=-=-=-=-
    if (millis() >= time_update) {
      if ((sensor_data_test_stepper == 1) || (sensor_data_test_stepper == 6)) {
        time_update = millis() + max_time_between_drinks + time_between_arrary_updates; //to trigger the Drink Reminder LED
      } else {
        time_update = millis() + time_between_arrary_updates;
      }
      for (int x = 0; x < 6; x++) {
        sensor_data[x] = sensor_data_test[sensor_data_test_stepper][x];
      }
      if (sensor_data_test_stepper < 9) {
        sensor_data_test_stepper++;
      }
    }
  }
  return (0);
}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-  Thermistor R to Celcius
int thermistor_conversion(int therm_analog) {
  double temperature_analog = (float)therm_analog; //done to avoid unintentional data trimming especially during the log calculation
  const double therm_r_ref = 47000;
  const double therm_sensor_r = 22000;
  const double therm_r = (3.3 - (temperature_analog / 1240.9090909)) / ((temperature_analog / 1240.9090909) / therm_sensor_r);
  const double therm_beta = 3862;
  return ((int)((((therm_beta * 298) / ((298 * log(therm_r / therm_r_ref)) + (therm_beta)))) - 273));
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

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-  OLED Display Update
void update_display(void) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  display.print("Volume (oz): ");
  display.println(sensor_data[2]);

  display.print("Temperature (C): ");
  display.println(sensor_data[1]);

  if (!sensor_data[2]) {
    display.println("No water in bottle!!!");
    display.println("");
  } else {
    display.println("");
    display.println("");
  }

  display.print("Last Drink: ");
  if (drink_time == 0) {
    display.println("N/A");
  } else {
    unsigned long cur_time = millis();
    unsigned long mils_elapsed = cur_time - drink_time;
    unsigned long minutes_elapsed = millis_to_min(mils_elapsed);

    if (minutes_elapsed >= 60) {
      int hours = minutes_elapsed / 60;
      int mins = minutes_elapsed % 60;
      display.print(hours);
      display.println(" hr");
      //display.print(mins);
      //display.println(" min ago");

    } else {
      display.print(minutes_elapsed);
      display.println(" min");
    }
  }

  display.println("Amount Drank (oz)");
  display.print("Last Hour: ");
  display.println(drank_last_hour);


  display.print("Last 24 Hours: ");
  display.println(drank_last_day());
  display.display();
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-  Check Low Battery
void check_low_battery() {
  if (sensor_data[3]) {
    // Shutdown if battery is low
    display_low_battery();
    delay(2000);
    display.clearDisplay();
    display.display();
    esp_deep_sleep_start();
  }
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-  Display Low Battery
void display_low_battery() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("Low Power");
  display.display();
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-  Update Stats
void update_statistics(int amount_drank) {
  unsigned long cur_time = millis();
  // Reset drink timer
  drink_time = cur_time;
  // Update amount of water drank
  drank_last_hour += amount_drank;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-  Update Last Day Array
void update_last_day_array(int new_hourly_volume) {
  for (int i = 22; i >= 0; i--) {
    drank_last_day_arr[i + 1] = drank_last_day_arr[i];
  }
  drank_last_day_arr[0] = new_hourly_volume;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-  Drank Last Day
// Calculates the amount of water drank in the last 24 hours
int drank_last_day() {
  int amount_drank = 0;
  for (int i = 0; i < 23; i++) {
    amount_drank += drank_last_day_arr[i];
  }
  amount_drank += drank_last_hour;
  return amount_drank;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-  Millis to Minute Conversion
// Converts milliseconds to minutes
unsigned long millis_to_min(unsigned long mils) {
  return (mils / 60000);
}
