#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Setup GPIO2 for LED output
int ledPin = 2;
// LED on flag
bool led_on = false;
// Total amount of water inside the bottle
int total_volume = 0;
// Amount of time since user drank
unsigned long drink_time = 0;
// Running timer to track hourly water consumption
unsigned long hour_time = 0;
// Amount of water drank in the last hour
int drank_last_hour = 0;
// Array to keep track of amount of water drank in the 23 hours before
int drank_last_day_arr[23] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Test 
//int drank_last_day_arr[24] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23};

//int sensor_data[6] = {0, 0, 0, 0, 0, 0}; // Battery voltage (mV), Temperature (C), Water Level (fl-oz), Battery Low?, Battery Charged?, Dumped?
/* "sensor_data" NOTES:
   Battery Voltage - reported in milliVolts to avoid having to use float, which takes more memory
   Temperature - reported in Celcius
   Water Level - reported in fluid ounces (this requires the use of bottle radius and height
   Battery Low? - boolean, 1 when battery is low & 0 when battery is charged
   Battery Charged? - boolean, 1 when battery is charged & 0 when battery wasn't just charged
   Dumped? - boolean, 1 when bottle was likely dumped & 0 when bottle wasn't dumped
   Boolean variables currently will not be cleared by the sensor code to ensure they are used properly
*/

// Test Data
int sensor_data_test[10][6] = 
  {{3700, 8, 32, 0, 0, 0},  // full, normal
  {3700, 12, 24, 0, 0, 0},  // one drink
  {3700, 15, 18, 0, 0, 0},  // two drinks
  {3700, 25, 0, 0, 0, 1},   // dumped
  {3700, 7, 32, 0, 0, 0},   // refilled
  {4200, 2, 32, 0, 1, 0},   // full, cold, charged
  {3800, 80, 24, 0, 0, 0},  // hot, one drink
  {3700, 12, 16, 0, 0, 0},  // two drinks
  {3700, 13, 12, 0, 0, 0},  // three drinks
  {3400, 13, 12, 1, 0, 0}}; // dead battery

int sensor_data[6] = {3700, 8, 32, 0, 0, 0}; 

void update_display() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  
  display.print("Volume (oz): "); 
  display.println(sensor_data[2]);
  
  display.print("Temperature (C): ");
  display.println(sensor_data[1]);

  if(!sensor_data[2]) {
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

    if (minutes_elapsed > 60) {
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

void display_low_battery(){
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("Low Power");
  display.display();
}

void setup() {
  // Set GPIO2 to output
  pinMode(ledPin, OUTPUT);

  // Initiate OLED screen
  Serial.begin(115200);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  delay(2000);
  
  // Initial measurements
  check_low_battery();
  total_volume = sensor_data[2];
  update_display();
  
  // Initiate time since last drink
  //drink_time = millis();
  hour_time = millis();
  delay(100);
}

void loop() {
  
  Serial.println(drink_time);
  Serial.println(millis());
  Serial.println("");
  unsigned long cur_time = millis();
  check_low_battery();

  for (int i=0; i<23; i++){
    Serial.print(drank_last_day_arr[i]);
    Serial.print(" ");
  }
  Serial.println("");

  // Reset hourly statistics if an hour has passed
  if ((cur_time - hour_time) > 3600000) {
    hour_time = cur_time;
    update_last_day_array(drank_last_hour);
    drank_last_hour = 0;
  }

  // If water level increaes update water statistics
  if (sensor_data[2] > total_volume){
    update_display();
  // If water level decreases, update water and user statistics depending on if they drank or dumped water 
  } else if (sensor_data[2] < total_volume){
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
      
  } else {
    delay(1000);
  }

  if (total_volume == 32) {
    sensor_data[2] = total_volume - 1;
  }
}

void update_statistics(int amount_drank){
  unsigned long cur_time = millis();

  // Reset drink timer
  drink_time = cur_time;

  // Update amount of water drank
  drank_last_hour += amount_drank;
}

// Update 24 hour array by inserting new volume data at index 0
void update_last_day_array(int new_hourly_volume) {
  for (int i = 22; i >= 0; i--) {
    drank_last_day_arr[i+1] = drank_last_day_arr[i];
  }
  drank_last_day_arr[0] = new_hourly_volume;
}

// Calculates the amount of water drank in the last 24 hours 
int drank_last_day(){
  int amount_drank = 0;
  for (int i = 0; i < 23; i++) {
    amount_drank += drank_last_day_arr[i];
  }
  amount_drank += drank_last_hour;
  return amount_drank;
}

// Converts milliseconds to minutes
unsigned long millis_to_min(unsigned long mils){
  unsigned long mins = (mils / 60000);
  return mins;
}
