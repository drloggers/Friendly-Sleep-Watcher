/****************************************************
 * Include file
 *****************************************************/
#include <Wire.h>
#include <rtc>
#include <FreqCount.h>
#include <LiquidCrystal.h>
#include <helper>
#include <EEPROM.h>

/****************************************************
 * Macros and Constants
 ****************************************************/
#define GSCALE 2 // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.
#define TIME_OUT 5 // One of the system's FSM transitions
#define ALARM_TIME_MET 6 // One of the system's FSM transitions
#define BUZZER_PIN 3 // Output PWM pin for the buzzer
#define motor_pin  6 // Output PWM pin for the motor
#define SNOOZE 10 // Minutes to snooze
#define KEYPAD_LEFT   1
#define KEYPAD_RIGHT  2
#define KEYPAD_SELECT 3
#define KEYPAD_MODE   4 
#define KEYPAD_NONE   0
#define INTERVAL      10

/****************************************************
 * Variable definition
 ****************************************************/
LiquidCrystal lcd(12,11,7,4,3,2);
// The different states of the system
enum states
{
  SHOW_TIME, // Displays the time and date
  SHOW_TIME_ALARM_ON, // Displays the time and date, and alarm is on
  SHOW_ALARM_TIME, // Displays the alarm time and goes back to time and date after 3 seconds
  SET_ALARM_HOUR, // Option for setting the alarm hours. If provided, it moves on to alarm minutes.
  // Otherwise, it times out after 5 seconds and returns to time and date
  SET_ALARM_MINUTES, // Option for setting the alarm minutes. If provided, it finally sets the alarm time and alarm.
  // Otherwise, it times out after 5 seconds and returns to time and date
  BUZZER_ON, // Displays the time and date, and buzzer is on (alarm time met)
  LAST_30_MINS,  // 30 mins before the set time
  LAST_15_MINS  // 15 mins before set time    
};
states state; // Holds the current state of the system

//rtcTime now; // Holds the current date and time information
int8_t button; // Holds the current button pressed
uint8_t alarmHours = 0, alarmMinutes = 0; // Holds the current alarm time
uint8_t tmpHours;
int prev_freq,prev_accel[3];
boolean alarm = false; // Holds the current state of the alarm
unsigned long timeRef;
extern unsigned long f_freq;
extern volatile unsigned char f_ready;
int pinLed=13;
byte second=0, minute=0, hour=0, dayOfWeek=0, month=0, year=0, day=0;
int store = 0, addr = 0;

/******************************************************
 * Initial Setup
 ******************************************************/
void setup() {
  pinMode(pinLed, OUTPUT);
  Wire.begin();
  Serial.begin(9600);        // connect to the serial port
  lcd.begin(20,4);
  FreqCount.begin(1000);     // pins not usable for pwm generation 3, 9, 10, 11
  //initMMA8452();
  Serial.println("Sleep Watcher");
  state = SHOW_TIME; // Initial state of the FSM
  // Uncomment this to set the current time on the RTC module
  // RTC.adjust(DateTime(__DATE__, __TIME__));
  second=30, minute=10, hour=3, dayOfWeek=2, month=5, year=14, day=29;
  addr = 0;
  store = 0;
}

int deviation(float accel[3],int freq){
  int devia;
  devia = abs(prev_freq - freq);
  devia += abs(accel[0]-prev_accel[0]) + abs(accel[1]-prev_accel[1]) + abs(accel[2]-prev_accel[2]);
  prev_freq = freq;
  prev_accel[0] = accel[0];
  prev_accel[1] = accel[1];
  prev_accel[2] = accel[2];
  return devia;
} 

uint8_t read_button(){
  static int NUM_KEYS=4;
  static int key_val[4] ={
    KEYPAD_LEFT,KEYPAD_RIGHT,KEYPAD_SELECT,KEYPAD_MODE  };
  int k;
  for (k = 0; k < NUM_KEYS; k++)
  {
    if (digitalRead(key_val[k]))
    {
      return key_val[k];
    }
  }
  if (k >= NUM_KEYS)
    k = -1;     // No valid key pressed
  return k;
}

// Looks at the provided trigger (event)
// and performs the appropriate state transition
// If necessary, sets secondary variables
void transition(uint8_t trigger)
{
  switch (state)
  {
  case SHOW_TIME:
    if ( trigger == KEYPAD_LEFT ) state = SHOW_ALARM_TIME;
    else if ( trigger == KEYPAD_RIGHT ) { 
      alarm = true; 
      state = SHOW_TIME_ALARM_ON; 
    }
    else if ( trigger == KEYPAD_SELECT ) state = SET_ALARM_HOUR;
    break;
  case SHOW_TIME_ALARM_ON:
    if ( trigger == KEYPAD_LEFT ) state = SHOW_ALARM_TIME;
    else if ( trigger == KEYPAD_RIGHT ) { 
      alarm = false; 
      state = SHOW_TIME; 
    }
    else if ( trigger == KEYPAD_SELECT ) state = SET_ALARM_HOUR;
    else if ( trigger == ALARM_TIME_MET ) { 
      analogWrite(BUZZER_PIN, 220); 
      state = BUZZER_ON; 
      analogWrite(motor_pin,220);    
    }
    break;
  case SHOW_ALARM_TIME:
    if ( trigger == TIME_OUT ) { 
      if ( !alarm ) state = SHOW_TIME;
      else state = SHOW_TIME_ALARM_ON; 
    }
    break;
  case SET_ALARM_HOUR:
    if ( trigger == KEYPAD_SELECT ) state = SET_ALARM_MINUTES;
    else if ( trigger == TIME_OUT ) { 
      if ( !alarm ) state = SHOW_TIME;
      else state = SHOW_TIME_ALARM_ON; 
    }
    break;
  case SET_ALARM_MINUTES:
    if ( trigger == KEYPAD_SELECT ) { 
      alarm = true; 
      state = SHOW_TIME_ALARM_ON; 
    }
    else if ( trigger == TIME_OUT ) { 
      if ( !alarm ) state = SHOW_TIME;
      else state = SHOW_TIME_ALARM_ON; 
    }
    break;
  case BUZZER_ON:
    if ( trigger == KEYPAD_RIGHT || trigger == KEYPAD_LEFT ) { 
      state = SHOW_TIME_ALARM_ON; 
    }
    if(FreqCount.read() > 90){
      analogWrite(BUZZER_PIN, 0);
      analogWrite(motor_pin, 0);
      alarm = false;
      state = SHOW_TIME;
    }
    break;
  }
}

// Displays the current date and time, and also an alarm indication
// e.g. SAT 04 JAN 2014, 22:59:10 ALARM
void showTime(){
  getFullTime();
  const char* dayName[] = { 
    "SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"       };
  const char* monthName[] = { 
    "JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"       };
  lcd.clear();
  lcd.print(String(dayName[dayOfWeek]) + " " +
    (day < 10 ? "0" : "") + day + " " +
    monthName[month-1] + " " + year);
  lcd.setCursor(0,1);
  lcd.print((hour < 10 ? "0" : "") + String(hour) + ":" +
    (minute < 10 ? "0" : "") + minute + ":" +
    (second < 10 ? "0" : "") + second + (alarm ? " ALARM" : ""));
}

// Displays the current alarm time and transitions back to show
// date and time after 2 sec (+ 1 sec delay from inside the loop function)
// e.g. Alarm Time HOUR: 08 MIN: 20
void showAlarmTime()
{
  lcd.clear();
  lcd.print("Alarm Time");
  lcd.setCursor(0,1);
  lcd.print(String("HOUR: ") + ( alarmHours < 9 ? "0" : "" ) + alarmHours +
    " MIN: " + ( alarmMinutes < 9 ? "0" : "" ) + alarmMinutes);
  delay(2000);
  transition(TIME_OUT);
}

// Checks if the alarm time has been met,
// and if so initiates a state transition
void checkAlarmTime()
{
  if ( hour == alarmHours && minute == alarmMinutes ) transition(ALARM_TIME_MET);
}

// The first of a 2 part process for setting the alarm time
// Receives the alarm time hour. If not provided within 5 sec,
// times out and returns to a previous (time and date) state
void setAlarmHours()
{
  unsigned long timeRef;
  boolean timeOut = true;
  lcd.clear();
  lcd.print("Alarm Time");

  tmpHours = 0;
  timeRef = millis();
  lcd.setCursor(0,1);
  lcd.print("Set hours: 0");
  while ( (unsigned long)(millis() - timeRef) < 5000 )
  {
    uint8_t button = read_button();

    if ( button == KEYPAD_RIGHT )
    {
      tmpHours = tmpHours < 23 ? tmpHours + 1 : tmpHours;
      lcd.setCursor(11,1);
      lcd.print(" ");
      lcd.setCursor(11,1);
      if ( tmpHours < 10 ) lcd.print(" ");
      lcd.print(tmpHours);
      timeRef = millis();
    }
    else if ( button == KEYPAD_LEFT )
    {
      tmpHours = tmpHours > 0 ? tmpHours - 1 : tmpHours;
      lcd.setCursor(11,1);
      lcd.print(" ");
      lcd.setCursor(11,1);
      if ( tmpHours < 10 ) lcd.print(" ");
      lcd.print(tmpHours);
      timeRef = millis();
    }
    else if ( button == KEYPAD_SELECT )
    {
      while ( read_button() != KEYPAD_NONE) ;
      timeOut = false;
      break;
    }
    delay(150);
  }

  if ( !timeOut ) transition(KEYPAD_SELECT);
  else transition(TIME_OUT);
}

// The second of a 2 part process for setting the alarm time
// Receives the alarm time minutes. If not provided within 5 sec,
// times out and returns to a previous (time and date) state
// If minutes are provided, sets the alarm time and turns the alarm on
void setAlarmMinutes()
{
  unsigned long timeRef;
  boolean timeOut = true;
  uint8_t tmpMinutes = 0;
  lcd.clear();
  lcd.print("Alarm Time");

  timeRef = millis();
  lcd.setCursor(0,1);
  lcd.print("Set minutes: 0");
  while ( (unsigned long)(millis() - timeRef) < 5000 )
  {
    uint8_t button = read_button();
    if ( button == KEYPAD_RIGHT )
    {
      tmpMinutes = tmpMinutes < 55 ? tmpMinutes + 5 : tmpMinutes;
      lcd.setCursor(13,1);
      lcd.print(" ");
      lcd.setCursor(13,1);
      if ( tmpMinutes < 10 ) lcd.print(" ");
      lcd.print(tmpMinutes);
      timeRef = millis();
    }
    else if ( button == KEYPAD_LEFT )
    {
      tmpMinutes = tmpMinutes > 0 ? tmpMinutes - 5 : tmpMinutes;
      lcd.setCursor(13,1);
      lcd.print(" ");
      lcd.setCursor(13,1);
      if ( tmpMinutes < 10 ) lcd.print(" ");
      lcd.print(tmpMinutes);
      timeRef = millis();
    }
    else if ( button == KEYPAD_SELECT )
    {
      while ( read_button() != KEYPAD_NONE ) ;
      timeOut = false;
      break;
    }
    delay(150);
  }

  if ( !timeOut )
  {
    alarmHours = tmpHours;
    alarmMinutes = tmpMinutes;
    transition(KEYPAD_SELECT);
  }
  else transition(TIME_OUT);
}

/* while(1){
 if (FreqCount.available()) {
 unsigned long count = FreqCount.read();
 Serial.println(count);
 }
 }*/

/*
  
 
 second = (byte) (20); 
 minute = (byte) (2);
 hour  = (byte) (11);
 dayOfWeek = (byte) (5);
 dayOfMonth = (byte) (29);
 month = (byte) (5);
 year= (byte) (14);
 setDateDs1307();
 getDateDs1307();
 Serial.println("");
/*
 while(1){
 
 getDateDs1307();
 delay(4000);
 while(1){
 Wire.beginTransmission(0x1c);
 
 int accelCount[3];  // Stores the 12-bit signed value
 readAccelData(accelCount);  // Read the x/y/z adc values
 float x,y,z;
 float prev_x=0,prev_y=0,prev_z=0;
 // Now we'll calculate the accleration value into actual g's
 float accelG[3];  // Stores the real accel value in g's
 for (int i = 0 ; i < 3 ; i++)
 {
 accelG[i] = (float) accelCount[i] / ((1<<12)/(2*GSCALE));  // get actual g value, this depends on scale being set
 x = accelG[1];
 y = accelG[0];
 z = accelG[2]; 
 }
 // Serial.println("accelometer data");
 //Serial.println(x);
 Serial.println(x*1000);
 Serial.println(",");
 Serial.println(y*1000);
 Serial.println(",");
 Serial.println(z*1000);
 //Serial.println(z);
 delay(10);  // Delay here for visibility
 status = Wire.endTransmission();
 if(status !=0){
 Serial.println("i2c transmission not terminated for accelerometer"); 
 }
 //getDateDs1307();
 //Serial.println("");
 if (FreqCount.available()) {
 unsigned long count = FreqCount.read();
 //Serial.print("Freq = ");
 // Serial.println(count/1000);
 }
 delay(500);
 } */



int Is30_min_diff(){
  byte alarmTime,actualTime;
  alarmTime = alarmHours *60 + alarmMinutes;
  getFullTime();
  actualTime = hour * 60 + minute;
  if(alarmTime - actualTime > 30)
    return 0;
  else if((alarmTime - actualTime) < 30 && (alarmTime - actualTime) > 0)
    return 1;
  else
    return 2;
}

int Is15_min_diff(){
  byte alarmTime,actualTime;
  alarmTime = alarmHours *60 + alarmMinutes;
  getFullTime();
  actualTime = hour * 60 + minute;
  if(alarmTime - actualTime > 15)
    return 0;
  else if((alarmTime - actualTime) < 15 && (alarmTime - actualTime) > 0)
    return 1;
  else
    return 2;
}

void log_movement(int data){
  EEPROM.write(addr, data);
  store++;
  if(store == INTERVAL){
    addr++;
    if(addr == 512)
      addr = 0;
    store = 0; 
  }
}

int read_logs(int address){
  return EEPROM.read(address);
}

int Is_movement(int logs[]){
  int avg=0;
  for(int i=0;i<addr;i++)
    avg += logs[i];
  avg /= addr;
  if(avg > 20)
    return 1;
  else
    return 0;
}
/******************************************************
 * Main Code Starts Here
 ******************************************************/
void loop() { // Has the main control of the FSM (1Hz refresh rate)
  int status;
  int accelCount[3];  // Stores the 12-bit signed value
  float accelG[3];  // Stores the real accel value in g's
  unsigned long count;
  timeRef = millis();
  int movement, address;
  int logs[512];
  Serial.print("State : ");
  Serial.println(state);
  // Uses the current state to decide what to process
  switch (state)
  {
  case SHOW_TIME:
    showTime();
    if(Is30_min_diff() == 1){

      readAccelData(accelCount);  // Read the x/y/z adc values
      // Now we'll calculate the accleration value into actual g's
      for (int i = 0 ; i < 3 ; i++){
        accelG[i] = (float) accelCount[i] / ((1<<12)/(2*GSCALE));  // get actual g value, this depends on scale being set
      }
      if (FreqCount.available())
        count = FreqCount.read();
      movement = deviation(accelG,(count/1000));
      log_movement(movement);
    }
    else if(Is15_min_diff() == 1){
      readAccelData(accelCount);  // Read the x/y/z adc values
      // Now we'll calculate the accleration value into actual g's
      for (int i = 0 ; i < 3 ; i++){
        accelG[i] = (float) accelCount[i] / ((1<<12)/(2*GSCALE));  // get actual g value, this depends on scale being set
      }
      if (FreqCount.available())
        count = FreqCount.read();
      movement = deviation(accelG,(count/1000));
      log_movement(movement);
      for (int i = 0 ; i < addr ; i++)
        logs[i] = read_logs(i);
      if(!Is_movement(logs)){
        analogWrite(motor_pin,50);
      }
    }

    break;
  case SHOW_TIME_ALARM_ON:
    showTime();
    checkAlarmTime();
    break;
  case SHOW_ALARM_TIME:
    showAlarmTime();
    break;
  case SET_ALARM_HOUR:
    setAlarmHours();
    if ( state != SET_ALARM_MINUTES ) break;
  case SET_ALARM_MINUTES:
    setAlarmMinutes();
    break;
  case BUZZER_ON:
    showTime();
    break;
  }

  // Waits about 1 sec for events (button presses)
  // If a button is pressed, it blocks until the button is released
  // and then it performs the applicable state transition
  while ( (unsigned long)(millis() - timeRef) < 970 )
  {
    if ( (button = read_button()) != KEYPAD_NONE )
    {
      while ( read_button() != KEYPAD_NONE ) ;
      transition(button);
      break;
    }
  }
}


