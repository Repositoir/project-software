#include <Arduino.h>
#include <Servo.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>


/* RULES & INTRO
 *
 * For the code we will follow some standard naming conventions:
 *
 * 1. Variable, parameters, arguments names will be in camel case (eg. thisIsCamelCase).
 *
 * 2. Functions will be small case with underscores for spaces (eg. void this_is_a_function())
 *
 * 3. When creating a function controlling a component try to include the state as one of the parameters.
 *    See functions below as examples. It simplifies code.
 *
 * 4. Comment what every function does.
 *
 * 5. Use all caps in constants and pin definitions (eg. const int BUZZER = 13;)
 *
 * 6. This list is appendable.
*/


// Pins
#define RED_LED 1   // OP
#define GREEN_LED 13 // OP
#define BUZZER 3    // OP
#define SERVO 8    // ATTACH
#define ROT_A 5     //INP
#define ROT_B 10     //INP
#define ROT_BTN 7   //INP_PL
#define SR_DATA 6  // OP
#define SR_LAT 2    // OP
#define SR_CLK 9   // OP
#define BCD_A 4   // OP
#define BCD_B 17    // OP
#define BCD_C 16    // OP
#define BCD_D 15    // OP
#define BUTTON 0  // INP_PL
#define DP1 14      // OP
#define DP2 11      // OP
#define DP3 12    // OP

// All other consts
#define DEBOUNCE_TIME 60
#define LOCKED 100
#define UNLOCKED 101
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_RESET (-1)
#define SCREEN_ADDRESS 0x3C



const int DEC_DIGITS[10] = {1, 79, 18, 6, 76, 36, 96, 15, 0, 12}; // 32 4
const int BCD_PINS[4] = {BCD_A, BCD_B, BCD_C, BCD_D};


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Servo s1;
Encoder Rotary(ROT_A,ROT_B);

/// Controls state of LEDs that display what state the safe is in
/// Sets value of each LED (high/low) depending on what state is
/// @param state is the state of safe (UNLOCKED/LOCKED)
void led_control(int state){
    if (state == UNLOCKED) {
        digitalWrite(RED_LED, LOW);
        digitalWrite(GREEN_LED, HIGH);
    } else {
        digitalWrite(RED_LED, HIGH);
        digitalWrite(GREEN_LED, LOW);
    }
}

void writeArrayIntoEEPROM(int address, byte num1, byte num2, byte num3)
{
  int numbers[3] = {num1, num2, num3};
  Serial.println("111");
  int addressIndex = address;  //Address will be updated in each loop
  for (int i = 0; i < 3; i++)    //A loop runs 3 times to store 3 seperate values of the password
  {
    EEPROM.write(addressIndex, numbers[i]);   //Store the value into 1 address
    addressIndex += 1;   //Shift to another address to store another value
    Serial.println(numbers[i]);
  }
}


void keepLockingState(int address, byte number)
{
  EEPROM.write(address, number);
}

void readLockingState(int address)
{
  EEPROM.read(address);
}

/// Controls buzzer output based on state (LOCKED/UNLOCKED)
/// Plays two short notes of same freq when state = LOCKED
/// Plays two notes in ascending order when state = UNLOCKED
/// @param state is state of safe
void buzzer_control(int state){
    if (state == UNLOCKED) {
        tone(BUZZER, 500);
        delay(100);
        tone(BUZZER, 600);
        delay(100);
        noTone(BUZZER);
    } else {
        tone(BUZZER, 440);
        delay(100);
        noTone(BUZZER);
        delay(5);
        tone(BUZZER, 440);
        delay(100);
        noTone(BUZZER);
    }
}

/// Reads rotary encoder adjusted for debouncing
/// @returns 0 if no rotation, 1 if anti-clockwise and -1 is clockwise
/// Do at 9600 baud
int read_encoder() {
  static long RotaryNum;
  static long RotaryTurn;
  RotaryTurn = Rotary.read();
  if (RotaryTurn != RotaryNum) {
    //Serial.print(RotaryTurn);
    //Serial.print(RotaryNum);
    if ((RotaryTurn % 4) == 0) {
      if (RotaryNum < RotaryTurn){
            RotaryNum = RotaryTurn;
        return 1;}
      else{
          RotaryNum = RotaryTurn;
          return 2;}
      }
    }
  return 0;

    }
/// Controls the servo motor
/// Moves to 180 degree when UNLOCKED and 10 when LOCKED
/// @param state is state of safe
void servo_control(int state){
    if (state == UNLOCKED){
        s1.write(90);
    } else {
        s1.write(10);
    }
}

/// @returns for how long the rotary encoder button was pressed
/// Compares by the difference between two clocks starting at button press and button release
int rotary_button(){

    static unsigned long time, time2;
    long elapsed_time = 0;
    delay(5);
    if (digitalRead(ROT_BTN) == LOW) {
      time = millis();
      while (!digitalRead(ROT_BTN)) {

      }

      time2 = millis();

      elapsed_time = time2 - time;
    }

    if (elapsed_time > 1500) {
      return 1;  // long
    } else if (elapsed_time > 0) {
      return -1;  // short
    } else {
      return 0;
    }
}

/// Checks whether safe is open or closed based on reading button state
/// @returns true (1) if safe is CLOSED and false (0) if OPEN
bool open_close_button(){
  const unsigned long time = millis();
  static int oldState = 1;
  static unsigned long oldTime = 0;
  const int newState = digitalRead(BUTTON);

  if (newState != oldState && (time - oldTime) > DEBOUNCE_TIME){
    oldTime = time;
    oldState = newState;

    if (newState == LOW) return true;
  }

  return false;
} 

/// Checks if passcode entered is same as correct passcode
/// For loop iterates through two arrays containing user passcode and real passcode
/// @returns true if passcode correct else false
bool check_passcode(const int array1[3], int array2[3]){
    for (int i = 0; i < 3; ++i){
        if (array1[i] != array2[i]) return false;
    }
    return true;
}

/// Selection of seven segment display is made based on decimal point
/// Cycles between 1,2,3 based on rotary encoder button press (short)
void seven_seg_select(int index){

  if (index == 0){
    digitalWrite(DP1, LOW);
    digitalWrite(DP2, HIGH);
    digitalWrite(DP3, HIGH);
  } else if (index == 1) {
    digitalWrite(DP1, HIGH);
    digitalWrite(DP2, LOW);
    digitalWrite(DP3, HIGH);
  } else if (index == 2){
    digitalWrite(DP1, HIGH);
    digitalWrite(DP2, HIGH);
    digitalWrite(DP3, LOW);
  }
}

/// Displays a seconds counter on oled display
/// Latch and Latch Clear built into the function
/// @param seconds is for number of seconds remaining to be displayed on the oled
void timer_oled_display(int seconds){
    while (seconds >= 0){
        display.clearDisplay();

        display.setTextSize(2);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.print("Locked for");

        display.setTextSize(4);
        display.setCursor(40, 30);
        display.print(seconds);

        display.display();

        delay(1000);
        seconds--;
    }
}

/// Displays attempts remaining on display
/// Latch and Latch Clear built into the function
/// @param attempts is for number of attempt remaining to be displayed on the oled.
void attempts_oled_display(int attempts){
    display.clearDisplay();

    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Attempts:");

    display.setTextSize(4);
    display.setCursor(40, 30);
    display.print(attempts);

    display.display();
}

/// Loop that runs through 1, 2, 4 and 8, as inputs on the BCD
/// Perform a bitwise shift to the right
/// Performs an and gate with 1, this helps us see which is high and which is low
/// @param is decimal number to display
void bcd_display_num(int number){
    for (int i = 0; i < 4; i++){
        digitalWrite(BCD_PINS[i], (number >> i) & 1);
    }
}

/// Displays the number on seven segment display using shift register
/// Using shiftOut choses number based on DEC_DIGITS array
/// @param number1 is the first number to display
/// @param number2 is the second number to display
void sr_display_num(int number1, int number2){
    digitalWrite(SR_LAT, LOW);
    shiftOut(SR_DATA, SR_CLK,  LSBFIRST, DEC_DIGITS[number1]);
    shiftOut(SR_DATA, SR_CLK,  LSBFIRST, DEC_DIGITS[number2]);
    digitalWrite(SR_LAT, HIGH);
}

void displayDigit(int bcdNum, int srNum1, int srNum2){
  bcd_display_num(bcdNum);
  sr_display_num(srNum1, srNum2);
}

void setup() {
    Serial.begin(9600);
    const int lock_address = 20;
    s1.attach(SERVO);

    if(!display.begin(SSD1306_PAGEADDR, SCREEN_ADDRESS)) {
        Serial.println("SSD1306 allocation failed");
        for(;;);
    }

    display.display();
    delay(2000);
    display.clearDisplay();

    attempts_oled_display(3);

    pinMode(RED_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);
    pinMode(BUZZER, OUTPUT);
    pinMode(ROT_A, INPUT);
    pinMode(ROT_B, INPUT);
    pinMode(ROT_BTN, INPUT_PULLUP);
    pinMode(DP1, OUTPUT);
    pinMode(DP2, OUTPUT);
    pinMode(DP3, OUTPUT);
    pinMode(SR_CLK, OUTPUT);
    pinMode(SR_DATA, OUTPUT);
    pinMode(SR_LAT, OUTPUT);
    pinMode(BCD_A, OUTPUT);
    pinMode(BCD_B, OUTPUT);
    pinMode(BCD_C, OUTPUT);
    pinMode(BCD_D, OUTPUT);
    const int ADDRESS = 5; 
    byte num1;
    byte num2;
    byte num3;  //Passcode
     //Read the passcode in the addresses

}

void loop() {
  static int ssd = 0,  stateOfSafe = LOCKED, cooldown = 30;
  static int userPscd[3] = {0,0,0};
  static int userAttempts = 3;
  const int PASSCODE[3] = {EEPROM.read(5),EEPROM.read(6),EEPROM.read(7)};  
  if (userAttempts != 3 && userAttempts != EEPROM.read(20)){
    userAttempts = EEPROM.read(20);
    attempts_oled_display(EEPROM.read(20));
  }
  else if (userAttempts == 3 && userAttempts != EEPROM.read(20)){
    userAttempts = EEPROM.read(20);
    attempts_oled_display(EEPROM.read(20));

  }
  led_control(stateOfSafe);
  servo_control(stateOfSafe);
  if (ssd > 2) ssd = 0;
  int rotarybut = rotary_button();
  if (rotarybut == -1){
    ssd++;
  }
  else if (rotarybut == 1 && stateOfSafe == LOCKED){
    if (check_passcode(PASSCODE, userPscd)){
      stateOfSafe = UNLOCKED;
      servo_control(stateOfSafe);
      led_control(stateOfSafe);
      buzzer_control(stateOfSafe);
      cooldown = 30;
      EEPROM.write(20,3);
      attempts_oled_display(EEPROM.read(20));
    } else {

      userAttempts--;
      EEPROM.write(20, userAttempts);
      Serial.println(EEPROM.read(20));
      buzzer_control(stateOfSafe);
      attempts_oled_display(EEPROM.read(20));
  }}
  else if (rotarybut == 1 && stateOfSafe == UNLOCKED){
        Serial.println(userPscd[0]);
            Serial.println(userPscd[1]);
                Serial.println(userPscd[2]);
    writeArrayIntoEEPROM(5, (byte) userPscd[0], userPscd[1], userPscd[2]);
  }
  
  if (userAttempts == 0){
    timer_oled_display(cooldown);
    cooldown+=30;
    userAttempts = 3;
    EEPROM.write(20, userAttempts);
    attempts_oled_display(EEPROM.read(20));
  }
  int rotaryanswer = read_encoder();
  if (rotaryanswer == 1) {
    *(userPscd + ssd)-=1;
    if (*(userPscd + ssd) < 0) *(userPscd + ssd) = 9;
  }
  else if (rotaryanswer == 2) {*(userPscd + ssd)+=1;}
    if (*(userPscd + ssd) >9) *(userPscd + ssd) =0;

  if (open_close_button() && stateOfSafe == UNLOCKED){
    stateOfSafe = LOCKED;
    servo_control(stateOfSafe);
    led_control(stateOfSafe);
    attempts_oled_display(EEPROM.read(20));
  }

  displayDigit(userPscd[0], userPscd[1], userPscd[2]);
  seven_seg_select(ssd);



}