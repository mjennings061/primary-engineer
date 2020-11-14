/*  Updated: 14/11/2020
 *  Author: Jordan Smart
 *  https://wiki.seeedstudio.com/Grove-PIR_Motion_Sensor/#resources
 * 
 *  R13 replaced with 1M R - Hald default sensitivity
 * 
 *  R14 replaced with 10K R - Max range
 * 
 */
 
#define DEBUG_LED_PIN 3
// The compiler will replace any mention of PIR_PIN with the value 3 at compile time.
#define PIR_PIN 4

// DEBUG Control
#define DEBUG

// Allows DEBUG terminal output using DEBUG_PRINT("TEXT")
#ifdef DEBUG
 #define DEBUG_PRINT(x)  Serial.println (x)
 #define DEBUG_LED(y,z) digitalWrite(y,z)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_LED(z,y)
#endif

void PIR() {
  if(digitalRead(PIR_PIN)==HIGH){
    // do something
    DEBUG_LED(3, HIGH);
    DEBUG_PRINT("PIR_DET");
  }
  else{
    // do nothing   
    DEBUG_LED(3, LOW);
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(DEBUG_LED_PIN, OUTPUT);
  pinMode(PIR_PIN, INPUT);
}

void loop() {
  PIR();
  delay(200);
}
