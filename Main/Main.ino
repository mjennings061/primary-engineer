/*Some base code was taken from https://forum.arduino.cc/index.php?topic=422821.0
    This code has now been heavily edited after consulting header files/datasheet etc.
    
    Relevant headers can be found in windows @:
    c:\Users\USERNAME\AppData\Local\Arduino15\packages\arduino\tools\CMSIS\4.0.0-atmel\Device\ATMEL\samd21\include\....
    Useful folders are ...\component\ (definition of software api, particularly gclk.h & tc.h)
    & ...\instance\ (definition of the actual device peripherals, particularly gclk.h & tc4.h)
    Another useful header file here is: samd21g18a.h (Part Number of IC in the MKR1000)

    SAMD21 Family Datasheet can be found here: https://www.microchip.com/wwwproducts/en/ATSAMD21G18

    Pin 0 switch is pulled up to Vcc pin on MKR1000 with a 1K resistor.    
    100nF Cap was used on Pin 0 switch in tested set-up for hardware based pin debounce.

    *** TODO ***
    - Would like to add potentiometer input to vary the timer interrupt period for flashing to end, or allow code to be edited to do so (in terms of seconds)
    - Wi-Fi connectivity, set-up, etc.
    - Would like to add ability to sense when a car has already passed second sensor and send signal to stop flashing, this would require tracking the number of vehicles etc.
*/

#include <SPI.h>
#include <WiFi101.h>


// Declare Pins for LED output etc.
const int HOST = 7;
const int LED = 2;
const int PIR = 0;

// Declare PIR interrupt flag variable so it can be accessed globally
volatile int PIR_Detection_Flag = 0;


char ssid[] = "roadflash-host";        // your network SSID (name)
char pass[] = "password";   // your network password (use for WPA, or use as key for WEP)
//int keyIndex = 0x4567;                // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;


void vehicle_detection_ISR() {
  
  // Vehicle is detected as PIR has been triggered, flag this.
  PIR_Detection_Flag = 1;

  Serial.println("Vehicle detected"); // Debug

  // Begin counting on timer 4, waiting for Channel Compare 0 interrupt to flag
  REG_TC4_INTENCLR = TC_INTENCLR_MC0;     // Disable TC4 CC0 overflow to reset value (Counter value is WP when TC4 interrupt is enabled)
  REG_TC4_COUNT32_COUNT = 0x00;   // Reset Counter value
  REG_TC4_INTENSET = TC_INTENSET_MC0;     // Enable TC4 CC0 overflow interrupt
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);        // Wait for registers to be written
  
  Serial.println("Interrupt Reset & Enabled"); // Debug
}





void setup() {
  //Assign LED pin and PIR pin
  pinMode(LED, OUTPUT);
  pinMode(PIR, INPUT);
  pinMode(HOST, INPUT);

  Serial.begin(9600);

  while(!Serial){
    ;
  }
  
  
  
  // **** WIFI ****
  Serial.println("Running WiFi set-up");
  WiFi_setup();
  Serial.println("WiFi set-up complete");


  // **** TIMER 4****
  Serial.println("Running Timer set-up");
  timer_4_setup();
  Serial.println("Timer set-up complete");


  // **** INTERRUPT - Remove later ****
  // Assign vehicle detection interrupt to the PIR pin, when pin pulled LOW
  attachInterrupt(digitalPinToInterrupt(PIR), vehicle_detection_ISR, LOW);

  
  
  // Indicate start-up / set-up of device is complete with LED.
  digitalWrite(LED, HIGH);
  delay(2000);
  digitalWrite(LED, LOW);  
}


void loop() {

  // If the PIR has been triggered and flag is raised then flash the LED
  while (PIR_Detection_Flag == 1){
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
  }
 
}
