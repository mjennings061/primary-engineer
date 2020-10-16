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

// Declare Pins for LED output etc.
const int LED = 2;
const int PIR = 0;

// Declare PIR interrupt flag variable so it can be accessed globally
volatile int PIR_Detection_Flag = 0;



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




void TC4_Handler()    // Interrupt Service Routine (ISR) for timer TC4
{
  // Check for match counter compare channel 0 (MC0) interrupt
  if (TC4->COUNT32.INTFLAG.bit.MC0 && TC4->COUNT32.INTENSET.bit.MC0)             
  {
    // Stop the LED from blinking as PIR hasn;t triggered in 20 seconds
    PIR_Detection_Flag = 0;

    Serial.println("Timer interrupt has occurred"); // Debug

    
    REG_TC4_INTENCLR = TC_INTENCLR_MC0;   // Disable the CC0 channel interrupt
    REG_TC4_INTFLAG = TC_INTFLAG_MC0;         // Clear the MC0 interrupt flag
    while (TC4->COUNT32.STATUS.bit.SYNCBUSY);        // Wait for synchronisation of registers
  }
}


void timer_setup() {
   // Set up the generic clock (GCLK4) used to clock timers
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(3) |          // Divide the 48MHz clock source by divisor 3: 48MHz/3=16MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  Serial.println("GLK was setup");

  // Feed GCLK4 to TC4 and TC5
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TC4 and TC5
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TC4_TC5;     // Feed the GCLK4 to TC4 and TC5
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
 
  REG_TC4_CTRLA |= TC_CTRLA_MODE_COUNT32;           // Set the counter to 32-bit mode - This will use both TC4 & TC5 to count 
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);        // Wait for synchronization


  REG_TC4_COUNT32_CC0 = 0x7270E;                  // Set the counter top register to trigger after 30 sec, 15625Hz (below) * 30 = 468750 = 0x7270E
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);        // Wait for synchronization
  

  Serial.println("Timer set-up complete. Enabling interrupt"); // Debug
  
  NVIC_SetPriority(TC4_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC4 to 0 (highest)
  NVIC_EnableIRQ(TC4_IRQn);         // Connect TC4 to Nested Vector Interrupt Controller (NVIC)

  REG_TC4_INTFLAG |= TC_INTFLAG_MC1 | TC_INTFLAG_MC0 | TC_INTFLAG_OVF;        // Clear the interrupt flags to be safe
  REG_TC4_INTENCLR = TC_INTENCLR_MC0;     // Disable TC4 CC0 interrupt

 
  REG_TC4_CTRLA |= TC_CTRLA_PRESCALER_DIV1024 |     // Set prescaler to 1024, 48MHz/3(GCLK divider)/1024(prescaler) = 15625 Hz
                   TC_CTRLA_ENABLE;               // Enable TC4
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);        // Wait for synchronization
  
  Serial.println("Timer interrupt set-up complete");
}






void setup() {
  //Assign LED pin and PIR pin
  pinMode(LED, OUTPUT);
  pinMode(PIR, INPUT);

  Serial.begin(9600);

  

  Serial.println("Running Timer set-up");
  timer_setup();

  // Indicate start-up of device with LED.
  digitalWrite(LED, HIGH);
  delay(2000);
  digitalWrite(LED, LOW);

  // Assign vehicle detection interrupt to the PIR pin, when pin pulled LOW
  attachInterrupt(digitalPinToInterrupt(PIR), vehicle_detection_ISR, LOW);
  
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
