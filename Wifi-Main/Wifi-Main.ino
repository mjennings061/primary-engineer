/* Some code for timers was found and heabily edited here: https://forum.arduino.cc/index.php?topic=422821.0
    This code has now been heavily edited after consulting header files/datasheet etc.
    
    Relevant headers can be found in windows @ a folder similar to below, but whenever there are updates to Arduino IDE this changes so only use as a guide:
    c:\Users\USERNAME\AppData\Local\Arduino15\packages\arduino\tools\CMSIS\4.0.0-atmel\Device\ATMEL\samd21\include\....
    
    Useful folders are:
    ...\component\ (definition of software api, i.e. class and struct definitions for different peripherals particularly useful files are: gclk.h (clock generation register setting values) & tc.h (timer register setting values)
    & ...\instance\ (definition of the actual device peripherals and their register addresses, in this case particularly useful files are: gclk.h, tc3.h, & tc4.h)
    Another useful header file here is: samd21g18a.h (Part Number of IC in the MKR1000)

    SAMD21 Family Datasheet can be found here, this is useful for knowing order to set registers, what peripherals are available without having to read through header files: 
    https://www.microchip.com/wwwproducts/en/ATSAMD21G18

    Pin 0 switch is pulled up to Vcc pin on MKR1000 with a 1K resistor for interrupt debug using a switch. 
    100nF Cap was used on Pin 0 switch in tested set-up for hardware based pin debounce.

    *** TODO ***
    - Would like to add ability to sense when a car has already passed second sensor and send signal to stop flashing, this would require tracking the number of vehicles etc.
    - Have multiple devices, this may require us of WiFiNINA library, this would not be trivial.
*/

#include <SPI.h>
#include <WiFi101.h>
#include <Adafruit_NeoPixel.h>
#include <WiFiUDP.h>


// Declare Pins for LED output etc.
#define HOST 6
#define LED 7
#define PIR 0
#define TIME A1 // A pot connected to this pin will determine the time the roadflash will flash for, these must be set to the same value for both units. Serial printout at startup will allow you to determine if they match


/********** Timer IRQ Variables **********/
volatile int Colour = 0; // Used by Timer 3 IRQ to track what colour the Jewel is.
long Timer_Value; // Used to set Timer 4 overflow value.


/********** WiFi/UDP Setup **********/
char ssid[] = "roadflash-host";        // your network SSID (name)

WiFiUDP Udp;

char packetBuffer[255]; //buffer to hold incoming packet
char  ReplyBuffer[] = "acknowledged";       // a string to send back

unsigned int localPort = 2390;      // local port to listen on

int Host; // Set at startup to check whether the unit will be hosting the WiFi AP or a device connecting to it

int status = WL_IDLE_STATUS;


/************ Neopixel Setup ***************/
// How many NeoPixels are attached to the Arduino?
#define ledCount  7

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(ledCount, LED, NEO_GRB + NEO_KHZ800);

// Declare the neopixel colours so the Timer IRQs can use them
volatile uint32_t black = strip.Color(0, 0, 0);
volatile uint32_t red = strip.Color(255, 0, 0);
volatile uint32_t green = strip.Color(0, 255, 0);
volatile uint32_t blue = strip.Color(0, 0, 255);
volatile uint32_t amber = strip.Color(255, 100, 0);
volatile uint32_t aqua = strip.Color(0, 255, 255);
volatile uint32_t purple = strip.Color(255, 0, 255);
volatile uint32_t white = strip.Color(255, 255, 255);




void vehicle_detection_ISR() {
  // Vehicle is detected as PIR has been triggered, flag this.
  delayMicroseconds(20); // Software debounce

  if (!(digitalRead(PIR))){ // Software debounce - check that the pin is still low, if it is, sned a message to the other device
    if(Host){ // If the unit is the host send to the device IP address
      noInterrupts(); // Ensure that the sending of the message to the other unit is not interrupted
      
      Udp.beginPacket(IPAddress(192, 168, 1, 5), 2390);
      Udp.write("Triggered");
      Udp.endPacket();

      interrupts();

      // Begin counting on timer 4, waiting for Channel Compare 0 interrupt to flag, I chose to write to the registers here rather than using a function to save time in the interrupt
      REG_TC4_INTENCLR = TC_INTENCLR_MC0;     // Disable TC4 CC0 overflow interrupt to reset value (Counter value is WP when TC4 interrupt is enabled)
      REG_TC4_COUNT32_COUNT = 0x00;   // Reset Counter value
      while (TC4->COUNT32.STATUS.bit.SYNCBUSY);        // Wait for registers to be written
      
      REG_TC4_INTENSET = TC_INTENSET_MC0;     // Enable TC4 CC0 overflow interrupt
      while (TC4->COUNT32.STATUS.bit.SYNCBUSY);        // Wait for registers to be written

      detachInterrupt(digitalPinToInterrupt(PIR)); // Detach the PIR interrupt so we don't exit in the middle of something 
      }
      
    else if(!Host){ // If the unit is the device send to the host IP address
      noInterrupts(); // Ensure that the sending of the message to the other unit is not interrupted
      
      Udp.beginPacket(IPAddress(192, 168, 1, 1), 2390);
      Udp.write("Triggered");
      Udp.endPacket();
      
      interrupts();
      
      // Begin counting on timer 4, waiting for Channel Compare 0 interrupt to flag, I chose to write to the registers here rather than using a function to save time in the interrupt
      REG_TC4_INTENCLR = TC_INTENCLR_MC0;     // Disable TC4 CC0 overflow interrupt to reset value (Counter value is WP when TC4 interrupt is enabled)
      REG_TC4_COUNT32_COUNT = 0x00;   // Reset Counter value
      while (TC4->COUNT32.STATUS.bit.SYNCBUSY);        // Wait for registers to be written
      
      REG_TC4_INTENSET = TC_INTENSET_MC0;     // Enable TC4 CC0 overflow interrupt
      while (TC4->COUNT32.STATUS.bit.SYNCBUSY);        // Wait for registers to be written
      
      detachInterrupt(digitalPinToInterrupt(PIR)); // Detach the PIR interrupt so we don't exit in the middle of something important
      }
  }
}



void setup() {
  Serial.begin(115200);
  
  // Setup Neopixel
  setupLed();
  testLed();
  Serial.println("NeoPixel Ready");
  
  pinMode(HOST, INPUT); // Check the HOST pin to see if the unit is set as the Host or the device.
  Host = digitalRead(HOST);

  pinMode(TIME, INPUT);
  int Flash_Time_Coeff = analogRead(TIME); // Get Pot. value to set timer value (amount of time to flash for).
  Timer_Value = (382 * Flash_Time_Coeff) + 78125; // Using the analog sensor value, set the time to flash for based on the calculated by analog sensor range (0--1024) and possible timer reg. values (78125(5 sec)--468750(30 sec)).
  Serial.println(Timer_Value);
  Serial.println(" - Timer Register Value");

  
  // **** WIFI/UDP ****
  Serial.print("WiFi Firmware Version: ");
  Serial.println(WiFi.firmwareVersion());
  Serial.println("Running WiFi set-up");
  WiFi_setup();
  Serial.println("WiFi set-up complete");
  
  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);
  

  // **** TIMER 4 - Time to flash the light for ****
  Serial.println("Running Timer 4 set-up");
  timer_4_setup();
  Serial.println("Timer 4 set-up complete");

  // **** TIMER 3 - Flashing the light ****
  Serial.println("Running Timer 3 set-up");
  timer_3_setup();
  Serial.println("Timer 3 set-up complete");
  

  // **** INTERRUPT ****
  // Assign vehicle detection interrupt to the PIR pin, when pin pulled LOW
  attachInterrupt(digitalPinToInterrupt(PIR), vehicle_detection_ISR, LOW); 
}


void loop() {
  if (Host){ // As the host is the AP, only run this code if we are the host.
    // compare the previous WiFi status to the current WiFi status
    if (status != WiFi.status()) {
      // it has changed update the variable
      status = WiFi.status();

    if (status == WL_AP_CONNECTED) {
      byte remoteMac[6];

      // a device has connected to the AP
      Serial.print("Device connected to AP, MAC address: ");
      WiFi.APClientMacAddress(remoteMac);
      printMacAddress(remoteMac);
    } 
    
    else {
      // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
      }
    } 
  }
  
  // Whether the device is the host or not, we want to recieve messages. If there's data available, read the packet
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    noInterrupts();
    
    Serial.print("Received From ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    
    if(strstr(packetBuffer, "Triggered")){ // Check if it's a 'Triggered' message from the other unit
      detachInterrupt(digitalPinToInterrupt(PIR)); // Detach the PIR interrupt so that we don't trigger when the car passes, the timer interrupt will reattach it again.

      Activate_Timers(); // Turn on the timers and begin flashing.
      
      Serial.println("Timers Active");
    }
    interrupts();
  }
 
}
