/*
 * Primary Engineer - Road Flash Project
 * Authors - Michael Jennings, Andrew Wilson, Jordan Smart
 * 
 * Road-narrowing oncoming traffic warning system
 * Compiled for the Arduino Nano
 * 
 * PIR Sensor Info:
 * https://wiki.seeedstudio.com/Grove-PIR_Motion_Sensor/#resources
 * R13 replaced with 1M R - Hald default sensitivity
 * R14 replaced with 10K R - Max range
 */

#include <SPI.h>
#include <RH_RF69.h>
#include <Adafruit_NeoPixel.h>


#define LED           7 // Neopixel Data is on Pin D7
#define PIR           3 // PIR Output is on Pin D3


/************ Radio Setup ***************/
// https://learn.adafruit.com/adafruit-rfm69hcw-and-rfm96-rfm95-rfm98-lora-packet-padio-breakouts/using-the-rfm69-radio
#define RF69_FREQ 434.0
#define RFM69_CS      9
#define RFM69_INT     2
#define RFM69_RST     8
//#define RFM69_LED     13

RH_RF69 rf69(RFM69_CS, RFM69_INT); // Singleton instance of the radio driver
int16_t packetnum = 0;  // packet counter, we increment per xmission

/********** Variables for tracking vehicle **********/
volatile boolean trigger = 0;  // 1 when the PIR or compass is triggered
volatile boolean flashing = 0;   // LED currently flashing? 1=true
boolean rxTrigger = 0;  // received trigger via RF


/************ Neopixel Setup ***************/
// How many NeoPixels are attached to the Arduino?
#define ledCount  7

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(ledCount, LED, NEO_GRB + NEO_KHZ800);

// Declare the neopixel colours so the IRQ can use them
volatile uint32_t black = strip.Color(0, 0, 0);
volatile uint32_t red = strip.Color(255, 0, 0);
volatile uint32_t green = strip.Color(0, 255, 0);
volatile uint32_t blue = strip.Color(0, 0, 255);
volatile uint32_t amber = strip.Color(255, 100, 0);
volatile uint32_t aqua = strip.Color(0, 255, 255);
volatile uint32_t purple = strip.Color(255, 0, 255);
volatile uint32_t white = strip.Color(255, 255, 255);



/********** IRQ Variables **********/
volatile int Colour = 0; // Used by Timer 2 IRQ to track what colour the Jewel is.

// Timer 2 overflows counter, the first one is used to count up to 0.5 Seconds, the second is used to count up to 30 Seconds
volatile int timer_count = 0;
volatile int timer_count_2 = 0;




void setup() {
  Serial.begin(115200);
  while(!Serial){
    ;
  }
  
  rf69_setup();
  Serial.println("RF Module OK");


  Timer_2_Setup(); //Timer 2 setup (Timer 2 is not used by anything)
  
  // Setup PIR interrupt (PIR will provide a rising edge)
  pinMode(PIR, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIR), triggered, RISING);
  Serial.println("PIR Interrupt Attached");
  
  // Setup Neopixel
  setupLed();
  testLed();  
  Serial.println("NeoPixel Ready");
}




/********** MAIN **********/
void loop(){
  sendWhenTriggered();  // check for a local trigger to broadcast trigger
  
  rxTrigger = rf69_rxTrigger(); // check if a 'triggered' message has been received

  if (rxTrigger){ // If a trigger has been received, start timer IRQ
    Serial.println("RX trigger"); // Debug
    rxTrigger = 0; // Set rxTrigger back to 0 or timer will keep being reset.
    TIMSK2 |= (1 << OCIE2A); // Start 30 sec timer with neopixel flashing every 500mS (Uses Counter Compare as it has setup in Timer setup, we're simply enabling the IF using one bit in a reg.)
  }
}





/********** Timer 2 ISR **********/
ISR(TIMER2_COMPA_vect){
  /* There are two variables used to actually track time here as timer 2 is only an 8 bit timer.
   *  One will count up to 500mS then swtich the neopixel on/off, and the other will count up to 30 Seconds then disable the IRQ
   *  The freq. after prescaler of the CLK is ~61MHz or a trig. every ~16.4mS: 500/16.4 = ~30 (Trigger 30 times before 0.5S has passed).
   *  Then if we want to turn off after 30S we want to see our 0.5S trigger occure a total of 30/0.5 = 60 times.*/
   
  if (timer_count < 30){ // 0.5 Seconds hasn't passed yet so lets increment our counter.
    timer_count++; 
  }

  else if (timer_count_2 < 60){ //0.5 Seconds passed so lets increment our 30 Second counter and reset the 0.5 Second one
    timer_count_2++;
    timer_count = 0;

    // Also lets change the neopixel colour to red if its off...
    if(!Colour){
      setStrip(red);
      Colour = 1;
    }

    // And off if its red
    else{
      setStrip(black);
      Colour = 0;
    }    
  }

  else{
    // 30 Seconds has passed to lets reset and disable everything in the IRQ etc. 
    Colour = 0;
    setStrip(black);
    
    timer_count = 0;
    timer_count_2 = 0;
    
    flashing = 0; // Reset the Flashing variable so that we can trigger on detect and send a message to the other Roadflash
    TIMSK2 = 0;
  }
}


// PIR ISR
void triggered(void) {
  trigger = 1; // Very simple here set this varibale to one, the main loop will check 'trigger' and send a message if this variable is 1
}




// check if the hardware interrupt is triggered and send an RF char
void sendWhenTriggered(void) {
  detachInterrupt(digitalPinToInterrupt(PIR)); // Detach the PIR interrupt so we don't exit in the middle of something important
  if (trigger && !flashing) { 
    //If we're not flashing and trigger is active, send our String to the other unit and if it was successful reset the trigger
    char toSend[20] = "Triggered";
    boolean success = rf69_sendChar(toSend);
    if(success){
      trigger = 0;
    }
  }
  
  else{
    trigger = 0;
  }
  attachInterrupt(digitalPinToInterrupt(PIR), triggered, FALLING); // Reattach out interrupt now that we're done
}


boolean rf69_rxTrigger(void) {
  if (rf69.available()) {
    // Should be a message for us now, set flashing so that we don't send any triggers to the other unit.
    flashing = 1;
    
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN]; // Buffer for received message.
    uint8_t len = sizeof(buf);
    
    if (rf69.recv(buf, &len)) {
      if (!len){ // If there's NULL or no message, also any other possibilities exit out of loop
        return 0;
      }
      
      // Print out our message and where it came from.
      buf[len] = 0;
      Serial.print("Received [");
      Serial.print(len);
      Serial.print("]: ");
      Serial.print((char*)buf);

      Serial.print("   RSSI: ");
      Serial.println(rf69.lastRssi(), DEC);

      // If we recieved the "Triggered" message we want to send an acknowledge and exit back to main.
      if (strstr((char*)buf, "Triggered")) {
        sendAcknowledge();
        return 1;
      }

      // Error with the content of message - or its not "Triggered" so let's exit.
      else {
        return 0;
      }
    }

    // Something has went wrong
    else {
      Serial.println("\t Receive failed");
      return 0;
    }
  }

  // No message recieved, let's exit.
  else {
    return 0;
  }
}


void sendAcknowledge(void){
  // Send an acknowledge message of INT '1'
  uint8_t data[] = "1";
  rf69.send(data, sizeof(data));
  rf69.waitPacketSent();
  Serial.println("Sent Acknowledge"); // Debug
}




boolean rf69_sendChar(char radiopacket[20]) {
  itoa(packetnum++, radiopacket + 13, 10);
  Serial.print("Sending: "); 
  Serial.println(radiopacket);

  // Send a message!
  rf69.send((uint8_t *)radiopacket, strlen(radiopacket));
  rf69.waitPacketSent();

  // Now wait for a reply
  uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  
  if (rf69.waitAvailableTimeout(500))  {
    // Should be a reply message for us now
    if (rf69.recv(buf, &len)) {
      Serial.print("\Acknowledge Recieve: ");
      Serial.println((char*)buf);
      return 1;
    } 
    
    else {
      Serial.println("\t Receive failed");
      return 0;
    }
  } 
  
  else {
    Serial.println("\t No reply, is another RFM69 listening?");
    return 0;
  }
}


void rf69_setup(void) {
  //  pinMode(RFM69_LED, OUTPUT);
  
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather RFM69 TX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
                  };
  rf69.setEncryptionKey(key);

  //  pinMode(RFM69_LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}
