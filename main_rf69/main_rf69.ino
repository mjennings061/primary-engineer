/*
 * Primary Engineer - Road Flash Project
 * Authors - Michael Jennings, Andrew Wilson, Jordan Smart
 * 
 * Road-narrowing oncoming traffic warning system
 * Compiled for the Arduino Nano
 */

#include <SPI.h>
#include <RH_RF69.h>

/************ Radio Setup ***************/
//https://learn.adafruit.com/adafruit-rfm69hcw-and-rfm96-rfm95-rfm98-lora-packet-padio-breakouts/using-the-rfm69-radio
#define RF69_FREQ 434.0
#define RFM69_CS      9
#define RFM69_INT     2
#define RFM69_RST     8
//#define RFM69_LED     13
#define LED           7
#define PIR           3 // INT1 (D3)

RH_RF69 rf69(RFM69_CS, RFM69_INT); // Singleton instance of the radio driver
int16_t packetnum = 0;  // packet counter, we increment per xmission
volatile boolean trigger = 0;  // 1 when the PIR or compass is triggered
boolean flashing = 0;   // LED currently flashing? 1=true
boolean firstFlash = 0;
boolean rxTrigger = 0;  // received trigger via RF

// LED flashing
unsigned long ledMillisPrevious = 0;
const long interval = 250;
boolean ledState = 0;
// LED Timeout
unsigned long timeoutMillisPrevious = 0;
const long timeoutInterval = 10000;
boolean timeout = 0;


void setup() {
  Serial.begin(115200);
  rf69_setup();
  pinMode(LED, OUTPUT);
  digitalWrite(LED, 0);
  pinMode(PIR, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIR), triggered, RISING);
}


void loop() {
  sendWhenTriggered();  // check for a local trigger to broadcast trigger
  rxTrigger = rf69_rxTrigger(); // check if a 'triggered' message has been received
  if (rxTrigger) { // if a trigger has been received
    flashing = 1; // start flashing
  }
  ledControl(); // control LED flashing
}








// check if the hardware interrupt is triggered and send an RF char
void sendWhenTriggered(void) {
  if (trigger) {
    detachInterrupt(digitalPinToInterrupt(PIR));
    if (!flashing) { // only tell the other device to flash when currently off
      char toSend[20] = "Triggered";
      boolean success = rf69_sendChar(toSend);
    }
    trigger = 0;
    attachInterrupt(digitalPinToInterrupt(PIR), triggered, FALLING);
  }
}


// control LED flashing
void ledControl(void) {
  if (flashing) { //if the system has been told to start flashing
    if (firstFlash) { //if it is the first flash of LED
      timeoutMillisPrevious = millis();
      firstFlash = 0;
    }
    // turn off the LED if it has been on too long
    unsigned long timeoutMillisCurrent = millis();
    if (timeoutMillisCurrent - timeoutMillisPrevious >= timeoutInterval) {
      timeoutMillisPrevious = timeoutMillisCurrent;
      flashing = 0;
    }

    // flash LED using millis()
    unsigned long ledMillisCurrent = millis();
    if (ledMillisCurrent - ledMillisPrevious >= interval) {
      // save the last time you blinked the LED
      ledMillisPrevious = ledMillisCurrent;
      // if the LED is off turn it on and vice-versa:
      if (ledState == LOW) {
        ledState = HIGH;
      } else {
        ledState = LOW;
      }
      // set the LED with the ledState of the variable:
      digitalWrite(LED, ledState);
    }//end if interval
  } else {
    firstFlash = 1;
    digitalWrite(LED, 0);
  }//end else flashing
}//end ledControl


boolean rf69_rxTrigger(void) {
  if (rf69.available()) {
    // Should be a message for us now
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN]; // received message buffer
    uint8_t len = sizeof(buf);
    if (rf69.recv(buf, &len)) {
      if (!len) return 0;
      buf[len] = 0;
      Serial.print("Received [");
      Serial.print(len);
      Serial.print("]: ");
      Serial.print((char*)buf);
      Serial.print("\t RSSI: ");
      Serial.print(rf69.lastRssi(), DEC);

      if (strstr((char *)buf, "Triggered")) {
        sendAck();
        return 1;
      } else {
        return 0;
      }
    } else {
      Serial.println("\t Receive failed");
      return 0;
    }
  } else {
    return 0;
  }
}


void sendAck(void) {
  // Send ACK!
  uint8_t data[] = "1";
  rf69.send(data, sizeof(data));
  rf69.waitPacketSent();
  Serial.println("\t Sent ACK");
  //  Blink(RFM69_LED, 40, 3); //blink LED 3 times, 40ms between blinks
}


// ISR
void triggered(void) {
  trigger = 1;
  Serial.println("Trigger");
}


boolean rf69_sendChar(char radiopacket[20]) {
  itoa(packetnum++, radiopacket + 13, 10);
  Serial.print("Sending "); Serial.print(radiopacket);

  // Send a message!
  rf69.send((uint8_t *)radiopacket, strlen(radiopacket));
  rf69.waitPacketSent();

  // Now wait for a reply
  uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  if (rf69.waitAvailableTimeout(500))  {
    // Should be a reply message for us now
    if (rf69.recv(buf, &len)) {
      Serial.print("\t ACK ");
      Serial.println((char*)buf);
      //      Blink(RFM69_LED, 50, 3); //blink RFM69_LED 3 times, 50ms between blinks
      return 1;
    } else {
      Serial.println("\t Receive failed");
      return 0;
    }
  } else {
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

void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i = 0; i < loops; i++)  {
    digitalWrite(PIN, HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN, LOW);
    delay(DELAY_MS);
  }
}
