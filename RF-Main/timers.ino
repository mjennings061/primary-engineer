/*Please refere to MicroChip datasheet: http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
 * And, google (Or other generic search engine)... A lot */




void Timer_2_Setup(void){
  noInterrupts();// We don;t want to be interrupted babay...

    //Set timer2 interrupt at ~61Hz
    TCCR2A = 0;// Set entire TCCR1A register to 0
    TCCR2B = 0;// Same for TCCR1B
    TCNT2  = 0;// Initialize counter value to 0
    
    // Set compare register value to maximum
    OCR2A = 255;// = (16*10^6) / (3*1024) - 1 (must be <65536)

    // Set the pre-scaler to 1024
    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);  
    // enable timer compare interrupt
    
    Serial.println("Timer 2 Programmed");

  interrupts();// Allow interrupts again now that we're done
}
