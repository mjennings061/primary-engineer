void Activate_Timers(){ // Used in the main loop when a triggered message is recieved to activate the timers for flashing the neopixel
  // Begin counting on timer 4, waiting for Channel Compare 0 interrupt to flag
  REG_TC4_INTENCLR = TC_INTENCLR_MC0;     // Disable TC4 CC0 overflow interrupt to reset value (Counter value is WP when TC4 interrupt is enabled)
  REG_TC4_COUNT32_COUNT = 0x00;   // Reset Counter value
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);        // Wait for registers to be written
      
  REG_TC4_INTENSET = TC_INTENSET_MC0;     // Enable TC4 CC0 overflow interrupt
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);        // Wait for registers to be written
      
  // Begin counting on timer 6, this is where the flashing of neopixel is handled
  REG_TC3_INTENCLR = TC_INTENCLR_MC0;     // Disable TC3 CC0 overflow to reset value (Counter value is WP when TC3 interrupt is enabled)
  REG_TC3_COUNT16_COUNT = 0x00;   // Reset Counter 3 value
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for registers to be written

  REG_TC3_INTENSET = TC_INTENSET_MC0;     // Enable TC3 CC0 overflow interrupt
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for registers to be written
}


void TC4_Handler()    // Interrupt Service Routine (ISR) for timer 4/5 - TC4
{
  // Check for match counter compare channel 0 (MC0) interrupt
  if (TC4->COUNT32.INTFLAG.bit.MC0 && TC4->COUNT32.INTENSET.bit.MC0)             
  {
    REG_TC3_INTENCLR = TC_INTENCLR_MC0;   // Disable the CC0 channel interrupt for timer 3 as 30 seconds has passed
    REG_TC3_COUNT16_COUNT = 0x00;   // Reset Counter 3 value
    REG_TC3_INTFLAG = TC_INTFLAG_MC0;         // Clear the MC0 interrupt flag for timer 3 for good measure
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronisation of TC3 registers (wait for the register to be written to)
    
    REG_TC4_INTENCLR = TC_INTENCLR_MC0;   // Disable the CC0 channel interrupt for this timer (4/5)
    REG_TC3_COUNT16_COUNT = 0x00;   // Reset Counter 3 value
    while (TC4->COUNT32.STATUS.bit.SYNCBUSY);        // Wait for synchronisation of TC4 registers

    setStrip(black); //Make sure the neopixel is turned off

    attachInterrupt(digitalPinToInterrupt(PIR), vehicle_detection_ISR, LOW); // Re-attach the vehicle detection interrupt as 30 sec has passed.
    
    REG_TC4_INTFLAG = TC_INTFLAG_MC0;         // Clear the MC0 interrupt flag for this timer (again, 4/5)
    while (TC4->COUNT32.STATUS.bit.SYNCBUSY);        // Wait for synchronisation of TC4 registers
  }
}




void TC3_Handler()    // Interrupt Service Routine (ISR) for timer TC3, this is as short as I could make it.
{
  // Check for match counter compare channel 0 (MC0) interrupt, and that it is supposed to be enabled in the first place
  if (TC3->COUNT16.INTFLAG.bit.MC0 && TC3->COUNT16.INTENSET.bit.MC0)             
  {

    // Also lets change the neopixel colour to red if its off...
    if(!Colour){
      setStrip(red);
      Colour = 1;
    }

    // ...and off if its red
    else{
      setStrip(black);
      Colour = 0;
    }
    
    REG_TC3_INTENCLR = TC_INTENCLR_MC0;     // Disable TC3 CC0 overflow to reset value (Counter value is WP when TC3 interrupt is enabled)
    REG_TC3_COUNT16_COUNT = 0x00;   // Reset Counter 3 value
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronisation of registers (wait for the register to be written to)

    REG_TC3_INTENSET = TC_INTENSET_MC0;     // Enable TC3 CC0 overflow interrupt
    REG_TC3_INTFLAG = TC_INTFLAG_MC0;         // Clear the MC0 interrupt flag for timer 3 to exit
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for registers to be written
    }
}




/********** TIMER 4 SETUP **********/
void timer_4_setup() {
  noInterrupts();
   // Set up the generic clock (GCLK4) used to clock timers
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(3) |          // Divide the 48MHz clock source by divisor 3: 48MHz/3=16MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  Serial.println("CLK Generator for timer 4 setup complete");

  // Feed GCLK4 to TC4 and TC5
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TC4 and TC5
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TC4_TC5;     // Feed the GCLK4 to TC4 and TC5
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
 
  REG_TC4_CTRLA |= TC_CTRLA_MODE_COUNT32;           // Set the counter to 32-bit mode - This will use both TC4 & TC5 to count 
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);        // Wait for synchronization


  REG_TC4_COUNT32_CC0 = Timer_Value;                  // Set the counter top register to trigger after 30 sec, 15625Hz (below) * TIME, for 30 seconds = 468750 and for 5 seconds = 78125. This is calculated from pot. reading at startup.
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);        // Wait for synchronization
  

  Serial.println("Timer 4 set-up complete. Enabling interrupt"); // Debug
  
  NVIC_SetPriority(TC4_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC4 to 0 (highest)
  NVIC_EnableIRQ(TC4_IRQn);         // Connect TC4 to Nested Vector Interrupt Controller (NVIC)

  REG_TC4_INTFLAG |= TC_INTFLAG_MC1 | TC_INTFLAG_MC0 | TC_INTFLAG_OVF;        // Clear the interrupt flags to be safe
  REG_TC4_INTENCLR = TC_INTENCLR_MC0;     // Disable TC4 CC0 interrupt

 
  REG_TC4_CTRLA |= TC_CTRLA_PRESCALER_DIV1024 |     // Set prescaler to 1024, 48MHz/3(GCLK divider)/1024(prescaler) = 15625 Hz
                   TC_CTRLA_ENABLE;               // Enable TC4
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);        // Wait for synchronization
  
  Serial.println("Timer interrupt set-up complete");

  interrupts();
}




/********** TIMER 3 SETUP **********/
void timer_3_setup() {
  noInterrupts();
  
  // Set up the generic clock (GCLK5) used to clock timers
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(3) |          // Divide the 48MHz clock source by divisor 3: 48MHz/3=16MHz
                    GCLK_GENDIV_ID(5);            // Select Generic Clock (GCLK) 5
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization


  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK5
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(5);          // Select GCLK5
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  Serial.println("GLK for timer 3 was setup");

  // Feed GCLK5 to TC2 and TC3
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK5 to TC2 and TC3
                     GCLK_CLKCTRL_GEN_GCLK5 |     // Select GCLK5
                     GCLK_CLKCTRL_ID_TCC2_TC3;     // Feed the GCLK5 to TC2 and TC3
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
 
  REG_TC3_CTRLA |= TC_CTRLA_MODE_COUNT16;           // Set the counter to 16-bit mode - This will use only TC3 as a 16 bit counter (unlike above where I have set a 32 bit counter up) 
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization


  REG_TC3_COUNT16_CC0 = 7812;                  // Set the counter top register to trigger after 0.5 sec, 15625Hz (below) * TIME, for 0.5 seconds = 7812.5 (0x1E84 in hex.)
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization
  

  Serial.println("Timer 3 set-up complete. Enabling interrupt"); // Debug
  
  NVIC_SetPriority(TC3_IRQn, 4);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC3 to 1 (second highest)
  NVIC_EnableIRQ(TC3_IRQn);         // Connect TC4 to Nested Vector Interrupt Controller (NVIC)

  REG_TC3_INTFLAG |= TC_INTFLAG_MC1 | TC_INTFLAG_MC0 | TC_INTFLAG_OVF;        // Clear the interrupt flags to be safe
  REG_TC3_INTENCLR = TC_INTENCLR_MC0;     // Disable TC3 CC0 interrupt

 
  REG_TC3_CTRLA |= TC_CTRLA_PRESCALER_DIV1024 |     // Set prescaler to 1024, 48MHz/3(GCLK divider)/1024(prescaler) = 15625 Hz
                   TC_CTRLA_ENABLE;               // Enable TC3
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization
  
  Serial.println("Timer 3 interrupt set-up complete");

  interrupts();
}
