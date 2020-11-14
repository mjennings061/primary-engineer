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


void timer_4_setup() {
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
