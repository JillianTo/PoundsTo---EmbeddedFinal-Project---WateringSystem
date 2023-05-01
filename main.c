#include <msp430.h>

#define MOISTURE_STR_LEN 4
#define WATER_OPEN_THRES 100
#define TEMP_THRES 30
#define CALADC_15V_30C *((unsigned int *)0x1A1A) // Temperature Sensor Calibration-30C //6682 // See device datasheet for TLV table memory mapping
#define CALADC_15V_85C *((unsigned int *)0x1A1C) // Temperature Sensor Calibration-High Temperature (85 for Industrial, 105 for Extended)


char moistureStr[MOISTURE_STR_LEN];
char waterOpen = 0x00; // 0 for closed, 1 for opened

void gpioInit();
void adcInit();
void adcTempInit();
void adcMoistureInit();
void timerInit();
void uartInit();
void rotateServo();
void shortToCharArr(unsigned short num);

void main(void) {
    WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;

    unsigned char strIdx;
    unsigned char waterOpenCount = 0;
    unsigned short moisture;
    unsigned short moistureThres = 2400;
    volatile long temp;
    volatile float calTemp;

    gpioInit();
    adcInit();
    timerInit();
    uartInit();

    _delay_cycles(5); // Wait for ADC ref to settle

    while(1){

        // send ';' to end last sent value
        _delay_cycles(20000);
        while((UCA1IFG & UCTXIFG)==0); //Wait until the UART transmitter is ready //UCTXIFG
        UCA1TXBUF = ';'; //Transmit the received data.

        // read ADC for moisture level
        adcMoistureInit();
        ADCCTL0 |= ADCENC | ADCSC; // Sampling and conversion start
        while((ADCCTL0 & ADCIFG) == 0); // check the flag, while its low just wait
        _delay_cycles(200000);
        moisture = ADCMEM0;
        ADCCTL0 &= ~ADCIFG;

        // change moisture threshold to current moisture if P2.3 button is pressed
        if(!(P2IN & BIT3)) {
                moistureThres = moisture;
        }

        // convert moisture to string and send over UART
        shortToCharArr(moisture);
        strIdx = 0;
        while(strIdx < MOISTURE_STR_LEN) {
            while((UCA1IFG & UCTXIFG)==0); //Wait Unitl the UART transmitter is ready //UCTXIFG
            UCA1TXBUF = moistureStr[strIdx]; //Transmit the received data.
            strIdx++;
        }

        // check current moisture to open or close water
        if(moisture > moistureThres) { // if currently above threshold
            if(!waterOpen) { // if water is not already open
                rotateServo(); // open
            }
            waterOpenCount++; // increment water open count to keep track of water level
        } else if(waterOpen) { // if water is open but moisture is below threshold
            rotateServo(); // close
        }

        // if waterOpenCount is above threshold, water is probably empty
        if(waterOpenCount > WATER_OPEN_THRES) {
            P1OUT |= BIT0;          // turn on P1.0 red LED
        }

        // if P4.1 button pressed, water has been replaced
        if(!(P4IN & BIT1)) {
            waterOpenCount = 0;
            P1OUT &= ~BIT0;
        }

        // read temperature
        adcTempInit();
        ADCCTL0 |= ADCENC | ADCSC; // Sampling and conversion start
        while((ADCCTL0 & ADCIFG) == 0); // check the flag, while its low just wait
        _delay_cycles(200000);
        temp = ADCMEM0;
        ADCCTL0 &= ~ADCIFG;
        calTemp = (temp-CALADC_15V_30C)*(85-30)/(CALADC_15V_85C-CALADC_15V_30C)+30;

        // turn off green LED if temperature is too high
        if(calTemp > TEMP_THRES) {
            P6OUT &= ~BIT6;
        } else if(!(P6OUT & BIT6)) { // if P6.6 is 0 and temp is less than threshold
            P6OUT |= BIT6;
        }

    }
}

void gpioInit(){

    // moisture sensor, P1.1
    P1DIR |= BIT1;
    P1OUT |= BIT1;
    P1SEL0 |= BIT1; // ADC A1
    P1SEL1 |= BIT1; // ADC A1

    // button, P2.3
    P2DIR &= ~BIT3; // set P2.3 to input
    P2REN |= BIT3; // enable P2.3 resistor
    P2OUT |= BIT3; // set P2.3 resistor to pull-up
    P2IES |= BIT3; // P2.3 High -> Low edge
    //P2IE |= BIT3; // P2.3 interrupt enable
    //P2IFG &= ~BIT3; // clear P2.3 interrupt flag

    // button, P4.1
    P4DIR &= ~BIT1; // set P4.1 to input
    P4REN |= BIT1; // enable P4.1 resistor
    P4OUT |= BIT1; // set P4.1 resistor to pull-up
    P4IES |= BIT1; // P4.1 High -> Low edge
    //P4IE |= BIT1; // P4.1 interrupt mode
    //P4IFG &= ~BIT1; // clear P4.1 interrupt flag

    // servo, P6.3
    P6OUT &= ~BIT3; // reset P6.3 output
    P6DIR |= BIT3; // set P6.3 to output
    P6SEL0 |= BIT3; // PWM mode for P6.3
    P6SEL1 &= ~BIT3; // PWM mode for P6.3

    // LED, P1.0
    P1OUT &= ~BIT0; // Clear P1.0 output latch for a defined power-on state
    P1DIR |= BIT0; // Set P1.0 to output direction

    // LED, P6.6
    P6OUT |= BIT6; // start P6.6 on
    P6DIR |= BIT6; // Set P6.6 to output direction

}

void adcInit() {
    ADCCTL0 |= ADCSHT_8 | ADCON;                                  // ADC ON, sample period>30us
    ADCCTL1 |= ADCSHP;                                            // s/w trig, single ch/conv, MODOSC
    ADCCTL2 &= ~ADCRES;                                           // clear ADCRES in ADCCTL
    ADCCTL2 |= ADCRES_2;                                          // 12-bit conversion results
    ADCIE |= ADCIE0;                                               // Enable the Interrupt request for a completed ADC_B conversion
}

void adcTempInit() {
    ADCMCTL0 = ADCSREF_1 | ADCINCH_12;                           // ADC input ch A12 => temp sense

    // Configure reference
    PMMCTL0_H = PMMPW_H;                                          // Unlock the PMM registers
    PMMCTL2 |= INTREFEN | TSENSOREN;                              // Enable internal reference and temperature sensor
    __delay_cycles(400);                                          // Delay for reference settling
}

void adcMoistureInit(){ // ADC A1
    ADCMCTL0 = ADCINCH_1;                                   // A1 ADC input select; Vref=AVCC
}

void timerInit() { // Timer B3 for servo PWM
    TB3CCR0 = 23260-1; // PWM period
    TB3CTL = TBSSEL_2 | MC_1; // SMCLK, up mode
    TB3CCTL4 = OUTMOD_7; // CCR4 reset/set
}

void uartInit() {

    // Configure UART pins
    P4SEL0 |= BIT2 | BIT3; // set 2-UART pin as second function
    P4SEL1 &= ~BIT2; // set 2-UART pin as second function
    P4SEL1 &= ~ BIT3; // set 2-UART pin as second function

    // Configure UART
    UCA1CTLW0 |= UCSWRST;
    UCA1CTLW0 |= UCSSEL__SMCLK;
    UCA1BRW = 8; // 115200
    UCA1MCTLW = 0xD600;
    UCA1CTLW0 &= ~UCSWRST; // Initialize eUSCI
    UCA1IE |= UCRXIE; // Enable USCI_A0 RX interrupt

}

void spiInit() {
    P1SEL0 |= BIT4 | BIT5 | BIT6 | BIT7;      // set 4-SPI pin as second function

      UCA0CTLW0 |= UCSWRST;                     // **Put state machine in reset**
                                                // 4-pin, 8-bit SPI slave
      UCA0CTLW0 |= UCSYNC|UCCKPL|UCMSB|UCMODE_1|UCSTEM;
                                                // Clock polarity high, MSB
      UCA0CTLW0 |= UCSSEL__ACLK;                // ACLK
      UCA0BR0 = 0x02;                           // BRCLK = ACLK/2
      UCA0BR1 = 0;                              //
      UCA0MCTLW = 0;                            // No modulation
      UCA0CTLW0 &= ~UCSWRST;                    // **Initialize USCI state machine**
      UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
}

void rotateServo() {
    if(TB3CCR4 > 2000) { // rotate servo CCW
        for(TB3CCR4 = 2600; TB3CCR4 > 550; TB3CCR4--) { // rotate servo CCW, less than 180 degrees because the motor gets stuck
                _delay_cycles(500); // speed of rotation
        }
    } else { // rotate servo CW
        for(TB3CCR4 = 350; TB3CCR4 < 2600; TB3CCR4++) { // rotate servo 190 degrees CW
            _delay_cycles(500); // speed of rotation
        }
    }
    waterOpen ^= 0x01; // toggle water open

}

void shortToCharArr(unsigned short num) {

    static char chars[] = "0123456789abcdefghijklmnopqrstuvwxyz";
    unsigned char strIdx = MOISTURE_STR_LEN;

    do {
        strIdx--;
        moistureStr[strIdx] = chars[num%10];
        num/=10;
    } while(strIdx > 0);

}
