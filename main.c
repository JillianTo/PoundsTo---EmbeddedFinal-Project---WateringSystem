#include <msp430.h>

#define MOISTURE_STR_LEN 4 // number of digits of moisture value to be sent through UART
#define WATER_OPEN_THRES 100 // a count of how many while loops can occur with the water hatch is open before the water runs out
#define TEMP_THRES 30 // the threshold temperature in celsius where it is too hot for a plant
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
    WDTCTL = WDTPW | WDTHOLD; // stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5; // disable the GPIO power-on default high-impedance mode

    unsigned char strIdx;
    unsigned char waterOpenCount = 0;
    unsigned short moisture;
    unsigned short moistureThres = 1200; // 2400
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
        UCA1TXBUF = ';'; // transmit ';'

        // read ADC for moisture level
        adcMoistureInit(); // set ADC to A1 for reading moisture sensor
        ADCCTL0 |= ADCENC | ADCSC; // sampling and conversion start
        while((ADCCTL0 & ADCIFG) == 0); // check the ADC interrupt flag, while it's low, wait
        _delay_cycles(200000);
        moisture = ADCMEM0;
        ADCCTL0 &= ~ADCIFG; // clear ADC interrupt flag

        // change moisture threshold to current moisture if P2.3 button is pressed
        if(!(P2IN & BIT3)) {
                moistureThres = moisture;
        }

        // convert moisture to string and send over UART
        shortToCharArr(moisture);
        strIdx = 0;
        while(strIdx < MOISTURE_STR_LEN) { // go from start of string to end of string
            while((UCA1IFG & UCTXIFG)==0); // wait until the UART transmitter is ready //UCTXIFG
            UCA1TXBUF = moistureStr[strIdx]; // transmit the character at strIdx in moistureStr
            strIdx++; // go to next character in moistureStr
        }

        // check current moisture to open or close water
        if(moisture > moistureThres) { // lower values is more moist, so a current ADC reading above threshold is too dry
            if(!waterOpen) { // if water is not already open
                rotateServo(); // open
            }
            waterOpenCount++; // increment water open count to keep track of water level
        } else if(waterOpen) { // if water is open but moisture is below threshold
            rotateServo(); // close
        }

        // if waterOpenCount is above threshold, water is probably empty
        if(waterOpenCount > WATER_OPEN_THRES) {
            P1OUT |= BIT0; // turn on P1.0 red LED
        }

        // if P4.1 button pressed, water has been replaced
        if(!(P4IN & BIT1)) {
            waterOpenCount = 0; // reset how long water hatch has been opened
            P1OUT &= ~BIT0; // turn off P1.0 red LED
        }

        // read temperature
        adcTempInit(); // set ADC to A12 for reading temperature senor
        ADCCTL0 |= ADCENC | ADCSC; // sampling and conversion start
        while((ADCCTL0 & ADCIFG) == 0); // check the flag, while its low just wait
        _delay_cycles(200000);
        temp = ADCMEM0;
        ADCCTL0 &= ~ADCIFG;
        calTemp = (temp-CALADC_15V_30C)*(85-30)/(CALADC_15V_85C-CALADC_15V_30C)+30; // convert raw ADC value into celsius

        // turn off green LED if temperature is too high
        if(calTemp > TEMP_THRES) { // if current temperature is greater than threshold
            P6OUT &= ~BIT6; // turn off P6.6 green LED
        } else if(!(P6OUT & BIT6)) { // if P6.6 is 0 and temperature is less than threshold
            P6OUT |= BIT6; // turn on P6.6 green LED
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

    // button, P4.1
    P4DIR &= ~BIT1; // set P4.1 to input
    P4REN |= BIT1; // enable P4.1 resistor
    P4OUT |= BIT1; // set P4.1 resistor to pull-up
    P4IES |= BIT1; // P4.1 High -> Low edge

    // servo, P6.3
    P6OUT &= ~BIT3; // reset P6.3 output
    P6DIR |= BIT3; // set P6.3 to output
    P6SEL0 |= BIT3; // PWM mode for P6.3
    P6SEL1 &= ~BIT3; // PWM mode for P6.3

    // red LED, P1.0
    P1OUT &= ~BIT0; // Clear P1.0 output latch for a defined power-on state
    P1DIR |= BIT0; // Set P1.0 to output direction

    // green LED, P6.6
    P6OUT |= BIT6; // start P6.6 on
    P6DIR |= BIT6; // Set P6.6 to output direction

}

void adcInit() {
    ADCCTL0 |= ADCSHT_8 | ADCON; // ADC ON, sample period>30us
    ADCCTL1 |= ADCSHP;  // software trigger, single channel conversion, MODOSC
    ADCCTL2 &= ~ADCRES;  // clear ADCRES in ADCCTL
    ADCCTL2 |= ADCRES_2;  // 12-bit conversion results
    ADCIE |= ADCIE0; // enable the interrupt request for a completed ADC_B conversion
}

void adcTempInit() {
    ADCMCTL0 = ADCSREF_1 | ADCINCH_12; // ADC input ch A12 => temp sense

    // Configure reference
    PMMCTL0_H = PMMPW_H; // unlock the PMM registers
    PMMCTL2 |= INTREFEN | TSENSOREN; // enable internal reference and temperature sensor
    __delay_cycles(400);  // delay for reference settling
}

void adcMoistureInit() {
    ADCMCTL0 = ADCINCH_1;  // A1 ADC input select; Vref=AVCC
}

void timerInit() { // timer B3 for servo PWM
    TB3CCR0 = 23260-1; // PWM period
    TB3CTL = TBSSEL_2 | MC_1; // SMCLK, up mode
    TB3CCTL4 = OUTMOD_7; // CCR4 reset/set
}

void uartInit() {

    // configure UART pins
    P4SEL0 |= BIT2 | BIT3; // set 2-UART pin as second function
    P4SEL1 &= ~BIT2; // set 2-UART pin as second function
    P4SEL1 &= ~ BIT3; // set 2-UART pin as second function

    // configure eUSCI_A1 for UART
    UCA1CTLW0 |= UCSWRST;
    UCA1CTLW0 |= UCSSEL__SMCLK;
    UCA1BRW = 8; // 115200
    UCA1MCTLW = 0xD600;
    UCA1CTLW0 &= ~UCSWRST; // Initialize eUSCI
    UCA1IE |= UCRXIE; // Enable USCI_A0 RX interrupt

}

void rotateServo() {
    if(TB3CCR4 > 2000) { // servo is to the right, rotate servo CCW
        for(TB3CCR4 = 2600; TB3CCR4 > 550; TB3CCR4--) { // rotate servo CCW, less than 180 degrees because the motor gets stuck
                _delay_cycles(500); // speed of rotation
        }
    } else { // servo is to the left, rotate servo CW
        for(TB3CCR4 = 350; TB3CCR4 < 2600; TB3CCR4++) { // rotate servo 190 degrees CW
            _delay_cycles(500); // speed of rotation
        }
    }
    waterOpen ^= 0x01; // toggle water open

}

void shortToCharArr(unsigned short num) {

    static char chars[] = "0123456789abcdefghijklmnopqrstuvwxyz";
    unsigned char strIdx = MOISTURE_STR_LEN;

    do { // goes from end of moistureStr to beginning
        strIdx--;
        moistureStr[strIdx] = chars[num%10]; // convert least significant digit in num to its ASCII value
        num/=10; // integer divide num by 10
    } while(strIdx > 0);

}
