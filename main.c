#include <msp430.h>

void gpioInit();
void timerInit();
void adcInit();
void uartInit();
void i2cInit();
void intToCharArr(int value, char* str, int base);
void rotateServo();
#define MCLK_FREQ_MHZ 1 // MCLK = 1MHz

char txData[100];
int moisture;
int strIdx;
unsigned int maxTime = 65535;
int moistureThreshold = 2000;

int main(void) {
	WDTCTL = WDTPW + WDTHOLD; // stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5; // disable GPIO power-on default high-Z

	gpioInit();
	timerInit();
	adcInit();
	uartInit();
	i2cInit();

	__bis_SR_register(LPM3_bits | GIE); // enter LPM3, enable interrupts
	__no_operation(); // for debugger

}

void gpioInit() {
    // moisture sensor, P1.1
    P1DIR |= BIT1;
    P1OUT |= BIT1;

    // button, P2.3
    P2DIR &= ~BIT3; // set P2.3 to input
    P2REN |= BIT3; // enable P2.3 resistor
    P2OUT |= BIT3; // set P2.3 resistor to pull-up
    P2IES |= BIT3; // P2.3 High -> Low edge
    P2IE |= BIT3; // P2.3 interrupt enable
    P2IFG &= ~BIT3; // clear P2.3 interrupt flag

    // button, P4.1
    P4DIR &= ~BIT1; // set P4.1 to input
    P4REN |= BIT1; // enable P4.1 resistor
    P4OUT |= BIT1; // set P4.1 resistor to pull-up
    P4IES |= BIT1; // P4.1 High -> Low edge
    P4IE |= BIT1; // P4.1 interrupt mode
    P4IFG &= ~BIT1; // clear P4.1 interrupt flag

    // servo, P6.3
    P6OUT &= ~BIT3; // reset P6.3 output
    P6DIR |= BIT3; // set P6.3 to output
    P6SEL0 |= BIT3; // PWM mode for P6.3
    P6SEL1 &= ~BIT3; // PWM mode for P6.3
}

void timerInit() {
    // Timer B1
    TB1CCTL0 = CCIE; // TBCCR0 interrupt enabled
    TB1CCR0 = maxTime; // interrupt when timer down from 65,535 to 0
    TB1CTL = TBSSEL_1 | MC_2 | ID_3; // ACLK, continuous mode, clock divider of 8

    // Timer B3 for servo PWM
    TB3CCR0 = 23260-1; // PWM period
    TB3CTL = TBSSEL_2 | MC_1; // SMCLK, up mode
    TB3CCTL4 = OUTMOD_7; // CCR4 reset/set
}

void adcInit() {
    // Configure ADC A1 pin
    P1SEL0 |= BIT1;
    P1SEL1 |= BIT1;

    // configure ADC10
    ADCCTL0 |= ADCSHT_2 | ADCON;
    ADCCTL1 |= ADCSHP;
    ADCCTL2 &= ~ADCRES;
    ADCCTL2 |= ADCRES_2;
    ADCMCTL0 |= ADCINCH_1 | ADCSREF_1;

    // configure reference module
    PMMCTL0_H = PMMPW_H;                                      // Unlock the PMM registers
    PMMCTL2 = INTREFEN | REFVSEL_0;                           // Enable internal 1.5V reference
    while(!(PMMCTL2 & REFGENRDY));                            // Poll till internal reference settles
}

void uartInit() {

    // Configure UART pins
    P4SEL0 |= BIT2 | BIT3;                    // set 2-UART pin as second function
    P4SEL1 &= ~BIT2;                    // set 2-UART pin as second function
    P4SEL1 &= ~ BIT3;                    // set 2-UART pin as second function

    // Configure UART
    UCA1CTLW0 |= UCSWRST;
    UCA1CTLW0 |= UCSSEL__SMCLK;
    UCA1BRW = 8; // 115200
    UCA1MCTLW = 0xD600;
    UCA1CTLW0 &= ~UCSWRST; // Initialize eUSCI
    UCA1IE |= UCRXIE; // Enable USCI_A0 RX interrupt

}

void i2cInit() {
    // initialize eUSCI, https://dev.ti.com/tirex/explore/node?node=A__AJvrfV9xG53GsZcavUE26w__com.ti.MSP430_ACADEMY__bo90bso__1.00.05.06
    UCB0CTLW0 = UCSWRST; // software reset enabled
    UCB0CTLW0 |= UCMODE_3 | UCSYNC; // I2C mode, sync mode
    //UCB0I2COA0 = SLAVE_ADDR | UCOAEN; // own address and enable
    UCB0CTLW0 &= ~UCSWRST; // clear reset register
    UCB0IE |= UCRXIE + UCSTPIE; // interrupt when there is data in RX register and stop bit has occurred

    // temp sensor
    P4SEL0 &= ~BIT6; // P4.6 I2C SDA, user guide pg102
    P4SEL0 &= ~BIT7; // P4.7 I2C SCL, user guide pg102
    P4SEL1 |= BIT6; // P4.6 I2C SDA, user guide pg102
    P4SEL1 |= BIT7; // P4.7 I2C SCL, user guide pg102
}

void intToCharArr(int value, char* str, int base) {
    static char num[] = "0123456789abcdefghijklmnopqrstuvwxyz";
    char* wstr=str;
    char* begin;
    char* end;
    char aux;
    int sign;

    // Validate that base is between 2 and 35 (inclusive)
    if (base<2 || base>35){
        *wstr='\0';
        return;
    }

    // Get magnitude of value
    sign=value;
    if (sign < 0)
        value = -value;

    // Perform integer-to-string conversion
    do
        *wstr++ = num[value%base]; //create the next number in converse by taking the modulus
    while(value/=base); // stop when you get a 0 for the quotient
    if(sign < 0) //attach sign character, if needed
        *wstr++='-';
    *wstr='\0'; //Attach a null character at end of char array. The string is in reverse order at this point

    // reverse string
    begin = str;
    end = wstr-1;
    while(end>begin)
        aux=*end, *end--=*begin, *begin++=aux;
}

void rotateServo() {
    if(TB3CCR4 > 2000) { // rotate servo CCW
        for(TB3CCR4 = 2600; TB3CCR4 > 550; TB3CCR4--) { // rotate servo CCW, less than 180 degrees because the motor gets stuck
                _delay_cycles(500); // speed of rotation
        }
    } else {
        for(TB3CCR4 = 350; TB3CCR4 < 2600; TB3CCR4++) { // rotate servo 190 degrees CW
            _delay_cycles(500); // speed of rotation
        }
    }

}

// port 2 interrupt service routine
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void) {
    P2IFG &= ~BIT3; // clear P2.3 IFG
    rotateServo();
}

// port 4 interrupt service routine
#pragma vector=PORT4_VECTOR
__interrupt void Port_4(void) {
    P4IFG &= ~BIT1; // clear P4.1 IFG
}

// Timer B1 interrupt service routine
#pragma vector = TIMER1_B0_VECTOR
__interrupt void Timer1_B0_ISR(void) {
    ADCCTL0 |= ADCENC | ADCSC; // enable and start conversion
    moisture = ADCMEM0;
    ADCCTL0 &= ~ADCIFG; // clear ADC interrupt flag
    if(moisture > moistureThreshold) {
        rotateServo();
    }
    intToCharArr(moisture,txData,10);
    strIdx = 0;
    txData[strIdx] = ';';
    while(txData[strIdx] != '\0') { // transmit data until null character is reached
        while((UCA1IFG&UCTXIFG) == 0); // wait until UART transmitter is ready
        UCA1TXBUF = txData[strIdx++]; // transmit received data one character at a time
    }
    TB1CCR0 += maxTime; // reset timer count
}
