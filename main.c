#include <msp430.h>

short moisture;
const char moistureThreshold = 55;
const char moistureStrLen = 5;
char moistureStr[moistureStrLen];

void uartInit();
void gpioInit();
void adcInit();
void timerB3Init();
void rotateServo();
void shortToCharArr(short num);

void main(void) {
    WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;

    char strIdx;

    gpioInit();
    uartInit();
    timerB3Init();

    _delay_cycles(5); // Wait for ADC ref to settle
    adcInit();

    while(1){

        _delay_cycles(20000);
        while((UCA1IFG & UCTXIFG)==0); //Wait until the UART transmitter is ready //UCTXIFG
        UCA1TXBUF = ';'; //Transmit the received data.

        ADCCTL0 |= ADCENC | ADCSC; // Sampling and conversion start
        while((ADCCTL0 & ADCIFG) == 0); // check the Flag, while its low just wait
        _delay_cycles(200000);
        moisture = ADCMEM0;
        ADCCTL0 &= ~ADCIFG;

        shortToCharArr(moisture);
        strIdx = 0;
        while(strIdx < moistureStrLen) {
            while((UCA1IFG & UCTXIFG)==0); //Wait Unitl the UART transmitter is ready //UCTXIFG
            UCA1TXBUF = moistureStr[strIdx]; //Transmit the received data.
            strIdx++;
        }
    }
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

    // LED, P1.0
    P1OUT &= ~BIT0; // Clear P1.0 output latch for a defined power-on state
    P1DIR |= BIT0; // Set P1.0 to output direction

    // LED, P6.6
    P6OUT &= ~BIT6; // Clear P6.6 output latch for a defined power-off state
    P6DIR |= BIT6; // Set P6.6 to output direction
}

void adcInit(){

    // configure ADC10
    ADCCTL0 |= ADCSHT_8 | ADCON;                             // ADCON, S&H period 30us
    ADCCTL1 |= ADCSHP;                                       // ADCCLK = MODOSC; sampling timer // | ADCSSEL_3 | ADCDIV_7
    ADCCTL2 &= ~ADCRES;                                      // clear ADCRES in ADCCTL
    ADCCTL2 |= ADCRES_2;                                     // 12-bit conversion results
    ADCMCTL0 |= ADCINCH_1;                                   // A1 ADC input select; Vref=AVCC
    ADCIE |= ADCIE0;                                         // Enable ADC conv complete interrupt

}

void timerB3Init() {
    // Timer B3 for servo PWM
    TB3CCR0 = 23260-1; // PWM period
    TB3CTL = TBSSEL_2 | MC_1; // SMCLK, up mode
    TB3CCTL4 = OUTMOD_7; // CCR4 reset/set

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

}

void shortToCharArr(short num) {

    static char chars[] = "0123456789abcdefghijklmnopqrstuvwxyz";
    short strIdx = moistureStrLen-2;

    while(strIdx > -1) {
        moistureStr[strIdx] = chars[num%10];
        strIdx--;
        num/=10;
    }

    moistureStr[moistureStrLen-1] = '\0';

}

/*// ADC interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC_VECTOR
__interrupt void ADC_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC_VECTOR))) ADC_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(ADCIV,ADCIV_ADCIFG)) {
        case ADCIV_NONE:
            break;
        case ADCIV_ADCOVIFG:
            break;
        case ADCIV_ADCTOVIFG:
            break;
        case ADCIV_ADCHIIFG:
            break;
        case ADCIV_ADCLOIFG:
            break;
        case ADCIV_ADCINIFG:
            break;
        case ADCIV_ADCIFG:
            moisture = ADCMEM0;
            __bic_SR_register_on_exit(LPM0_bits); // Clear CPUOFF bit from LPM0
            break;
        default:
            break;
    }
}*/
