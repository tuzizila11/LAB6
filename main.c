/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 *
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//******************************************************************************
//   MSP430G2xx3 Demo - USCI_A0, 9600 UART Echo ISR, DCO SMCLK
//
//   Description: Echo a received character, RX ISR used. Normal mode is LPM0.
//   USCI_A0 RX interrupt triggers TX Echo.
//   Baud rate divider with 1MHz = 1MHz/9600 = ~104.2
//   ACLK = n/a, MCLK = SMCLK = CALxxx_1MHZ = 1MHz
//
//                MSP430G2xx3
//             -----------------
//         /|\|              XIN|-
//          | |                 |
//          --|RST          XOUT|-
//            |                 |
//            |     P1.2/UCA0TXD|------------>
//            |                 | 9600 - 8N1
//            |     P1.1/UCA0RXD|<------------
//
//   D. Dang
//   Texas Instruments Inc.
//   February 2011
//   Built with CCS Version 4.2.0 and IAR Embedded Workbench Version: 5.10
//******************************************************************************


#include <msp430.h>

/* Global variables */
unsigned int i = 0, count = 0;
unsigned int data[10];
volatile char txBuffer[6];

/* Function Prototypes */
void portInit(void);
unsigned int sampleADC(void);
void itoa(unsigned int);

int main(void)
{
    portInit();
    while (1)
    {
//        unsigned int adcVal = sampleADC();
//        itoa(adcVal);
//        __delay_cycles(10000);
    }
}


#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
    UCA0TXBUF = txBuffer[i++];                 // TX next character

    if (i == sizeof txBuffer - 1)              // TX over?
        IE2 &= ~UCA0TXIE;                       // Disable USCI_A0 TX interrupt
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
    if (UCA0RXBUF == 'u')                     // 'u' received?
    {
        i = 0;
        IE2 |= UCA0TXIE;                        // Enable USCI_A0 TX interrupt
    }
}

// Timer A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A(void)
{
    unsigned int val;
    P1OUT ^= 0x01;                            // Toggle P1.0

    data[9] = data[8];
    data[8] = data[7];
    data[7] = data[6];
    data[6] = data[5];
    data[5] = data[4];
    data[4] = data[3];
    data[3] = data[2];
    data[2] = data[1];
    data[1] = data[0];
    data[0] = sampleADC();

    val = data[0] + data[1] + data[2] + data[3] + data[4] + data[5] + data[6]
            + data[7] + data[8] + data[9];
    val = val/10;


    itoa(val);

    i = 0;
    IE2 |= UCA0TXIE;                        // Enable USCI_A0 TX interrupt
    P1OUT ^= 0x01;                            // Toggle P1.0
}

/*
 * Function: portInit
 * ---------------------
 * Initializes microcontroller registers and configures ADC
 */
void portInit(void)
{
    WDTCTL = WDTPW + WDTHOLD;                       // Stop WDT
    if (CALBC1_1MHZ == 0xFF)                        // If calibration constant erased
    {
        while (1);                                  // do not load, trap CPU!!
    }
    DCOCTL = 0;                                     // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_1MHZ;                          // Set DCO
    DCOCTL = CALDCO_1MHZ;

    /* Configure UART */
    P1SEL = BIT1 + BIT2;                            // P1.1 = RXD, P1.2=TXD
    P1SEL2 = BIT1 + BIT2;                           // P1.1 = RXD, P1.2=TXD
    UCA0CTL1 |= UCSSEL_2;                           // SMCLK
    UCA0BR0 = 104;                                  // 1MHz 9600
    UCA0BR1 = 0;                                    // 1MHz 9600
    UCA0MCTL = UCBRS0;                              // Modulation UCBRSx = 1
    UCA0CTL1 &= ~UCSWRST;                           // **Initialize USCI state machine**
    IE2 |= UCA0RXIE;                                // Enable USCI_A0 RX interrupt

    /* Configure GPIO */
    P2OUT = 0x00;                                   // reset all P2 output pins to clear 7-seg
    P2SEL &= ~BIT6;                                 // turn off XIN to enable P2.6
    P1DIR |= 0x01;                                  // P1.0 output
    P1OUT &= ~0x01;

    /* Configure Timer */
    TACTL = TASSEL_2 + MC_1 + ID_3;                 // SMCLK, upmode, 1Mhz/4 = 125KHz
    CCTL0 = CCIE;                                   // CCR0 interrupt enabled
    CCR0 = 12500-1;                                 // 2Hz = 125KHz / 62500


    /* Configure ADC Channel */
    P1SEL |= BIT5;                                  // set P1.5 to analog input
    ADC10CTL1 = INCH_5 + ADC10DIV_3;                // select channel A5, ADC10CLK/3
    ADC10CTL0 = ADC10SHT_3 + MSC + ADC10ON;         // sample/hold 64 cycle, multiple sample, turn on ADC10
    ADC10AE0 |= BIT5;                               // enable P1.5 for analog input

    __bis_SR_register(GIE);                         // interrupts enabled
}

unsigned int sampleADC(void){
    ADC10CTL0 |= ENC + ADC10SC; // enable and start conversion
    while ((ADC10CTL1 & ADC10BUSY) == 0x01); // wait until sample operation is complete
    return ADC10MEM;
}

void itoa(unsigned int num){

    txBuffer[3] = (num % 10) + 48;
    num /= 10;
    txBuffer[2] = (num % 10) + 48;
    num /= 10;
    txBuffer[1] = (num % 10) + 48;
    num /= 10;
    txBuffer[0] = (num % 10) + 48;

    txBuffer[4] = 13; // Return character
    txBuffer[5] = 10; // New Line
}
