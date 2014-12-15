/*
 * board.c - msp430f5529 launchpad configuration
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#include "msp/msp430g2553.h"
#include "simplelink.h"
#include "board.h"

#define XT1_XT2_PORT_SEL            P5SEL
#define XT1_ENABLE                  (BIT4 + BIT5)
#define XT2_ENABLE                  (BIT2 + BIT3)
#define PMM_STATUS_ERROR  1
#define PMM_STATUS_OK     0
#define XT1HFOFFG   0

P_EVENT_HANDLER                pIraEventHandler = 0;

unsigned char IntIsMasked;


#ifdef SL_IF_TYPE_UART
#define ASSERT_UART(expr) {  if (!(expr)) { while(1) ;}}

unsigned char error_overrun = FALSE;
_uartFlowctrl uartFlowctrl;
_uartFlowctrl *puartFlowctrl = &uartFlowctrl;
#endif

int registerInterruptHandler(P_EVENT_HANDLER InterruptHdl , void* pValue)
{
    pIraEventHandler = InterruptHdl;

    return 0;
}


void CC3100_disable() {
	P1OUT &= ~BIT3;
}


void CC3100_enable() {
	P1OUT |= BIT3;
}

void CC3100_InterruptEnable(void) {
	IE2 |= UCA0RXIE;
}

void CC3100_InterruptDisable() {
	IE2 |= UCA0RXIE;
}

void MaskIntHdlr() {
    IntIsMasked = TRUE;
}

void UnMaskIntHdlr() {
    IntIsMasked = FALSE;
}

void set_rts() {
    P1OUT |= BIT4;
}

void clear_rts() {
    P1OUT &= ~BIT4;
}

void initClk() {
    // Set system clock to 1MHz
    BCSCTL1 = CALBC1_1MHZ; // basic clock system control register
    DCOCTL = CALDCO_1MHZ; // digital clock oscilator control register

    // Globally enable interrupts
    __enable_interrupt();
}

void stopWDT()
{
    WDTCTL = WDTPW + WDTHOLD;
}

void initLEDs() {
    P1OUT &= ~(BIT0 | BIT6);
    P1DIR |= BIT0 | BIT6;
    P1SEL &= ~(BIT0 | BIT6);
}

void turnLedOn(char ledNum) { // TODO simplify this function and eliminate switch case statement
    switch(ledNum) {
      case LED1:
        P1OUT |= BIT0;
        break;
      case LED2:
        P1OUT |= BIT6;
        break;
    }
}

void turnLedOff(char ledNum) { // TODO simplify this function and eliminate switch case statement
    switch(ledNum) {
      case LED1:
        P1OUT &= ~BIT0;
        break;
      case LED2:
        P1OUT &= ~BIT6;
        break;
    }
}


void toggleLed(char ledNum) { // TODO simplify this function and eliminate switch case statement
    switch(ledNum) {
      case LED1:
        P1OUT ^= BIT0;
        break;
      case LED2:
        P1OUT ^= BIT6;
        break;
    }

}

unsigned char GetLEDStatus() {
  unsigned char status = 0;

  if(P1OUT & BIT0)
    status |= (1 << 0); // TODO replace shift number 0 by proper LED1 value
  if(P1OUT & BIT6)
    status |= (1 << 1); // TODO replace shift number 1 by proper LED2 value

  return status;
}

#pragma vector=PORT1_VECTOR
__interrupt void Port1_ISR(void)
{
    /* Context save interrupt flag before calling interrupt vector. */
    /* Reading interrupt vector generator will automatically clear IFG flag */

    switch (__even_in_range(P1IV, P1IV_P1IFG7))
    {
        /* Vector  P1IV_NONE:  No Interrupt pending */
        case  P1IV_NONE:
            break;

        /* Vector  P1IV_P1IFG0:  P1IV P1IFG.0 */
        case  P1IV_P1IFG0:
            break;

        /* Vector  P1IV_P1IFG1:  P1IV P1IFG.1 */
        case  P1IV_P1IFG1:
            break;

        /* Vector  P1IV_P1IFG2:  P1IV P1IFG.2 */
        case  P1IV_P1IFG2:
            break;

        /* Vector  P1IV_P1IFG3:  P1IV P1IFG.3 */
        case  P1IV_P1IFG3:
            break;

        /* Vector  P1IV_P1IFG4:  P1IV P1IFG.4 */
        case  P1IV_P1IFG4:
            break;

        /* Vector  P1IV_P1IFG5:  P1IV P1IFG.5 */
        case  P1IV_P1IFG5:
            break;

        /* Vector  P1IV_P1IFG1:  P1IV P1IFG.6 */
        case  P1IV_P1IFG6:
            break;

        /* Vector  P1IV_P1IFG7:  P1IV P1IFG.7 */
        case  P1IV_P1IFG7:
            break;

        /* Default case */
        default:
            break;
    }
}

void Delay(unsigned long interval) {
    while(interval--)
        __delay_cycles(1000); //1 second, 1000 cycles at 1Mhz
}



// TODO remember who UART interruptions work, read user guide
// responsible for filling RX buffer
/*!
    \brief          The UART A0 interrupt handler

    \param[in]      none

    \return         none

    \note

    \warning
*/
#pragma vector=USCIAB0RX_VECTOR
__interrupt void CC3100_UART_ISR(void)
{
	unsigned char ByteRead;

	if(UCA0STAT & UCRXERR) { // character received with error
		if(UCA0STAT & UCOE) // a character was transfered to buff before the previous was read
		    error_overrun = TRUE;
		ASSERT_UART(0); // lock, a infinite loop
	}

	ByteRead = UCA0RXBUF;

	if(puartFlowctrl->bActiveBufferIsJitterOne == TRUE) {
		if(puartFlowctrl->JitterBufferFreeBytes > 0) {
		    puartFlowctrl->JitterBuffer[puartFlowctrl->JitterBufferWriteIdx] = ByteRead;
		    puartFlowctrl->JitterBufferFreeBytes--;
		    puartFlowctrl->JitterBufferWriteIdx++;

		    if((FALSE == IntIsMasked) && (NULL != pIraEventHandler))
			pIraEventHandler(0);
		}
		else if(P2OUT & BIT3) // MCU CTS high, CC3100 request stop receive
			ASSERT_UART(0); //lock, a infinite loop

		if(puartFlowctrl->JitterBufferFreeBytes <= UART_READ_JITTER_RTS_GUARD) {
		    set_rts();
		    puartFlowctrl->bRtsSetByFlowControl = TRUE;
		}

		if(puartFlowctrl->JitterBufferWriteIdx > (UART_READ_JITTER_BUFFER_SIZE - 1))
		    puartFlowctrl->JitterBufferWriteIdx = 0;
	}
	else
		puartFlowctrl->pActiveBuffer[puartFlowctrl->ActiveBufferWriteCounter++] = ByteRead;
}

