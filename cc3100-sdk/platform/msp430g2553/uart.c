/*
 * uart.h - msp430f5529 launchpad uart interface implementation
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

#ifdef SL_IF_TYPE_UART
#include "simplelink.h"
#include "board.h"
#include <msp430.h>
#include "uart.h"
#include <msp/msp430g2553.h>

#define CTS_LINE_IS_HIGH        (P2IN & BIT3)

extern unsigned char IntIsMasked;

extern P_EVENT_HANDLER  pIraEventHandler;

extern _uartFlowctrl *puartFlowctrl;


int uart_Close(Fd_t fd)
{
    // Disable WLAN Interrupt ...
    CC3100_InterruptDisable();

    return NONOS_RET_OK;
}


Fd_t uart_Open(char *ifName, unsigned long flags) {
	unsigned char i;

	IntIsMasked = FALSE;

	puartFlowctrl->JitterBufferFreeBytes = UART_READ_JITTER_BUFFER_SIZE;
	puartFlowctrl->JitterBufferWriteIdx = 0;

	puartFlowctrl->pActiveBuffer = puartFlowctrl->JitterBuffer;
	puartFlowctrl->bActiveBufferIsJitterOne = TRUE;

	puartFlowctrl->JitterBufferReadIdx = 0xFF;

	for(i = 0; i < UART_READ_JITTER_BUFFER_SIZE; i++)
		puartFlowctrl->JitterBuffer[i] = 0xCC;


	//TODO what this code snippet does?
	// P2.3 - WLAN enable full DS
	P2SEL &= ~BIT3;
	P2OUT &= ~BIT3;
	P2DIR |= BIT3;

	// Configure Pin 1.2/1.1 for RX/TX
	P1SEL |= BIT2 + BIT1;               // P1.1,2 = USCI_A0 TXD/RXD
	P1SEL2 |= BIT2 + BIT1;

	UCA0CTL1 |= UCSWRST;                // Stop USCI state machine

	// Enable RX Interrupt on UART
	IFG2 &= ~ (UCA0TXIFG | UCA0RXIFG);
	IE2 |= UCA0RXIE;

	UCA0CTL1 |= UCSSEL_2; 		// Use SMCLK
	UCA0BR0 = 0x08;         		// 1MHz/115200= 8.68, int(8.68) = 8 (see User's Guide)
	UCA0BR1 = 0x00;         
	UCA0MCTL = UCBRS_6;     		// round(0.68 * 8) = 6, UCOS16=0 (UCA0BR < 16)
	//maximum TX error -7.8% to 6.4%, maximum RX error -9.6% to 16.1%

	UCA0CTL1 &= ~UCSWRST;               // Initialize USCI state machine

	// XXX configure RTS ans CTS pins aren't possible with 20 pins MCUs
	// TODO use proto cable to put this functionality in other pins
	// Configure Pin 1.4 and 2.3 as RTS as CTS
	// MCU RTS
	P1SEL &= ~BIT4;
	P1OUT &= ~BIT4;
	P1DIR |= BIT4;
	// MCU CTS
	P2SEL &= ~BIT3;
	P2DIR &= ~BIT3;
	P2REN |= BIT3;
	    
	Delay(50);

	// Enable WLAN interrupt
	CC3100_InterruptEnable();

	clear_rts();


	return NONOS_RET_OK;
}


int uart_Read(Fd_t fd, unsigned char *pBuff, int len)
{
	/* Disable interrupt to protect reorder of bytes */
	__disable_interrupt();

	puartFlowctrl->pActiveBuffer = pBuff;
	puartFlowctrl->bActiveBufferIsJitterOne = FALSE;
	puartFlowctrl->ActiveBufferWriteCounter = 0;

	/* Copy data received in Jitter buffer to the user buffer */
	while(puartFlowctrl->JitterBufferFreeBytes != UART_READ_JITTER_BUFFER_SIZE) {
		if(puartFlowctrl->JitterBufferReadIdx == (UART_READ_JITTER_BUFFER_SIZE - 1))
			puartFlowctrl->JitterBufferReadIdx = 0;
		else
			puartFlowctrl->JitterBufferReadIdx++;

		puartFlowctrl->pActiveBuffer[puartFlowctrl->ActiveBufferWriteCounter++] = puartFlowctrl->JitterBuffer[puartFlowctrl->JitterBufferReadIdx];

		puartFlowctrl->JitterBufferFreeBytes ++;
	}


	puartFlowctrl->bRtsSetByFlowControl = FALSE;
	__enable_interrupt();
	clear_rts();

	/* wait till all remaining bytes are received */
	while(puartFlowctrl->ActiveBufferWriteCounter < len);

	puartFlowctrl->bActiveBufferIsJitterOne = TRUE;

	return len;
}


int uart_Write(Fd_t fd, unsigned char *pBuff, int len) {
	int len_to_return = len;

	while (len--) {
		while (!(IFG2 & UCA0TXIFG) || CTS_LINE_IS_HIGH) ;
		UCA0TXBUF = *(pBuff++);
	}
	return len_to_return;
}
