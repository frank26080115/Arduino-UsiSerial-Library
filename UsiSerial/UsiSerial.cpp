/* UsiSerial by me@frank-zhao.com
 *  
 * UsiSerial is a simple wrapper around the code from AVR307 so that Arduino can use USI to implement a hardware serial port
 *  
  Copyright (c) 2013 Frank Zhao
  All rights reserved.

  UsiSerial is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation, either version 3 of
  the License, or (at your option) any later version.

  UsiSerial is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with UsiSerial. If not, see
  <http://www.gnu.org/licenses/>.
*/

#include "UsiSerial.h"
#include "USI_UART_config.h"

USISerial::USISerial()
{
	// do nothing in initializer
}

// AVR307 does not calculate baudrate at runtime, set the baudrate in USI_UART_config.h
void USISerial::begin()
{
	USI_UART_Initialise_Receiver();
	USI_UART_Initialise_Transmitter();
}

void USISerial::end()
{
	flush();
	TCCR1 = 0;
	TCNT1 = 0;
	TIFR |= _BV(TOV1);
	TIMSK &= ~_BV(TOIE1);
	USICR = 0;
	USISR = 0;
	GIFR |= _BV(PCIF);
	GIMSK &= ~_BV(PCIE);
	PCMSK &= ~(_BV(0) | _BV(1));
	DDRB &= ~(_BV(0) | _BV(1));
	PORTB &= ~(_BV(0) | _BV(1));
}

int USISerial::available(void)
{
	return (unsigned int)(UART_RX_BUFFER_SIZE + UART_RxHead - UART_RxTail) % UART_RX_BUFFER_SIZE;
}

int USISerial::peek(void)
{
	if (UART_RxHead == UART_RxTail) {
		return -1;
	} else {
		return UART_RxBuf[UART_RxTail];
	}
}

int USISerial::read(void)
{
	USI_UART_Receive_Byte();
}

void USISerial::flush()
{
	USI_UART_Flush_Buffers();
}

size_t USISerial::write(uint8_t c)
{
	USI_UART_Transmit_Byte(c);
	return 1;
}

USISerial::operator bool() {
	return true;
}

USISerial UsiSerial; // user accessible instance