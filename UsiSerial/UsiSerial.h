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

#ifndef USISERIAL_H_
#define USISERIAL_H_

#include <stdint.h>
#include <Stream.h>


class USISerial : public Stream
{
	private:
	public:
		USISerial();
		void begin(); // AVR307 does not calculate baudrate at runtime, set the baudrate in USI_UART_config.h
		void end();
		virtual int available(void);
		virtual int peek(void);
		virtual int read(void);
		virtual void flush(void);
		virtual size_t write(uint8_t);
		inline size_t write(unsigned long n) { return write((uint8_t)n); }
		inline size_t write(long n) { return write((uint8_t)n); }
		inline size_t write(unsigned int n) { return write((uint8_t)n); }
		inline size_t write(int n) { return write((uint8_t)n); }
		using Print::write; // pull in write(str) and write(buf, size) from Print
		operator bool();
};

extern USISerial UsiSerial; // user accessible instance

#endif