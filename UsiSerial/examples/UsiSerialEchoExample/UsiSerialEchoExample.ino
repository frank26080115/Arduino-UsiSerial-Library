/* UsiSerial by me@frank-zhao.com
 *  
 * UsiSerial is a simple wrapper around the code from AVR307 so that Arduino can use USI to implement a hardware serial port
 *
 * This is a simple echo example.
 * The USI class works almost exactly like HardwareSerial
 * So you can use print and println and other functions like that
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

#include <UsiSerial.h>

void setup()
{
  UsiSerial.begin();
}

void loop()
{
  if (UsiSerial.available() > 0) {
    UsiSerial.write(UsiSerial.read());
  }
}