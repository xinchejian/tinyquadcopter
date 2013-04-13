/*
 * CMPS03 library example code
 *
 * Reads values from a CMPS03 sensor and writes to the serial
 * interface at 9600 baud.
 *
 * This file is part of the grappendorf.net Arduino Libraries.
 *
 * The contents of this file are subject to the Apache License Version
 * 2.0 (the "License"); you may not use this file except in compliance with
 * the License. You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Software distributed under the License is distributed on an "AS IS" basis,
 * WITHOUT WARRANTY OF ANY KIND, either express or implied. See the License
 * for the specific language governing rights and limitations under the
 * License.
 *
 * The Original Code is grappendorf.net Arduino Libraries / CMPS03.
 *
 * The Initial Developer of the Original Code is
 * Dirk Grappendorf (www.grappendorf.net)
 * Portions created by the Initial Developer are Copyright (C) 2008-2011
 * the Initial Developer. All Rights Reserved.
 */

#include "Wire.h"
#include "CMPS03.h"

CMPS03 cmps03;

void setup()
{
  Serial.begin(9600);
  Wire.begin();
}

void loop()
{
  Serial.println(cmps03.read());
  delay(1000);
}
