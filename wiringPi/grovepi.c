
/*
 * grovepi.c:
 *  Extend wiringPi with the grovepi
 *  Copyright (c) 2014 Wyliodrin
 ***********************************************************************
 * This file is part of wiringPi:
 *  https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with wiringPi.
 *    If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#include <stdio.h>
#include <pthread.h>

#include "wiringPi.h"
#include "wiringPiI2C.h"

 int pins[9];
 int pinsa[3];


/*
 * grovepiPinMode:
 *********************************************************************************
 */

static void grovepiPinMode (struct wiringPiNodeStruct *node, int pin, int mode)
{
  unsigned char buffer[] = {5, 0, 0, 0};
  pin = pin - node->pinBase;

  if (mode == INPUT)
  {
    mode = 0;
  }
  else
  {
    mode = 1;
  }

  buffer[1] = pin;
  buffer[2] = mode;

  wiringPiI2CWriteBuffer (node->fd, 1, buffer, 4) ;
}


/*
 * grovepiDigitalWrite:
 *********************************************************************************
 */

static void grovepiDigitalWrite (struct wiringPiNodeStruct *node, int pin, int value)
{
  unsigned char buffer[] = {2, 0, 0, 0};
  pin = pin - node->pinBase;

  buffer[1] = pin;
  buffer[2] = value;

  // printf ("digitalWrite %d\n", pin);

  wiringPiI2CWriteBuffer (node->fd, 1, buffer, 4) ;
  // perror ("digitalWrite");
}


/*
 * grovepiDigitalRead:
 *********************************************************************************
 */

static int grovepiDigitalRead (struct wiringPiNodeStruct *node, int pin)
{
  unsigned char buffer[] = {1, 0, 0, 0};
  pin = pin - node->pinBase;
  int res = 0;

  buffer[1] = pin;

  wiringPiI2CWriteBuffer (node->fd, 1, buffer, 4) ;

  delay (1);

  res = wiringPiI2CReadReg8 (node->fd, 1);
  if (res >= 0)
  {
    pins[pin] = res;
  }
  return pins[pin];
}


/*
 * grovepiAnalogWrite:
 *********************************************************************************
 */

static void grovepiAnalogWrite (struct wiringPiNodeStruct *node, int pin, int value)
{
  unsigned char buffer[] = {4, 0, 0, 0};
  pin = pin - node->pinBase;

  buffer[1] = pin;
  buffer[2] = value;

  wiringPiI2CWriteBuffer (node->fd, 1, buffer, 4) ;
}


/*
 * grovepiDigitalRead:
 *********************************************************************************
 */

static int grovepiAnalogRead (struct wiringPiNodeStruct *node, int pin)
{
  unsigned char buffer[] = {3, 0, 0, 0};
  pin = pin - node->pinBase;

  int res = 0;

  buffer[1] = pin;

  wiringPiI2CWriteBuffer (node->fd, 1, buffer, 4) ;
  // perror ("analog read error");

  delay (1);
  wiringPiI2CReadReg8 (node->fd, 1);

  res = wiringPiI2CReadBuffer (node->fd, 0, buffer, 4);
  if (res >= 4)
  {
    res = buffer[1]*256+buffer[2];
    if (res < 1024) 
      pinsa[pin] = res;
  }
  return pinsa[pin];
}


/*
 * grovepiSetup:
 *  Create a new instance of an grovepi I2C GPIO interface. 
 *********************************************************************************
 */

int grovepiSetup (const int pinBase, const int i2cAddress)
{
  int fd ;
  struct wiringPiNodeStruct *node ;

  if ((fd = wiringPiI2CSetup (i2cAddress)) < 0)
    return fd ;

  node = wiringPiNewNode (pinBase, 8) ;

  node->fd              = fd ;
  node->pinMode         = grovepiPinMode ;
  node->digitalRead     = grovepiDigitalRead ;
  node->digitalWrite    = grovepiDigitalWrite ;
  node->analogRead      = grovepiAnalogRead ;
  node->analogWrite     = grovepiAnalogWrite ;

  return 0 ;
}
