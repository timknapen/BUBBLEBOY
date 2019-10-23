/***************************************************************************
  This is a library example for the HMC5883 magnentometer/compass

  Designed specifically to work with the Adafruit HMC5883 Breakout
  http://www.adafruit.com/products/1746

  *** You will also need to install the Adafruit_Sensor library! ***

  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries with some heading example from
  Love Electronics (loveelectronics.co.uk)

  This program is free software: you can redistribute it and/or modify
  it under the terms of the version 3 GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ***************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345);

void displaySensorDetails(void)
{
  sensor_t sensor;
  compass.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

float x = 0;
float y = 0;
float z = 0;


/* measured extremes:
    -41.65 24.94 -29.18 25.30 -32.63 32.33
*/
float minx = -41.65;
float maxx = 24.94;

float miny = -29.18;
float maxy = 25.30;

float minz = -32.63;
float maxz = 32.33;



void setup(void)
{
  Serial.begin(9600);
  Serial.println("HMC5883 Magnetometer Test");
  Serial.println("");

  if (!compass.begin())
  {
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while (1);
  }

  /* Display some basic information on this sensor */
  displaySensorDetails();
}

void loop(void)
{
  /* Get a new sensor event */
  sensors_event_t event;
  compass.getEvent(&event);

  float tx = -1  + 2 * (event.magnetic.x - minx) / (maxx - minx);
  float ty = -1  + 2 * (event.magnetic.y - miny) / (maxy - miny);
  float tz = -1  + 2 * (event.magnetic.z - minz) / (maxz - minz);

  // filter out some noise
  // x += (tx - x) / 3;
  // y += (ty - y) / 3;
  // z += (tz - z) / 3;
  x = tx;
  y = ty;
  z = tz;

  /*
     // calculation of calibration data
    minx = min(x, minx);
    miny = min(y, miny);
    minz = min(z, minz);

    maxx = max(x, maxx);
    maxy = max(y, maxy);
    maxz = max(z, maxz);


    Serial.print(minx);
    Serial.print(" ");
    Serial.print(maxx);
    Serial.print(" ");

    Serial.print(miny);
    Serial.print(" ");
    Serial.print(maxy);
    Serial.print(" ");

    Serial.print(minz);
    Serial.print(" ");
    Serial.println(maxz);


  */


  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(y, x);

  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  //float declinationAngle = 0.013;
  //Lille/ float declinationAngle = 0.013
  //Paris/ float declinationAngle = 0.0157
  //heading += declinationAngle;

  // Correct for when signs are reversed.
  if (heading < 0) {
    heading += 2 * PI;
  }

  // Check for wrap due to addition of declination.
  if (heading > 2 * PI) {
    heading -= 2 * PI;
  }
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180 / M_PI;

  //Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  //*
  Serial.print(x);
  Serial.print(" ");

  Serial.print(y);
  Serial.print(" ");

  Serial.print(z);
  Serial.print(" ");

  Serial.println(headingDegrees);
  //*/
  delay(50);
}
