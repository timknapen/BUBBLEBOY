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
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

/*
# Ranges when flippin Z all over the place:
# X -40.45 to 33.27
# Y -45.73 to 26.55
# Z -72.35 to 17.24
*/
float minX = -40.45; //-20.64;
float maxX = 33.27;  //15;

float minY = -45.73; // -26;
float maxY = 26.55;  // 9;

float minZ = -72.35; // -63.57;
float maxZ = 17.24;  // 4.08;

float x = 0;
float y = 0;
float z = 0;

bool bUpdateMinMax = true; // set to true to record min max values
bool bUseMinMax = true;
bool bOutputRange = false; // set to true to print min max

unsigned long lastMessage = 0;

#define BUF_LENGTH 3
float angleBuffer[BUF_LENGTH];
int bufPos = 0;

void setup(void)
{
  if (bUpdateMinMax)
  {
    minX = 999;
    maxX = -999;

    minY = 999;
    maxY = -999;

    minZ = 999;
    maxZ = -999;
  }

  pinMode(LED_BUILTIN, OUTPUT);
  while (!Serial)
  {
    // blink LED ?
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
  Serial.begin(57600);
  Serial.println("HMC5883 Magnetometer Test");
  Serial.println("");

  /* Initialise the sensor */
  if (!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while (1)
      ;
  }

  /* Display some basic information on this sensor */
  displaySensorDetails();

  lastMessage = millis();

  // empty buffer
  for (int i = 0; i < BUF_LENGTH; i++)
  {
    angleBuffer[i] = 0;
  }
}

void loop(void)
{
  /* Get a new sensor event */
  sensors_event_t event;
  mag.getEvent(&event);

  float curx = event.magnetic.x;
  float cury = event.magnetic.y;
  float curz = event.magnetic.z;

  if (bUpdateMinMax)
  {
    minX = min(curx, minX);
    minY = min(cury, minY);
    minZ = min(curz, minZ);

    maxX = max(curx, maxX);
    maxY = max(cury, maxY);
    maxZ = max(curz, maxZ);
  }
  if (bUseMinMax)
  {
    // set to middle point as origin?
    // normalize?
    curx -= (minX + maxX) / 2;
    cury -= (minY + maxY) / 2;
    curz -= (minZ + maxZ) / 2;

    float xrange = maxX - minX;
    float yrange = maxY - minY;
    float zrange = maxZ - minZ;

    // rescale to fixed range (-100, 100);
    if (xrange != 0 && yrange != 0 && zrange != 0)
    {
      curx /= xrange;
      cury /= yrange;
      curz /= zrange;

      //curx *= 1023;
      //cury *= 1023;
      //curz *= 1023;

      //curx += 512;
      //cury += 512;
      //curz += 512;
    }
  }

  // zeno out noise
  x += (curx - x) / 2;
  y += (cury - y) / 2;
  z += (curz - z) / 2;

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(y, x);

  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  // float declinationAngle = 1.12; // for Paris
  // heading += declinationAngle;

  // Correct for when signs are reversed.
  if (heading < 0)
  {
    heading += 2 * M_PI;
  }

  // Check for wrap due to addition of declination.
  if (heading > 2 * M_PI)
  {
    heading -= 2 * M_PI;
  }

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180.0 / M_PI;

  //fill angle buffer
  angleBuffer[bufPos] = headingDegrees;
  bufPos++;
  bufPos %= BUF_LENGTH;

  float avgHeading = 0;
  for (int i = 0; i < BUF_LENGTH; i++)
  {
    avgHeading += angleBuffer[i];
  }
  avgHeading /= BUF_LENGTH;

  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  // print out the raw data for graph
  Serial.print(x * 1023 + 512);
  Serial.print(" ");
  Serial.print(y * 1023 + 512);
  Serial.print(" ");
  Serial.print(z * 1023 + 512);
  Serial.print("    ");
  Serial.print(avgHeading);

  Serial.println();
  delay(100);

  unsigned long now = millis();
  if (now > lastMessage + 2000 && bOutputRange)
  {
    lastMessage = now;
    Serial.println("# Ranges:");

    Serial.print("# X ");
    Serial.print(minX);
    Serial.print(" to ");
    Serial.println(maxX);

    Serial.print("# Y ");
    Serial.print(minY);
    Serial.print(" to ");
    Serial.println(maxY);

    Serial.print("# Z ");
    Serial.print(minZ);
    Serial.print(" to ");
    Serial.println(maxZ);

    Serial.println();
  }
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("# ------------------------------------");
  Serial.print("# Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("# Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("# Unique ID:    ");
  Serial.println(sensor.sensor_id);
  Serial.print("# Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println(" uT");
  Serial.print("# Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println(" uT");
  Serial.print("# Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println(" uT");
  Serial.println("# ------------------------------------");
  Serial.println("");
  delay(500);
}