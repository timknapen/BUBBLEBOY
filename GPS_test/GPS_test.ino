#include <TinyGPS.h>
//#include <SoftwareSerial.h>

/* This sample code demonstrates the normal use of a TinyGPS object.
   It assumes that you have a 4800-baud serial GPS device hooked up
   to a serial port, or SoftwareSerial on pins 4(rx) and 3(tx).
*/

TinyGPS gps;

// Use one of these to connect your GPS
// ------------------------------------
#define gpsPort Serial1

char gpsBuf[32];
float latitude, longitude, elevation;

void setup()
{
  Serial.begin(115200);

  gpsPort.begin(9600);

  Serial.print("Simple TinyGPS library v. ");
  Serial.println(TinyGPS::library_version());
  Serial.println("by Mikal Hart");
  Serial.println();
}

void loop()
{
  readGPS();
}

void readGPS()
{

  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (gpsPort.available())
    {
      char c = gpsPort.read();
      Serial.write(c);   // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  unsigned long age;
  if (newData)
  {
    gps.f_get_position(&latitude, &longitude, &age);
    elevation = gps.f_altitude();
  Serial.println("");
    Serial.print("LAT=");
    Serial.print(latitude == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : latitude, 6);
    Serial.print(" LON=");
    Serial.print(longitude == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : longitude, 6);
    Serial.print(" ALT=");
    Serial.println(elevation == TinyGPS::GPS_INVALID_F_ALTITUDE ? 0.0 : elevation, 6);
    Serial.println("");
  }
  else
  {

    //satellites in view
    uint32_t *satz = gps.trackedSatellites();
    uint8_t sat_count = 0;
    for (int i = 0; i < 24; i++)
    {
      if (satz[i] != 0)
      { //exclude zero SNR sats
        sat_count++;
        byte strength = (satz[i] & 0xFF) >> 1;
        byte prn = satz[i] >> 8;
        sprintf(gpsBuf, "PRN %d: ", prn);
        Serial.print(gpsBuf);
        Serial.print(strength);
        Serial.println("dB");
      }
    }
  }

  //date time
  int year;
  uint8_t month, day, hour, minutes, second, hundredths;
  gps.crack_datetime(&year, &month, &day, &hour, &minutes, &second, &hundredths, &age);
  sprintf(gpsBuf, "GPS time: %d/%02d/%02d %02d:%02d:%02d", year, month, day, hour, minutes, second);
  Serial.println("");
  Serial.println(gpsBuf);

  gps.stats(&chars, &sentences, &failed);
  gps.stats(&chars, &sentences, &failed);
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
  if (chars == 0)
  {
    Serial.println("** No characters received from GPS: check wiring **");
  }
}