#include <math.h>
#include <Adafruit_GPS.h>

#include <SoftwareSerial.h>
SoftwareSerial myLCD(0,4);
SoftwareSerial myGPS(3,2);

Adafruit_GPS GPS(&myGPS);
#define GPSECHO false

boolean usingInterrupt = false;
void useInterrupt(boolean);

//Declarations
const float deg2rad = 0.01745329251994;
const float rEarth = 6371000.0;
float range = 3000;

int gpsWasFixed = HIGH;
int ledFix = 4;

String here;
String there = "N44 44.266, W076 50.776";

void setup() {
  myLCD.begin(9600); // set up LCD for 9600 baud  
  Serial.begin(115200); // debug on serial port
  
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  useInterrupt(true);
  delay(1000);
}

void loop() {
  // Parse GPS and recalculate RANGE
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }
  
  if (GPS.fix) {
    gpsWasFixed = HIGH;
    digitalWrite(ledFix, HIGH);
    
    Serial.print(GPS.latitude, 4); Serial.println(GPS.lat);
    Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);

    here = gps2string((String) GPS.lat, GPS.latitude, (String) GPS.lon, GPS.longitude);
    range = haversine(string2lat(here), string2lon(here), string2lat(there), string2lon(there));
    
    clearLCD();
    selectLineOne();
    myLCD.write("Distance to goal");
    selectLineTwo();
    
    char charVal[10];
    dtostrf(range, 4, 3, charVal);
    myLCD.write(charVal);
    
    delay(500);
  }
  else {
    Serial.println("No fix yet...");
    
    clearLCD();
    selectLineOne();
    myLCD.write("Hi John; Take me");
    selectLineTwo();
    myLCD.write("outside & wait..");
    delay(200);
  }
  
  if (range < 20.0) {
    clearLCD();
    selectLineOne();
    myLCD.write("Goal 01 unlocked");
    selectLineTwo();
    myLCD.write(">>> bit.ly/jgm01");
    delay(5000);
  }
}

SIGNAL(TIMER0_COMPA_vect) {
  // Interrupt is called once a millisecond, looks for any new GPS data, and stores it
  char c = GPS.read();
  if (GPSECHO) {
    if (c) UDR0 = c;
    if (c) Serial.print(c);
  }
}

void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

String int2fw (int x, int n) {
  // returns a string of length n (fixed-width)
  String s = (String) x;
  while (s.length() < n) {
    s = "0" + s;
  }
  return s;
}

String gps2string (String lat, float latitude, String lon, float longitude) {
  // returns "Ndd mm.mmm, Wddd mm.mmm";
  int dd = (int) latitude / 100;
  int mm = (int) latitude % 100;
  int mmm = (int) round(1000 * (latitude - floor(latitude)));
  String gps2lat = lat + int2fw(dd, 2) + " " + int2fw(mm, 2) + "." + int2fw(mmm, 3);
  dd = (int) longitude / 100;
  mm = (int) longitude % 100;
  mmm = (int) round(1000 * (longitude - floor(longitude)));
  String gps2lon = lon + int2fw(dd, 3) + " " + int2fw(mm, 2) + "." + int2fw(mmm, 3);
  String myString = gps2lat + ", " + gps2lon;
  return myString;
}

float string2lat (String myString) {
  // returns radians: e.g. String myString = "N38 58.892, W076 29.177";
  float lat = ((myString.charAt(1) - '0') * 10.0)
            + ((myString.charAt(2) - '0') * 1.0)
            + ((myString.charAt(4) - '0') / 6.0)
            + ((myString.charAt(5) - '0') / 60.0)
            + ((myString.charAt(7) - '0') / 600.0)
            + ((myString.charAt(8) - '0') / 6000.0)
            + ((myString.charAt(9) - '0') / 60000.0);
  Serial.print("float lat: ");
  Serial.println(lat);
  lat *= deg2rad;
  if (myString.charAt(0) == 'S')
    lat *= -1; // correct for hemisphere
  return lat;
}

float string2lon (String myString) {
  // returns radians: e.g. String myString = "N38 58.892, W076 29.177";
  float lon = ((myString.charAt(13) - '0') * 100.0)
            + ((myString.charAt(14) - '0') * 10.0)
            + ((myString.charAt(15) - '0') * 1.0)
            + ((myString.charAt(17) - '0') / 6.0)
            + ((myString.charAt(18) - '0') / 60.0)
            + ((myString.charAt(20) - '0') / 600.0)
            + ((myString.charAt(21) - '0') / 6000.0)
            + ((myString.charAt(22) - '0') / 60000.0);
  Serial.print("float lon: ");
  Serial.println(lon);
  lon *= deg2rad;
  if (myString.charAt(12) == 'W')
    lon *= -1; // correct for hemisphere
  return lon;
}

float haversine (float lat1, float lon1, float lat2, float lon2) {
  // returns the great-circle distance between two points (radians) on a sphere
  float h = sq((sin((lat1 - lat2) / 2.0))) + (cos(lat1) * cos(lat2) * sq((sin((lon1 - lon2) / 2.0))));
  float d = 2.0 * rEarth * asin(sqrt(h));
  //Serial.println(d);
  return d;
}

