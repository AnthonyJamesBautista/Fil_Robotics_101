#include <deg2utm.h>
#include <GPS_module.h>

//variable declarations for latitude and longitude
double lt;
double ln;
//variable declarations for UTM coordinates
double UTMNorthing;
double UTMEasting;

void setup() {
Serial.begin (9600);
GPS_initialise(); //initilize GPS module library
}

void loop() {
if (gps_avail())
{
lt = get_lat();
ln = get_lon();
deg2utm(lt, ln, &UTMNorthing, &UTMEasting);
//Print UTM coordinates in Serial monitor
Serial.print("Northing: "); Serial.print(UTMNorthing, 3);
Serial.print ("\t");
Serial.print ("Easting: "); Serial.println(UTMEasting, 3);
delay (100);
}
}
