/*
  .cpp file
  Maverick robot library
  For the Summer School Robotics
  GPS Robotics
  Created by Samuel Wane, Harper Adams University, August 2015
  Adapted for single folder library Matt Butler April 2016
  Updated for wireless summer 2016
*/

#include "GPS_module.h"

const unsigned int MAX_INPUT = 200;

//Functions
static byte input_msg[MAX_INPUT];
static unsigned int input_pos = 0;               // nicht negativer int und immer der gleiche int
bool use_piksi_gps;

String lastMessage; // not happy with this
String lastMessage1;
String lastMessage2;

volatile long unsigned int lastChk = 666;

//Functions for the trailer
Trailerclass::Trailerclass() {}
Trailerclass::~Trailerclass() {}

Trailerclass trailer;

double Trailerclass::trailer_angleclass() {
  double _trailer_offset = 135;
  int sensorPin = A8;
  int sensorValue = analogRead(sensorPin); //0-1022
  double sensorAngle = double(sensorValue) / 1024 * 270;

  sensorAngle = sensorAngle - _trailer_offset;
  return sensorAngle;
}
void Trailerclass::set_trailer_centreclass(void)
{
  int sensorPin = A8;
  int sensorValue = analogRead(sensorPin); //0-1022
  double sensorAngle = double(sensorValue) / 1024 * 270;
  _trailer_offset = sensorAngle;
}

void set_trailer_centre(void)
{
  trailer.set_trailer_centreclass();
}

double trailer_angle(void)
{
  return trailer.trailer_angleclass();
}
//Functions for the timer
Timerclass::Timerclass() {}
Timerclass::~Timerclass() {}

Timerclass timer;

unsigned long Timerclass::get_timerclass(int timer_no) {
  return _timervar[timer_no];
}

void Timerclass::set_timerclass(int timer_no, unsigned long timerincrement)
{
  if (_timer_elapsed[timer_no])
  {
    _timervar[timer_no] = millis() + timerincrement;
    _timer_elapsed[timer_no] = false;
    // set_statei(_timerstate);
  }
  /*if (get_state()!=_timerstate)
    {
    _timervar=millis()+timerincrement;
    set_statei(_timerstate);
    }*/
}
bool Timerclass::timer_elapsedclass(int timer_no)
{
  bool elapsed = millis() > get_timerclass(timer_no);
  // if (elapsed) _timer_elapsed=true;
  _timer_elapsed[timer_no] = elapsed;
  return (elapsed);

}

unsigned long get_timer(int timer_no) {
  return timer.get_timerclass(timer_no);
}

void set_timer(int timer_no, unsigned long timerincrement) {
  timer.set_timerclass(timer_no, timerincrement);
}

bool timer_elapsed(int timer_no) {
  return timer.timer_elapsedclass(timer_no);
}

// public functions for the GPS
double gpstime, gpslat, gpslon, gpsdop, gpsalt;
int gpshour, gpsmin, gpssec;
double gpslatdd, gpslatmm, gpslondd, gpslonmm;
int gpsfix, gpsnosat;
char gpshemi, gpsew;
bool gpsavail;

Gps::Gps() {
}
Gps::~Gps() {
}
void Gps::setTime(double tme)
{
  _tme = tme;
}
void Gps::setLat(double lat)
{
  _lat = lat;
}
void Gps::setLon(double lon)
{
  _lon = lon;
}
void Gps::setDOP(double dop)
{
  _dop = dop;
}
void Gps::setAlt(double alt)
{
  _alt = alt;
}
void Gps::setHour(int hh)
{
  _hh = hh;
}
void Gps::setMin(int mm)
{
  _mm = mm;
}
void Gps::setSec(int ss)
{
  _ss = ss;
}
void Gps::setFix(int fix)
{
  _fix = fix;
}
void Gps::setSat(int sat)
{
  _sat = sat;
}
void Gps::setHemi(char hemi)
{
  _hemi = hemi;
}
void Gps::setEW(char ew)
{
  _ew = ew;
}
double Gps::getTime()
{
  return _tme;
}
double Gps::getLat()
{
  return _lat;
}
double Gps::getLon()
{
  return _lon;
}
double Gps::getDOP()
{
  return _dop;
}
double Gps::getAlt()
{
  return _alt;
}
int Gps::getHour()
{
  return _hh;
}
int Gps::getMin()
{
  return _mm;
}
int Gps::getSec()
{
  return _ss;
}
int Gps::getFix()
{
  return _fix;
}
int Gps::getSat()
{
  return _sat;
}
char Gps::getHemi()
{
  return _hemi;
}
char Gps::getEW()
{
  return _ew;
}

void Gps::setN(double n)
{
  _n = n;
}

void Gps::setE(double e)
{
  _e = e;
}

double Gps::getN()
{
  return _n;
}
double Gps::getE()
{
  return _e;
}

void Gps::setAvailable(bool availability)
{
  _status = availability;
}

bool Gps::isAvailable()
{
  return  _status;
}
Gps GPS; // create a global instance


//###################################### wireless class methods etc,

//#####################################

Wireless::Wireless() {
  purgeWirelessBuffer();
  _myID = '1';
  _runningProgram = false;
  _debugOn = false;
  _lastMessage = "";

}


//######################################

Wireless::~Wireless() {
}
Wireless w;

//########################################OK

void Wireless::sendCommand(char slaveID, char messageType) {

  // slaveID: an ascii character identifying the recipient of the message - x = broadcast
  // messageType: r - run a routine
  //              s - stop routine
  //              o - lon please
  //              a - lat please
  //              p - ping

  String message = "";
  message = message + slaveID + ',' + messageType; // concatenate string
  sendWirelessMessage(message); // use send message sending function to add error checking
  Serial.println("Sending command: " + message);

}

//############################################OK

void Wireless::sendMessage(char slaveID, String data) {
  // slaveID: an ascii character identifying the recipient of the message - x = broadcast
  String message = "";
  message = message + slaveID + ',' + data; // concatenate string
  sendWirelessMessage(message); // use send message sending function to add error checking
  Serial.println("Sending message: " + message);

}

//#########################################OK

String Wireless::ping(char slaveID) {
  sendCommand(slaveID, 'p');
  delay(1000);
  return getWirelessMessage(); // return the response as a string
}

//#########################################OK

String Wireless::runSlaveProgram(char slaveID) {
  sendCommand(slaveID, 'r');
  delay(1000);
  return getWirelessMessage(); // return the response as a string
}

//#########################################OK

String Wireless::stopSlaveProgram(char slaveID) {
  sendCommand(slaveID, 's');
  delay(1000);
  return getWirelessMessage(); // return the response as a string
}

//#########################################OK

double Wireless::getSlaveE(char slaveID) {

  sendCommand(slaveID, 'E');
  delay(1000);
  return getWirelessMessage().toFloat(); // return the response as a double
}

//#########################################OK

double Wireless::getSlaveN(char slaveID) {

  sendCommand(slaveID, 'N');
  delay(1000);
  return getWirelessMessage().toFloat(); // return the response as a double
}

//#########################################

void Wireless::processMessage() { // answer a command or save the message

  String message = getWirelessMessage();

  if (message[0] == 'x') {
    _messageAvailable = true;  // broadcast
    _lastMessage = message;
  }
  if (message[0] == _myID) {
    String reply = "";
    switch (message[2]) {
      case 'p': {
          reply = reply + _myID + " here";
          sendWirelessMessage(reply);
          Serial.println("Ping answered " + String(getMyID()));
        }
        break;
      case 'E': {
          double lt = get_lat();
          double ln = get_lon();
          double UTMNorthing, UTMEasting;
          deg2utm(lt, ln, &UTMNorthing, &UTMEasting);

          sendWirelessMessage(String(UTMEasting));
          Serial.println("E sent: " + String(UTMEasting));
        }
        break;
      case 'N': {
          double lt = get_lat();
          double ln = get_lon();
          double UTMNorthing, UTMEasting;
          deg2utm(lt, ln, &UTMNorthing, &UTMEasting);

          sendWirelessMessage(String(UTMNorthing));
          Serial.println("N sent: " + String(UTMNorthing));
        }
        break;
      case 'r': {
          _runningProgram = true;
          reply = reply + _myID + " Running";
          sendWirelessMessage(reply);
          Serial.println("Run process");
        }
        break;
      case 's': {
          _runningProgram = false;
          reply = reply + _myID + " Stopped";
          sendWirelessMessage(reply);
          Serial.println("Stop process");
        }
        break;
      default:
        _lastMessage = message.substring(2);
        _messageAvailable = true;
        Serial.println("Message available");
        break;
    }
  }//else {return "Not for me";}
}

//#########################################

bool Wireless::messageReceived() { // see if a message exists
  if (_messageAvailable) {
    return true;
  }
  else return false;
}

//#########################################

String Wireless::getWirelessMessage() { // get the message (strip off error check)

  //Serial.println("Checking for message");
  if (!Serial3.available()) { /////////////////////////////<<
    return "";
  }

  String inData = "";
  long unsigned int timeout = millis();
  char received = Serial3.read(); /////////////////////////////<<

  while ((received != '\n')) {
    if (Serial3.available()) { /////////////////////////////<<
     inData = inData + received; 
      received = Serial3.read();  /////////////////////////////<<
    }
    if ((millis() - timeout) > 1000)
    {
      return "terminator error";
    }
  }

  if (_debugOn) Serial.println("Raw message received: " + inData);
  String lastDigit;
  if (inData.length() > 11) {
    lastDigit = String(inData.length() - 3);
  }
  else lastDigit = String(inData.length() - 2); // number that should be transmitted as string
  // decode error checking
  if ((inData[inData.length() - 2]) == (lastDigit[lastDigit.length() - 1])) {

    if (inData.length() > 11) {
      return inData.substring(0, inData.length() - 3);
    }
    else {
      return inData.substring(0, inData.length() - 2);
    }
  }
  else {
    return "corruption error";
  }
}

//#########################################

void Wireless::sendWirelessMessage(String message) { // send a message
  Serial3.println(message + String(message.length())); // add error checking /////////////////////////////<<
  if (_debugOn) Serial.println("Raw message sent: " + message + String(message.length()));

}

//#########################################

int Wireless::purgeWirelessBuffer() { // purge any existing messages or garbage

  int counter = 0;
  while (Serial3.available()) { /////////////////////////////<<
    delay(10); // clean up any message in mid-transmission too
    char dump = Serial3.read(); /////////////////////////////<<
    counter ++;
  }
  return counter;
}

//#########################################

void Wireless::setMyID(char myID) {
  _myID = myID;

}

//#########################################

char Wireless::getMyID() {
  return _myID;
}

//#########################################

String Wireless::getMessage() {
  _messageAvailable = false;
  return _lastMessage;
}

//#########################################

bool Wireless::runProgram() {
  if (_runningProgram) return true; else return false;
}

void Wireless::showDebugInfo(bool turnOn) {
  if (turnOn) _debugOn = true; else _debugOn = false;
}

int bytesToInt(int b4, int b3, int b2, int b1)    //4 bytes verschieben, zusammensetzen und zu dezimalzahl wandeln
{
  int resultat = 0;
  resultat = (b4 << 24) | (b3 << 16) | (b2 << 8) | b1;   //bytes in resultatvariable nach links verschieben bis aus 4 bytes eine zahl wird
  return resultat;
}
void msg_analyse(byte byte_msg[29])        //funktion um die herausgefilterte msg zu analysieren
{
  double east = 0;
  double north = 0;
  north = bytesToInt(byte_msg[12], byte_msg[11], byte_msg[10], byte_msg[9]);        //vertikale abweichung von basis in  (aufruf der Funktion und welche bytes dazu verwendet werden)
  east = bytesToInt(byte_msg[16], byte_msg[15], byte_msg[14], byte_msg[13]);        //horizontale abweichung von basis in mm (aufruf der Funktion und welche bytes dazu verwendet werden)
  GPS.setN(north);
  GPS.setE(east);
  GPS.setAvailable(true);
}

void processIncomingPiksiByte()
{
  //Serial.println(inByte , HEX);
  byte inByte;
  static byte input_msg [MAX_INPUT];
  static unsigned int input_pos = 0;               // nicht negativer int und immer der gleiche int
  if (Serial1.available())
  {
    inByte = Serial1.read();
    if (inByte == 0x55)
    {
      if (input_pos == 29)
      {
        /* for( int i = 0 ; i < input_pos ; i++)
          {
           Serial.print(input_msg [i],HEX);       // wenn hex 55 print gesammter puffer [input_msg)
           Serial.print(" ");
          }
          Serial.println();                       // neue Zeile nach kpl. msg print
        */
        if ((input_msg[0] == 0x03) && (input_msg[1] == 0x02))
        {
          msg_analyse (input_msg);              //funktion um die herausgefilterte msg zu analysieren
        }
      }
      input_pos = 0;                            // und setzte input_pos wieder auf 0
    }
    else
    {
      if (input_pos < (MAX_INPUT - 1))         // nur wenn pufferlänge nicht erreicht ist (in unserem Falle, wird das nicht der Fall sein)
      {
        input_msg [input_pos] = inByte;        // schreibt byte in puffer an position 0
        input_pos = input_pos + 1;             // erhöht position im Puffer um 1
      }
    }
  }

}
/* GPS string comes as: (http://aprs.gids.nl/nmea/#gga)
  $GPGGA,044909.000,5248.0310,N,00206.1113,W,1,06,2.5,73.4,M,48.3,M,,0000*7F
  $GPGSA,A,3,29,12,31,14,25,32,,,,,,,3.2,2.5,2.0*3F
  $GPRMC,044909.000,A,5248.0310,N,00206.1113,W,0.02,226.14,250917,,,A*7A

  $--GGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx
  hhmmss.ss = UTC of position
  llll.ll = latitude of position
  a = N or S
  yyyyy.yy = Longitude of position
  a = E or W
  x = GPS Quality indicator (0=no fix, 1=GPS fix, 2=Dif. GPS fix)
  xx = number of satellites in use
  x.x = horizontal dilution of precision
  x.x = Antenna altitude above mean-sea-level
  M = units of antenna altitude, meters
  x.x = Geoidal separation
  M = units of geoidal separation, meters
  x.x = Age of Differential GPS data (seconds)
  xxxx = Differential reference station ID


*/
/*New GPS split routine, not returning DOP, Sat, but returns lat long ok-check 3 Nov 2017*/
bool nmea_split(char input[]) //Used for parsing Serial. from PC
/*Returns string split in global variable:
  char str_split_result[20][15]; //20 strings of up to 15 characters each
  first item is in str_split_result[0]
  and value returned = number of strings split
*/

/*char str[] ="- This, a sample string.";
  char * pch;
  printf ("Splitting string \"%s\" into tokens:\n",str);
  pch = strtok (str," ,.-");
*/
{
  char str_split_result[20][15];
  char * token;
  int n = 0;
  bool gpsavail = false;
  token = strtok(input, ",");
  while (token != NULL)
  {
    strcpy(str_split_result[n], token);

    n++;
    token = strtok(NULL, ",");
  }


  String str(str_split_result[0]);
  if (str == "$GPGGA")
  {
    //044909.000
    gpsavail = true;
    gpstime = strtod(str_split_result[1], NULL);
    //conv H,M,S from 044909.000
    gpshour = (int)(gpstime / 10000);
    gpsmin = (int)((gpstime - ((double)gpshour * 10000)) / 100);
    gpssec = (int)(gpstime - ((double)gpshour * 10000) - ((double)gpsmin * 100));
    gpslat = strtod(str_split_result[2], NULL);
    //Convert to ddmm.mmmm
    gpslatdd = floor(gpslat / 100);
    gpslatmm = gpslat - (gpslatdd * 100);
    gpslatmm = (floor(gpslatmm * 1000)) / 1000;
    gpslat = ((gpslatmm / 60.0) + gpslatdd);
    String hemi(str_split_result[3]);
    if (hemi == "N") gpshemi = 'N'; else gpshemi = 'S';
    gpslon = strtod(str_split_result[4], NULL);
    //Convert to dddmm.mmmm
    gpslondd = floor(gpslon / 100);
    gpslonmm = gpslon - (gpslondd * 100);
    //gpslonmm=(floor(gpslonmm *10000))/10000;
    gpslon = ((gpslonmm / 60.0) + gpslondd);
    String ew(str_split_result[5]);
    if (ew == "W") {
      gpsew = 'W';
      gpslon = -gpslon;
    } else gpsew = 'E';
    gpsfix = (int)strtol(str_split_result[6], NULL, 0); //0,1,2
    gpsnosat = (int)strtol(str_split_result[7], NULL, 0);
    gpsdop = strtod(str_split_result[8], NULL); //DOP
    gpsalt = strtod(str_split_result[9], NULL); //alt
  }
  return gpsavail;
}
/*End of new GPS split routine*/
void read_sparkfun_GPS()
{
  static char gpsstring[100];
  static char pcstring[100];
  static int readinggps = 0, readpos = 0;
  static int readingpc = 0, readpcpos = 0;
  char c;
  double la = 0, lo = 0;
  double tt, dop = 0;
  //Need to process S and W to get negative values etc
  if (Serial1.available())
  {
    c = Serial1.read();
    //Serial.print(c);
    if (c == '$') {
      readinggps = 1;
      readpos = 0;
    }
    if (readinggps)gpsstring[readpos++] = c;
    //Serial.println(gpsstring);
    if (c == '\r')
    {
      //Serial.println(gpsstring);
      readinggps = 0; gpsstring[readpos] = 0;
      //gpsstring now has '$GPRMC,044909.000,A,5248.0310,N,00206.1113,W,0.02,226.14,250917,,,A*7A'
      if (nmea_split(gpsstring))
      {
        GPS.setAvailable(true);
        GPS.setTime(gpstime);
        GPS.setLat(gpslat);
        GPS.setLon(gpslon);
        GPS.setDOP(gpsdop);
        GPS.setAlt(gpsalt);
        GPS.setFix(gpsfix);
        GPS.setSat(gpsnosat);
        GPS.setHemi(gpshemi);
        GPS.setEW(gpsew);

      } else GPS.setAvailable(false);
    }
  }//if Serial1.available...
}

//Timer2 Overflow Interrupt Vector, called every 1ms
ISR(TIMER2_OVF_vect) {
  if (!use_piksi_gps) read_sparkfun_GPS();
  if (use_piksi_gps) processIncomingPiksiByte();
  //processIncomingPiksiByte();
  TCNT2 = 130;           //Reset Timer to 130 out of 255
  TIFR2 = 0x00;          //Timer2 INT Flag Reg: Clear Timer Overflow Flag
};


void display(String topLine) {
  lastMessage1 = ""; lastMessage2 = "";
  if (topLine != lastMessage) {
    clear_LCD();
    Serial3.print(topLine.substring(0, 16));
    lastMessage = topLine;
  }
}


void display(String topLine, String bottomLine ) {
  lastMessage = "";
  if ((topLine != lastMessage1) || (bottomLine != lastMessage2)) {
    clear_LCD();
    Serial3.print(topLine.substring(0, 16));
    nextline_LCD();
    Serial3.print(bottomLine.substring(0, 16));
    lastMessage1 = topLine;
    lastMessage2 = bottomLine;
  }
}

// access the GPS
bool gps_avail() {
  return GPS.isAvailable();
}
double get_lat() {

  return GPS.getLat();
}
double get_lon() {
  return GPS.getLon();
}
double get_N() {
  return GPS.getN();
}
double get_E() {
  return GPS.getE();
}

double get_Time() {
  return GPS.getTime();
}
double get_DOP() {
  return GPS.getDOP();
}
double get_Alt() {
  return GPS.getAlt();
}
int get_Hour() {
  return GPS.getHour();
}
int get_Min() {
  return GPS.getMin();
}
int get_Sec() {
  return GPS.getSec();
}
int get_Fix() {
  return GPS.getFix();
}
int get_Sat() {
  return GPS.getSat();
}
char get_Hemi() {
  return GPS.getHemi();
}
char get_EW() {
  return GPS.getEW();
}
void serialEvent2() { // triggers processing of wireless by the wireless object
  w.processMessage();
}

void clear_LCD() {
  Serial3.write(0xFE); Serial3.write(0x01); //Clear LCD
}
void nextline_LCD() {
  Serial3.write(0xFE); Serial3.write(192); //Next LCD Line
}

void GPS_initialise() {
  Serial1.begin(9600);

  TCCR2B = 0x00;        //Disbale Timer2 while we set it up
  TCNT2  = 130;         //Reset Timer Count to 130 out of 255
  TIFR2  = 0x00;        //Timer2 INT Flag Reg: Clear Timer Overflow Flag
  TIMSK2 = 0x01;        //Timer2 INT Reg: Timer2 Overflow Interrupt Enable
  TCCR2A = 0x00;        //Timer2 Control Reg A: Normal port operation, Wave Gen Mode normal
  TCCR2B = 0x02;        //Timer2 Control Reg B: Timer Prescaler set to 64  gives 0.5ms
}

