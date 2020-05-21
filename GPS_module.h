/*
  .h file
  Maverick robot library
  For the Summer School Robotics
  GPS Robotics
  Created by Samuel Wane, Harper Adams University, August 2015
  Library folder adapted by Matt Butler March 2016
  Updated for wireless summer 2016
*/


#ifndef GPS_module_h
#define GPS_module_h

#include "Arduino.h"
#include "deg2utm.h"

class Trailerclass
{
  public:
    Trailerclass();
    ~Trailerclass();
    double trailer_angleclass(void);
    void set_trailer_centreclass(void);
  private:
    double _trailer_offset = -1;
};

class Timerclass
{
  public:
    Timerclass();
    ~Timerclass();
    unsigned long get_timerclass(int timer_no);
    void set_timerclass(int timer_no, unsigned long timerincrement);
    bool timer_elapsedclass(int timer_no);
  private:
    unsigned long _timervar[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    bool _timer_elapsed[10] = {true, true, true, true, true, true, true, true, true, true};
};

class Gps
{
  public:
    Gps();
    ~Gps();
    void setTime(double hh);
    void setLat(double lat);
    void setLon(double lon);
    void setDOP(double dop);
    void setAlt(double alt);
    void setHour(int hh);
    void setMin(int mm);
    void setSec(int ss);
    void setFix(int fix);
    void setSat(int sat);
    void setHemi(char hemi);
    void setEW(char EW);

    double getTime();
    double getLat();
    double getLon();
    double getDOP();
    double getAlt();
    int getHour();
    int getMin();
    int getSec();
    int getFix();
    int getSat();
    char getHemi();
    char getEW();

    void setE(double e);
    void setN(double n);
    double getN();
    double getE();

    bool isAvailable();
    void setAvailable(bool availability);
  private:
    double _tme = 0;
    double _lat = 0;
    double _lon = 0;
    double _dop = 0;
    double _alt = 0;
    int _hh = 0;
    int _mm = 0;
    int _ss = 0;
    int _fix = 0;
    int _sat = 0;
    char _hemi;
    char _ew;

    double _e = 0;
    double _n = 0;

    bool _status = false;
};


class Wireless
{
  public:
    Wireless(); // wireless object constructor
    ~Wireless(); // wireless object destructor
    void sendMessage(char slaveID, String data); // slave id 'x' is broadcast
    String ping(char slaveID);                  // expect response like " 4 here"
    double getSlaveE(char slaveID);           // use .toFloat()
    double getSlaveN(char slaveID);
    void processMessage();             // this is called automatically every time serial2 gets a byte
    bool messageReceived();              // used to find out if a message is available
    String getMessage();           //then collect it
    void setMyID(char myID);             // default is '1'. User should set.
    char getMyID();                      // to check current ID set
    int purgeWirelessBuffer();           // clear serial 2 buffer. Performed automatically at start
    bool runProgram();  // should my program run?
    String runSlaveProgram(char slaveID); // start or stop program on slave
    String stopSlaveProgram(char slaveID);
    void showDebugInfo(bool turnOn); // see all communication
  private:
    bool _messageAvailable; // indicates a message is available
    void sendWirelessMessage(String message); // used internally
    String getWirelessMessage();              // used internally
    void sendCommand(char slaveID, char messageType); // used internally
    char _myID;          // use public get and set methods
    String _lastMessage; // the last message received
    bool _runningProgram;    // used internally
    bool _debugOn;     // used internally to store debug status
};

extern Wireless w; // wireless object globally available

//Forward declarations
double trailer_angle();
void set_trailer_centre();
unsigned long get_timer(int timer_no);
void set_timer(int timer_no, unsigned long timerincrement);
bool timer_elapsed(int timer_no);
bool gps_avail();
double get_lat();
double get_lon();
double get_N();
double get_E();
double get_Time();
double get_DOP();
double get_Alt();
int get_Hour();
int get_Min();
int get_Sec();
int get_Fix();
int get_Sat();
char get_Hemi();
char get_EW();

//void initialise(bool use_piksi);
void GPS_initialise();

double convtime(char s[]);
double convlat(char s[]);
double convlon(char s[]);
double convdop(char s[]);

void clear_LCD();
void nextline_LCD();

void display(String topLine);
void display(String topLine, String bottomLine ); // override

#endif

