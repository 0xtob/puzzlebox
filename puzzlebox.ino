/****************************************************************
 * Wedding Puzzlebox                                            *
 *                                                              *
 * A box that needs to be carried to three specific locations   *
 * until it opens. Based on the Reverse GeoCache Puzzle by      *
 * Mikal Hart                                                   *
 * http://arduiniana.org/projects/the-reverse-geo-cache-puzzle/ *
 *                                                              *
 * Tested on Arduino Uno.                                       *
 * Requires the TinyGPS Library                                 *
 * http://arduiniana.org/libraries/tinygps/                     *
 ****************************************************************/

#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <LiquidCrystal.h>
#include <Servo.h>
#include <EEPROM.h>

#define GPS_RX_PIN 2
#define GPS_TX_PIN 3
#define SERVO_PIN  5

// Fake locking for debugging
#define SERVO_LOCKED    90
#define SERVO_UNLOCKED  180

//Uncomment for actual locking
//#define SERVO_LOCKED    0
//#define SERVO_UNLOCKED  90

#define SERVO_DELAY    1000 // Tims im ms we wait for the servo to move to its position
#define MESSAGE_DELAY  3000 // How long a message is displayed in ms.

// Addresses of persistent information in EEPROM
#define EEPROM_STATE  0 // Values: targets starting from 0. 255 means final target found.

#define STATE_ALL_TARGETS_FOUND      255

#define GPS_TIMEOUT_SECS  300 // How to long wait for a GPS fix
#define DIST_MONITOR_SECS 120 // How long to show the distance monitor (after a GPS fix has been established)

#define GPSBAUD 4800

TinyGPS gps;
SoftwareSerial uart_gps(GPS_RX_PIN, GPS_TX_PIN);
LiquidCrystal lcd(8, 9, 10, 11, 12, 13);
Servo servo;

struct Target {
  float latitude, longitude, unlock_distance;
  int n_hint_msgs;
  // At most 4 msgs of 2 lines of 16 chars
  char hint_msgs[4][2][17];   // Hint the box shows after turn on before this target is reached
  int n_arrive_msgs;
  char arrive_msgs[4][2][17]; // Message the box shows when this target is reached
};

const int N_TARGETS = 3;

struct Target targets[N_TARGETS] = {
                                      { 50.785599, 6.082625, 0.05,
                                        4,
//                                         "XXXXXXXXXXXXXXXX",  "XXXXXXXXXXXXXXXX"
                                        { {"Hallo Welt und"   , "Tim und Anne!"},
                                          {"Besucht 3 Orte"   , "und ich geh auf"},
                                          //{"Lasst uns ein"    , "Spiel spielen."},
                                          {"Ich funktioniere" , "nur draussen."},
                                          {"Ich sage nur wie" , "weit zum Ziel"} },
                                        2,
                                        { {"Bravo! War aber"  , "auch einfach."},
                                          {"Macht ein Foto!"  , "Und weiter gehts"} }
                                      },
                                      
                                      { 51.659196, 6.612073, 0.05,
                                        3,
//                                         "XXXXXXXXXXXXXXXX", "XXXXXXXXXXXXXXXX"
                                        { {"Das neue Ziel"   , ""},
                                          {"Wie Chiptunes"   , "und 8 Bit"},
                                          {"und Kibbelsche"  , "und Cooper '72" } },
                                        2,
                                        { {"Ihr habts"       , "fast geschafft!"},
                                          {"Schickt ne Mail!", ""} }
                                      },
                                      
                                      { 50.568649, 6.203665, 0.10,
                                        3,
//                                         "XXXXXXXXXXXXXXXX", "XXXXXXXXXXXXXXXX"
                                        { {"Voran, voran nur", "immer im Lauf"},
                                          {"Zum Tour Eiffel" , "wo der Roehricht"},
                                          {"knistert im"     , "Heiderauche"} },
                                        2,
                                        { {"Ihr habts"       , "geschafft!"},
                                          {"Achievement"     , "unlocked!"} }
                                      }
                                      
                                   };

void servoSet(int state)
{ 
  // Disable GPS serial temporarily, because it interferes with the Servo
  // TODO: Find out why!
  uart_gps.end();

  servo.attach(SERVO_PIN);
 
  int curr_state = servo.read();
  
  if(curr_state != state) {  
    servo.write(state);
    delay(SERVO_DELAY);
  }
  
  servo.detach();
  
  uart_gps.begin(GPSBAUD);
}

int getState()
{
  return EEPROM.read(EEPROM_STATE);
}

void setState(int state)
{
  EEPROM.write(EEPROM_STATE, state);
}

void setup()
{  
  lcd.begin(16, 2);
  Serial.begin(115200);
  uart_gps.begin(GPSBAUD);
  Serial.println("TimAnneBox Debug Console");
  lcd.clear();
}

void showMessage(char *line1, char *line2)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
  delay(MESSAGE_DELAY);
}

void requestTurnOff()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Bitte schaltet");
  lcd.setCursor(0, 1);
  lcd.print("mich aus.");
  while(1);
}

// Collect GPS data for one second
bool feedGps()
{
  bool new_data = false;
  unsigned int start = millis();
  while (millis() - start < 1000) {
    while(uart_gps.available()) {
      if(gps.encode(uart_gps.read())) {
        new_data = true;
      }
    }
  }
  return new_data;
}

// Shows the distance monitor.
// Return values:
//   1: target was reached
//   0: target was not reached
//  -1: no GPS signal found
int monitorDistance(double target_latitude, double target_longitude, float unlock_distance)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Lokalisiere ...");
  Serial.println("Searching sattelites ..."); // The person reading the serial output is assumed to be technically more proficient than the person reading the display.
  
  unsigned int startsec = millis() / 1000;
  bool got_gps_signal = false;
  
  while(1) {
    unsigned int secs_passed = millis() / 1000 - startsec;
    if(!got_gps_signal && (secs_passed > GPS_TIMEOUT_SECS) ) {
      Serial.println("GPS Timeout");
      return -1;
    }
    if(got_gps_signal && (secs_passed > DIST_MONITOR_SECS) ) {
      Serial.println("Monitor timeout");
      return 0;
    }
    
    // Hack for debugging: If pin5 is shorted to +5V, we return success.
    pinMode(5, INPUT);
    if(analogRead(5) == 1023) {
      return 1;
    }
    
    if(!feedGps()) {
      continue;
    }
    
    got_gps_signal = true;
    
    Serial.println("data");

    
    float latitude, longitude, altitude, km_to_target;
    char latstr[8] = {0}, lonstr[8] = {0}, altstr[8] = {0}, diststr[8] = {0};
    gps.f_get_position(&latitude, &longitude);
    altitude = gps.f_altitude();
    km_to_target = gps.distance_between(latitude, longitude, target_latitude, target_longitude) / 1000.0;
      
    dtostrf(latitude, 7, 4, latstr);
    dtostrf(longitude,7, 4, lonstr);
    dtostrf(altitude, 7, 4, altstr);
    dtostrf(km_to_target, 7, 2, diststr);
      
    Serial.print("Lat: ");
    Serial.println(latitude);
    Serial.print("Lon: ");
    Serial.println(longitude);
    Serial.print("Distance: ");
    Serial.println(km_to_target);
      
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Eure Entfernung:");
    lcd.setCursor(0, 1);
    lcd.print(diststr);
    lcd.print(" km");
      
    if(km_to_target < unlock_distance) {
      return 1;
    } 
  }
}

// Check if Pin5 is set to +5V and reset the state if so.
void checkReset()
{
  pinMode(5, INPUT);

  if(analogRead(5) == 1023) {
    Serial.println("Reset!");
    
    setState(0);
    
    // So we can close it
    // On next startup, the box will lock
    servoSet(SERVO_UNLOCKED); 
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("State reset.");
    
    while(1);
  }
}

void loop()
{
  checkReset();
  
  int state = getState();
  
  if(state == STATE_ALL_TARGETS_FOUND) {
    Serial.println("Target already found");
    servoSet(SERVO_UNLOCKED);
    showMessage("Alles Gute!Buck,",
                "Helga,Medha,Tob");
  } else {
    Serial.println("Target not found");
    servoSet(SERVO_LOCKED);
      
    // Show hints
    for(int msg=0; msg < targets[state].n_hint_msgs; ++msg) {
      showMessage(targets[state].hint_msgs[msg][0], targets[state].hint_msgs[msg][1]);
    }
      
    // Show distance monitor
    double target_lat = targets[state].latitude;
    double target_lon = targets[state].longitude;
    double target_unlock_dist = targets[state].unlock_distance;
    
    int res = monitorDistance(target_lat, target_lon, target_unlock_dist);
      
    // If the current target was reached
    if(res == 1) {
      
      // If it was the last target
      if(state == N_TARGETS-1) {
        showMessage("Ziel erreicht!",
                    "Macht mich auf!");
        setState(STATE_ALL_TARGETS_FOUND);
        servoSet(SERVO_UNLOCKED);
      } else {
        // Show the arrival message
        for(int msg=0; msg < targets[state].n_arrive_msgs; ++msg) {
          showMessage(targets[state].arrive_msgs[msg][0], targets[state].arrive_msgs[msg][1]);
        }
          
        // Proceed to the next target
        state++;
        setState(state);
          
        // Show hint for next target
        for(int msg=0; msg < targets[state].n_hint_msgs; ++msg) {
          showMessage(targets[state].hint_msgs[msg][0], targets[state].hint_msgs[msg][1]);
        }
        
        // Show the distance monitor, but don't expect to reach the target
        target_lat = targets[state].latitude;
        target_lon = targets[state].longitude;
        target_unlock_dist = targets[state].unlock_distance;
        
        monitorDistance(target_lat, target_lon, target_unlock_dist);
      }
        
    } else if(res == -1) {
      showMessage("Kein GPS Signal", "");
    }
  }
    
  requestTurnOff();
}

