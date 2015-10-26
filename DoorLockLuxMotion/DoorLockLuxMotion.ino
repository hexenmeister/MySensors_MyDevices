#include <SPI.h>
#include <MySensor.h>
#include <DHT.h>

#include <MyUtils.h>
//#include <BH1750.h>
#include <AS_BH1750A.h>
#include <Wire.h>

//-------- define these in your sketch, if applicable ----------------------------------------------------------
// You can reduce the memory footprint of this handler by declaring that there will be no pin change interrupts
// on any one or two of the three ports.  If only a single port remains, the handler will be declared inline
// reducing the size and latency of the handler.
//#define NO_PORTB_PINCHANGES // to indicate that port b will not be used for pin change interrupts
//#define NO_PORTC_PINCHANGES // to indicate that port c will not be used for pin change interrupts
//#define NO_PORTD_PINCHANGES // to indicate that port d will not be used for pin change interrupts
// if there is only one PCInt vector in use the code can be inlined
// reducing latency and code size
// define DISABLE_PCINT_MULTI_SERVICE below to limit the handler to servicing a single interrupt per invocation.
//#define       DISABLE_PCINT_MULTI_SERVICE
//-------- define the above in your sketch, if applicable ------------------------------------------------------

#define VERSION "1.1"

#define MYDEBUG 0
#define REPEATER 0
#define USE_WATCHDOG 1

//-----------------------------------------------------------------------------------------------
#define MAX_U_LONG 4294967295;

// Sensor-Child-IDs
#define CHILD_ID_LIGHT 1
#define CHILD_ID_MOTION 2

#define CHILD_ID_DOOR 5
#define CHILD_ID_LOCK 6

// receive Controller command
#define CHILD_CCMD_ID 100 

// Sensors PIN Config
#define MOT_SENSOR_PIN 3              // The digital input you attached your motion sensor.  (Only 2 and 3 generates interrupt!)
// remove INTERRUPT definition to disable interrupts
#define INTERRUPT MOT_SENSOR_PIN-2    // Usually the interrupt = pin -2 (on uno/nano anyway)
#define INTERRUPT_MODE CHANGE

// Use pin change interrupts for DOOR and LOCK contacts
// remove PC_INTERRUPT definition to disable pinchange interrupts
#define PC_INTERRUPT dummy

#if defined (PC_INTERRUPT)
#include <PinChangeInt.h>
#endif

#define DOOR_SENSOR_PIN 6
#define LOCK_SENSOR_PIN 7

// LED pins
//#define PIN_LED_RED 6
#define PIN_LED_GREEN 5

#define PIN_LED_A_B A0
#define PIN_LED_A_G A1
#define PIN_LED_A_Y A2
#define PIN_LED_A_R A3

// Send time limits
#define TIME_MAX_REPLAY_LUX 600000 // Maximum time to send values (Lux) even if not changed
#define TIME_MIN_REPLAY_LUX 1000   // Minimum time to send values (Lux) even if changed
#define TIME_MIN_REPLAY_MOT 1000   // Minimum time to send values (Motion) even if changed

#define TIME_MAX_REPLAY_DOOR 600000 // Maximum time to send door status even if not changed
#define TIME_MIN_REPLAY_DOOR 70
#define TIME_MAX_REPLAY_LOCK 600000 // Maximum time to send lock status even if not changed
#define TIME_MIN_REPLAY_LOCK 70

// Sleep time between Distance reads (in milliseconds), remove to disable
//#define SLEEP_TIME 50

// Ideen: Parameter aus der Ferne aendern und in EEPROM speichern: MIN/MAX Time, Sende-Grenzwerte, LED-Benutzung

//-----------------------------------------------------------------------------------------------

MySensor gw;

//BH1750 lightSensor;
AS_BH1750A lightSensor;

MyMessage msgLux(CHILD_ID_LIGHT, V_LIGHT_LEVEL);
float lastLux = -1;

MyMessage msgMot(CHILD_ID_MOTION, V_TRIPPED);
uint16_t lastMot = 0;

MyMessage msgDoor(CHILD_ID_DOOR, V_TRIPPED);
uint16_t lastDoor = 0;

MyMessage msgLock(CHILD_ID_LOCK, V_LOCK_STATUS);
uint16_t lastLock = 0;

boolean metric = true;

// Sensoren vorhanden?
boolean mot_present = true;  // can not be autodetected
boolean door_present = true; // can not be autodetected
boolean lock_present = true; // can not be autodetected
boolean lux_present = false; // can be autodetected

// LEDs
#include <AS_Blink.h>
AS_Blink ledR(PIN_LED_A_R, cMillis);
AS_Blink ledG(PIN_LED_A_G, cMillis);
AS_Blink ledB(PIN_LED_A_B, cMillis);
AS_Blink ledY(PIN_LED_A_Y, cMillis);

//-----------------------------------------------------------------------------------------------

void setup()
{
  // configure Watchdog - startet das Device im Falle eines Freezes neu
  #if USE_WATCHDOG > 0
    // Damit Watchdog korrekt funktioniert, muss bei mehreren Boards (z.B. Pro MiniU) ein anderer Bootloader installiert werden. Z.B. Optiboot
    // s. http://sysmagazine.com/posts/189744/ oder Originalpost: http://habrahabr.ru/post/189744/
    wdt_disable();
    // set watchdog
    wdt_enable(WDTO_8S);
    //wdt_reset();
  #endif

  // configure pins for the status LEDs
  //pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);

  // Test the LEDs
  //digitalWrite(PIN_LED_RED,1); // LED red
  //delay(100);
  digitalWrite(PIN_LED_GREEN, 1); // LED green
  delay(100);
  ledG.setOn();
  delay(100);
  ledY.setOn();
  delay(100);
  ledR.setOn();
  delay(100);
  ledB.setOn();

  // configure MySesnsors system
  #if REPEATER == 1
    // The third argument enables repeater mode.
    // Keep node awake all time (no slepping!). Pump the radio network by calling process() in your loop(). The MySensors library will take care of all routing in the background.
    gw.begin(incomingMessage, AUTO, true);
  #else
    // Normal Node
    gw.begin(incomingMessage, AUTO, false);
  #endif

  // autodetect and initialize sensors

  // autodetect BH1750
  if (!lightSensor.begin(RESOLUTION_AUTO_HIGH,false)) {
    lux_present = false;
    #if MYDEBUG > 0
      Serial.println("LightSensor not found");
    #endif
  } else {
    lux_present = true;
    #if MYDEBUG > 0
      Serial.println("LightSensor found");
    #endif
  }

  // sets right mode for the sensor pins
  pinMode(MOT_SENSOR_PIN, INPUT);
  pinMode(DOOR_SENSOR_PIN, INPUT_PULLUP);
  digitalWrite(DOOR_SENSOR_PIN, HIGH);
  pinMode(LOCK_SENSOR_PIN, INPUT_PULLUP);
  digitalWrite(LOCK_SENSOR_PIN, HIGH);

  // Send the sketch version information to the gateway and Controller
  //gw.sendSketchInfo("Temp+Hum+Lux+Motion", "1.0");
  const char *sketch_name = "Door+Lock/Motion+Lux sensor (";
  //+(mot_present?"M ":""+lux_present?"L ":""+dht_present?"T H":"");
  if (mot_present) {
    sketch_name = strcat(const_cast<char*>(sketch_name), "M");
  }
  if (lux_present) {
    sketch_name = strcat(const_cast<char*>(sketch_name), "L");
  }
  if (door_present) {
    sketch_name = strcat(const_cast<char*>(sketch_name), "d");
  }
  if (lock_present) {
    sketch_name = strcat(const_cast<char*>(sketch_name), "l");
  }
  sketch_name = strcat(const_cast<char*>(sketch_name), ")");

  #if MYDEBUG > 0
    Serial.print("Sketch: ");
    Serial.print(sketch_name);
    Serial.print(" Version: ");
    Serial.println(VERSION);
  #endif

  gw.sendSketchInfo(sketch_name, VERSION);

  // Register all sensors to gw (they will be created as child devices)
  if (mot_present) {
    gw.present(CHILD_ID_MOTION, S_MOTION);
  }

  if (door_present) {
    gw.present(CHILD_ID_DOOR, S_DOOR);
  }

  if (lock_present) {
    gw.present(CHILD_ID_LOCK, S_LOCK);
  }

  if (lux_present) {
    gw.present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
  }

  gw.present(CHILD_CCMD_ID, S_CUSTOM);

  //metric = gw.getConfig().isMetric;

  // attach interupts (if any used)
  #if defined (INTERRUPT)
  if(mot_present) {
    attachInterrupt(INTERRUPT, sendMot, INTERRUPT_MODE);  
  }
  #endif
  #if defined (PC_INTERRUPT)
  //PCintPort::attachInterrupt(DOOR_SENSOR_PIN, &sendDoor, CHANGE);
  //PCintPort::attachInterrupt(LOCK_SENSOR_PIN, &sendLock, CHANGE);
  // Beide obere PCInts zu aktivieren führte zum Aufhängen, daher beides in einer Methode
  PCintPort::attachInterrupt(LOCK_SENSOR_PIN, &sendDoorAndLock, CHANGE);
  #endif

  // turn the LEDs off
  ledG.setOff();
  delay(100);
  ledY.setOff();
  delay(100);
  ledR.setOff();
  delay(100);
  ledB.setOff();
  //delay(100);
  //digitalWrite(PIN_LED_RED,0); // LED red
  delay(100);
  digitalWrite(PIN_LED_GREEN, 0); // LED green


  #if USE_WATCHDOG > 0
    // set watchdog
    //wdt_enable(WDTO_4S);
    wdt_reset();
  #endif

  // TEST ONLY!
  ledG.setBlinkInterval(250);
  ledY.setBlinkInterval(800);
  ledR.setBlinkInterval(100);
  ledB.setBlinkInterval(800);
}

void sendDoorAndLock() {
  #if MYDEBUG > 0
    Serial.println("PC_INT handler activated");
  #endif

  sendDoor();
  sendLock();
}

void loop()
{
  #if USE_WATCHDOG > 0
    // watchdog reset
    wdt_reset();
  #endif

  // Blink
  blink();
  
  // send sensor events
  if (mot_present) {
    sendMot();
  }

  // Blink
  blink();
  
  if (door_present) {
    sendDoor();
  }

  // Blink
  blink();
  
  if (lock_present) {
    sendLock();
  }

  // Blink
  blink();
  
  if (lux_present) {
    sendLux();
  }

  #if defined (SLEEP_TIME)
    // Sleep until interrupt comes in on motion sensor. Send update every X minutes.
    // Auchtung! Wird Zeitprobleme geben, Korrektur notwendig.
    sleep(INTERRUPT, CHANGE, SLEEP_TIME);
  #endif

  #if REPEATER == 1
    // process radio messages (use for repeater and aktor nodes)
    gw.process();
  #endif

  // Blink
  blink();
}

void blink() {
  ledR.blink();
  ledG.blink();
  ledB.blink();
  ledY.blink();
}

//-----------------------------------------------------------------------------------------------
// TODO: ggf. nach MSUtils verlagern
unsigned long timeCorrection = 0;
unsigned long cMillis() {
  return millis() + timeCorrection;
}

void sleep(unsigned long ms) {
  gw.sleep(ms);
  timeCorrection += ms;
}

bool sleep(uint8_t interrupt, uint8_t mode, unsigned long ms) {
  bool ret = gw.sleep(interrupt, mode, ms);
  if (ret) {
    // interrupted
    // Statistisch dürfe im Mittel dir Hälfte der Zeit ein akzeptabler Wert bei Interrupts sein
    timeCorrection += (ms / 2);
  } else {
    timeCorrection += ms;
  }
  return ret;
}

//-----------------------------------------------------------------------------------------------

unsigned long lastTimeLux = cMillis() + TIME_MIN_REPLAY_LUX + 1;
//boolean luxReadingRunning=false;
void sendLux()
{
  unsigned long time = cMillis();

  // Zeitdifferenz zum letzten Senden
  unsigned long delayTime = 0;
  // Auf Ueberlauf pruefen
  if (time < lastTimeLux)
  {
    // Ueberlauf: Delay ist Zeit zum MaxWert plus Zeit ab Null
    delayTime = MAX_U_LONG - lastTimeLux + time;
  } else {
    // Kein Ueberlauf: einfache Differenz
    delayTime = time - lastTimeLux;
  }

#if MYDEBUG > 1
  Serial.print("TimeDiff: ");
  Serial.println(delayTime);
#endif

  // Mindestabstand beachten
  if (delayTime < TIME_MIN_REPLAY_LUX)
  {
    return;
  }


  // Senden, nur wenn sich der Wert geändert hat
  // Nach verstreichen eines definierten Intervals soll in jedem Fall gesendet werden

  boolean sendAnyway = delayTime >= TIME_MAX_REPLAY_LUX;
  
  lightSensor.startMeasurementAsync(cMillis);
  while(!lightSensor.isMeasurementReady()) {
    //delay(lightSensor.nextDelay());
    unsigned long x = lightSensor.nextDelay();
    unsigned long st=cMillis();
    while(cMillis()<st+x) {
      delay(10);
      blink();
    } 
  }
  float lux=lightSensor.readLightLevelAsync();
  
  #if MYDEBUG > 0
    Serial.print("Lux: ");
    Serial.println(lux);
  #endif

  float diff = abs(lux - lastLux);
  boolean doSend = false;
  // Weil exponentielle Funktion, mehrere Stufen (vereinfacht)
  if (lastLux < 1) {
    doSend = diff > 0.5;
  } else if (lastLux < 5) {
    doSend = diff > 1;
  } else if (lastLux < 10) {
    doSend = diff > 2;
  } else if (lastLux < 100) {
    doSend = diff > 10;
  } else if (lastLux < 1000) {
    doSend = diff > 100;
  } else if (lastLux < 10000) {
    doSend = diff > 1000;
  } else {
    doSend = diff > 10000;
  }

  if (sendAnyway || doSend) {
    //gw.send(msgLux.set(lux));
    gw.send(msgLux.set(lux, 2));
    lastLux = lux;
    // Zeit merken
    lastTimeLux = cMillis();


    Serial.print("->Lux: ");
    Serial.println(lux);
  }
}

unsigned long lastTimeDoor = 0;
boolean doorStatus = false;
void sendDoor() {
  unsigned long time = cMillis();

  // Zeitdifferenz zum letzten Senden
  unsigned long delayTime = 0;
  // Auf Ueberlauf pruefen
  if (time < lastTimeDoor)
  {
    // Ueberlauf: Delay ist Zeit zum MaxWert plus Zeit ab Null
    delayTime = MAX_U_LONG - lastTimeDoor + time;
  } else {
    // Kein Ueberlauf: einfache Differenz
    delayTime = time - lastTimeDoor;
  }

  // Mindestabstand beachten
  if (delayTime < TIME_MIN_REPLAY_DOOR)
  {
    return;
  }

  // Senden, nur wenn sich der Wert geändert hat
  // Nach verstreichen eines definierten Intervals soll in jedem Fall gesendet werden

  boolean sendAnyway = delayTime >= TIME_MAX_REPLAY_DOOR;

  boolean newDoorStatus = digitalRead(DOOR_SENSOR_PIN) == HIGH;

#if MYDEBUG > 0
  Serial.print("Door: ");
  Serial.println(newDoorStatus ? "off" : "on");
#endif

  if (newDoorStatus != doorStatus || sendAnyway) {
    doorStatus = newDoorStatus;
    gw.send(msgDoor.set(newDoorStatus ? "0" : "1")); // Send value to gw
    // Zeit merken
    lastTimeDoor = cMillis();
  }
}

unsigned long lastTimeLock = 0;
boolean lockStatus = false;
void sendLock() {
  unsigned long time = cMillis();

  // Zeitdifferenz zum letzten Senden
  unsigned long delayTime = 0;
  // Auf Ueberlauf pruefen
  if (time < lastTimeLock)
  {
    // Ueberlauf: Delay ist Zeit zum MaxWert plus Zeit ab Null
    delayTime = MAX_U_LONG - lastTimeLock + time;
  } else {
    // Kein Ueberlauf: einfache Differenz
    delayTime = time - lastTimeLock;
  }

  // Mindestabstand beachten
  if (delayTime < TIME_MIN_REPLAY_LOCK)
  {
    return;
  }

  // Senden, nur wenn sich der Wert geändert hat
  // Nach verstreichen eines definierten Intervals soll in jedem Fall gesendet werden

  boolean sendAnyway = delayTime >= TIME_MAX_REPLAY_LOCK;

  boolean newLockStatus = digitalRead(LOCK_SENSOR_PIN) == HIGH;

#if MYDEBUG > 0
  Serial.print("Lock: ");
  Serial.println(newLockStatus ? "off" : "on");
#endif

  if (newLockStatus != lockStatus || sendAnyway) {
    lockStatus = newLockStatus;
    gw.send(msgLock.set(newLockStatus ? "0" : "1")); // Send value to gw
    // Zeit merken
    lastTimeLock = cMillis();
  }
}

unsigned long lastTimeMot = cMillis() + TIME_MIN_REPLAY_MOT + 1;
void sendMot()
{
  unsigned long time = cMillis();

  // Zeitdifferenz zum letzten Senden
  unsigned long delayTime = 0;
  // Auf Ueberlauf pruefen
  if (time < lastTimeMot)
  {
    // Ueberlauf: Delay ist Zeit zum MaxWert plus Zeit ab Null
    delayTime = MAX_U_LONG - lastTimeMot + time;
  } else {
    // Kein Ueberlauf: einfache Differenz
    delayTime = time - lastTimeMot;
  }
  // Mindestabstand beachten
  if (delayTime < TIME_MIN_REPLAY_MOT)
  {
    return;
  }

  // Read digital motion value
  boolean tripped = digitalRead(MOT_SENSOR_PIN) == HIGH;
  digitalWrite(PIN_LED_GREEN, tripped ? 1 : 0); //TEST TODO

#if MYDEBUG > 0
  Serial.print("Motion: ");
  Serial.println(tripped);
#endif

  if (tripped && !lastMot) // muss zwischendurch mal 'false' werden
  {
    //gw.send(msgMot.set(tripped?"1":"0"));  // Send tripped value to gw
    gw.send(msgMot.set("1"));  // Send tripped value to gw
    // Zeit merken
    lastTimeMot = cMillis();
  }

  lastMot = tripped;
}

int controllerCmd = 0;
void incomingMessage(const MyMessage &message) {
  // We only expect one type of message from controller. But we better check anyway.
  if (message.isAck()) {
    Serial.println("This is an ack from gateway");
  }

  if (message.type == V_VAR1) {
    // new controller command
    controllerCmd = message.getInt();
    // Format xyzzzz => x => LED-Nr. (), y => State (0->off, 1->on, 2->blink, 3->PIR-State, 4->door/lock state), zzzz => Blink interval *10ms (100 = 1000ms)
    

    // Zustände LEDs (Door/Lock, Windows...)
    // Green:  on -> Door closed and locked; blinking slow -> door closed but not locked; blinking fast-> door is open
    // Yellow: reserved
    // Red:    alarm (reserved)
    // Blue:   on -> all Windows closed; blinking slow -> at least one windows is tilted, all other are closed; blinking fast -> at least one window ist open; off -> unknown 


    // Write some debug info
    Serial.print("Incoming change for sensor:");
    Serial.print(message.sensor);
    Serial.print(", New status: ");
    Serial.println(message.getInt());
  }
}

