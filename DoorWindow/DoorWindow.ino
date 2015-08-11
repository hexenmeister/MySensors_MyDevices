#include <SPI.h>
#include <MySensor.h> 

#define SKETCH_NAME "Door Window Sensor"
#define VERSION "1.1"

#define DEBUG 1
#define REPEATER 0
#define USE_WATCHDOG 0

//-----------------------------------------------------------------------------------------------
#define MAX_U_LONG 4294967295;

// Sensor-Child-IDs
#define CHILD_ID_DOOR_WIN 1

// Switch pin
#define SWITCH_PIN 3           // Arduino Digital I/O pin for button/reed switch or IR-Sensor
#define INTERRUPT SWITCH_PIN-2 // Usually the interrupt = pin -2 (on uno/nano anyway)

// LED pins
#define PIN_LED 7

// Send time limits
#define TIME_MAX_REPLAY_DOOR_WIN 600000 // Maximum time to send values even if not changed
#define TIME_MIN_REPLAY_DOOR_WIN 100    // Minimum time to send values even if changed

unsigned long SLEEP_TIME = 1000; // Sleep time between Distance reads (in milliseconds) (be careful with watchdogbe careful with watchdog (if used)!)

//-----------------------------------------------------------------------------------------------

MySensor gw;

MyMessage msg(CHILD_ID_DOOR_WIN, V_TRIPPED);
uint16_t lastVal=0;

void setup()
{ 
  #if USE_WATCHDOG > 0
  // Damit Watchdog korrekt funktioniert, muss bei mehreren Boards (z.B. Pro MiniU) ein anderer Bootloader installiert werden. Z.B. Optiboot
  // s. http://sysmagazine.com/posts/189744/ oder Originalpost: http://habrahabr.ru/post/189744/
  wdt_disable();
  // set watchdog
  wdt_enable(WDTO_8S);
  //wdt_reset();
  #endif
  
  #if DEBUG > 0
    Serial.print("Sketch: ");
    Serial.print(SKETCH_NAME);
    Serial.print(" Version: ");
    Serial.println(VERSION);
  #endif
  
  // initialize repeater or normal node
  #if REPEATER == 1
    // The third argument enables repeater mode.
    // Keep node awake all time (no slepping!). Pump the radio network by calling process() in your loop(). The MySensors library will take care of all routing in the background.
    gw.begin(NULL, AUTO, true);
  #else
    // Normal Node
    gw.begin();
  #endif

  // Setup the buttons
  //pinMode(SWITCH_PIN, INPUT);
  // Activate internal pull-ups
  //digitalWrite(SWITCH_PIN, HIGH);
  // should be a better way to activate the pull-ups
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  
  // Define LED pins
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED,0); // LED off

  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo(SKETCH_NAME, VERSION);

  // Register binary input sensor to sensor_node (they will be created as child devices)
  // You can use S_DOOR, S_MOTION or S_LIGHT here depending on your usage. 
  // If S_LIGHT is used, remember to update variable type you send in. See "msg" above.
  gw.present(CHILD_ID_DOOR_WIN, S_DOOR);  

  //metric = gw.getConfig().isMetric;
  
  #if USE_WATCHDOG > 0
  // set watchdog
  //wdt_enable(WDTO_2S);
  wdt_reset();
  #endif
}

void loop()      
{ 
  #if USE_WATCHDOG > 0
  // watchdog reset
  wdt_reset();
  #endif
  
  #if PIN_LED > 0
  // switch led off
  digitalWrite(PIN_LED,0);
  sendMsg();
  #endif
  
  #if defined (SLEEP_TIME)
    // Sleep until interrupt comes in on motion sensor. Send update every X minutes. 
    // Auchtung! Wird Zeitprobleme geben, Korrektur notwendig.
    gw.sleep(INTERRUPT, CHANGE, SLEEP_TIME);
  #endif
  
  #if REPEATER == 1
    // process radio messages (use for repeater and aktor nodes)
    gw.process();
  #endif
}

//-----------------------------------------------------------------------------------------------

unsigned long lastTime=cMillis()+TIME_MIN_REPLAY_DOOR_WIN+1;
void sendMsg()     
{
  unsigned long time = cMillis();
  
  // Zeitdifferenz zum letzten Senden
  unsigned long delayTime = 0;
  // Auf Ueberlauf pruefen
  if(time<lastTime) 
  {
    // Ueberlauf: Delay ist Zeit zum MaxWert plus Zeit ab Null
    delayTime = MAX_U_LONG-lastTime+time;
  } else {
    // Kein Ueberlauf: einfache Differenz
    delayTime = time-lastTime;
  }
  
  // Mindestabstand beachten
  if(delayTime<TIME_MIN_REPLAY_DOOR_WIN)
  {
    return;
  }

  boolean sendAnyway = delayTime >= TIME_MAX_REPLAY_DOOR_WIN;
  
  // Read digital motion value
  boolean tripped = digitalRead(SWITCH_PIN) == HIGH; 
  
  if(sendAnyway || tripped != lastVal)
  {
    #if PIN_LED > 0
    // switch led on
    digitalWrite(PIN_LED,1);
    #endif

    #if DEBUG > 0
      Serial.print("Value: ");
      Serial.println(tripped);
    #endif
    
    gw.send(msg.set(tripped?"1":"0"));  // Send tripped value to gw 
    // Zeit merken
    lastTime=cMillis();
  }

  lastVal = tripped;
}

//-----------------------------------------------------------------------------------------------
// TODO: ggf. nach MSUtils verlagern
unsigned long timeCorrection = 0;
unsigned long cMillis() {
  return millis()+timeCorrection;
}

void sleep(unsigned long ms) {
  gw.sleep(ms);
  timeCorrection += ms;
}

bool sleep(uint8_t interrupt, uint8_t mode, unsigned long ms) {
  bool ret = gw.sleep(interrupt,mode, ms);
  if(ret) {
    // interrupted
    // Statistisch dürfe im Mittel dir Hälfte der Zeit ein akzeptabler Wert bei Interrupts sein
    timeCorrection += (ms/2);
  } else {
    timeCorrection += ms;
  }
  return ret;
}

//-----------------------------------------------------------------------------------------------

