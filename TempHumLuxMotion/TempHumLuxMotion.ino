#include <SPI.h>
#include <MySensor.h>  
#include <DHT.h>  

//#include <BH1750.h>
#include <AS_BH1750.h>
#include <Wire.h> 

#define VERSION "1.2"

#define DEBUG 0
#define REPEATER 1

//-----------------------------------------------------------------------------------------------
#define MAX_U_LONG 4294967295;

// Sensor-Child-IDs
#define CHILD_ID_LIGHT 1
#define CHILD_ID_MOTION 2
#define CHILD_ID_TEMP 3
#define CHILD_ID_HUM 4

// Sensors PIN Config
#define HUMIDITY_SENSOR_DIGITAL_PIN 4 // The digital input you attached DHT sensot
#define MOT_SENSOR_PIN 3              // The digital input you attached your motion sensor.  (Only 2 and 3 generates interrupt!)
#define INTERRUPT MOT_SENSOR_PIN-2    // Usually the interrupt = pin -2 (on uno/nano anyway)
#define INTERRUPT_MODE CHANGE

// LED pins
//#define PIN_LED_RED 6
#define PIN_LED_GREEN 5

// Send time limits
#define TIME_MAX_REPLAY_LUX 600000 // Maximum time to send values (Lux) even if not changed
#define TIME_MAX_REPLAY_TH 600000  // Maximum time to send values (TH) even if not changed
#define TIME_MIN_REPLAY_LUX 1000   // Minimum time to send values (Lux) even if changed
#define TIME_MIN_REPLAY_TH  60000  // Minimum time to send values (TH) even if changed
#define TIME_MIN_REPLAY_MOT 1000   // Minimum time to send values (Motion) even if changed

#define MAX_DIFF_T 1 // Send immediately when the difference is greater than X degree
#define MAX_DIFF_H 5 // Send immediately when the difference is greater than X percent


//unsigned long SLEEP_TIME = 1000; // Sleep time between Distance reads (in milliseconds)
// Aufpassen bei SleepTime wg. Watchdog!
#define SLEEP_TIME 1000 // Sleep time between Distance reads (in milliseconds) (be careful with watchdogbe careful with watchdog (if used)!)


// Ideen: Parameter aus der Ferne aendern und in EEPROM speichern: MIN/MAX Time, Sende-Grenzwerte, LED-Benutzung

//-----------------------------------------------------------------------------------------------

MySensor gw;

//BH1750 lightSensor;
AS_BH1750 lightSensor;

MyMessage msgLux(CHILD_ID_LIGHT, V_LIGHT_LEVEL);
float lastLux=-1;

MyMessage msgMot(CHILD_ID_MOTION, V_TRIPPED);
uint16_t lastMot=0;

boolean metric = true; 

DHT dht;
float lastTemp=-999;
float lastHum=-1;

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);

//-----------------------------------------------------------------------------------------------
boolean mot_present = true; // can not be autodetected
boolean dht_present = false;
boolean lux_present = false;
//-----------------------------------------------------------------------------------------------

void setup()  
{
  #if REPEATER == 1
    // The third argument enables repeater mode.
    // Keep node awake all time (no slepping!). Pump the radio network by calling process() in your loop(). The MySensors library will take care of all routing in the background.
    gw.begin(NULL, AUTO, true);
  #else
    // Normal Node
    gw.begin();
  #endif
  
  // Init Sensors
  
  // autodetect DHT
  dht.setup(HUMIDITY_SENSOR_DIGITAL_PIN);
  // warm up
  delay(dht.getMinimumSamplingPeriod());
  float temperature = dht.getTemperature();
  dht_present=!isnan(temperature);
  #if DEBUG > 0
    if(!dht_present) { 
      Serial.println("LightSensor not found");
    } else {
      Serial.println("LightSensor found");
    }
  #endif

  // autodetect BH1750
  if(!lightSensor.begin()) {
    lux_present = false;
    #if DEBUG > 0
      Serial.println("LightSensor not found");
    #endif
  } else {
    lux_present = true;
    #if DEBUG > 0
      Serial.println("LightSensor found");
    #endif
  }
  
  // sets the motion sensor digital pin as input
  pinMode(MOT_SENSOR_PIN, INPUT);

  // Define LED pins
  //pinMode(PIN_LED_RED, OUTPUT);  
  pinMode(PIN_LED_GREEN, OUTPUT);    
  //digitalWrite(PIN_LED_RED,0); // LED red
  digitalWrite(PIN_LED_GREEN,0); // LED green
  
  // Send the sketch version information to the gateway and Controller
  //gw.sendSketchInfo("Temp+Hum+Lux+Motion", "1.0");
  const char *sketch_name = "Universal sensor (";
  //+(mot_present?"M ":""+lux_present?"L ":""+dht_present?"T H":"");
  if(mot_present) { sketch_name = strcat(const_cast<char*>(sketch_name),"M"); }
  if(lux_present) { sketch_name = strcat(const_cast<char*>(sketch_name),"L"); }
  if(dht_present) { sketch_name = strcat(const_cast<char*>(sketch_name),"TH"); }  
   sketch_name = strcat(const_cast<char*>(sketch_name),")");
  
  #if DEBUG > 0
    Serial.print("Sketch: ");
    Serial.print(sketch_name);
    Serial.print(" Version: ");
    Serial.println(VERSION);
  #endif
  
  gw.sendSketchInfo(sketch_name, VERSION);

  // Register all sensors to gw (they will be created as child devices)
  if(mot_present) {
    gw.present(CHILD_ID_MOTION, S_MOTION);  
  }
  
  if(lux_present) {
    gw.present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
  }
  
  if(dht_present) {
    gw.present(CHILD_ID_HUM, S_HUM);
    gw.present(CHILD_ID_TEMP, S_TEMP);
  }
  
  //metric = gw.getConfig().isMetric;  

  #if defined (INTERRUPT)
  if(mot_present) {
    attachInterrupt(INTERRUPT, sendMot, INTERRUPT_MODE);  
  }
  #endif
}

void loop()      
{ 
  // send sensor events
  if(mot_present) {
    sendMot();
  }
  
  if(lux_present) {
    sendLux();
  }
  
  if(dht_present) {
    sendTH();
  }
  
  #if defined (SLEEP_TIME)
    // Sleep until interrupt comes in on motion sensor. Send update every X minutes. 
    // Auchtung! Wird Zeitprobleme geben, Korrektur notwendig.
    gw.sleep(INTERRUPT,CHANGE, SLEEP_TIME);
  #endif
  
  #if REPEATER == 1
    // process radio messages (use for repeater and aktor nodes)
    gw.process();
  #endif
}

//-----------------------------------------------------------------------------------------------

unsigned long lastTimeTH=millis()+TIME_MIN_REPLAY_TH+1;
void sendTH()
{
  unsigned long time = millis();
  
  // Zeitdifferenz zum letzten Senden
  unsigned long delayTime = 0;
  
  // Auf Ueberlauf pruefen
  if(time<lastTimeTH) 
  {
    // Ueberlauf: Delay ist Zeit zum MaxWert plus Zeit ab Null
    delayTime = MAX_U_LONG-lastTimeTH+time;
  } else {
    // Kein Ueberlauf: einfache Differenz
    delayTime = time-lastTimeTH;
  }
  
  #if DEBUG > 1
    Serial.print("TimeDiff: ");
    Serial.println(delayTime);
  #endif
  
  // Mindestabstand beachten
  if(delayTime<TIME_MIN_REPLAY_TH)
  {
    return;
  }
  
  // Senden, nur wenn sich der Wert geändert hat
  // Nach verstreichen eines definierten Intervals soll in jedem Fall gesendet werden
  
  boolean sendAnyway = delayTime >= TIME_MAX_REPLAY_TH;
  
  // DHT22 braucht 2 Sekunden
  // delay(dht.getMinimumSamplingPeriod());

  float temperature = dht.getTemperature();
  if (isnan(temperature)) {
    #if DEBUG > 0
      Serial.println("Failed reading temperature from DHT");
    #endif
  } 
  else if (sendAnyway || (abs(temperature - lastTemp) > MAX_DIFF_T)) 
  {
    lastTemp = temperature;
    if (!metric) {
      temperature = dht.toFahrenheit(temperature);
    }
    
    #if DEBUG > 0
      Serial.print("T: ");
      Serial.println(temperature);
    #endif
    
    gw.send(msgTemp.set(temperature, 1));
    lastTimeTH=millis();
  }

  float humidity = dht.getHumidity();
  if (isnan(humidity)) {
    #if DEBUG > 0
      Serial.println("Failed reading humidity from DHT");
    #endif
  } 
  else if (sendAnyway || abs(humidity - lastHum) > MAX_DIFF_H)
  {
      lastHum = humidity;
      
      #if DEBUG > 0
        Serial.print("H: ");
        Serial.println(humidity);
      #endif
      
      gw.send(msgHum.set(humidity, 1));
      lastTimeTH=millis();
  }
}


unsigned long lastTimeLux=millis()+TIME_MIN_REPLAY_LUX+1;
void sendLux()    
{     
  
  // Prüfen, ob Sensor antwortet
  if(!lightSensor.isPresent()) {
    #if DEBUG > 0
      Serial.println("LightSensor not found");
    #endif
    return;  
  }
    
  unsigned long time = millis();
  
  // Zeitdifferenz zum letzten Senden
  unsigned long delayTime = 0;
  // Auf Ueberlauf pruefen
  if(time<lastTimeLux) 
  {
    // Ueberlauf: Delay ist Zeit zum MaxWert plus Zeit ab Null
    delayTime = MAX_U_LONG-lastTimeLux+time;
  } else {
    // Kein Ueberlauf: einfache Differenz
    delayTime = time-lastTimeLux;
  }
  
  #if DEBUG > 1
    Serial.print("TimeDiff: ");
    Serial.println(delayTime);
  #endif
  
  // Mindestabstand beachten
  if(delayTime<TIME_MIN_REPLAY_LUX)
  {
    return;
  }
  
  
  // Senden, nur wenn sich der Wert geändert hat
  // Nach verstreichen eines definierten Intervals soll in jedem Fall gesendet werden
  
  boolean sendAnyway = delayTime >= TIME_MAX_REPLAY_LUX;
  
  //uint16_t lux = lightSensor.readLightLevel();// Get Lux value
  float lux = lightSensor.readLightLevel();// Get Lux value
  
  #if DEBUG > 0
    Serial.print("Lux: ");
    Serial.println(lux);
  #endif
  
  float diff = abs(lux - lastLux);
  boolean doSend = false;
  // Weil exponentielle Funktion, mehrere Stufen
  if(lastLux<1) {
    doSend = diff>0.5;
  } else if (lastLux<5) {
    doSend = diff>1;
  } else if (lastLux<10) {
    doSend = diff>2;
  } else if (lastLux<100) {
    doSend = diff>10;
  } else if (lastLux<1000) {
    doSend = diff>100;
  } else if (lastLux<10000) {
    doSend = diff>1000;      
  } else {
    doSend = diff>10000;
  }

 
  // Senden, wenn Abweichung > als 1% ist.
  //uint16_t aDiff = abs(lux - lastLux);
  //uint16_t pAbw = (min(lux,lastLux)/100);
  
  //#if DEBUG > 1
  //Serial.print("aDiff: ");
  //Serial.println(aDiff);
  //Serial.print("pAbw: ");
  //Serial.println(pAbw);
  //#endif
  
  //if (sendAnyway || aDiff > pAbw) {
  if (sendAnyway || doSend) {
      //gw.send(msgLux.set(lux));
      gw.send(msgLux.set(lux,2));
      lastLux = lux;
      // Zeit merken
      lastTimeLux=millis();
  }
}

unsigned long lastTimeMot=millis()+TIME_MIN_REPLAY_MOT+1;
void sendMot()     
{
  unsigned long time = millis();
  
  // Zeitdifferenz zum letzten Senden
  unsigned long delayTime = 0;
  // Auf Ueberlauf pruefen
  if(time<lastTimeMot) 
  {
    // Ueberlauf: Delay ist Zeit zum MaxWert plus Zeit ab Null
    delayTime = MAX_U_LONG-lastTimeMot+time;
  } else {
    // Kein Ueberlauf: einfache Differenz
    delayTime = time-lastTimeMot;
  }
  // Mindestabstand beachten
  if(delayTime<TIME_MIN_REPLAY_MOT)
  {
    return;
  }
  
  // Read digital motion value
  boolean tripped = digitalRead(MOT_SENSOR_PIN) == HIGH; 
    digitalWrite(5,tripped?1:0); //TEST TODO
  
  #if DEBUG > 0
    Serial.print("Motion: ");
    Serial.println(tripped);
  #endif
  
  if(tripped && !lastMot) // muss zwischendurch mal 'false' werden
  {
    //gw.send(msgMot.set(tripped?"1":"0"));  // Send tripped value to gw 
    gw.send(msgMot.set("1"));  // Send tripped value to gw 
    // Zeit merken
    lastTimeMot=millis();
  }

  lastMot = tripped;
}

