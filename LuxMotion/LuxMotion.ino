#include <SPI.h>
#include <MySensor.h>  
//#include <NewPing.h>
//#include <DHT.h>  

#include <BH1750.h>
#include <Wire.h> 

#define DEBUG 0
//-----------------------------------------------------------------------------------------------
#define MAX_U_LONG 4294967295;

#define CHILD_ID_LIGHT 1
#define CHILD_ID_MOTION 2
#define DIGITAL_INPUT_SENSOR 3   // The digital input you attached your motion sensor.  (Only 2 and 3 generates interrupt!)
#define INTERRUPT DIGITAL_INPUT_SENSOR-2 // Usually the interrupt = pin -2 (on uno/nano anyway)
#define TIME_MAX_REPLAY 600000 // Maximum time to send values (Lux) even if not changed
#define TIME_MIN_REPLAY 500 // Minimum time to send values (Lux) even if changed

#define PIN_LED_RED 5
#define PIN_LED_GREEN 6

unsigned long SLEEP_TIME = 1000; // Sleep time between Distance reads (in milliseconds)

// Ideen: Parameter aus der Ferne aendern und in EEPROM speichern: MIN/MAX Time, Sende-Grenzwerte, LED-Benutzung

//-----------------------------------------------------------------------------------------------

MySensor gw;
//boolean metric = true; 

BH1750 lightSensor;
MyMessage msgLux(CHILD_ID_LIGHT, V_LIGHT_LEVEL);
uint16_t lastLux;

MyMessage msgMot(CHILD_ID_MOTION, V_TRIPPED);
uint16_t lastMot;

//-----------------------------------------------------------------------------------------------

void setup()  
{
  gw.begin();
  
  lightSensor.begin();
  
  // sets the motion sensor digital pin as input
  pinMode(DIGITAL_INPUT_SENSOR, INPUT);

  // Define LED pins
  pinMode(PIN_LED_RED, OUTPUT);  
  pinMode(PIN_LED_GREEN, OUTPUT);    
  digitalWrite(PIN_LED_RED,0); // LED red
  digitalWrite(PIN_LED_GREEN,0); // LED green
  
  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo("Lux+Motion", "1.0");

  // Register all sensors to gw (they will be created as child devices)
  gw.present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
  gw.present(CHILD_ID_MOTION, S_MOTION);
  
  //metric = gw.getConfig().isMetric;
  
//  analogWrite(5,50);
//  analogWrite(6,250);
  
}


void loop()      
{ 
  // Ereignisse
  sendMot();
  sendLux();
  
  // Sleep until interrupt comes in on motion sensor. Send update every two minute. 
  //gw.sleep(INTERRUPT,CHANGE, SLEEP_TIME);
  gw.process();
}

//-----------------------------------------------------------------------------------------------

unsigned long lastTimeLux=millis();

void sendLux()    
{     
  unsigned long time = millis();
  
  // Zeitdifferenz zum letzten Senden
  unsigned long delay = 0;
  // Auf Ueberlauf pruefen
  if(time<lastTimeLux) 
  {
    // Ueberlauf: Delay ist Zeit zum MaxWert plus Zeit ab Null
    delay = MAX_U_LONG-lastTimeLux+time;
  } else {
    // Kein Ueberlauf: einfache Differenz
    delay = time-lastTimeLux;
  }
  
  #if DEBUG > 1
  Serial.print("TimeDiff: ");
  Serial.println(delay);
  #endif
  
  // Mindestabstand beachten
  if(delay<TIME_MIN_REPLAY)
  {
    return;
  }
  
  
  // Senden, nur wenn sich der Wert geändert hat
  // Nach verstreichen eines definierten Intervals soll in jedem Fall gesendet werden
  
  boolean sendAnyway = delay >= TIME_MAX_REPLAY;
  
  uint16_t lux = lightSensor.readLightLevel();// Get Lux value
  
  #if DEBUG > 0
  Serial.print("Lux: ");
  Serial.println(lux);
  #endif
  
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
  if (sendAnyway || lux!=lastLux) {
      gw.send(msgLux.set(lux));
      lastLux = lux;
      // Zeit merken
      lastTimeLux=millis();
  }

}

unsigned long lastTimeMot=millis();
void sendMot()     
{
  unsigned long time = millis();
  
  // Zeitdifferenz zum letzten Senden
  unsigned long delay = 0;
  // Auf Ueberlauf pruefen
  if(time<lastTimeMot) 
  {
    // Ueberlauf: Delay ist Zeit zum MaxWert plus Zeit ab Null
    delay = MAX_U_LONG-lastTimeMot+time;
  } else {
    // Kein Ueberlauf: einfache Differenz
    delay = time-lastTimeMot;
  }
  // Mindestabstand beachten
  if(delay<TIME_MIN_REPLAY)
  {
    return;
  }
  
  // Read digital motion value
  boolean tripped = digitalRead(DIGITAL_INPUT_SENSOR) == HIGH; 
    digitalWrite(6,tripped?1:0); //TEST
  
  #if DEBUG > 0
  Serial.print("Motion: ");
  Serial.println(tripped);
  #endif
  
      
  if(tripped && !lastMot)
  {
    //gw.send(msgMot.set(tripped?"1":"0"));  // Send tripped value to gw 
    gw.send(msgMot.set("1"));  // Send tripped value to gw 
    // Zeit merken
    lastTimeMot=millis();
  }

  lastMot = tripped;
}

