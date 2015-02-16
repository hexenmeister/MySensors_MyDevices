#include <SPI.h>
#include <MySensor.h>  
#include <NewPing.h>
#include <DHT.h>  

#include <BH1750.h>
#include <Wire.h> 

#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1
#define CHILD_ID_DIS 2
#define CHILD_ID_LIGHT 3
#define CHILD_ID_MOTION 4
#define TRIGGER_PIN  6  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     5  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define HUMIDITY_SENSOR_DIGITAL_PIN 4
#define DIGITAL_INPUT_SENSOR 3   // The digital input you attached your motion sensor.  (Only 2 and 3 generates interrupt!)
#define INTERRUPT DIGITAL_INPUT_SENSOR-2 // Usually the interrupt = pin -2 (on uno/nano anyway)
#define MAX_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define TIME_MAX_REPLAY 600000 // Maximum time to send values (T/H/L) even if not changed

unsigned long TH_TIME = 60000; // time between Temp/Humidity reads (in milliseconds)
unsigned long DS_TIME = 1000; // Sleep time between Distance reads (in milliseconds)

MySensor gw;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
MyMessage msgDis(CHILD_ID_DIS, V_DISTANCE);
int lastDist;
boolean metric = true; 

DHT dht;
float lastTemp;
float lastHum;
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);

BH1750 lightSensor;
MyMessage msgLux(CHILD_ID_LIGHT, V_LIGHT_LEVEL);
uint16_t lastlux;

MyMessage msgMot(CHILD_ID_MOTION, V_TRIPPED);

void setup()  
{ 
  gw.begin();
  
  dht.setup(HUMIDITY_SENSOR_DIGITAL_PIN); 
  lightSensor.begin();
  
  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo("Hum+Temp+Dist+Lux", "1.0");

  // Register all sensors to gw (they will be created as child devices)
  gw.present(CHILD_ID_HUM, S_HUM);
  gw.present(CHILD_ID_TEMP, S_TEMP);
  gw.present(CHILD_ID_DIS, S_DISTANCE);
  gw.present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
  
  pinMode(DIGITAL_INPUT_SENSOR, INPUT);      // sets the motion sensor digital pin as input
  // Register all sensors to gw (they will be created as child devices)
  gw.present(CHILD_ID_MOTION, S_MOTION);
  
  boolean metric = gw.getConfig().isMetric;
}

// Senden, nur wenn sich der Wert geändert hat
// TODO: Nach verstreichen eines definierten Intervals soll in jedem Fall gesendet werden

void sendDist(boolean sendAnyway)
{
  int dist = metric?sonar.ping_cm():sonar.ping_in();
  Serial.print("Ping: ");
  Serial.print(dist); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  Serial.println(metric?" cm":" in");

  if (sendAnyway || dist != lastDist) {
      gw.send(msgDis.set(dist));
      lastDist = dist;
  }
}

void sendTH(boolean sendAnyway)
{
  delay(dht.getMinimumSamplingPeriod());

  float temperature = dht.getTemperature();
  if (isnan(temperature)) {
      Serial.println("Failed reading temperature from DHT");
  } else if (sendAnyway || temperature != lastTemp) {
    lastTemp = temperature;
    if (!metric) {
      temperature = dht.toFahrenheit(temperature);
    }
    gw.send(msgTemp.set(temperature, 1));
    Serial.print("T: ");
    Serial.println(temperature);
  }
  
  float humidity = dht.getHumidity();
  if (isnan(humidity)) {
      Serial.println("Failed reading humidity from DHT");
  } else if (sendAnyway || humidity != lastHum) {
      lastHum = humidity;
      gw.send(msgHum.set(humidity, 1));
      Serial.print("H: ");
      Serial.println(humidity);
  }
}

void sendLux(boolean sendAnyway)    
{     
  uint16_t lux = lightSensor.readLightLevel();// Get Lux value
  Serial.println(lux);
  // TODO: Senden, wenn ABweichung > als 10% (?) ist.
  if (sendAnyway || lux != lastlux) {
      gw.send(msgLux.set(lux));
      lastlux = lux;
  }
  
}

void sendMot()     
{
  // TODO: Inaktivitaetszeit?
  // Read digital motion value
  boolean tripped = digitalRead(DIGITAL_INPUT_SENSOR) == HIGH; 
        
  Serial.println(tripped);
  if(tripped)
  {
    //gw.send(msgMot.set(tripped?"1":"0"));  // Send tripped value to gw 
    gw.send(msgMot.set("1"));  // Send tripped value to gw 
  }
}

//unsigned long cdelay=0;
unsigned long start=0;
unsigned long timeMaxReplay = 0;

void loop()      
{ 
  unsigned long time = millis();
  
  sendMot();

  boolean sendAnyway = false;
  if(time-timeMaxReplay>=TIME_MAX_REPLAY) {
    sendAnyway = true;
  }

  if(time-start>=DS_TIME)
  {
    sendDist(sendAnyway);
    sendLux(sendAnyway);
  }
  //if(cdelay>=TH_TIME)
  if(time-start>=TH_TIME)
  {
    //cdelay=0;
    start=time;//millis();
    sendTH(sendAnyway);
  }

  //gw.sleep(DS_TIME);
  
  // Sleep until interrupt comes in on motion sensor. Send update every two minute. 
  gw.sleep(INTERRUPT,CHANGE, DS_TIME);
  
  //cdelay+=DS_TIME;
}

