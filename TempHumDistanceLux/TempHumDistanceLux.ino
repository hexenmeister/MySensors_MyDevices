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
#define TRIGGER_PIN  6  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     5  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define HUMIDITY_SENSOR_DIGITAL_PIN 3
#define MAX_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
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
  boolean metric = gw.getConfig().isMetric;
}

// Senden, nur wenn sich der Wert geändert hat
// TODO: Nach verstreichen eines definierten Intervals soll in jedem Fall gesendet werden

void sendDist()
{
  int dist = metric?sonar.ping_cm():sonar.ping_in();
  Serial.print("Ping: ");
  Serial.print(dist); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  Serial.println(metric?" cm":" in");

  if (dist != lastDist) {
      gw.send(msgDis.set(dist));
      lastDist = dist;
  }
}

void sendTH()
{
  delay(dht.getMinimumSamplingPeriod());

  float temperature = dht.getTemperature();
  if (isnan(temperature)) {
      Serial.println("Failed reading temperature from DHT");
  } else if (temperature != lastTemp) {
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
  } else if (humidity != lastHum) {
      lastHum = humidity;
      gw.send(msgHum.set(humidity, 1));
      Serial.print("H: ");
      Serial.println(humidity);
  }
}

void sendLux()    
{     
  uint16_t lux = lightSensor.readLightLevel();// Get Lux value
  Serial.println(lux);
  // TODO: Senden, wenn ABweichung > als 10% (?) ist.
  if (lux != lastlux) {
      gw.send(msgLux.set(lux));
      lastlux = lux;
  }
  
}

unsigned long cdelay=0;
void loop()      
{ 
  sendDist();
  sendLux();
  if(cdelay>=TH_TIME)
  {
    cdelay=0;
    sendTH();
  }
  gw.sleep(DS_TIME);
  cdelay+=DS_TIME;
}

