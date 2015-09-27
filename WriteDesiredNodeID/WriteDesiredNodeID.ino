#include <SPI.h>
#include <EEPROM.h>  
#include <MySensor.h>  

#define NODE_ID 250

void setup()  
{ 
  Serial.begin(BAUD_RATE);
  
  Serial.print("Write desiren NodeID: ");
  Serial.print(NODE_ID);
  Serial.println(" Please wait...");

  uint8_t _nodeId = NODE_ID;
  hw_writeConfig(EEPROM_NODE_ID_ADDRESS, _nodeId);

  Serial.println("Done. You're ready to go!");
}

void loop()      
{ 
  boolean b = true;
  while(true){
    digitalWrite(13, b?0:1);     
    b=!b;
    delay(100);
  }
}

