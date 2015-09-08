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

  hw_writeConfig(EEPROM_NODE_ID_ADDRESS, NODE_ID);

  Serial.println("Done. You're ready to go!");
}

void loop()      
{ 
  // Nothing to do here...
}

