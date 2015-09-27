#include <avr/wdt.h>

// Prüfung, ob Watchdog funktioniert, oder das Davice in einer Reset-Schleife hängen bleibt

void setup() {
  wdt_disable(); //  bis hier kommt man wg. bootloop nicht mehr
  Serial.begin(9600);
  Serial.println("Setup..");
  
  Serial.println("Wait 5 sec..");
  delay(5000); // Pause, um beim bootloop Zeit zum reprogrammieren zu habn
  wdt_enable (WDTO_8S); // 
  Serial.println("Watchdog enabled.");
}

int timer = 0;

void loop(){
  // Wenns funktioniert, dann durch Blinken anzeigen
  boolean b = true;
  while(true){
    timer++;
    Serial.println(timer);
    digitalWrite(13, b?0:1);     
    b=!b;
    delay(1000);
  }
//  wdt_reset();
}
