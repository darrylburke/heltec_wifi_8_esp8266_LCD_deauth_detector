// by Ray Burnette 20161013 compiled on Linux 16.3 using Arduino 1.6.12

#include <ESP8266WiFi.h>
#include <Button.h>
#include "./functions.h"

unsigned long c = 0;


#define disable 0
#define enable  1
// uint8_t channel = 1;
unsigned int channel = 1;
#define SW1  14
#define SWLOW 12
#define SW2 13

Button button1(SW1);
Button button2(SW2);

  
void setupSwitch() {
  pinMode(SWLOW,OUTPUT);
  digitalWrite(SWLOW,LOW);
  button1.begin();
  button2.begin();
}
void setup() {
  Serial.begin(115200);
  setupSwitch();
  Serial.println(F("Based off of ESP8266 mini-sniff by Ray Burnette http://www.hackster.io/rayburne/projects"));
  
  heltecLCDsetup();
  heltecDrawBC();
  Serial.println("Init V4");



  
  wifi_set_opmode(STATION_MODE);            // Promiscuous works only with station mode
  wifi_set_channel(channel);
  wifi_promiscuous_enable(disable);
  wifi_set_promiscuous_rx_cb(promisc_cb);   // Set up promiscuous callback
  wifi_promiscuous_enable(enable);
   heltecDisplayMsg("Scanning",0,1,false,false,false);
   Serial.println("Scanning");
}
int counter = 0;
int maxcounter=15;
int mcounter=0;

void loop() {
  

  counter++;
  if (counter > maxcounter){
    counter=0;
    mcounter++;
    if (!tracking){
      String msg="Scanning....";
      msg+=(String)mcounter;
      heltecDisplayMsg(msg,0,1,false,true,false);
    }
   // print_beacons();
   // print_devices();
  }else {
  // Serial.print(counter);
  }

  channel = 1;
  wifi_set_channel(channel);
  while (true) {
    nothing_new++;                          // Array is not finite, check bounds and adjust if required
    if (nothing_new > 200) {
      nothing_new = 0;
      channel++;
      if (channel == 15) break;             // Only scan channels 1 to 14
      wifi_set_channel(channel);
    }
    delay(1);  // critical processing timeslice for NONOS SDK! No delay(0) yield()
    // Press keyboard ENTER in console with NL active to repaint the screen
//    if ((Serial.available() > 0) && (Serial.read() == '\n')) {
//      Serial.println("\n-------------------------------------------------------------------------------------\n");
//      print_beacons();
//      print_devices();
//     // for (int u = 0; u < clients_known_count; u++) print_client(clients_known[u],true);
//     // for (int u = 0; u < aps_known_count; u++) print_beacon(aps_known[u],true);
//      Serial.println("\n-------------------------------------------------------------------------------------\n");
//    }
  }
  //check counts to see if max reached to determine flooding

  if(button1.pressed()){
  //  Serial.println("Tracking Pressed");
    heltecClear();
     heltecDisplayMsg("Tracking",0,1,false,false,false);
    tracking=true;
  }
  if (button2.pressed()){
  //  Serial.println("Reset Pressed");
    
    heltecClear();
    heltecDisplayMsg("Scanning",0,1,false,false,false);
    tracking=false;
  }
  if (!tracking){
   //   Serial.println("S");
    check_flooder();
  } else {
  //  Serial.println("T");
  }

 
}

