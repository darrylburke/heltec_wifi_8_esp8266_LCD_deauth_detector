// This-->tab == "functions.h"

// Expose Espressif SDK functionality
extern "C" {
#include "user_interface.h"
  typedef void (*freedom_outside_cb_t)(uint8 status);
  int  wifi_register_send_pkt_freedom_cb(freedom_outside_cb_t cb);
  void wifi_unregister_send_pkt_freedom_cb(void);
  int  wifi_send_pkt_freedom(uint8 *buf, int len, bool sys_seq);
}

#include <ESP8266WiFi.h>
#include "./structures.h"

unsigned long prevTime = 0;
unsigned long curTime = 0;



#define MAX_APS_TRACKED 30
#define MAX_CLIENTS_TRACKED 100

bool tracking = false;
bool running=false;
beaconinfo aps_known[MAX_APS_TRACKED];                    // Array to save MACs of known APs
int aps_known_count = 0;                                  // Number of known APs
int nothing_new = 0;
clientinfo clients_known[MAX_CLIENTS_TRACKED];            // Array to save MACs of known CLIENTs
int clients_known_count = 0;                              // Number of known CLIENTs
long beacon_packet_counts[MAX_APS_TRACKED];
long device_packet_counts[MAX_CLIENTS_TRACKED];
long prev_beacon_packet_counts[MAX_APS_TRACKED];
long prev_device_packet_counts[MAX_CLIENTS_TRACKED];
int beacon_rssi[MAX_APS_TRACKED];
int device_rssi[MAX_CLIENTS_TRACKED];

int max_ap_packets=25;
int max_device_packets=8;

//used to keep track of last flooder for real  time tracking
String lastFlooderType="";
uint8_t lastFlooderbssid[ETH_MAC_LEN];

#include "heltec.h"

int register_beacon(beaconinfo beacon)
{
  int known = 0;   // Clear known flag
  for (int u = 0; u < aps_known_count; u++)
  {
    if (! memcmp(aps_known[u].bssid, beacon.bssid, ETH_MAC_LEN)) {
        beacon_packet_counts[u]++;
        aps_known[u].rssi=beacon.rssi;
        beacon_rssi[u]=beacon.rssi;
      known = 1;
      break;
    }   // AP known => Set known flag
  }
  if (! known)  // AP is NEW, copy MAC to array and return it
  {
    memcpy(&aps_known[aps_known_count], &beacon, sizeof(beacon));
    beacon_packet_counts[aps_known_count]=1;
     beacon_rssi[aps_known_count]=aps_known[aps_known_count].rssi;
    aps_known_count++;

    if ((unsigned int) aps_known_count >=
        sizeof (aps_known) / sizeof (aps_known[0]) ) {
      Serial.printf("exceeded max aps_known\n");
      aps_known_count = 0;
    }
  }
  return known;
}

int register_client(clientinfo ci)
{
  int known = 0;   // Clear known flag
//  if (ci.station[5]=53 && ci.station[0]==0x40){
//    for (int i = 0; i < 6; i++) Serial.printf("%02x", ci.station[i]); 
//    Serial.printf(" : %d\n",ci.rssi);
//  }
  for (int u = 0; u < clients_known_count; u++)
  {
    if (! memcmp(clients_known[u].station, ci.station, ETH_MAC_LEN)) {
      clients_known[u].rssi=ci.rssi;
       device_packet_counts[u]++;
       device_rssi[u]=ci.rssi;
      known = 1;
      break;
    }
  }
  if (! known)
  {
    memcpy(&clients_known[clients_known_count], &ci, sizeof(ci));
    device_packet_counts[clients_known_count]=1;
    device_rssi[clients_known_count]=clients_known[clients_known_count].rssi;
       
    clients_known_count++;
    
    if ((unsigned int) clients_known_count >=
        sizeof (clients_known) / sizeof (clients_known[0]) ) {
      Serial.printf("exceeded max clients_known\n");
      clients_known_count = 0;
    }
  }
  return known;
}

void print_beacon(beaconinfo beacon,bool crlf)
{
  if (beacon.err != 0) {
    //Serial.printf("BEACON ERR: (%d)  ", beacon.err);
  } else {
    Serial.printf("BEACON: <============ [%32s]  ", beacon.ssid);
    for (int i = 0; i < 6; i++) Serial.printf("%02x", beacon.bssid[i]);
    Serial.printf("   %2d", beacon.channel);
     if (crlf){
      Serial.printf("   %4d\r\n", beacon.rssi);
    }else {
      Serial.printf("   %4d", beacon.rssi);
    }
    
   
  }
}

void print_client(clientinfo ci,bool crlf)
{
  int u = 0;
  int known = 0;   // Clear known flag
  if (ci.err != 0) {
    // nothing
  } else {
    Serial.printf("DEVICE: ");
    for (int i = 0; i < 6; i++) Serial.printf("%02x", ci.station[i]);
    Serial.printf(" ==> ");
    int uindex=0;
    for (u = 0; u < aps_known_count; u++)
    {
      if (! memcmp(aps_known[u].bssid, ci.bssid, ETH_MAC_LEN)) {
        Serial.printf("[%32s]", aps_known[u].ssid);
        uindex=u;
        known = 1;     // AP known => Set known flag
        break;
      }
    }

    if (! known)  {
      Serial.printf("   Unknown/Malformed packet \r\n");
      //  for (int i = 0; i < 6; i++) Serial.printf("%02x", ci.bssid[i]);
    } else {
      Serial.printf("%2s", " ");
      for (int i = 0; i < 6; i++) Serial.printf("%02x", ci.ap[i]);
      Serial.printf("  %3d", aps_known[u].channel);
      if (crlf){
        Serial.printf("   %4d\r\n", device_rssi[uindex]);
      }else {
        Serial.printf("   %4d",device_rssi[uindex]);
      }
      
    }
  }
}

void print_header(){
  Serial.println(F("Type:   /-------MAC------/-----WiFi Access Point SSID-----/  /----MAC---/  Chnl  RSSI       Packets"));
}

bool check_flooder() {
for (int u = 0; u < aps_known_count; u++)
  {
    int diff = (beacon_packet_counts[u]-prev_beacon_packet_counts[u]);
    if ( diff > max_ap_packets){
      beaconinfo beacon = aps_known[u];
      char msg[60];
      sprintf(msg,"A[%s]%d/%d", beacon.ssid,beacon_rssi[u],diff);
      Serial.println(msg);
      heltecDisplayMsg((String)msg,100,1,false,true,false);

     lastFlooderType="AP";
     memcpy( lastFlooderbssid, beacon.bssid, ETH_MAC_LEN );
    
     
    }
    prev_beacon_packet_counts[u] = beacon_packet_counts[u];
  }
 for (int u = 0; u < clients_known_count; u++)
  {
    int diff = device_packet_counts[u]-prev_device_packet_counts[u];
   if (diff > max_device_packets){

    clientinfo client = clients_known[u];
     char msg[60];
      sprintf(msg,"C[%02x:%02x:%02x:%02x:%02x:%02x]%d/%d",client.station[0],client.station[1],client.station[2],client.station[3],client.station[4],client.station[5],device_rssi[u],diff);
      Serial.println(msg);
      heltecDisplayMsg((String)msg,100,1,false,true,false);
      lastFlooderType="CL";
      memcpy( lastFlooderbssid, client.station, ETH_MAC_LEN );
      
    
   }
   prev_device_packet_counts[u]=device_packet_counts[u];
  }
}
void print_beacons(){

   Serial.println("----------------Beacons--------------------------");
 print_header();
 for (int u = 0; u < aps_known_count; u++)
  {
  Serial.print(u);
  Serial.print(": ");
  print_beacon(aps_known[u],false);
  Serial.printf(" - %12ld\r\n", beacon_packet_counts[u]);
  }
    Serial.println("------------------------------------------------");
}
void print_devices(){

   Serial.println("----------------Clients--------------------------");
    print_header();
 for (int u = 0; u < clients_known_count; u++)
  {

  print_client(clients_known[u],false);
  Serial.printf(" - %12ld\r\n", device_packet_counts[u]);
  }
    Serial.println("------------------------------------------------");
}

bool array_cmp(uint8_t array1[ETH_MAC_LEN],uint8_t array2[ETH_MAC_LEN])
{
  
  for (int x=0;x<ETH_MAC_LEN;x++) {
    if (array1[x] != array2[x]) return false;
  }
  return true;
}

void promisc_cb(uint8_t *buf, uint16_t len)
{
  if (running) return;
 
  running=true;
//  if (tracking){
//    Serial.print("Tracking:[");
//    Serial.print(lastFlooderType);
//    Serial.print("] ");
//   char msg[60];
//   sprintf(msg," [%02x:%02x:%02x:%02x:%02x:%02x]",lastFlooderbssid[0],lastFlooderbssid[1],lastFlooderbssid[2],lastFlooderbssid[3],lastFlooderbssid[4],lastFlooderbssid[5]);
//   Serial.println(msg);
//  }
 
  

  
  int i = 0;
  uint16_t seq_n_new = 0;
  if (len == 12) {
    struct RxControl *sniffer = (struct RxControl*) buf;
  } else if (len == 128) {
    struct sniffer_buf2 *sniffer = (struct sniffer_buf2*) buf;
    struct beaconinfo beacon = parse_beacon(sniffer->buf, 112, sniffer->rx_ctrl.rssi);

    if (tracking) {

      if (lastFlooderType == "AP") {
          if (array_cmp(lastFlooderbssid, beacon.bssid) == true) {
              char msg[60];
              sprintf(msg,"A[%s]%d", beacon.ssid,beacon.rssi);
            //  Serial.println(msg);
              heltecDisplayMsg((String)msg,0,1,false,true,false);
          }
      }
      
    }else {

      if (register_beacon(beacon) == 0) {
     // print_beacon(beacon,true);
        nothing_new = 0;
      }
      
    }
    
  } else {
    struct sniffer_buf *sniffer = (struct sniffer_buf*) buf;
    //Is data or QOS?
    if ((sniffer->buf[0] == 0x08) || (sniffer->buf[0] == 0x88)) {
      struct clientinfo ci = parse_data(sniffer->buf, 36, sniffer->rx_ctrl.rssi, sniffer->rx_ctrl.channel);
     // if (memcmp(ci.bssid, ci.station, ETH_MAC_LEN)) {

         if (tracking){
            
            
            if (lastFlooderType == "CL") {
            //  Serial.print("X");
               if (array_cmp(lastFlooderbssid, ci.station) == true) {
                    char msg[60];
                    sprintf(msg,"C[%02x:%02x:%02x:%02x:%02x:%02x] %d",ci.station[0],ci.station[1],ci.station[2],ci.station[3],ci.station[4],ci.station[5],ci.rssi);
                //    Serial.println(msg);
                    heltecDisplayMsg((String)msg,0,1,false,true,false);
                }
            }
          
         } else {
              if (register_client(ci) == 0) {
             //   print_client(ci,true);
                nothing_new = 0;
              }
          
         }
        
   
    }
  }
   running=false;;
}

