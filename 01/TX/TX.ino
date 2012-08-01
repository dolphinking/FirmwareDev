#include <JeeLib.h>

int mynode = 1;
#define freq RF12_868MHZ
#define group 1

typedef struct { char type; int power1, power2, power3, battery; } PayloadTX;      // create structure - a neat way of packaging data for RF comms
PayloadTX emontx;

unsigned long timer;

void setup()
{
  Serial.begin(9600);
  Serial.println("tx");
  
  rf12_initialize(mynode, freq, group);
}

void loop()
{
  //------------------------------------------------------------------
  // RX data
  //------------------------------------------------------------------
  if (rf12_recvDone())
  {
    if (rf12_crc == 0 && (rf12_hdr & RF12_HDR_CTL) == 0)  // and no rf errors
    {
      int node_id = (rf12_hdr & 0x1F);
      char type = rf12_data[0];
        
      if (type=='i')
      {
        int target = rf12_data[1];
        int newid = rf12_data[2];
        if (target==mynode) mynode = newid;
        rf12_initialize(mynode, freq, group);
      }
    }
  }
  
  //------------------------------------------------------------------
  // Transmit data
  //------------------------------------------------------------------
  emontx.type = 'd';
  emontx.power1 = 520;
  emontx.power2 = 1209;
  emontx.power3 = 1000;
  emontx.battery = 3300;
  
  if ((millis()-timer)>1000)
  {
    timer = millis();
    int i = 0; while (!rf12_canSend() && i<10) {rf12_recvDone(); i++;}
    rf12_sendStart(0, &emontx, sizeof emontx);
    rf12_sendWait(0);
  }
}
