#include <JeeLib.h>

#define MYNODE 20
#define freq RF12_868MHZ
#define group 1 

typedef struct { char type; int power1, power2, power3, battery; } PayloadTX;      // create structure - a neat way of packaging data for RF comms
PayloadTX emontx;

unsigned long timer;

//int idmap[30][1];
int autoid = 1;

char instr[30];
int pos = 0;

void setup()
{
  Serial.begin(9600);
  Serial.println("Base");
  
  rf12_initialize(MYNODE, freq, group);
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
      
      if (node_id==1)
      {
        autoid ++;
        char data[] = {'i',1,autoid};
        int i = 0; while (!rf12_canSend() && i<10) {rf12_recvDone(); i++;}
        rf12_sendStart(0, data, sizeof data);
        rf12_sendWait(0);
      }
        
      if (type=='d')
      {
        Serial.print("DATA: ");
        Serial.print(node_id);
        Serial.print(" ");
        byte n = rf12_len;
        for (byte i=1; i<n; i+=2)
        {
          int num = ((unsigned char)rf12_data[i+1] << 8 | (unsigned char)rf12_data[i]);
          if (i>1) Serial.print(",");
          Serial.print(num);
        }
        Serial.println();
      }
    }
  }

}
