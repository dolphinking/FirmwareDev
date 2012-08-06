int use = 354;
int gen = 2200;

// The packet to be sent is constructed in a char array that could be then max length of the packet.
char packet[50]; byte len = 0;

// CONSTRUCTOR FUNCTIONS

// Adds a type byte to the packet
void addtype(byte type) {packet[len++] = type;} 

// Adds an integer to the packet
void addint(int intval)
{
  packet[len++] = intval & 255;
  packet[len++] = intval >> 8;
}

// DECONSTRUCTOR FUNCTIONS

// Reads an integer at postion p
int readint(byte p)
{
  return ((unsigned char)packet[p+1] << 8 | (unsigned char)packet[p]);
}


void setup()
{
  Serial.begin(9600);
  
  // On the transmitter
  // Construct a packet holding 
  // | 10 | use | 10 | gen
  // | 1-byte | 2-bytes | 1-byte | 2-bytes |
  addtype(10); addint(use);
  addtype(10); addint(gen);
  
  // On the reciever
  // Deconstruct the packet
  byte pos = 0;
  while (pos<len)
  {
    // type 10 determines how to read the following bytes of data.
    // in this example we have arbitrary set type 10 to be an intger type ( 2 bytes)
    if (packet[pos]==10)
    {
      Serial.println(readint(pos+1));
      pos+=3;
    }
  }
  
  Serial.println("finished");
}

void loop() { }
