// To measure approximate realpower in a 3 phase system with one voltage sensor on one phase
// We need to create two voltage waveforms that are offset by +120 degrees and +240 degrees
// A good solution is to use a circular buffer

// Buffer size
#define SIZE 10

// How far back in history do we want to go? MAX is SIZE - 1
#define OFFSET 9

double buffer[SIZE];
unsigned char pos = (SIZE-1);

// A dummy variable
double val = 0;

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  // Create a random example value
  val = random(300);
  
  // Insert the value in the buffer at the next position
  if (--pos == 255) pos = SIZE-1;
  buffer[pos] = val;
  
  // Get the value at offset
  // % is http://en.wikipedia.org/wiki/Modulo_operation
  unsigned char offset_pos = (pos + OFFSET) % SIZE;
  double offset_val = buffer[offset_pos];
  
  // Print to serial
  Serial.print (val);
  Serial.print (' ');
  Serial.println (offset_val);
  
  delay(1000);
}
