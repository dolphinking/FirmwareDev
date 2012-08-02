
unsigned long val = 0;
unsigned long timer;

byte pin = 2;  // ADC pin

void setup()
{
  Serial.begin(9600);

  // Set ADC reference to external VFREF and first defined port
  // openenergymonitor.blogspot.co.uk/2012/08/low-level-adc-control-admux.html
  ADMUX = 0x40 | (pin & 0x07);  
  ADCSRA |= (1 << ADEN);        // Enable ADC
  ADCSRA |= (1 << ADATE);       // Enable auto-triggering
  ADCSRA |= (1 << ADIE);        // Enable ADC Interrupt

  sei();

  ADCSRA=0xEF;
}

ISR(ADC_vect){
  val ++;
}

void loop()
{
  if ((millis()-timer)>1000)
  {
    timer = millis();
    
    Serial.println(val);
    val = 0;
  }
  
}
