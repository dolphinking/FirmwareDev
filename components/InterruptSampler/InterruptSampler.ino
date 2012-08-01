
unsigned long val = 0;
unsigned long timer;

void setup()
{
  Serial.begin(9600);

  ADMUX = (0x02 & 0x07); // Set ADC reference to external VFREF and first defined port
  ADCSRA |= (1 << ADEN);  // Enable ADC
  ADCSRA |= (1 << ADATE); // Enable auto-triggering
  ADCSRA |= (1 << ADIE);  // Enable ADC Interrupt

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
