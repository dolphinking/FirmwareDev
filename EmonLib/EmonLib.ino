/*

  Development version of new EmonLib, that uses the atmega's free running interrupt based ADC sampling mode.

  //-----------------------------------------------------------------------------------------------------------------  
  
  FILTER PROBLEM
  
  I've been trying to implement the integer and long based high pass filter as used by Jorg Becker and AVR465 example
  but having trouble getting it to work its currently reporting the rms voltage at about 14000 rather than 244 V

  //-----------------------------------------------------------------------------------------------------------------  

  FURTHER DEVELOPMENT
  
  - zero crossing detection with complete measurements being made on an integer number of wavelengths, interestingly AVR465 does not do this
    it sets a constant number of samples that corresponds to an integer number of wavelengths for both 50Hz and 60Hz mains frequency. It must
    assume that variations in mains frequency and system sample rate are small enough not to significantly affect the reading. It would be
    integereting to do the maths to work out how valid this assumtion is.
    
  - turn ct1, ct2, ct3 on and off as needed
  - test how sample rate is affected by rf transmitts and ds18b20 temperature readings
  - wrap up as a arduino library
  - record accumulated energy in eprom
  
  - Dev Question: MartinR uses a phase locked loop approach to always sample over an integer number of wavelengths and sample
                  in the same place each time. But would a method that deviates slightly each time be better given issues around
                  ADC step size relative to signal, sampling at slightly different places each time could average out errors caused 
                  by sampled value being one step up or down from actual value..
              
  - Dev Question: MartinR uses low pass filters instead of high pass filters, what is the best solution?
  
  - Note: an important thing to watch is that the calc() function completes before the next adc sample starts as otherwise
          sample values get mixed up.
  
  //-----------------------------------------------------------------------------------------------------------------  

  Credit goes to the following people for developing and pushing forward this approach over the last year
  on which the code below is based and takes insight from:
  
  MartinR: http://openenergymonitor.org/emon/node/1535
  Robin Emley: http://openenergymonitor.org/emon/sites/default/files/Mk2i_PV_Router_rev5a.ino_.zip
  Pcunha: https://github.com/pcunha-lab/emonTxFirmware/tree/master/emonTx_Interrupts
  Jorg Becker: (a modification of Pcunha's code) http://openenergymonitor.org/emon/sites/default/files/EmonTx_Interrupt_JB.zip

  The code is also based on the great AVR465.c example.

  Licence: GNU GPL V3
  Author: Trystan Lea
  
*/

int analog_inputs_enabled[6] = {1,1,1,1,0,0};
signed int analog_input_values[6] = {0,0,0,0,0,0};

int conversion_in_progress = 0;
int sample_in_register;
int next_conversion = 0;

unsigned long timer;

double realPower, apparentPower, powerFactor, Vrms, Irms;

//Calibration coeficients
//These need to be set in order to obtain accurate results
double VCAL = 275;
double ICAL = 111.1;
double PHASECAL= 1.7;

int inPinV = 2;
signed int lastSampleV,sampleV;
signed long shifted_filteredV;
float sumV;

int inPinI1 = 0;
signed int lastSampleI1,sampleI1;
signed long shifted_filteredI1;
float sumI1,sumP1;

int inPinI2 = 1;
signed int lastSampleI2,sampleI2;
signed long shifted_filteredI2;
float sumI2,sumP2;

int inPinI3 = 3;
signed int lastSampleI3,sampleI3;
signed long shifted_filteredI3;
float sumI3,sumP3;

int numberOfSamples = 0;

void setup()
{
  Serial.begin(9600);

  // REFS0 sets AVcc with external capacitor on AREF pin
  // CT1PIN sets the analog input pin to start reading from
  ADMUX = _BV(REFS0) | next_conversion;
  
  ADCSRA = _BV(ADATE) | _BV(ADIE);
  
  ADCSRA |= _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);

  ADCSRA |= _BV(ADEN) | _BV(ADSC);
}

// ISR(ADC_vect) is the function called after each single channel ADC conversion is complete
// this function handle's cycling through all enabled analog inputs storing the result in the analog_input_values array
// The calc() function is called once all enabled analog inputs have been sampled.
ISR(ADC_vect)
{ 
  sample_in_register = conversion_in_progress;
  conversion_in_progress = next_conversion;
  
  analog_input_values[sample_in_register] = ADC;
  
  // Set the adc channel to read from the sample after the current one already underway
  // the value of which will not be in the next ISR call but the one after.
  
  // cycle through analog inputs to the next input which is enabled
  
  boolean next_conversion_set=false;
  while (next_conversion_set==false)
  {
    next_conversion++;
    // If we've looped through all analog inputs then go back to the start and call the calc function
    if (next_conversion>5) next_conversion = 0;
    if (analog_inputs_enabled[next_conversion]) {ADMUX = _BV(REFS0) | next_conversion; next_conversion_set = true;}
    
    // If we're starting at input zero again then a whole set of inputs have been sampled, time to do calcs
    if (next_conversion==0) calc();
  } 
}

void calc()
{
  signed long TempLong;
  static signed long filteredV;
  static signed long filteredI1;
  static signed long filteredI2;
  static signed long filteredI3;
  
  signed long n;
  
  numberOfSamples++;                                   // Count number of times looped.

  // VOLTAGE
  lastSampleV=sampleV;                                 // Used for digital high pass filter
  sampleV = analog_input_values[inPinV];               // Read in raw voltage signal

  TempLong = (long)(sampleV-lastSampleV)<<8;           // (sampleV-lastSampleV) * 256.0
  TempLong += shifted_filteredV;
  shifted_filteredV = TempLong - (TempLong>>8);
  filteredV = (shifted_filteredV+128)>>8;              // 128 as being  ½ << 8 
  sumV += filteredV * filteredV;

  // CT1
  lastSampleI1=sampleI1;
  sampleI1 = analog_input_values[inPinI1];
  
  TempLong = (long)(sampleI1-lastSampleI1)<<8;
  TempLong += shifted_filteredI1;
  shifted_filteredI1 = TempLong - (TempLong>>8);
  filteredI1 = (shifted_filteredI1+128)>>8;
  
  sumI1 += filteredI1 * filteredI1;
  sumP1 += filteredV * filteredI1;
  
  // CT2
  lastSampleI2=sampleI2;
  sampleI2 = analog_input_values[inPinI2]; 
  
  TempLong = (long)(sampleI2-lastSampleI2)<<8;
  TempLong += shifted_filteredI2;
  shifted_filteredI2 = TempLong - (TempLong>>8);
  filteredI2 = (shifted_filteredI2+128)>>8;
  
  sumI2 += filteredI2 * filteredI2;
  sumP2 += filteredV * filteredI2;
  
  // CT3
  lastSampleI3=sampleI3;
  sampleI3 = analog_input_values[inPinI3];
  
  TempLong = (long)(sampleI3-lastSampleI3)<<8;
  TempLong += shifted_filteredI3;
  shifted_filteredI3 = TempLong - (TempLong>>8);
  filteredI3 = (shifted_filteredI3+128)>>8;
  
  sumI3 += filteredI3 * filteredI3;
  sumP3 += filteredV * filteredI3;
  
  /*
  
  FLOATING POINT:
  filteredI1 = 0.996 * (filteredI1+sampleI1-lastSampleI1)
  
  BITWISE SIMPLEST FORM:
  n = lastFilteredI1 + sampleI1 - lastSampleI1;
  filteredI1 = ((n<<8)-n+128)>>8;

  BITWISE WORKS BETTER WITH SMALLER SIGNALS:
  TempLong = (long)(sampleI1-lastSampleI1)<<8;
  TempLong += shifted_filteredI1;
  shifted_filteredI1 = TempLong - (TempLong>>8);
  filteredI1 = (shifted_filteredI1+128)>>8;
  
  */
  
}

void loop()
{
  if ((millis()-timer)>1000)
  {
    timer = millis();
      
    //-------------------------------------------------------------------------------------------------------------------------
    // 3) Post loop calculations
    //------------------------------------------------------------------------------------------------------------------------- 
    //Calculation of the root of the mean of the voltage and current squared (rms)
    //Calibration coeficients applied. 
    
    int SUPPLYVOLTAGE = 3300;
    
    double V_RATIO = VCAL *((SUPPLYVOLTAGE/1000.0) / 1023.0);
    Vrms = V_RATIO * sqrt(sumV / numberOfSamples); 
    
    double I_RATIO = ICAL *((SUPPLYVOLTAGE/1000.0) / 1023.0);
    Irms = I_RATIO * sqrt(sumI1 / numberOfSamples); 

    //Calculation power values
    realPower = V_RATIO * I_RATIO * sumP1 / numberOfSamples;
    apparentPower = Vrms * Irms;
    powerFactor=realPower / apparentPower;

    //Reset accumulators
    sumV = 0;
    
    sumI1 = 0;
    sumP1 = 0;
    
    sumI2 = 0;
    sumP2 = 0;
    
    sumI3 = 0;
    sumP3 = 0;
    
    numberOfSamples = 0;
    
    Serial.print(realPower);
    Serial.print(' ');
    Serial.print(apparentPower);
    Serial.print(' ');
    Serial.print(Vrms);
    Serial.print(' ');
    Serial.print(Irms);
    Serial.print(' ');
    Serial.print(powerFactor);
    Serial.println(' ');
  }
}
