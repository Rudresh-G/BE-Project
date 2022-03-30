#include <FanMonitor.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/* ===================== SET PARAMETERS ========================= */

//define input pins
#define inputPin_Vltg A1
#define inputPin_Crnt A0
#define inputPin_TP1 4
#define inputPin_TP2 5
#define inputPin_TP3 6
#define sensePin 8
#define fanPWM 3
#define speedSelect_Pin 2

//Define Onewire
OneWire ds18x20[] = { inputPin_TP1, inputPin_TP2,inputPin_TP3 };
const int oneWireCount = sizeof(ds18x20)/sizeof(OneWire);
DallasTemperature sensor[oneWireCount];

//Define Fan Monitor
FanMonitor _fanMonitor = FanMonitor(sensePin, FAN_TYPE_BIPOLE);

//Time syncronization variables
const unsigned long samplingInterval = 2000;
unsigned long pre_time = 0;    //stores the time when previous sample was taken
unsigned long cycle_count = 0;



//initialize Variables
double V_probe = 0.0;   //voltage across probed device
double V_adc = 0.0;     //voltage at ADC for voltage sensing
double I_adc = 0.0;     //voltage at ADC for current sensing
double Current = 0.0;   //current through probed device
float Temps[] = {0.0, 0.0, 0.0}; // array holds temperaure readings from probes TP1 & TP2
const float dutyCycles[] = {0.0f, 0.2f, 0.5f, 0.8f, 1.0f};
unsigned int volatile dutySelect = 0;
unsigned long Time = 0; //store fan tacho pulse interval
unsigned int rpm = 0;   //stores RPM of fan


//Define constants
const float R1 = 30100.0;                 //Define resistances used for voltage divider
const float R2 = 7500.0;
const float vRef = 5.1;             //reference voltage
const double scale_factor = 0.066;  // for ACS712-30A current sensing module (66mV/Amp)
const double resConvert = 1024;     //range ADC which is 10-bit for UNO
double resADC = vRef/resConvert;    //resolution of ADC
double zeroPoint = vRef/2;             //no load output (ie. when current=0) of ACS712 is 2.5V, can be corrected

//10bit value at ADC
int adc_value = 0;

//Serial command
char serial_input = ' ';
bool carry_out = false;
bool display_info = false;
bool headers = false;

//Auto reset
void(*resetFunc) (void) = 0;    //declare reset function at address 0





/* ===================== CODE BEGINS HERE ======================= */

void setup() {
  
  //enable outputs for Timer 2
    pinMode(fanPWM,OUTPUT); //1A 
    setupTimer2();
    pinMode(speedSelect_Pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(speedSelect_Pin),changeDutyISR,FALLING);
    setPWM2(dutyCycles[dutySelect]);   // set duty cycle on pin 9
    //pinMode(sensePin, HIGH);
  
  // Begin communication:
  Serial.begin(115200);
  //_fanMonitor.begin();

  if (display_info) {
    Serial.println("MEASUREMENT");
    Serial.print("============Ready with ");
    Serial.print(oneWireCount);
    Serial.println(" Temperature Sensors================");
    Serial.println("Enter 'b' to begin measurement and 't' to terminate");
  }
  
  if (headers){
    Serial.println("Voltage[V],Current[mA],Temp-1[C],Temp-2[C],Temp-3[C],Fan_Speed[RPM],PWM_Duty[%]");
  }

  //analogReference(DEFAULT);      //If using internal 1.1V reference, probing voltage should be <5.51V
  
  // Start up the library on all defined bus-wires
  DeviceAddress deviceAddress;
  for (int i = 0; i < oneWireCount; i++) {;
    sensor[i].setOneWire(&ds18x20[i]);
    sensor[i].begin();
    if (sensor[i].getAddress(deviceAddress, 0)) sensor[i].setResolution(deviceAddress, 12);
  }

}


void loop() {


  if (Serial.available() > 0 )
  {
    switch (Serial.read())
    {
    case 'b':
      carry_out = true;
      break;
    case 't':
      carry_out = false;
      resetFunc();          //for reseting arduino
      break;
    }
  }
  if (carry_out)
  {
      
      cycle_count += 1;
      
      //Voltage Measurement
      V_adc = resADC * analogRead(inputPin_Vltg);    // Read Analogue input and determine voltage at bridge
      V_probe += (V_adc*(R1 + R2)/R2);     //Calculate Input voltage

      //Current Measurement
      I_adc = (resADC * analogRead(inputPin_Crnt));   
      Current += ((I_adc - zeroPoint)/ scale_factor);    // Convert Vout into Current using Scale Factor 

      //Temperature Measurement
      // request to all devices on the bus
      for (int i = 0; i < oneWireCount; i++) {
        sensor[i].requestTemperatures();
      }
      for (int i = 0; i < oneWireCount; i++) {
        Temps[i] += sensor[i].getTempCByIndex(0);
      }

      //RPM measurement
      //rpm = _fanMonitor.getSpeed();

      unsigned volatile long curr_time = millis();
      //Print Results once every sampling interval
      if(curr_time - pre_time >= samplingInterval) {
        Serial.print(V_probe/cycle_count);
        Serial.print(",");
        Serial.print(Current/cycle_count);
        Serial.print(",");
        Serial.print(Temps[0]/cycle_count);
        Serial.print(",");
        Serial.print(Temps[1]/cycle_count);
        Serial.print(",");
        Serial.print(Temps[2]/cycle_count);
        Serial.print(",");
        Serial.print(rpm);
        Serial.print(",");
        Serial.println(100*dutyCycles[dutySelect],0);

        //reinitiate buffers
        V_probe = 0;
        Current = 0;
        Temps[0] = 0;
        Temps[1] = 0;
        Temps[2] = 0;
        
        cycle_count = 0;
        pre_time = curr_time;
      }
      
      }
    
  }



/*================== Functions =====================*/

  //configure Timer 1 (pins 9,10) to output 25kHz PWM
void setupTimer1(){
    //Set PWM frequency to about 25khz on pins 9,10 (timer 1 mode 10, no prescale, count to 320)
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
    TCCR1B = (1 << CS10) | (1 << WGM13);
    ICR1 = 320;
    OCR1A = 0;
    OCR1B = 0;
}
//configure Timer 2 (pin 3) to output 25kHz PWM. Pin 11 will be unavailable for output in this mode
void setupTimer2(){
    //Set PWM frequency to about 25khz on pin 3 (timer 2 mode 5, prescale 8, count to 79)
    TIMSK2 = 0;
    TIFR2 = 0;
    TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
    TCCR2B = (1 << WGM22) | (1 << CS21);
    OCR2A = 79;
    OCR2B = 0;
}
//equivalent of analogWrite on pin 9
void setPWM1A(float f){
    f=f<0?0:f>1?1:f;
    OCR1A = (uint16_t)(320*f);
}
//equivalent of analogWrite on pin 10
void setPWM1B(float f){
    f=f<0?0:f>1?1:f;
    OCR1B = (uint16_t)(320*f);
}
//equivalent of analogWrite on pin 3
void setPWM2(float f){
    f=f<0?0:f>1?1:f;
    OCR2B = (uint8_t)(79*f);
}


//Interrupt function for changing duty cycle
void changeDutyISR() {
  dutySelect += dutySelect>=0 and dutySelect<4?1:-4;
  setPWM2(dutyCycles[dutySelect]);
}

/*//Reading RPM of fan
void getRPMS() {
 Time = pulseIn(sensePin, HIGH);
 rpm = (1000000 * 60) / (Time * 4);
 
 } 
 */
