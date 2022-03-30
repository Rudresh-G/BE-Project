/* --- SET PARAMETERS --- */

//define input pin
#define inputPin A0

//Define resistances used
float R1 = 27000.0;
float R2 = 10000.0;

//Voltages for input and bridge
float V_input = 0.0;
float V_adc = 0.0;
float current = 0.0;
float power = 0.0;

//reference voltage
const float Vref = 1.10;

//10bit value at ADC
int adc_value = 0;

//Serial command
char serial_input = ' ';
bool carry_out = false;
bool headers = true;

/* --- CODE BEGINS HERE --- */

void setup() {
  // Begin communication:
Serial.begin(9600);
//Serial.println("VOLTAGE MEASUREMENT");
//Serial.println("Enter 'b' to begin measurement and 't' to terminate");
if (headers){
  Serial.println("voltage[V],Current[mA],Power[mW]");
}

analogReference(INTERNAL);
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
      break;
    }
  }
  if (carry_out)
  {
      // Read Analogue input
      adc_value = analogRead(inputPin);

      // Determine voltage at bridge
      V_adc = (adc_value*Vref)/1024.0;

      //Calculate Input voltage
      V_input = V_adc*(R1 + R2)/R2;

      //calculate current and power
      current = V_adc/R2*1000;
      power = V_input*current*1000;

      //Print Results
      Serial.print(V_input);
      Serial.print(",");
      Serial.print(current);
      Serial.print(",");
      Serial.println(power);
      
      //delay
      delay(1000);
      }
    
  }
