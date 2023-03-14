/*
SCADA remote monitoring of control panel
Adapted from https://stackoverflow.com/questions/73701813/nrf24l01-arduino-mega-esp32s
NRF tutorial here: https://forum.arduino.cc/t/sending-multiple-values-with-nrf24l01/535766/4
... and here: https://forum.arduino.cc/t/sending-multiple-values-with-nrf24l01/535766/7
...and here https://forum.arduino.cc/t/simple-nrf24l01-2-4ghz-transceiver-demo/405123

========pinout information========
========OMRON data========
Gen1 on/off (D) - D3
Gen1 voltage (A) - A0
Gen1 Current (A) - A1
Gen2 on/off (D) - D4
Gen2 voltage (A) - A2
Gen2 Current (A) - A3
Pump1 on/off (D) - D5
Pump1 voltage (A) - A4
Pump1 Current (A) - A5
Pump2 on/off (D) - D6
Pump2 voltage (A) - A6
Pump2 Current (A) - A7
Power limit % (A) - A8
========enclosure temp and humidity sensor========
Temperature box DHT (D) - D2
========NRF pins========
CE - 8
CSN - 7

*/
//==============================LIBRARIES==============================
#include <Arduino.h> //as usual, delete this if working in Arduino IDE
#include <DHT.h> //include the DHT.h library (Adafruit Unified Sensor library pack)
#include <SPI.h> //include the SPI (serial peripheral interface) library
#include <RF24.h> //include the RF24 library for the NRF24 module
#include <nRF24L01.h> //probably needed for proper functioning of the NRF24 module

//==============================LIBRARY DEPENDANT DEFINITIONS==============================
#define DHTPIN1 2 //digital pin where the DHT data pin is connected to
#define DHTTYPE1 DHT11 //DHT11 sensor type
DHT dht_sensor1(DHTPIN1, DHTTYPE1); //initialize the DHT library object dht_sensor1 type DHTTYPE1 connected to pin DHTPIN1 

const int pinCE = 8; //NRF pin CE connected to pin 8 of the arduino
const int pinCSN = 7;  //NRF pin CSN connected to pin 7 of the arduino
RF24 tx_module(pinCE, pinCSN); //name of the module and pin connections
const byte address[6] = {"42"}; //address for the module, AFAIK both TX and RX modules should have same value here

//==============================PIN DEFINITIONS==============================
int gen1_switch_pin = 3; //digital pin where the gen1 ON/OFF status is connected to
int gen1_volts_pin = A0; //analog pin where the gen1 voltage sensor is connected to 
int gen1_amps_pin = A1; //analog pin where the gen1 current sensor is connected to

int gen2_switch_pin = 4; //digital pin where the gen2 ON/OFF status is connected to
int gen2_volts_pin = A2; //analog pin where the gen2 voltage sensor is connected to 
int gen2_amps_pin = A3; //analog pin where the gen2 current sensor is connected to

int pump1_switch_pin = 5; //digital pin where the pump1 ON/OFF status is connected to
int pump1_volts_pin = A4; //analog pin where the pump1 voltage sensor is connected to 
int pump1_amps_pin = A5; //analog pin where the pump1 current sensor is connected to

int pump2_switch_pin = 6; //digital pin where the pump2 ON/OFF status is connected to
int pump2_volts_pin = A6; //analog pin where the pump2 voltage sensor is connected to 
int pump2_amps_pin = A7; //analog pin where the pump2 current sensor is connected to

int powerlim_pin = A8; //analog pin where the power limit sensor is connected to

//==============================VARIABLES==============================
unsigned long prevmillis = 0; //for timer use (if needed to reduce sensor jitter)
const long interval = 200; //timer interval to reduce sensor jitter

int h = 0; //humidity value as read from dht_sensor1 NOTE for NRF use - 2 bytes
int t = 0; //temperature value as read from dht_sensor1 NOTE for NRF use - 2 bytes

int gen1_switch_state = 0; //state of gen1 (ON/OFF) NOTE for NRF use - 2 byte
int gen1_volts_val = 0; //voltage on gen1 NOTE for NRF use - 2 bytes
int gen1_amps_val = 0; //current on gen1NOTE for NRF use - 2 bytes

int gen2_switch_state = 0; //state of gen2 (ON/OFF) NOTE for NRF use - 2 byte
int gen2_volts_val = 0; //voltage on gen2 NOTE for NRF use - 2 bytes
int gen2_amps_val = 0; //current on gen2 NOTE for NRF use - 2 bytes

int pump1_switch_state = 0; //state of pump1 (ON/OFF) NOTE for NRF use - 2 byte
int pump1_volts_val = 0; //voltage on pump1 NOTE for NRF use - 2 bytes
int pump1_amps_val = 0; //current on pump1 NOTE for NRF use - 2 bytes

int pump2_switch_state = 0; //state of pump2 (ON/OFF) NOTE for NRF use - 2 byte
int pump2_volts_val = 0; //voltage on pump2 NOTE for NRF use - 2 bytes
int pump2_amps_val = 0; //current on pump2 NOTE for NRF use - 2 bytes

int powerlim_val = 0; //power limit value NOTE for NRF use - 2 bytes
//NOTE for NRF use - total so far 15 variables taking up 30 bytes

int v_max = 1000; //define the maximum voltage in the panel when input value on arduino is 5V (1023)
int a_max = 1935; //define the maximum current in the panel when input value on arduino is 5V (1023)
int plim_max = 100; //define the multiplier (percent) for the power limit sensor in the panel when input value on arduino is 5V (1023)


//==============================STRINGS FOR EASIER READING OF SERIAL==============================
char gen1_onstate[] = "ON ";
char gen1_offstate[] = "OFF";
char gen2_onstate[] = "ON ";
char gen2_offstate[] = "OFF";
char pump1_onstate[] = "ON ";
char pump1_offstate[] = "OFF";
char pump2_onstate[] = "ON ";
char pump2_offstate[] = "OFF";

//============================================================
void setup() {
Serial.begin(115200); //start serial communications for debugging
dht_sensor1.begin();  //start the dht_sensor1 object
tx_module.begin(); //initialize the TX module
tx_module.openWritingPipe(address); //open communication pipe
tx_module.setPALevel(RF24_PA_MAX); //set power of NRF module, can be changed to RF24_PA_MIN for lowest power consumption, BUT MUST BE SAME ON THE RX MODULE!!!
tx_module.setDataRate(RF24_250KBPS); //set bit rate to lowest possible value for best range
tx_module.setPayloadSize(32);
tx_module.stopListening(); //puts the NRF module into TRANSMIT MODE

//==============================PIN MODES==============================
pinMode(gen1_switch_pin, INPUT_PULLUP);
pinMode(gen1_volts_pin, INPUT);
pinMode(gen1_amps_pin, INPUT);

pinMode(gen2_switch_pin, INPUT_PULLUP);
pinMode(gen2_volts_pin, INPUT);
pinMode(gen2_amps_pin, INPUT);

pinMode(pump1_switch_pin, INPUT_PULLUP);
pinMode(pump1_volts_pin, INPUT);
pinMode(pump1_amps_pin, INPUT);

pinMode(pump2_switch_pin, INPUT_PULLUP);
pinMode(pump2_volts_pin, INPUT);
pinMode(pump2_amps_pin, INPUT);

pinMode(powerlim_pin, INPUT);

}


//============================================================
void loop() {

unsigned long currmillis = millis(); //for timer use to reduce sensor jitter

  if (currmillis - prevmillis >= interval){ //check if enough time has passed since last reading
    h = dht_sensor1.readHumidity(); //read humidity...
    //h_int = h_raw; //...and convert to integer
    t = dht_sensor1.readTemperature(); //read temperature in C...
    //t_int = t_raw; //...and convert to integer
    prevmillis = currmillis; //reset timer
  }

gen1_switch_state = !digitalRead(gen1_switch_pin); //read (and reverse due to INPUT_PULLUP) the gen1 state
gen1_volts_val = map(analogRead(gen1_volts_pin), 0, 1023, 0, v_max); //read and map voltage on gen1
gen1_amps_val = map(analogRead(gen1_amps_pin), 0, 1023, 0, a_max); //read and map current on gen1

gen2_switch_state = !digitalRead(gen2_switch_pin); //read (and reverse due to INPUT_PULLUP) the gen2 state
gen2_volts_val = map(analogRead(gen2_volts_pin), 0, 1023, 0, v_max); //read and map voltage on gen2
gen2_amps_val = map(analogRead(gen2_amps_pin), 0, 1023, 0, a_max); //read and map current on gen2

pump1_switch_state = !digitalRead(pump1_switch_pin); //read (and reverse due to INPUT_PULLUP) the pump1 state
pump1_volts_val = map(analogRead(pump1_volts_pin), 0, 1023, 0, v_max); //read and map voltage on pump1
pump1_amps_val = map(analogRead(pump1_amps_pin), 0, 1023, 0, a_max); //read and map current on pump1

pump2_switch_state = !digitalRead(pump2_switch_pin); //read (and reverse due to INPUT_PULLUP) the pump2 state
pump2_volts_val = map(analogRead(pump2_volts_pin), 0, 1023, 0, v_max); //read and map voltage on pump2
pump2_amps_val = map(analogRead(pump2_amps_pin), 0, 1023, 0, a_max); //read and map current on pump2

powerlim_val = map(analogRead(powerlim_pin), 0, 1023, 0, plim_max); //read and map power limit value

//==============================STORE DATA IN THE ARRAY nrf_data_packet TO BE SENT OVER THE NRF MODULE==============================
int nrf_data_packet[14]; //array of variables to be sent, the array seems one smaller than total number of variables because 0-9 not 1-10

nrf_data_packet[0] = h;
nrf_data_packet[1] = t;

nrf_data_packet[2] = gen1_switch_state;
nrf_data_packet[3] = gen1_volts_val;
nrf_data_packet[4] = gen1_amps_val;

nrf_data_packet[5] = gen2_switch_state;
nrf_data_packet[6] = gen2_volts_val;
nrf_data_packet[7] = gen2_amps_val;

nrf_data_packet[8] = pump1_switch_state;
nrf_data_packet[9] = pump1_volts_val;
nrf_data_packet[10] = pump1_amps_val;

nrf_data_packet[11] = pump2_switch_state;
nrf_data_packet[12] = pump2_volts_val;
nrf_data_packet[13] = pump2_amps_val;

nrf_data_packet[14] = powerlim_val;

//==============================SEND DATA OVER tx_module==============================
tx_module.write(&nrf_data_packet, 32); //sends the values to be transmitted

//==============================SERIAL OUTPUT==============================
Serial.print(" Humidity =:"); Serial.print(h); Serial.print("%"); Serial.print("["); Serial.print(nrf_data_packet[0]); Serial.print("]");//output humidity to serial, and value stored in array position[0]
Serial.print(" Temperature =:"); Serial.print(t); Serial.print("C"); Serial.print("["); Serial.print(nrf_data_packet[1]); Serial.print("]"); //output temperature to serial, and value stored in array position[1]

if (gen1_switch_state == 1){Serial.print("   G1S:"); Serial.print(gen1_onstate);} //output gen1 ON state
else {Serial.print("   G1S:"); Serial.print(gen1_offstate);} //output gen1 OFF state
Serial.print("["); Serial.print(nrf_data_packet[2]); Serial.print("]"); //value stored in array position[x]
Serial.print(" G1V:"); Serial.print(gen1_volts_val); Serial.print("V");Serial.print("["); Serial.print(nrf_data_packet[3]); Serial.print("]"); //output voltage on gen1, and value stored in array position[x]
Serial.print(" G1A:"); Serial.print(gen1_amps_val); Serial.print("A");Serial.print("["); Serial.print(nrf_data_packet[4]); Serial.print("]"); //output current on gen1

if (gen2_switch_state == 1){Serial.print("   G2S:"); Serial.print(gen2_onstate);} //output gen2 ON state
else {Serial.print("   G2S:"); Serial.print(gen2_offstate);} //output gen2 OFF state
Serial.print("["); Serial.print(nrf_data_packet[5]); Serial.print("]"); //value stored in array position[x]
Serial.print(" G2V:"); Serial.print(gen2_volts_val); Serial.print("["); Serial.print(nrf_data_packet[6]); Serial.print("]"); Serial.print("V"); //output gen2 voltage, and value stored in array position[x]
Serial.print(" G2A:"); Serial.print(gen2_amps_val); Serial.print("A"); Serial.print("["); Serial.print(nrf_data_packet[7]); Serial.print("]"); //output gen2 current, and value stored in array position[x]

if (pump1_switch_state == 1){Serial.print("   P1S:"); Serial.print(pump1_onstate);} //output pump1 ON state
else {Serial.print("   P1S:"); Serial.print(pump1_offstate);} //output pump1 OFF state
Serial.print("["); Serial.print(nrf_data_packet[8]); Serial.print("]"); //value stored in array position[x]
Serial.print(" P1V:"); Serial.print(pump1_volts_val); Serial.print("V"); Serial.print("["); Serial.print(nrf_data_packet[9]); Serial.print("]"); //output pump1 voltage, and value stored in array position[x]
Serial.print(" P1A:"); Serial.print(pump1_amps_val); Serial.print("A"); Serial.print("["); Serial.print(nrf_data_packet[10]); Serial.print("]"); //output pump1 current, and value stored in array position[x]

if (pump2_switch_state == 1){Serial.print("   P2S:"); Serial.print(pump2_onstate);} //output pump2 ON state
else {Serial.print("   P2S:"); Serial.print(pump2_offstate);} //output pump2 OFF state
Serial.print("["); Serial.print(nrf_data_packet[11]); Serial.print("]"); //value stored in array position[x]
Serial.print(" P2V:"); Serial.print(pump2_volts_val); Serial.print("V"); Serial.print("["); Serial.print(nrf_data_packet[12]); Serial.print("]"); //output pump2 voltage, and value stored in array position[x]
Serial.print(" P2A:"); Serial.print(pump2_amps_val); Serial.print("A"); Serial.print("["); Serial.print(nrf_data_packet[13]); Serial.print("]"); //output pump2 current, and value stored in array position[x]

Serial.print("   P_lim(%)"); Serial.print(powerlim_val); Serial.print("%"); Serial.print("["); Serial.print(nrf_data_packet[14]); Serial.print("]"); //output power limit value, and value stored in array position[x]

Serial.println(" "); //carriage return

}