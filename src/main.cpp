/*
 * Class for Spunder
 * A Spunder consists of a pressure transducer and a relay controlled solenoid valve
 * 
 * Default setup here is 4 Spunders one thermometer for all four (bulk-aging) Spunders
 * Board - Arduino Mega
 * ADC - 10 bit (0 - 1024)
 * Transducer - 3 wires 5v, ground, signal
 * Relay      - 2 wires switchleg and ground
 * 
 * 7/1/21 added streaming library for more concise output 
*/

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Streaming.h>

#define NUMBER_OF_SPUNDERS 4
#define RELAY_OPEN HIGH
#define ONE_WIRE_BUS 21

// Setup a oneWire instance & pass it to Dallas Temperature.
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Arduino pins and desired carbonation in vols.
const int SENSOR_PINS[NUMBER_OF_SPUNDERS] = {0, 1, 2, 3};      // analog pin of the arduino that the transducer is connected to
const int RELAY_PINS[NUMBER_OF_SPUNDERS] = {4, 5, 6, 7};       // digital pin of the arduino relay is connected to. (current pins are default for relay shield)
float DESIRED_VOLS[NUMBER_OF_SPUNDERS] = {4.0, 4.0, 4.0, 4.0}; // target carbonation in volumes of co2

class Spunder
{
public:
  int id;                                      // Spunder ID number
  int sensor_pin;                              // Arduino pin of the transducer
  int sensor_rating = 60;                      // Pressure rating of the transducer in PSI
  int sensor_offset = 102;                     // Bits from 0v - .5v
  int sensor_fullscale = 1024 - sensor_offset; // Bits from 0v - 4.5v
  int relay_pin;                               // Arduino pin of the spunder valve relay
  int stored_time;                             // Time of last mins_since_vent
  int mins_since_vent;                         // Time since last vent
  float vols_setpoint;                         // Desired co2 in vols
  float vols_value;                            // Actual co2 in vols
  float psi_setpoint;                          // PSI target
  float psi_value;                             // Actual PSI
  float tempC;                                 // Temp in Celsius
  float tempF;                                 // Temp in Fahrenheit

  void init_vols_setpoint(float vs) { vols_setpoint = vs; }

  float get_tempC() { return sensors.getTempCByIndex(0); }

  float get_tempF() { return sensors.getTempFByIndex(0); }

  float get_psi_value() { return (analogRead(sensor_pin) - sensor_offset) * sensor_rating / float(sensor_fullscale - sensor_offset); }

  float get_psi_setpoint() { return (-16.669 - (.0101059 * tempF)) + (.00116512 * (tempF * tempF)) + (.173354 * tempF * vols_setpoint) + (4.24267 * vols_setpoint) - (.0684226 * (vols_setpoint * vols_setpoint)); }

  float get_vols_value() { return (psi_value / psi_setpoint) * vols_setpoint; }

  float get_vent_value() { return (millis() - stored_time) / 60000; }
};

// Create an array of Spunders
Spunder spunder_arr[NUMBER_OF_SPUNDERS];

// Start serial and spunders, set valves to closed.
void setup()
{
  Serial.begin(9600);
  sensors.begin();

  for (int i = 0; i < NUMBER_OF_SPUNDERS; i++)
  {
    spunder_arr[i].id = i + 1;
    spunder_arr[i].relay_pin = RELAY_PINS[i];
    spunder_arr[i].sensor_pin = SENSOR_PINS[i];
    spunder_arr[i].vols_setpoint = DESIRED_VOLS[i];
    spunder_arr[i].stored_time = millis();

    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], !RELAY_OPEN);
  }
}

// Get Data -> print it out.
void loop()
{
  sensors.requestTemperatures();
  Serial << ("{") << endl;

  for (int i = 0; i < NUMBER_OF_SPUNDERS; i++)
  {
    spunder_arr[i].tempC = spunder_arr[i].get_tempC();
    spunder_arr[i].tempF = spunder_arr[i].get_tempF();
    spunder_arr[i].psi_setpoint = spunder_arr[i].get_psi_setpoint();
    spunder_arr[i].psi_value = spunder_arr[i].get_psi_value();
    spunder_arr[i].vols_value = spunder_arr[i].get_vols_value();
    spunder_arr[i].mins_since_vent = spunder_arr[i].get_vent_value();

    if (spunder_arr[i].psi_value > spunder_arr[i].psi_setpoint)
    {
      digitalWrite(spunder_arr[i].relay_pin, RELAY_OPEN);
      delay(100);
      digitalWrite(spunder_arr[i].relay_pin, !RELAY_OPEN);
      spunder_arr[i].stored_time = millis();
      spunder_arr[i].mins_since_vent = 0;
    }
    else
    {
      spunder_arr[i].mins_since_vent = (millis() - spunder_arr[i].stored_time) / 60000;
    }

    Serial << "\"" << spunder_arr[i].id << "\": {" << endl;
    Serial << "\"psi target\" : " << spunder_arr[i].psi_setpoint << ", " << endl;
    Serial << "\"actual psi\" : " << spunder_arr[i].psi_value << ", " << endl;
    Serial << "\"vol target\" : " << spunder_arr[i].vols_setpoint << ", " << endl;
    Serial << "\"actual vols\" : " << spunder_arr[i].vols_value << ", " << endl;
    Serial << "\"temperature\" : " << spunder_arr[i].tempC << ", " << endl;
    Serial << "\"mins_since_vent\" : " << spunder_arr[i].mins_since_vent;
    if (i != (NUMBER_OF_SPUNDERS - 1))
    {
      Serial << " }, " << endl;
      Serial << " " << endl;
    }
    else
    {
      Serial << " }" << endl;
      Serial << "}" << endl;
    }
  }
  delay(5000);
}
