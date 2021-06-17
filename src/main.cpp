/*
 * Class for Spunder
 * A Spunder consists of a pressure transducer and a relay controlled solenoid valve
 * 
 * Default setup here is 4 Spunders one thermometer for all four (bulk-aging) Spunders
 * Board - Arduino Mega
 * ADC - 10 bit (0 - 1024)
 * Transducer - 3 wires 5v, ground, signal
 * Relay      - 2 wires switchleg and ground 
*/

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
using namespace std;

#define NUMBER_OF_SPUNDERS 4
#define RELAY_OPEN HIGH
#define ONE_WIRE_BUS 3

// Setup a oneWire instance & pass it to Dallas Temperature.
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Arduino pins and desired carbonation in vols.
const int SENSOR_PINS[NUMBER_OF_SPUNDERS] = {0, 1, 2, 3};
const int RELAY_PINS[NUMBER_OF_SPUNDERS] = {4, 5, 6, 7};
float DESIRED_VOLS[NUMBER_OF_SPUNDERS] = {1.0, 2.0, 3.0, 4.0};
class Spunder
{
public:
  const static int sensor_rating = 60;     // Pressure rating of the transducer in PSI
  const static int sensor_offset = 102;    // Bits from 0v - .5v
  const static int sensor_fullscale = 922; // Bits from .5v - 5v

  int id;          // Spunder ID number
  int sensor_pin;  // Arduino pin of the transducer
  int relay_pin;   // Arduino pin of the spunder valve relay
  int stored_time; // Time of last vent
  int vent;        // Time since last vent

  float vols_setpoint; // Desired co2 in vols
  float vols_value;    // Actual co2 in vols
  float psi_setpoint;  // PSI target
  float psi_value;     // Actual PSI
  float tempC;         // Temp in Celsius
  float tempF;         // Temp in Fahrenheit

  void init_vols_setpoint(float vs) { vols_setpoint = vs; }

  float get_tempC() { return sensors.getTempCByIndex(0); }

  float get_tempF() { return sensors.getTempFByIndex(0); }

  float get_psi_value() { return (analogRead(sensor_pin) - sensor_offset) * sensor_rating / float(sensor_fullscale - sensor_offset); }

  float get_psi_setpoint(float vols_setpoint, float tempF) { return (-16.669 - (.0101059 * tempF)) + (.00116512 * (tempF * tempF)) + (.173354 * tempF * vols_setpoint) + (4.24267 * vols_setpoint) - (.0684226 * (vols_setpoint * vols_setpoint)); }

  float get_vols_value(float psi_value, float psi_setpoint, float vols_setpoint) { return (psi_value / psi_setpoint) * vols_setpoint; }

  int get_vent_value(float psi_value, float psi_setpoint, int stored_time, int relay_pin) { return (millis() - stored_time) / 60000; }
};
Spunder spunder_arr[NUMBER_OF_SPUNDERS];

// start serial and spunders, set valves to closed.
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
    spunder_arr[i].stored_time = 0;

    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], !RELAY_OPEN);
  }
}

void loop()
{
  sensors.requestTemperatures();
  Serial.println("{");
  for (int i = 0; i < NUMBER_OF_SPUNDERS; i++)
  {
    spunder_arr[i].tempC = spunder_arr[i].get_tempC();
    spunder_arr[i].tempF = spunder_arr[i].get_tempF();
    spunder_arr[i].psi_setpoint = spunder_arr[i].get_psi_setpoint(spunder_arr[i].vols_setpoint, spunder_arr[i].tempF);
    spunder_arr[i].psi_value = spunder_arr[i].get_psi_value();
    spunder_arr[i].vols_value = spunder_arr[i].get_vols_value(spunder_arr[i].psi_value, spunder_arr[i].psi_setpoint, spunder_arr[i].vols_setpoint);
    spunder_arr[i].vent = spunder_arr[i].get_vent_value(spunder_arr[i].psi_value, spunder_arr[i].psi_setpoint, spunder_arr[i].stored_time, spunder_arr[i].relay_pin);

    if (spunder_arr[i].psi_value > spunder_arr[i].psi_setpoint)
    {
      digitalWrite(spunder_arr[i].relay_pin, RELAY_OPEN);
      delay(100);
      digitalWrite(spunder_arr[i].relay_pin, !RELAY_OPEN);
      spunder_arr[i].stored_time = millis();
      spunder_arr[i].vent = 0;
    }
    else
    {
      spunder_arr[i].vent = (millis() - spunder_arr[i].stored_time) / 60000;
    }
    Serial.print("\"");
    Serial.print(i + 1);
    Serial.print("\": ");
    Serial.print("{");
    Serial.print("\"psi target\": ");
    Serial.print(spunder_arr[i].psi_setpoint);
    Serial.print(", ");
    Serial.print("\"psi\": ");
    Serial.print(spunder_arr[i].psi_value);
    Serial.print(", ");
    Serial.print("\"vol target\": ");
    Serial.print(spunder_arr[i].vols_setpoint);
    Serial.print(", ");
    Serial.print("\"vols\": ");
    Serial.print(spunder_arr[i].vols_value);
    Serial.print(", ");
    Serial.print("\"temp\": ");
    Serial.print(spunder_arr[i].tempC);
    Serial.print(", ");
    Serial.print("\"time since vent\": ");
    Serial.print(spunder_arr[i].vent);

    if (i != (NUMBER_OF_SPUNDERS - 1))
    {
      Serial.println("}, ");
    }
    else
    {
      Serial.println("}");
      Serial.println("}");
    }
  }
  delay(5000);
}
