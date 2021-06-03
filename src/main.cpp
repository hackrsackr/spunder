#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define NUMBER_OF_SPUNDERS 4
#define SENSOR_RATING 30
#define OFFSET 102                     // bits from 0v - .5v
#define FULLSCALE 922                  // bits from .5v - 4.5v
#define RELAY_OPEN HIGH
#define ONE_WIRE_BUS 3

// Setup a oneWire instance & pass it to Dallas Temperature.
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

const int RELAY_PINS[NUMBER_OF_SPUNDERS] = {7, 6, 5, 4};
const int SENSOR_PINS[NUMBER_OF_SPUNDERS] = {0, 1, 2, 3};
float DESIRED_VOLS[NUMBER_OF_SPUNDERS] = {1.0, 2.0, 3.0, 4.0};

int stored_times[NUMBER_OF_SPUNDERS];

struct Spunder
{
  int id, RELAY_PIN, SENSOR_PIN, stored_time, vent;
  float vols_setpoint, vols_value, psi_setpoint, psi_value, tempC, tempF;
};

Spunder spunder_arr[NUMBER_OF_SPUNDERS];

void setup()
{
  // start serial and spunders, set valves to closed.
  Serial.begin(9600);
  sensors.begin();
  for (int i = 0; i < NUMBER_OF_SPUNDERS; i++)
  {
    spunder_arr[i].id = i + 1;
    spunder_arr[i].RELAY_PIN = RELAY_PINS[i];
    spunder_arr[i].SENSOR_PIN = SENSOR_PINS[i];
    spunder_arr[i].vols_setpoint = DESIRED_VOLS[i];
    spunder_arr[i].stored_time = 0;

    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], !RELAY_OPEN);
  }
}

float get_tempC()
{
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  return tempC;
}

float get_tempF()
{
  sensors.requestTemperatures();
  float tempF = sensors.getTempFByIndex(0);
  return tempF;
}

float get_psi_value(int SENSOR_PIN)
{
  float psi_value = (analogRead(SENSOR_PIN) - OFFSET) * SENSOR_RATING / float((FULLSCALE - OFFSET));
  return psi_value;
}

float get_psi_setpoint(float vol_setpoint, float tempF)
{
  float psi_setpoint = (-16.669 - (.0101059 * tempF)) + (.00116512 * (tempF * tempF)) + (.173354 * tempF * vol_setpoint) + (4.24267 * vol_setpoint) - (.0684226 * (vol_setpoint * vol_setpoint));
  return psi_setpoint;
}

float get_vols_value(float psi_value, float psi_setpoint, float vols_setpoint)
{
  float vols_value = float(psi_value / psi_setpoint) * vols_setpoint;
  return vols_value;
}

int get_vent_value(float psi_value, float psi_setpoint, int stored_time, int RELAY_PIN)
{
  int vent;
  if (psi_value > psi_setpoint)
  {
    digitalWrite(RELAY_PIN, RELAY_OPEN);
    delay(100);
    digitalWrite(RELAY_PIN, !RELAY_OPEN);
    stored_time = millis();
    vent = 0;
  }
  else
  {
    vent = (millis() - stored_time) / 60000;
  }
  return vent;
}

void loop()
{
  Serial.println("{");
  for (int i = 0; i < NUMBER_OF_SPUNDERS; i++)
  {
    spunder_arr[i].tempC = get_tempC();
    spunder_arr[i].tempF = get_tempF();
    spunder_arr[i].psi_setpoint = get_psi_setpoint(spunder_arr[i].vols_setpoint, spunder_arr[i].tempF);
    spunder_arr[i].psi_value = get_psi_value(SENSOR_PINS[i]);
    spunder_arr[i].vols_value = get_vols_value(spunder_arr[i].psi_value, spunder_arr[i].psi_setpoint, spunder_arr[i].vols_setpoint);
    spunder_arr[i].vent = get_vent_value(spunder_arr[i].psi_value, spunder_arr[i].psi_setpoint, spunder_arr[i].stored_time, spunder_arr[i].RELAY_PIN);

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
