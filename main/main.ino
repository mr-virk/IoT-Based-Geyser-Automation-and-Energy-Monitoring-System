// Blynk Token & Details
#define BLYNK_TEMPLATE_ID "id"
#define BLYNK_TEMPLATE_NAME "name"
#define BLYNK_AUTH_TOKEN "token"

// Libraries
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <ZMPT101B.h>
#include "EmonLib.h"
#include <Preferences.h>

Preferences preferences; 

EnergyMonitor emon1; // Create an instance for current sensor

// Wifi 2.4Ghz, WPA2
const char ssid[] = "WiFi1";
const char pass[] = "pass";

// Lcd Object
LiquidCrystal_I2C lcd(0x27, 16, 4); // Address, columns, rows 16x04 //Inter-Integrated Circuit

// Pin Defines
const int currentSensorPin = 32; // ADC pin for ACS712 sensor
const int temperatureSensorPin = 4;
const int heaterRelaySwitch = 25;
const int pumpRelaySwitch = 26;
const int waterSensoerPin = 35;
const int buzzerPin = 27;
const int ledRed = 5;
const int ledGreen = 18;
const int ledBlue = 19;
const int ledPumpRed = 12;
const int ledPumpGreen = 14;

// Virtual Pins
const int virtual_pin_voltage = V1; // Voltage
const int virtual_pin_load = V2;    // Watt
const int virtual_pin_ampere = V3;  // Ampere
const int virtual_pin_ect = V4;     // Electricity Consumed
const int virtual_pin_ectl = V5;    // Electricity consumed Today
const int virtual_pin_tt = V11;     // Tank Temperature
const int virtual_pin_wl = V12;     // Water Level
const int virtual_pin_st = V13;     // Set Temperature
const int virtual_pin_gs = V14;     // Geyser Switch

// Variables and Constants
int voltage_now = 0;
int energy_consumed_total = 0;

int tank_water_temp = 0;
int tank_water_temp_set = 0;
int water_Level_Percentage = 0;

// Heater Relay (Timer Function Variables)
static unsigned long relayStartTime = 0; // Variable to store the start time
unsigned long elapsedHours = 0;          // Variable to store the time in hours
unsigned long elapsedSeconds = 0;

// EEPROM (Electronically Eraseable Programmable Read Only Memory)
unsigned int fromEEPROM = 0;

// Current
double current_A = 0.0;      // Current in amperes
double apparentPower = 0.00; // Power in watts
double Wh_accumulated = 0.0;

// Volts 
#define SENSITIVITY 450.0f //Float
#define VOLTAGE_THRESHOLD 200.0f 
// Set a threshold value (adjust as needed)

// ZMPT101B sensor output connected to analog pin 34
// and the voltage source frequency is 50 Hz.
ZMPT101B voltageSensor(34, 50.0); // Pin, Frequency

// Flags
bool GeyserButton = false;
bool isGeyserOn = false;
bool isWaterHeating = false;
bool isWaterHeated = false;
bool isTankFull = false;
bool isPumpRunning = false;
bool isMannualOveridedOn = false;

// For Backgruound Tasks Main Timer
unsigned long previousTime = 0;
const unsigned long interval = 1000; // 1 second = 1000ms

unsigned long currentTime;

// LCD Icons Charaters Map
byte volts[8] = {
    B11100,
    B10101,
    B11101,
    B10100,
    B00000,
    B11101,
    B10001,
    B11100};
byte current[8] = {
    B00001,
    B00010,
    B00100,
    B00010,
    B00100,
    B01000,
    B00000,
    B00000};
byte fire[8] = {
    B00000,
    B10000,
    B10100,
    B11101,
    B11111,
    B11111,
    B11111,
    B01110};
byte drop[8] = {
    B00000,
    B00100,
    B01110,
    B11111,
    B11111,
    B11111,
    B01110,
    B00000};
byte temp[8] = {
    B01110,
    B01010,
    B01010,
    B01010,
    B01010,
    B10001,
    B10001,
    B01110};
byte pump[8] = {
    B00011,
    B11010,
    B01010,
    B11110,
    B10010,
    B10010,
    B10010,
    B11110};
byte load[8] = {
    B01010,
    B01010,
    B11111,
    B10001,
    B01110,
    B00100,
    B00100,
    B01000};
byte wifi[8] = {
    B00000,
    B11111,
    B10001,
    B00000,
    B01110,
    B01010,
    B00000,
    B00100};

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(temperatureSensorPin);
// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

void setup()
{
  Serial.begin(115200); // Serial rate

  // Pin Modes
  pinMode(buzzerPin, OUTPUT);
  pinMode(heaterRelaySwitch, OUTPUT);
  pinMode(pumpRelaySwitch, OUTPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledBlue, OUTPUT);
  pinMode(waterSensoerPin, INPUT);
  pinMode(ledPumpRed, OUTPUT);
  pinMode(ledPumpGreen, OUTPUT);

  // Keeping Relay's OFF
  digitalWrite(heaterRelaySwitch, LOW); //Low = Off, High = ON
  digitalWrite(pumpRelaySwitch, HIGH);  //HIGH = OFF, LOW = ON

  // init preference
  preferences.begin("myfile", false); //FileName, 

  delay(10); //Millisecond

  fromEEPROM = preferences.getUInt("toEEPROM", 0); //UnsignedIntger

  preferences.end();

  delay(10);

  // Volts Sensor Initialize
  voltageSensor.setSensitivity(SENSITIVITY);

  // Current Sensor Initialize
  emon1.current(currentSensorPin, 2.5); // Current: input pin, calibration.

  // Start the DS18B20 sensor //Temp
  sensors.begin();

  // Initialize LCD
  lcd.init(); //Intilize
  lcd.backlight(); // Backlight On
  lcd.clear();     // Clear LCD Screen

  // Create LCD Icons
  lcd.createChar(0, volts); //call name, Array
  lcd.createChar(1, current);
  lcd.createChar(2, fire);
  lcd.createChar(3, drop);
  lcd.createChar(4, temp);
  lcd.createChar(5, pump);
  lcd.createChar(6, load);
  lcd.createChar(7, wifi);

  for (int i; i <= 1; i++) //i++ , i= i+1
  {
    digitalWrite(buzzerPin, HIGH);
    delay(100);
    digitalWrite(buzzerPin, LOW);
    delay(100);
  }

  // Start Screen
  lcd.clear();

  delay(1000);

  lcd.setCursor(-4, 2);
  lcd.print(" Connecting to  ");
  lcd.setCursor(-4, 3);
  lcd.print("     WiFi...    ");

  // Start Wifi
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  lcd.clear(); // Clear LCD after Connecting to Wifi
}

void loop()
{
  Blynk.run(); // Run Blynk
  run_Tasks();
}

BLYNK_CONNECTED()
{
  Blynk.syncVirtual(V1, V2, V3, V4, V5, V11, V12, V13, V14);
}

BLYNK_WRITE(V13)
{
  // Read desired temperature level from slider
  tank_water_temp_set = param.asInt(); // assigning incoming value to variable //As Inteager

  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Value Updated!");
  lcd.setCursor(0, 1);
  lcd.print("Temperature Set: ");
  lcd.setCursor(10, 2);
  lcd.print(tank_water_temp_set);
  lcd.setCursor(13 - 4, 2);
  lcd.print("C");

  delay(1000);
  lcd.clear();
}

BLYNK_WRITE(V14)
{
  int switch_state = param.asInt();

  if (switch_state == 1)
  {
    GeyserButton = true;
  }
  else if (switch_state == 0)
  {
    GeyserButton = false;
  }
}

void run_Tasks()
{
  currentTime = millis();

  if (currentTime - previousTime >= interval)
  {
    previousTime = currentTime;

    // Code to execute every 1 second
    check_Voltage();
    check_current();
    waterLevelCheck();
    temperatureCheck();
    geyserFunction();
    heaterOnTime();
    LedNotification();
    lcd_Dashboard();

    // SerialPrintFunction();
  }
}

void check_Voltage()
{
  // Read the voltage
  voltage_now = voltageSensor.getRmsVoltage();

  voltage_now = constrain(voltage_now, 0, 240); //limitation, 

  // Check if the voltage is below the threshold
  if (voltage_now < VOLTAGE_THRESHOLD)
  {
    voltage_now = 0; // Set voltage to 0 volts
  }

  Blynk.virtualWrite(virtual_pin_voltage, voltage_now);
}

void check_current()
{
  float current_offset = 0.60; 
  current_A = emon1.calcIrms(1480); // Calculate Irms only emon = Energy Mointor

  // Calculate apparent power in watts
  apparentPower = current_A * 230;

  // Calculate energy consumed in watt-hours
  double Wh = apparentPower * (elapsedSeconds / 3600000.0); //3600000.0 = 1hr

  if (current_A <= current_offset)
  {
    current_A = 0.00;
    apparentPower = 0.00;
    energy_consumed_total = 0.00;
  }
  else if (current_A > current_offset && digitalRead(heaterRelaySwitch) == LOW) //Relay has Invert Logic
  {
    heaterOnTime();

    Wh_accumulated = Wh + elapsedSeconds;

    // calculate with before stored value
    energy_consumed_total = Wh_accumulated + fromEEPROM;

    int toEEPROM = energy_consumed_total; //Electronically Eraseable Programable Read Only Memory

    // Serial.print("Energy Consumed before EEPROM Write: ");
    // Serial.print(energy_consumed_total);
    // Serial.print("");

    // EEPROM FUNCTION

    // init preference
    preferences.begin("myfile", false);

    preferences.putUInt("toEEPROM", energy_consumed_total);

    delay(10);

    fromEEPROM = preferences.getUInt("toEEPROM", 0);
    // Serial.print("Data putted in EEPROM...");
    // Serial.println(fromEEPROM);

    preferences.end();
  }

  Blynk.virtualWrite(virtual_pin_ampere, current_A);
  Blynk.virtualWrite(virtual_pin_load, apparentPower);
  Blynk.virtualWrite(virtual_pin_ect, fromEEPROM);

  delay(10);
}

void temperatureCheck()
{
  sensors.requestTemperatures();
  tank_water_temp = sensors.getTempCByIndex(0);

  Blynk.virtualWrite(virtual_pin_tt, tank_water_temp);

  delay(100);
}

void SerialPrintFunction()
{
  // Print the current and power values
  Serial.print("Current: ");
  Serial.print(current_A); //200
  Serial.println(" A");
  //Current: 200 A

  Serial.println("--------------------");
  Serial.print("Power: ");
  Serial.print(apparentPower);
  Serial.println(" W");

  Serial.println("--------------------");

  // Voltage
  Serial.print("Voltage: ");
  Serial.print(voltage_now); // Calculation and Value display is done the rest is if you're using an OLED display

  Serial.println("--------------------");

  // Temperature
  Serial.print("Tank Temperature: ");
  Serial.print(tank_water_temp);
  Serial.println("ºC");

  Serial.println("--------------------");

  // Print the water percentage to the serial monitor
  Serial.print("Water Percentage: ");
  Serial.print(water_Level_Percentage);
  Serial.println("%");
}

void waterLevelCheck()
{
  // Read the analog value from the water level sensor
  int waterLevel = analogRead(waterSensoerPin);

  // Calculate the water level percentage based on the sensor readings
  water_Level_Percentage = map(waterLevel, 0, 4095, 0, 100); // 12-bit ADC //analoug to digital converter

  Blynk.virtualWrite(virtual_pin_wl, water_Level_Percentage);
}

void geyserFunction()
{
  if (tank_water_temp < tank_water_temp_set && GeyserButton == true)
  {
    if (isTankFull == true)
    {
      isWaterHeating = true;
      isWaterHeated = false;

      digitalWrite(heaterRelaySwitch, HIGH); // Relay Switch On

      Blynk.logEvent("notificationevent", "Geyser Heater is OFF! Tank Temperature Needed: " + tank_water_temp_set);

      if (tank_water_temp >= tank_water_temp_set)
      {
        isGeyserOn = false;

        isWaterHeating = false;
        isWaterHeated = true;

        digitalWrite(heaterRelaySwitch, LOW); // Relay Switch Off

        Blynk.logEvent("notificationevent", "Geyser Heater is ON! Temperature Needed: " + tank_water_temp_set);

        digitalWrite(ledGreen, LOW);
        digitalWrite(ledBlue, LOW);

        for (int i; i <= 3; i++)
        {
          digitalWrite(buzzerPin, HIGH);
          digitalWrite(ledRed, HIGH);
          delay(100);
          digitalWrite(buzzerPin, LOW);
          digitalWrite(ledRed, LOW);
          delay(100);
        }

        digitalWrite(ledRed, HIGH);
        digitalWrite(buzzerPin, LOW);
      }
      else if (GeyserButton == false)
      {
        isGeyserOn = false;

        isWaterHeating = false;

        digitalWrite(heaterRelaySwitch, LOW); // Relay Switch Off

        Blynk.logEvent("notificationevent", "Geyser Heater is Manually OFF!");
      }
    }
    else
    {
      pumpFunction();
    }
  }
  else
  {
    isGeyserOn = false;

    isWaterHeating = false;
    isWaterHeated = false;
  }
}

void pumpFunction()
{
  if (water_Level_Percentage < 70)
  {
    isTankFull = false;
    isPumpRunning = true;

    digitalWrite(pumpRelaySwitch, LOW);
    Blynk.logEvent("notificationevent", "Low Water Level: Tank Pump has been Started!");
  }
  else if (water_Level_Percentage >= 30)
  {
    isTankFull = true;
    isPumpRunning = false;

    digitalWrite(pumpRelaySwitch, HIGH);
    Blynk.logEvent("noficationevent", "Tank Full. Tank Pump has been stopped!");
  }
  else
  {
    digitalWrite(pumpRelaySwitch, HIGH);
  }
}

void heaterOnTime()
{
  bool relayIsOn = digitalRead(heaterRelaySwitch) == LOW; // Check if the relay is active (assuming active LOW)

  if (relayIsOn)
  {
    // Relay is currently on, continue timing
    unsigned long currentTime = millis();
    unsigned long elapsedMillis = currentTime - relayStartTime;
    elapsedHours = elapsedMillis / 3600000; // 3600000 ms = 1 hour
    elapsedSeconds = elapsedMillis / 1000;  // 1000 ms = 1 second
  }
  else
  {
    // Relay is off, reset the timer
    relayStartTime = 0;
    elapsedHours = 0;   // Reset elapsed hours when the relay turns off
    elapsedSeconds = 0; // Reset elapsed seconds when the relay turns off
  }

  // Serial.print("Elapsed Time (hours): ");
  // Serial.println(elapsedHours);
  // Serial.print("Elapsed Time (seconds): ");
  // Serial.println(elapsedSeconds);
}

void lcd_Dashboard()
{
  lcd.clear();

  // Volts
  lcd.setCursor(0, 0);
  lcd.write(0);
  lcd.setCursor(2, 0);
  lcd.print(voltage_now);
  lcd.setCursor(6, 0);
  lcd.print("V");

  // Currnet
  lcd.setCursor(8, 0);
  lcd.write(1);
  lcd.setCursor(10, 0);
  lcd.print(current_A);
  lcd.setCursor(14, 0);
  lcd.print("A");

  // Load
  lcd.setCursor(0, 1);
  lcd.write(6);
  lcd.setCursor(2, 1);
  lcd.print(apparentPower);
  lcd.setCursor(6, 1);
  lcd.print("W");

  // Energy Consumed Monthly
  lcd.setCursor(8, 1);
  lcd.write(1);
  lcd.setCursor(10, 1);
  lcd.print(fromEEPROM); // Energy Consumed Totoal
  lcd.setCursor(14, 1);
  lcd.print("Wh");

  // Water Level
  lcd.setCursor(-4, 2);
  lcd.write(3);
  lcd.setCursor(2 - 4, 2);
  lcd.print(water_Level_Percentage);
  lcd.setCursor(6 - 4, 2);
  lcd.print("%");

  // Temperature
  lcd.setCursor(8 - 4, 2);
  lcd.write(4);
  lcd.setCursor(10 - 4, 2);
  lcd.print(tank_water_temp);
  lcd.setCursor(14 - 4, 2);
  lcd.print("C");

  // Temp Set
  lcd.setCursor(8 - 4, 3);
  lcd.print(">");
  lcd.setCursor(10 - 4, 3);
  lcd.print(tank_water_temp_set);
  lcd.setCursor(14 - 4, 3);
  lcd.print("C");

  // Notifications
  if (WiFi.status() == WL_CONNECTED)
  {
    lcd.setCursor(0 - 4, 3);
    lcd.write(7);
  }
  else
  {
    lcd.setCursor(0 - 4, 3);
    lcd.print(' ');
  }

  // Geyser
  lcd.setCursor(2 - 4, 3);
  lcd.write(isWaterHeating ? 2 : ' ');

  // Pump
  lcd.setCursor(4 - 4, 3);
  lcd.write(isPumpRunning ? 5 : ' ');

  // Water Heated
  if (tank_water_temp >= tank_water_temp_set)
  {
    lcd.setCursor(6 - 4, 3);
    lcd.write(4);
  }
  else
  {
    lcd.setCursor(6 - 4, 3);
    lcd.write(' ');
  }
}

void LedNotification()
{
  if (tank_water_temp >= tank_water_temp_set)
  {
    digitalWrite(ledRed, HIGH);
    digitalWrite(ledGreen, LOW);
    digitalWrite(ledBlue, LOW);
  }

  else if (isWaterHeating == true)
  {
    digitalWrite(ledRed, LOW);
    digitalWrite(ledGreen, HIGH);
    digitalWrite(ledBlue, LOW);
  }
  else
  {
    digitalWrite(ledRed, LOW);
    digitalWrite(ledGreen, LOW);
    digitalWrite(ledBlue, HIGH);
  }

  // PUMP
  if (water_Level_Percentage >= 80)
  {
    digitalWrite(ledPumpGreen, HIGH);
    digitalWrite(ledPumpRed, LOW);
  }
  else if (water_Level_Percentage <= 60)
  {
    digitalWrite(ledPumpGreen, LOW);
    digitalWrite(ledPumpRed, HIGH);
  }
}
