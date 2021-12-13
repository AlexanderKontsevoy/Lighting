#include <Arduino.h>
#include <Wire.h>
#include "PCF8574_esp.h"
#include "PCA9685.h"
#include <WiFiClient.h>
#include <ESP32WebServer.h>
#include <WiFi.h>

const char* ssid = "<ssid>";
const char* password = "<password>";
unsigned long check_wifi = 30000;

const int switches_interrupt_pin0 = 25;
const int switches_interrupt_pin1 = 21;
const int sda_pin = 18;
const int scl_pin = 19;
const int a01_pin = 26;
const int a02_pin = 27;
const int a11_pin = 22;
const int a12_pin = 23;
const int pem_units_count = 5;
const int pem_module_count = 16;
const int switches_count = 32;
const int off = 0;
const int defalt_value = off;
const int lowest = 80;
const int ulow = 200;
const int low = 600;
const int third = 1365;
const int half = 2048;
const int high = 3072;
const int full = 0x01000;
const int battery_backup_pem_unit = 0;
const int battery_down_coeff = 2;

const int group_size = 3;
const int system_group_size = 7;
const uint8_t system_group = 0;
const uint8_t kitchen_group = 1;
const uint8_t hall_group = 2;
const uint8_t lobby_group = 3;
const uint8_t bedroom_group = 4;

const uint8_t washbowl_units_count = 1;
const uint8_t kitchen_units_count = 6;
const uint8_t lobby_units_count = 2;
const uint8_t lobby_left_units_count = 1;
const uint8_t lobby_right_units_count = 1;
const uint8_t hall_units_count = 24;
const uint8_t hall_medium_units_count = 8;
const uint8_t hall_big_units_count = 16;
const uint8_t bedroom_units_count = 8;
const uint8_t bedroom_left_units_count = 2;
const uint8_t bedroom_middle_units_count = 4;
const uint8_t bedroom_right_units_count = 2;

uint8_t washbowl_map[washbowl_units_count][2] = {{3,0}};
uint8_t kitchen_map[kitchen_units_count][2] = {{0,0},{0,1},{1,0},{1,1},{2,0},{2,1}};
uint8_t lobby_map[lobby_units_count][2] = {{0,4},{0,5}};
uint8_t lobby_left_map[lobby_left_units_count][2] = {{0,4}};
uint8_t lobby_right_map[lobby_right_units_count][2] = {{0,5}};
uint8_t hall_map[hall_units_count][2] = {{4,4},{3,4},{2,4},{1,4},{1,5},{2,5},{3,5},{4,5},{1,6},{1,7},{2,6},{2,7},{3,6},{3,7},{4,6},{4,7},{1,8},{1,9},{2,8},{2,9},{3,8},{3,9},{4,8},{4,9}};
uint8_t hall_medium_map[hall_medium_units_count][2] = {{4,4},{3,4},{2,4},{1,4},{1,5},{2,5},{3,5},{4,5}};
uint8_t hall_big_map[hall_big_units_count][2] = {{1,6},{1,7},{2,6},{2,7},{3,6},{3,7},{4,6},{4,7},{1,8},{1,9},{2,8},{2,9},{3,8},{3,9},{4,8},{4,9}};
uint8_t bedroom_map[bedroom_units_count][2] = {{0,2},{0,3},{1,2},{1,3},{2,2},{2,3},{3,2},{3,3}};
uint8_t bedroom_left_map[bedroom_left_units_count][2] = {{2,2},{2,3}};
uint8_t bedroom_middle_map[bedroom_middle_units_count][2] = {{0,2},{0,3},{1,2},{1,3}};
uint8_t bedroom_right_map[bedroom_right_units_count][2] = {{3,2},{3,3}};

boolean battery_load = false; // >50W load
boolean dc_ok = false;
boolean battery_fail = false;
boolean psu_0_active = false;
boolean psu_1_active = false;
boolean psu_2_active = false;
boolean psu_3_active = false;
boolean psu_fail = false;
boolean on_battery = false;

boolean kitchen_override = false;
boolean lobby_override = false;
boolean hall_override = false;
boolean bedroom_override = false;

uint8_t switches_kitchen_group[group_size] = {2, 3, 4};
uint8_t switches_lobby_group[group_size] = {5, 6, 7};
uint8_t switches_hall_group[group_size] = {16, 17, 18};
uint8_t switches_bedroom_group[group_size] = {19, 20, 21};
uint8_t switches_system_group[system_group_size] = {8, 9, 10, 11, 12, 13, 14};

uint8_t switches[switches_count] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t switches_ps[switches_count] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint16_t pwm[pem_units_count][pem_module_count] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
uint16_t pwm_ps[pem_units_count][pem_module_count] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
uint16_t pwm_max[pem_units_count][pem_module_count] = 
{
  {full, full, full, full, full, full, full, full, full, full, full, full, full, full, full, full},
  {full, full, full, full, full, full, full, full, full, full, full, full, full, full, full, full},
  {full, full, full, full, full, full, full, full, full, full, full, full, full, full, full, full},
  {full, full, full, full, full, full, full, full, full, full, full, full, full, full, full, full},
  {full, full, full, full, full, full, full, full, full, full, full, full, full, full, full, full}
};
uint16_t pwm_limit[pem_units_count][pem_module_count] = 
{
  {full, full, full, full, full, full, full, full, full, full, full, full, full, full, full, full},
  {full, full, full, full, full, full, full, full, full, full, full, full, full, full, full, full},
  {full, full, full, full, full, full, full, full, full, full, full, full, full, full, full, full},
  {full, full, full, full, full, full, full, full, full, full, full, full, full, full, full, full},
  {full, full, full, full, full, full, full, full, full, full, full, full, full, full, full, full}
};

PCF857x switches0(0x20, &Wire, true);
PCF857x switches1(0x21, &Wire, true);
PCA9685 pem1;
PCA9685 pem2;
PCA9685 pem3;
PCA9685 pem4;
PCA9685 pem5;
PCA9685 pems[pem_units_count] = {pem1, pem2, pem3, pem4, pem5};

ESP32WebServer server (80);

volatile bool switches_interrupt_flag = true;

void IRAM_ATTR detectsSwitchChanges()
{
  switches_interrupt_flag = true;
}

void setPWM(uint8_t unit, uint8_t module, uint16_t amount, bool force = false)
{
  if (amount == off || !on_battery)
    pwm[unit][module] = amount;
  else if (unit != battery_backup_pem_unit)
    pwm[unit][module] = off;
  else if (amount <= ulow)
    pwm[unit][module] = amount;
  else
    pwm[unit][module] = max((amount / battery_down_coeff), ulow);

  if (!force)
    pwm[unit][module] = min(pwm_limit[unit][module], pwm[unit][module]); //current limit
  
  pwm[unit][module] = min(pwm_max[unit][module], pwm[unit][module]); //absolute maximum
}

void setPWM(uint8_t unit[2], uint16_t amount, bool force = false)
{
  setPWM(unit[0], unit[1], amount, force);
}

void setKitchen(uint16_t pwmAmount, uint16_t washbowl)
{
  if (kitchen_override)
    return;
  setPWM(washbowl_map[0], washbowl);
  for (int i = 0; i < kitchen_units_count; i++)
    setPWM(kitchen_map[i], pwmAmount);
}

void setLobby(uint16_t first, uint16_t second)
{
  if (lobby_override)
    return;
  setPWM(lobby_left_map[0], first);
  setPWM(lobby_right_map[0], second);
}

void setBedroom(uint16_t left, uint16_t middle, uint16_t right)
{
  if (bedroom_override)
    return;
  for (int i = 0; i < bedroom_left_units_count; i++)
    setPWM(bedroom_left_map[i], left);
  for (int i = 0; i < bedroom_middle_units_count; i++)
    setPWM(bedroom_middle_map[i], middle);
  for (int i = 0; i < bedroom_right_units_count; i++)
    setPWM(bedroom_right_map[i], right);
}

void setHall(uint16_t pwmAmount)
{
  if (hall_override)
    return;
  for (int i = 0; i < hall_units_count; i++)
    setPWM(hall_map[i], pwmAmount);
}

void saveSwitchesState()
{
  for (int i = 0; i < switches_count; i++)
    switches_ps[i] = switches[i];
}

void savePWM()
{
  for (int i = 0; i < pem_units_count; i++)
    for (int j = 0; j < pem_module_count; j++)
      pwm_ps[i][j] = pwm[i][j];
}

int changesOffset(int pem_unit)
{
  for (int i = 0; i < pem_module_count; i++)
    if (pwm[pem_unit][i] != pwm_ps[pem_unit][i])
      return i;
  return -1;
}

int changesSize(int pem_unit, int offset)
{
  for (int i = pem_module_count - 1; i >= offset; i--)
    if (pwm[pem_unit][i] != pwm_ps[pem_unit][i])
      return i - offset + 1;
  return 0;
}

void applyChanges()
{
  boolean has_changes = false;
  
  for (int i = 0; i < pem_units_count; i++)
  {
    int offset = changesOffset(i);
    if (offset >= 0)
    {
      int size = changesSize(i, offset);
      uint16_t changes[size];
      for (int j = 0; j < size; j++)
      {
        changes[j] = pwm[i][j + offset];
      }
      pems[i].setChannelsPWM(offset, size, changes);
      has_changes = true;
    }
  }
  if (has_changes)
    savePWM();
}

void makeChanges()
{ 
  battery_load = switches[switches_system_group[0]];
  battery_fail = switches[switches_system_group[1]];
  dc_ok = switches[switches_system_group[2]];
  psu_0_active = switches[switches_system_group[4]];
  psu_1_active = switches[switches_system_group[3]];
  psu_2_active = switches[switches_system_group[6]];
  psu_3_active = switches[switches_system_group[5]];
  on_battery = !psu_0_active && !psu_1_active && !psu_2_active && !psu_3_active;
  psu_fail = (!psu_0_active || !psu_1_active || !psu_2_active || !psu_3_active) && !on_battery;

  int kitchen = 0;
  int hall = 0;
  int lobby = 0;
  int bedroom = 0;
  int kitchen_ps = 0;
  int hall_ps = 0;
  int lobby_ps = 0;
  int bedroom_ps = 0;
  for (int i = 0; i < group_size; i++)
  {
    int step = (int) pow(2, i);
    kitchen += switches[switches_kitchen_group[i]] * step;
    kitchen_ps += switches_ps[switches_kitchen_group[i]] * step;
    hall += switches[switches_hall_group[i]] * step;
    hall_ps += switches_ps[switches_hall_group[i]] * step;
    lobby += switches[switches_lobby_group[i]] * step;
    lobby_ps += switches_ps[switches_lobby_group[i]] * step;
    bedroom += switches[switches_bedroom_group[i]] * step;
    bedroom_ps += switches_ps[switches_bedroom_group[i]] * step;
  }

  if (kitchen_ps != kitchen)
    kitchen_override = false;
  if (lobby_ps != lobby)
    lobby_override = false;
  if (hall_ps != hall)
    hall_override = false;
  if (bedroom_ps != bedroom)
    bedroom_override = false;
  
  switch (kitchen)
  {
      case 1: setKitchen(0, third); break;      //001
      case 2: setKitchen(third, 0); break;      //010
      case 3: setKitchen(third, third); break;  //011
      case 4: setKitchen(half, half); break;    //100
      case 5: setKitchen(half, high); break;    //101
      case 6: setKitchen(high, half); break;    //110
      case 7: setKitchen(high, high); break;    //111
      default: setKitchen(0, 0); break;         //000
  }
  switch (hall)
  {
      case 1: setHall(lowest); break; //001
      case 2: setHall(ulow); break;   //010
      case 3: setHall(low); break;    //011
      case 4: setHall(third); break;  //100
      case 5: setHall(half); break;   //101
      case 6: setHall(high); break;   //110
      case 7: setHall(full); break;   //111
      default: setHall(0); break;     //000
  }
  switch (lobby)
  {
      case 1: setLobby(0, half); break;     //001
      case 2: setLobby(high, high); break;  //010
      case 3: setLobby(high, full); break;  //011
      case 4: setLobby(half, 0); break;     //100
      case 5: setLobby(half, half); break;  //101
      case 6: setLobby(full, high); break;  //110
      case 7: setLobby(full, full); break;  //111
      default: setLobby(0, 0); break;       //000
  }
  switch (bedroom)
  {
      case 1: setBedroom(0, 0, half); break;        //001
      case 2: setBedroom(0, half, 0); break;        //010
      case 3: setBedroom(0, high, high); break;     //011
      case 4: setBedroom(half, 0, 0); break;        //100
      case 5: setBedroom(half, 0, half); break;     //101
      case 6: setBedroom(high, high, 0); break;     //110
      case 7: setBedroom(high, high, high); break;  //111
      default: setBedroom(0, 0, 0); break;          //000
  }
  saveSwitchesState();
}

void checkSwitches()
{
  if (switches_interrupt_flag)
  {
    switches_interrupt_flag = false;
    //Задержка 100мс. т.к. если сразу начинать считывать с шины, 
    //сбрасывается флаг прерывания в контроллере выключателей, 
    //а дребезг контактов может вызывать панику ядра частыми прерываниями.
    delay(100);
    for (int i = 0; i < switches_count; i++)
      if (i < 16)
        switches[i] = switches0.read(i);
      else
        switches[i] = switches1.read(i - 16);
    makeChanges();
    applyChanges();
  }
}

int parseValue(String rawValue, int max, int default_value = -1)
{
  String temp = "";
  for (int i = 0; i < rawValue.length(); i++)
    if (isDigit(rawValue[i]))
      temp += rawValue[i];
  if (temp.isEmpty())
    return default_value;
  int intVal = temp.toInt();
  if (intVal > max)
    return default_value;
  return intVal;
}

uint16_t mapValue(int value, uint16_t max)
{
  return map(value, 0, 100, 0, max);
}

void setOverride(uint8_t map[][2], uint8_t valuePercent, int order, bool all, uint8_t total_units)
{
  int from = order;
  int to = order;
  if (all)
  {
    from = 0;
    to = total_units - 1;
  }
  else if (order < 0 || order >= total_units)
    return;
  
  for (int i = from; i <= to; i++)
    setPWM(map[i], mapValue(valuePercent, pwm_max[map[i][0]][map[i][1]]), true);
}

void setWashbowlOverride(uint8_t valuePercent)
{
  kitchen_override = true;
  setOverride(washbowl_map, valuePercent, 0, true, washbowl_units_count);
}

void setKitchenOverride(uint8_t valuePercent, int order, bool all)
{
  kitchen_override = true;
  setOverride(kitchen_map, valuePercent, order, all, kitchen_units_count);
}

void setLobbyLeftlOverride(uint8_t valuePercent)
{
  lobby_override = true;
  setOverride(lobby_left_map, valuePercent, 0, true, lobby_left_units_count);
}

void setLobbyRightlOverride(uint8_t valuePercent)
{
  lobby_override = true;
  setOverride(lobby_right_map, valuePercent, 0, true, lobby_right_units_count);
}

void setLobbyOverride(uint8_t valuePercent, int order, bool all)
{
  lobby_override = true;
  setOverride(lobby_map, valuePercent, order, all, lobby_units_count);
}

void setHallMediumOverride(uint8_t valuePercent, int order, bool all)
{
  hall_override = true;
  setOverride(hall_medium_map, valuePercent, order, all, hall_medium_units_count);
}

void setHallBigOverride(uint8_t valuePercent, int order, bool all)
{
  hall_override = true;
  setOverride(hall_big_map, valuePercent, order, all, hall_big_units_count);
}

void setHallOverride(uint8_t valuePercent, int order, bool all)
{
  hall_override = true;
  setOverride(hall_map, valuePercent, order, all, hall_units_count);
}

void setBedroomLeftOverride(uint8_t valuePercent, int order, bool all)
{
  bedroom_override = true;
  setOverride(bedroom_left_map, valuePercent, order, all, bedroom_left_units_count);
}

void setBedroomMiddleOverride(uint8_t valuePercent, int order, bool all)
{
  bedroom_override = true;
  setOverride(bedroom_middle_map, valuePercent, order, all, bedroom_middle_units_count);
}

void setBedroomRightOverride(uint8_t valuePercent, int order, bool all)
{
  bedroom_override = true;
  setOverride(bedroom_right_map, valuePercent, order, all, bedroom_right_units_count);
}

void setBedroomOverride(uint8_t valuePercent, int order, bool all)
{
  bedroom_override = true;
  setOverride(bedroom_map, valuePercent, order, all, bedroom_units_count);
}

String jsonStatus(uint8_t map[][2], uint8_t size)
{
  String result = "[";
  for (int i = 0; i < size; i++)
  {
    if (i > 0) result += ",";
    result += pwm[map[i][0]][map[i][1]] * 100 / min(pwm_limit[map[i][0]][map[i][1]], pwm_max[map[i][0]][map[i][1]]);
    //result += "%";
  }
  result += "]";
  return result;
}

String jsonStatuses()
{
  String result = "{\n";
  result += "\"washbowl\": " + jsonStatus(washbowl_map, washbowl_units_count);
  result += ",\n";
  result += "\"kitchen\": " + jsonStatus(kitchen_map, kitchen_units_count);
  result += ",\n";
  result += "\"lobby\": " + jsonStatus(lobby_map, lobby_units_count);
  result += ",\n";
  result += "\"hall\": " + jsonStatus(hall_map, hall_units_count);
  result += ",\n";
  result += "\"bedroom\": " + jsonStatus(bedroom_map, bedroom_units_count);
  result += "\n}";
  return result;
}

void handleNotFound()
{
  server.send(404, "text/plain", "Not Found");
}

void handleStatus()
{
  int argsCount = server.args();
  for (uint8_t i = 0; i < argsCount; i++)
  {
    String argName = server.argName(i);
    argName.toLowerCase();
    String argValue = server.arg(i);
    int value = parseValue(argValue, 100);
    if (value < 0)
      continue;
    if (argName.startsWith("w"))
      setWashbowlOverride(value);
    else if (argName.startsWith("k"))
      setKitchenOverride(value, parseValue(argName, kitchen_units_count), argName.length() == 1);
    else if (argName.startsWith("ll"))
      setLobbyLeftlOverride(value);
    else if (argName.startsWith("lr"))
      setLobbyRightlOverride(value);
    else if (argName.startsWith("l"))
      setLobbyOverride(value, parseValue(argName, lobby_units_count), argName.length() == 1);
    else if (argName.startsWith("hm"))
      setHallMediumOverride(value, parseValue(argName, hall_medium_units_count), argName.length() == 2);
    else if (argName.startsWith("hb"))
      setHallBigOverride(value, parseValue(argName, hall_big_units_count), argName.length() == 2);
    else if (argName.startsWith("h"))
      setHallOverride(value, parseValue(argName, hall_units_count), argName.length() == 1);
    else if (argName.startsWith("bl"))
      setBedroomLeftOverride(value, parseValue(argName, bedroom_left_units_count), argName.length() == 2);
    else if (argName.startsWith("bm"))
      setBedroomMiddleOverride(value, parseValue(argName, bedroom_middle_units_count), argName.length() == 2);
    else if (argName.startsWith("br"))
      setBedroomRightOverride(value, parseValue(argName, bedroom_right_units_count), argName.length() == 2);
    else if (argName.startsWith("b"))
      setBedroomOverride(value, parseValue(argName, bedroom_units_count), argName.length() == 1);
  }
  if (argsCount > 0)
    applyChanges();
  server.send(200, "application/json", jsonStatuses());
}

void setup()
{
  delay(1000);
  Serial.begin(250000);
  Wire.begin(sda_pin, scl_pin, 400000);
  pems[0].resetDevices();
  pems[0].init(B111110);
  pems[1].init(B111101);
  pems[2].init(B111011);
  pems[3].init(B110111);
  pems[4].init(B101111);
  for (int i = 0; i < pem_units_count; i++)
  {
    pems[i].setPWMFrequency(1000);
    pems[i].setAllChannelsPWM(defalt_value);
  }

  //Switches
  pinMode(a01_pin, OUTPUT);
  pinMode(a02_pin, OUTPUT);
  pinMode(a11_pin, OUTPUT);
  pinMode(a12_pin, OUTPUT);
  digitalWrite(a01_pin, LOW);
  digitalWrite(a02_pin, LOW);
  digitalWrite(a11_pin, LOW);
  digitalWrite(a12_pin, LOW);
  switches0.begin();
  switches1.begin();
  switches0.resetInterruptPin();
  switches1.resetInterruptPin();
  pinMode(switches_interrupt_pin0, INPUT);
  pinMode(switches_interrupt_pin1, INPUT);
  attachInterrupt(digitalPinToInterrupt(switches_interrupt_pin0), detectsSwitchChanges, FALLING);
  attachInterrupt(digitalPinToInterrupt(switches_interrupt_pin1), detectsSwitchChanges, FALLING);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  server.on("/status", handleStatus);
  server.onNotFound(handleNotFound);
  server.begin();
}

void loop()
{
  checkSwitches();
  if ((WiFi.status() != WL_CONNECTED) && (millis() > check_wifi))
  {
    WiFi.reconnect();
    check_wifi = millis() + 30000;
  }
  server.handleClient();
  delay(10);
}