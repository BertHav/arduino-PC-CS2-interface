/*********************************************************************
 * RocRailuino
 * Initial created 06 nov 2013 Bert Havinga
 *
 * This sketch runs on Arduino Uno R, Leonardo or Mega in
 * conjuction with a CAN bus shield
 * http://www.watterott.com/en/Arduino-CANdiy-Shield
 * this shield is a development as a part of the Railuino project.
 * In this solution the CS line has a conflict with the W5100
 * ethernet shield. To bypass this, build your own shield or view
 * yourself some pictures on the Rocrail forum in the DIY hardware
 * area.
 *
 * Rocrail should be configured to communicate with a Maerklin CS2
 *
 * Based on Railuino library. The library is stripped for use
 * in combination with RocRail©. The original Railuino library
 * didn't support the priobits in the CANheader and hangs when
 * two CAN packets are received before clearing the interrupt in
 * combination with another SPI device.
 *
 * excerpt from Railuino - Hacking your Märklin Idea
 * Copyright (C) 2012 Joerg Pleumann
 *
 * This example is free software; you can redistribute it and/or
 * modify it under the terms of the Creative Commons Zero License,
 * version 1.0, as published by the Creative Commons Organisation.
 * This effectively puts the file into the public domain.
 *
 * This example is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * LICENSE file for more details.
 *
 * REMARK
 * if this sketch hangs after modifying the code, try to change
 * first to resize the queuelength in QueueArray.h
 * The current configuration  is on the Edge with a Leonardo or UNO
 * This is due to limited SRAM space on the Arduino platform
 *
 * Shortcircuit detection is made by sensor number: see variable "shortDetect"
 * The shortcircuit detection circuit must connected to pin with variable pinSCD
 * A logical value "1" is treated as a shortcircuit signal
 * Short circuit detection can be disabled by the var: shortdetection */

#include <SPI.h>         // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include <EthernetUdp.h> // UDP library from: bjoern@cs.stanford.edu 12/30/2008
#include <Gateway.h>
//#include <SoftwareSerial.h>
#include <QueueArray.h>

//CAN BUS shield definition, this should be equal to the var in can/defaults.h
//and mapped to the pin number
//in case of using the default Watterott shield or most of the other commercial shields,
// do not forget to bent out the pin 10 and place a patch wire to pin MCP2515_CS (default = 6)
#define UDP_TX_PACKET_MAX_SIZE 39
const byte MCP2515_CS = 6;
const byte intPin = 2;
const byte msHash = 0x67;
const byte lsHash = 0x15;

const int decDevices = 4; // number of devices (switches) per decoder, normaly the half of the nr of outputs

//const word SchaltzeitDecoder = 250; //time in milliseconds
const long baud = 115200;
boolean CANintrc = false;  //debugflag
boolean gwlibtrc = false;  //debugflag library trace
boolean IPtrc = false;  //debugflag
boolean timetrc = false;  //debugflag
boolean swCmdtrc = false;  //debugflag
boolean swCmdfbtrc = false;
boolean UDPintrc = false;  //debugflag
boolean quetrc = false;
boolean counters = false;
boolean shortdetection = false; //true for enable short circuit detection

// boolean askStatus = true;  // ask for Volt Ampere Celcius

//start including ethernet
// Enter a MAC address and IP address for your controller below.
byte mac[] = { 0x08, 0x00, 0x2B, 0xBF, 0xFE, 0xED }; //My DEC history ;)
IPAddress ip(192, 168, 1, 147);
// in this sketch the iptx will be overwritten by the Rocrail server
// address during init. This sketch detects the server address automatic
// if this device is configured as a MCS2 controller with our ip address
IPAddress iptx(192, 168, 1, 255); //Address for Rocrail

// the router's gateway address:
byte gateway[] = { 192, 168, 1, 254 };
// the subnet:
byte subnet[] = { 255, 255, 255, 0 };

//The next two lines are Central Station (controller specific)
//The should look like the controller configuration in Rocrail
//
const unsigned int localPort   = 35731;      // local port to listen on (left in Rocrail form)
const unsigned int localTxPort = 35730;      // local port to send on (right in Rocrail form)

// An EthernetUDP instance to let us send and receive packets over UDP


TrackController ctrl(0x0000, gwlibtrc, false);

TrackMessage message;

EthernetUDP Udp;

EthernetServer server = EthernetServer(15731);

// buffers for receiving and sending data
byte packetTxBuffer[UDP_TX_PACKET_MAX_SIZE];
byte packetRxBuffer[13];
// byte packetTempBuffer[13];

// end including ethernet

QueueArray <TrackMessage> queue1;
QueueArray <TrackMessage> queue2;
QueueArray <TrackMessage> queue3;

//value below 150ms requeres a mature powersupply, the default Gleisbox power supply needs a minimum of 150
const byte accPauseTime = 85; //interval time in milli seconds between accessory commands
const byte packetwaittime = 17; // time to wait between packets

const word gk = 0x0000; // conform specs 0 is illegal!!! in this case random choosen geraetekennung, should be assigned by CS2
const word shortDetect = 512;    // sensor number to report for shortcircuit
const int pinSCD = 7;            // shortCircuit pin
byte mSwitch = 0;                    // The most recent contact value we know.
byte mPrevSwitch = 0;                // The previous contact value we know.
boolean power_ena = false;       // power status

unsigned long time;
unsigned long lstswtime;  // time for trace between received commands from Rocrail
// unsigned long lstpingtime = 0;  // time for pausing between requesting the UID from Gleisbox and MS2
unsigned long lstCUtime;  // time for trace between received CAN 2 UDPcommands to Rocrail
unsigned long pptime;  // time for trace between send and received CAN and UDPcommands
unsigned long lstpptime;  // last absolute time for trace between received CAN 2 UDPcommands to Rocrail
unsigned long lstqueswtime;  //pause time between issuing commands to GFP
unsigned long thisdlytime; //delay time for the handled accessory command
unsigned long quedlytime = 0; //time from previous handled accessory commands concurrently handled
unsigned long looptime = 0;
unsigned long prevlooptime = 0;
unsigned long gbpolltime = 0;
unsigned long lastinitpckttime = 0;
byte gb_uid_d0 = 0;  // store the UID of the Gleisbox
byte gb_uid_d1 = 0;
byte gb_uid_d2 = 0;
byte gb_uid_d3 = 0;
byte ms2_uid_d0 = 0;  // store the UID of the MS2
byte ms2_uid_d1 = 0;
byte ms2_uid_d2 = 0;
byte ms2_uid_d3 = 0;
byte lstChn = 0;     // last measurement channel requested

// time out in ms in case no feed back received on accessoiry command
// the default for the GFP internal is 1000ms.
unsigned long queuetimeout = 1100; 

// time for each queue after which we can assume we can send a new command
// will be modifified after receiving an acknowledge from the GFP
// that the command has succesful completed
unsigned long pktfromqueue1send; 
unsigned long pktfromqueue2send;
unsigned long pktfromqueue3send;

boolean ArduinoQueueError = false;
byte sendfromqueue = 0;
int currentSend[4]; // decoder adresses in current cycle to central station, adapted to 4 for while and Arduino 1.5.7
byte csAccSnd = 0; // Accessory commands send (max=3 for GFP)
byte roundRobin = 0;
//char char20buff[21];

#if defined(__UNO__)
const int maxentry = 11; //queue1.initialSize;  // max nr of queue entries Uno=10, Leonardo=18 with non resizable queue
#elif defined(__LEONARDO__)
const int maxentry = 18;
#elif defined(__MEGA__)
const int maxentry = 30;
#endif

int que1addr[maxentry];
int que2addr[maxentry];
int que3addr[maxentry];

byte obsmaxentry1top = 0;
byte obsmaxentry2top = 0;
byte obsmaxentry3top = 0;

// RocRail sends always 13 bytes
char sbuffer[13];
int i;
unsigned long UDPsnd = 0;
unsigned long UDPrx = 0;
unsigned long CANsnd = 0;
unsigned long CANrx = 0;

byte initstate = 8;  // number of sending init packets for the GB in case no MS2 is connected

void welcome() {
  Serial.println();
  Serial.println(F("(C) B. Havinga"));
  Serial.println(F("ver 14-mai-2015.109"));
  Serial.println(F("Image name: CAN_CS2_gwV20"));
  Serial.println();
}

void showIP() {
  Serial.print(F("local IP: "));
  if (Ethernet.localIP() == ip) {
    Serial.println(ip);
  }
  else {
    Serial.println(F("Error Ethernetshield not detected"));
  }
  Serial.print(F("Rxport: "));
  Serial.print(localPort);
  Serial.print(F(", Txport: "));
  Serial.println(localTxPort);
}

void show_wait_for_Rr_server() {
  showmac();
  showIP();
  Serial.println(F("Waiting for Rocrailserver"));
  Serial.println();
}

void showmac() {
  Serial.print(F("MAC: "));
  for (i = 0; i < 6; i++)
  {
    if (mac[i] < 16) Serial.print('0');
    Serial.print(mac[i], HEX);
    if (i < 5) Serial.print('-');
  }
  Serial.println();
}

void printUID(byte uid_d0, byte uid_d1, byte uid_d2,  byte uid_d3)
{
  if (uid_d0 + uid_d1 + uid_d2 + uid_d3 == 0)
  {
    Serial.println(F("not detected"));
  }
  else
  {
    Serial.print(F("UID: "));
    unsigned long DEVUID = ( ((unsigned long) uid_d0 << 24)
                             | ((unsigned long) uid_d1 << 16)
                             | ((int) uid_d2 << 8)
                             | uid_d3  );
    Serial.println(DEVUID, HEX);
  }
  //  Serial.println(char20buff);
}

void showUIDs()
{
  Serial.print(F("Gleisbox "));
  printUID(gb_uid_d0, gb_uid_d1, gb_uid_d2, gb_uid_d3);
  Serial.print(F("MS2 "));
  printUID(ms2_uid_d0, ms2_uid_d1, ms2_uid_d2, ms2_uid_d3);
}

void showInterval(unsigned long ptime, unsigned long p2ptime) {
  Serial.print(F(" Interval (ms): "));
  Serial.print(ptime);
  Serial.print(F(" -> p2p (ms): "));
  Serial.println(p2ptime);
}

int freeRam () {
  Serial.print(F("Estimated free SRAM (bytes): "));
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void powerDown(byte MSBhash, byte LSBhash) {
  // switch off power from track by the GFP
  message.id1 = 0;
  message.id2 = 0;
  message.id3 = MSBhash;
  message.id4 = LSBhash;
  message.length = 5;
  for (int i = 0; i < 8; i++) {
    message.data[i] = 0;
  }
  ctrl.sendMessage(message);  // Send CAN bus msg to Maerklin device
  CANsnd++;
}

void setup() {
  Serial.begin(baud);
  // while (!Serial); //for debuging on Leonardo
  // set the printer of the queue for error logging
  queue1.setPrinter (Serial);
  queue2.setPrinter (Serial);
  queue3.setPrinter (Serial);
  Serial.println(F("start setup"));
  Serial.println(F("type ? for help"));
  Serial.println(F("init Ethernet now"));
  // start the Ethernet and UDP:
  // Sprinkle some magic pixie dust. (disable SD SPI on the Ethernetshield)
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  pinMode(pinSCD, INPUT);
  //digitalWrite(pinSCD, HIGH); // turn on pullup resistor

  //disable your new SPI until the w5100 is set up,
  //this is the MCP2515_CS Arduino pin defined according
  // to the chip register in defaults.h in CAN library
  pinMode(MCP2515_CS, OUTPUT);
  digitalWrite(MCP2515_CS, HIGH);

  // set up w5100
  Ethernet.begin(mac, ip, gateway, subnet);  // initialize Ethernet device
  noInterrupts();
  server.begin();
  Udp.begin(localPort);
  interrupts();
  // disable w5100 SPI
  digitalWrite(10, HIGH);
  //  Serial.println(F("Ethernet init complete"));
  //end ethernet setup
  Serial.setTimeout(2);
  //  showmac();
  show_wait_for_Rr_server();
  int packetSize = 0;

  while( (packetSize != 13) ) {
    // check for a debug command
    if (Serial.available() > 0) {
      setDebugflags(Serial.read());
    }
    if (packetSize != 0) {
       Serial.print(F("packetsize received: "));
       Serial.println(packetSize);
    }
    //    while (ctrl.receiveMessage(message)) { };
    noInterrupts();
    packetSize = Udp.parsePacket(); //waiting for a message from Rocrail
    EthernetClient client = server.available();
    if (client == true) {
    // read bytes from the incoming client and write them back
    // to any clients connected to the server:
      server.write(client.read());
    }
    interrupts();
  }
  // waiting for a Rocrail Ethernet CAN bus message via Ethernet
  noInterrupts();
  Udp.read(packetRxBuffer, 13);
  interrupts();
  UDPrx++;
  //Welcome msg inserted here to show always, even on Leonardo
  welcome();
  Serial.print(F("RocRailserver found on address: "));
  noInterrupts();
  iptx = Udp.remoteIP();
  interrupts();
  Serial.print(iptx);
  Serial.println();
  // store message that awaked the Arduino
  //  for (int i = 0; i < 13; i++) {
  //    packetTempBuffer[i] = packetRxBuffer[i];
  //  }
  // extract the Rocrail hash
  word rrHash = word(packetRxBuffer[2], packetRxBuffer[3]);
  // register Rocrail on the CAN bus
  ctrl.begin(rrHash);
  Serial.println(F("CAN initialized"));
  // send  the message that awaked us as a packet to the CAN bus
  message.id1 = packetRxBuffer[0]; //packetTempBuffer[0];
  message.id2 = packetRxBuffer[1];
  message.id3 = packetRxBuffer[2];
  message.id4 = packetRxBuffer[3];
  message.length = packetRxBuffer[4];
  for (int i = 0; i < 8; i++) {
    message.data[i] = packetRxBuffer[i + 5];
  }
  ctrl.sendMessage(message);  // Send CAN bus msg to Maerklin device
  CANsnd++;
  readDebugflags();
  //init the array for decoderaddresses that are to be processed by the GFP
  memset(currentSend, 0, sizeof(currentSend));
//  enableProtocols();
}

void  printOnOff(boolean key) {
  if (key == false) {
    Serial.println(F(" [OFF]"));
  }
  else {
    Serial.println(F(" [ON]"));
  }
}
void readDebugflags() {
  welcome();
  Serial.println(F("Debug toggle switches:"));
  Serial.print(F("b - Short circuit enabled"));
  printOnOff(shortdetection);
  Serial.print(F("c - CAN-in trace"));
  printOnOff(CANintrc);
  Serial.print(F("g - gateway library trace"));
  printOnOff(gwlibtrc);
  Serial.print(F("i - IP target trace"));
  printOnOff(IPtrc);
  Serial.print(F("m - timing trace"));
  printOnOff(timetrc);
  Serial.print(F("n - Show counters"));
  printOnOff(counters);
  Serial.print(F("q - Queue tracing"));
  printOnOff(quetrc);
  Serial.print(F("s - Switch-cmd trace"));
  printOnOff(swCmdtrc);
  Serial.print(F("t - switch-cmd feedback trace"));
  printOnOff(swCmdfbtrc);
  Serial.print(F("u - UDP"));
  printOnOff(UDPintrc);
  Serial.println();
  Serial.print(F("Queuedepth initialized defined in sketch: "));
  Serial.println(maxentry);
  if (queue1.initialSize != maxentry) {
    Serial.println(F("Error configuration mismatch depth in queuearray.h lib: "));
    Serial.println(queue1.initialSize);
  }
  Serial.print(F("Max observed queuedepth q1: "));
  Serial.print(obsmaxentry1top);
  Serial.print(F(", q2: "));
  Serial.print(obsmaxentry2top);
  Serial.print(F(", q3: "));
  Serial.println(obsmaxentry3top);
  Serial.println();
  Serial.print(F("Gleisbox UID: "));
  Serial.print(gb_uid_d0, HEX);
  Serial.print(gb_uid_d1, HEX);
  Serial.print(gb_uid_d2, HEX);
  Serial.println(gb_uid_d3, HEX);

  if ((ms2_uid_d0 | ms2_uid_d1 | ms2_uid_d2 | ms2_uid_d3) != 0)
  {
    Serial.print(F("MS2 detected, UID: "));
    Serial.print(ms2_uid_d0, HEX);
    Serial.print(ms2_uid_d1, HEX);
    Serial.print(ms2_uid_d2, HEX);
    Serial.println(ms2_uid_d3, HEX);
  }
  Serial.println();
  Serial.println(freeRam());
  Serial.println();
  showUIDs();
  showmac();
  showIP();
  Serial.println();
  Serial.print(F("UDP pckts Tx: "));
  Serial.println(UDPsnd);
  Serial.print(F("UDP pckts Rx: "));
  Serial.println(UDPrx);
  Serial.print(F("CAN pckts Tx: "));
  Serial.println(CANsnd);
  Serial.print(F("CAN pckts Rx: "));
  Serial.println(CANrx);
  Serial.println();
}

void setDebugflags(char dflg) {
  switch (dflg) {
    case 'b':
      if (shortdetection == false) {
        shortdetection = true;
      }
      else {
        shortdetection = false;
      }
      break;
    case 'c':
      if (CANintrc == false) {
        CANintrc = true;
      }
      else {
        CANintrc = false;
      }
      break;
    case 'g':
      if (gwlibtrc == false) {
        gwlibtrc = true;
      }
      else {
        gwlibtrc = false;
      }
      gwlibtrc = ctrl.setDebug(gwlibtrc);
      break;
    case 'i':
      if (IPtrc == false) {
        IPtrc = true;
      }
      else {
        IPtrc = false;
      }
      break;
    case 'm':
      if (timetrc == false) {
        timetrc = true;
      }
      else {
        timetrc = false;
      }
      break;
    case 'n':
      if (counters == false) {
        counters = true;
      }
      else {
        counters = false;
      }
      break;
    case 'q':
      if (quetrc == false) {
        quetrc = true;
      }
      else {
        quetrc = false;
      }
      break;
    case 's':
      if (swCmdtrc == false) {
        swCmdtrc = true;
      }
      else {
        swCmdtrc = false;
      }
      break;

    case 't':
      if (swCmdfbtrc == false) {
        swCmdfbtrc = true;
      }
      else {
        swCmdfbtrc = false;
      }
      break;
    case 'u':
      if (UDPintrc == false) {
        UDPintrc = true;
      }
      else {
        UDPintrc = false;
      }
      break;
  }
  readDebugflags();
  if (iptx[3] == 255) {
    show_wait_for_Rr_server();
  }
}
void writeMessage2Rocrail() {
  int i;
  packetTxBuffer[0] = message.id1;  // save prio and Command msbit
  packetTxBuffer[1] = message.id2;
  packetTxBuffer[2] = message.id3;
  packetTxBuffer[3] = message.id4;
  packetTxBuffer[4] = message.length;
  for (int i = 0; i < message.length; i++) {
    packetTxBuffer[i + 5] = message.data[i];
  }

  for (i = message.length; i < 8 ; i++) {
    packetTxBuffer[i + 5] = 0;
    message.data[i] = 0;  // clean the rest of the message
  }
  /*
    for (int i = 7; i > message.length - 1; i--) {
      packetTxBuffer[i + 5] = 0;
      message.data[i] = 0; // for logprinting
    }

  /*
    // show to which node we gonna send
    if (IPtrc == true) {
      Serial.print(F("To ip node: "));
      Serial.print(iptx);
      Serial.print(F(", port "));
      Serial.println(localTxPort);
    }
  */
  if ((swCmdfbtrc == true && packetTxBuffer[1] == 0x17) || (CANintrc == true)) { //feedback for switchcommand
    Serial.print(F("CAN -> UDP: "));
    Serial.print(message);
    if (timetrc) {
      time = millis() - lstCUtime;
      lstCUtime = millis();
      pptime = lstCUtime - lstpptime;
      lstpptime = lstCUtime;
      showInterval(time, pptime);
    }
    else {
      Serial.println();
    }
  }
  noInterrupts();
  // send a datagram, to the IP address and port that sent us the packet we received
  Udp.beginPacket(iptx, localTxPort);
  for (i = 0; i < 13; i++) {
    Udp.write(packetTxBuffer[i]);
  }
  Udp.endPacket();
  interrupts();
  UDPsnd++;
  if (counters) {
    Serial.print(F("UDP Packets send: "));
    Serial.println(UDPsnd);
  }
}

void errorQueueFull() {
  //Power off Rocrail
  message.id1 = 0;
  message.id2 = 0;
  message.id3 = msHash;  //with a pseudo hash
  message.id4 = lsHash;
  message.length = 5;
  for (int i = 0; i < 8; i++) {
    message.data[i] = 0;
  }
  writeMessage2Rocrail();
  // Befehl: System Halt to the track
  message.id1 = 0;
  message.id2 = 0;  // Systembefehl
  message.id3 = msHash;  // use own hash
  message.id4 = lsHash;
  message.length = 5;
  for (int i = 0; i < 8; i++) {
    message.data[i] = 0;  // to all
  }
  message.data[4] = 2; // Sub-CMD System Halt
  delay(queuetimeout); // be sure that the power off for coils is issued
  ctrl.sendMessage(message);  // Send CAN bus msg to Maerklin device
  CANsnd++;

  Serial.println(F("ERROR, Command denied and flushed"));
  if (!ArduinoQueueError) {
    Serial.println(F("ERROR, arduino queue full, powering down Command Station"));
  }
  ArduinoQueueError = true;
}

void printQueStat() {
  Serial.print(F(" q1d:"));
  Serial.print(queue1.count());
  Serial.print(F(" q2d:"));
  Serial.print(queue2.count());
  Serial.print(F(" q3d:"));
  Serial.println(queue3.count());
}

void printDCCMM(int decaddr) {
  Serial.print(F(" type "));
  if (decaddr & 0x0800) {
    Serial.print ("DCC");
  }
  else {
    Serial.print("MM");
  }
  Serial.print(F(" decaddr: "));
  Serial.print(decaddr & 0x07FF);
}

void showMessDecPos(byte qnr, int decaddr, byte i) {
  Serial.print(F("Message: "));
  Serial.print(message);
  printDCCMM(decaddr);
  Serial.print(F(" port: "));
  Serial.print(calculatePort());
  Serial.print(F(" queued to queue"));
  Serial.print(qnr);
  Serial.print(F(" at position: "));
  Serial.println(i);
}

void createQue1Entry(int decaddr) {
  // in case of queue full, power off
  if ((queue1.count() == maxentry) ||  ArduinoQueueError) {
    errorQueueFull();
    return;
  }
  queue1.enqueue(message);
  i = 0;
  while ((que1addr[i] != 0) && (i < sizeof(que1addr))) {
    i++;
  }
  que1addr[i] = decaddr;
  if (quetrc == true) {
    showMessDecPos(1, decaddr, i);
    printQueStat();
  }
  if (queue1.count() > obsmaxentry1top) {
    obsmaxentry1top = queue1.count();
  }
}

void createQue2Entry(int decaddr) {
  // in case of queue full, power off
  if ((queue2.count() == maxentry) || ArduinoQueueError) {
    errorQueueFull();
    return;
  }
  queue2.enqueue(message);
  i = 0;
  while ((que2addr[i] != 0) && (i < sizeof(que2addr))) {
    i++;
  }
  que2addr[i] = decaddr;
  if (quetrc == true) {
    showMessDecPos(2, decaddr, i);
    printQueStat();
  }
  if (queue2.count() > obsmaxentry2top) {
    obsmaxentry2top = queue2.count();
  }
}

void createQue3Entry(int decaddr) {
  // in case of queue full, power off
  if ((queue3.count() == maxentry) || ArduinoQueueError) {
    errorQueueFull();
    return;
  }
  queue3.enqueue(message);
  i = 0;
  while ((que3addr[i] != 0) && (i < sizeof(que3addr))) {
    i++;
  }
  que3addr[i] = decaddr;
  if (quetrc == true) {
    showMessDecPos(3, decaddr, i);
    printQueStat();
  }
  if (queue3.count() > obsmaxentry3top) {
    obsmaxentry3top = queue3.count();
  }
}

void createQueEntry(int decaddr) {
  // determine which queue has already an entry for this decoder
  for (int i = 0; i < queue1.count(); i++) {
    if (decaddr == que1addr[i]) {
      createQue1Entry(decaddr);  //queue this command if the decoder has already a command in this queue
      return;
    }
  }
  for (int i = 0; i < queue2.count(); i++) {
    if (decaddr == que2addr[i]) {
      createQue2Entry(decaddr);  //queue this command if the decoder has already a command in this queue
      return;
    }
  }
  for (int i = 0; i < queue3.count(); i++) {
    if (decaddr == que3addr[i]) {
      createQue3Entry(decaddr);  //queue this command if the decoder has already a command in this queue
      return;
    }
  }
  // up till now we did not find a match in any queue,
  // create an entry in the least loaded queue
  byte minval = 1;
  if (queue2.count() < queue1.count()) {
    minval = 2;
  }
  if (queue3.count() < queue2.count()) {
    minval = 3;
  }
  switch (minval) {
    case 1: createQue1Entry(decaddr);
      break;
    case 2: createQue2Entry(decaddr);
      break;
    case 3: createQue3Entry(decaddr);
      break;
  }
}


void registerAndSend(int decaddr, byte queue) {
  thisdlytime = (word(message.data[6], message.data[7])) * 10;
  queue--;  // zero indexed
  if (currentSend[queue] == 0) {
    csAccSnd++;  //increment the concurrent treated accessory commands
  }
  //store this decoderaddress
  currentSend[queue] = decaddr;
  ctrl.sendMessage(message);  // Send CAN bus msg to Maerklin device
  CANsnd++;
  if (counters) {
    Serial.print(F("CAN Packets send: "));
    Serial.println(CANsnd);
  }
  quedlytime = lstqueswtime; // to compute the delay time between by the queue send packets
  lstqueswtime = millis();
  switch (queue) {
      //store the timeout value in case a CAN ready messageg from GFP is missed
    case 0: pktfromqueue1send = lstqueswtime + thisdlytime + queuetimeout;
      break;
    case 1: pktfromqueue2send = lstqueswtime + thisdlytime + queuetimeout;
      break;
    case 2: pktfromqueue3send = lstqueswtime + thisdlytime + queuetimeout;
      break;
  }
  if (timetrc) {
    Serial.print(lstqueswtime);
    Serial.print(F(" Time since last accessory cmd: "));
    Serial.println(lstqueswtime - quedlytime);
    //    Serial.print(F("Delay time requested for accessory cmd: "));
    //    Serial.println(thisdlytime);
  }
  /*
    if (quetrc) {
      Serial.print(F("Current decoder: "));
      Serial.println(decaddr);
      Serial.print(F("From queue: "));
      Serial.println(queue + 1);
      Serial.print(F("Nr of cmds queued: "));
      Serial.println(csAccSnd);
      Serial.print(F("Decoder addresses currently handled (GFP):"));
      for (int i = 0; i < sizeof(currentSend); i++) {
        if (currentSend[i] != 0) {
          Serial.print(F(" "));
          Serial.print(currentSend[i]);
        }
      }
      Serial.println();
      Serial.println();
    }
  */
}

int calculateDecAddr() {
  // calculate decoder address
  int decaddr = word(message.data[2], message.data[3]);
  int pckttype = decaddr & 0x3800;
  decaddr = decaddr & 0x0FFF;
  decaddr = decaddr / decDevices;
  decaddr++; //offset=1
  decaddr = decaddr + pckttype;
  return (decaddr);
}

int calculatePort() {
  // calculate decoder address
  int decaddr = word(message.data[2], message.data[3]);
  int pckttype = decaddr & 0x3800;
  decaddr = decaddr & 0x0FFF;
  int decnr = decaddr / decDevices;
  int port = decaddr - (decnr * decDevices);
  port++;
  return (port);
}

void cleanEntry(byte i) {
  if (currentSend[i] != 0) {
    // clear the entry in currently handled
    currentSend[i] = 0;
    csAccSnd--;
  }
}

void cleanAccAddr() {
  // lookup for decoder address
  int curraddr = calculateDecAddr();
  byte i = 0;
  while ((!(curraddr == currentSend[i])) && (i < sizeof(currentSend))) {
    i++;
  }
  switch (i) {
      // mark the accessoire as handled and correct timeout for this queue
    case 0:
      if (timetrc) {
        Serial.print(pktfromqueue1send);
      }
      pktfromqueue1send = millis() + packetwaittime;
      cleanEntry(i);
      break;
    case 1:
      if (timetrc) {
        Serial.print(pktfromqueue2send);
      }
      pktfromqueue2send = millis() + packetwaittime;
      cleanEntry(i);
      break;
    case 2:
      if (timetrc) {
        Serial.print(pktfromqueue3send);
      }
      pktfromqueue3send = millis() + packetwaittime;
      cleanEntry(i);
      break;
    default:
      Serial.println();
      Serial.print(F("ERROR, device decaddr "));
      Serial.print(((curraddr & 0x07FF) - 1) * 4);
      printDCCMM(curraddr);
      Serial.println(F(" not found in list currentSend"));
      Serial.print(F("Decoder addresses currently handled (GFP):"));
      for (int i = 0; i < sizeof(currentSend); i++) {
        Serial.print(F(" "));
        Serial.print(currentSend[i] & 0x07FF);
      }
      Serial.println();
  }
  if (timetrc) {
    Serial.print(F(" changed to: "));
    Serial.print(millis());
    if (!quetrc) {
      Serial.println();
    }
  }
  if (timetrc && quetrc) {
    Serial.print(F("%S%-Acc ready. "));
    printDCCMM(curraddr);
    Serial.print(F(" dev nr: "));
    Serial.print(message.data[3], HEX);
    Serial.print(F(" from queue: "));
    Serial.print(i + 1);
    Serial.print(F(" still processed: "));
    Serial.println(csAccSnd);
  }
}

void sendByQueue() {
  int decaddr = calculateDecAddr();
  //determine if we have already a command executed for this decoder
  //TO DO change in a while loop
  /*
    byte hit = 3;
    for (int i = 0; i < sizeof(currentSend); i++) {
      if ((decaddr == currentSend[i]) && (currentSend[i] != 0)) {
        hit = i;
      }
    }
  */
  // The code below is not running well in Arduino V1.5.7
  // It is replaced by the code above. Who can explain why it does not run well?
  // for now the array is extended with one bogusbyte, and runs well. Array range checking implemented???
  byte hit = 0;
  while ((decaddr != currentSend[hit]) && (hit < 3)) {
    hit++;
  }
  // if found in array, hit to the correspondent queuenumber


  /*
      Serial.print(F("In current array: "));
      Serial.print(currentSend[0], HEX);
      Serial.print(F(" "));
      Serial.print(currentSend[1], HEX);
      Serial.print(F(" "));
      Serial.println(currentSend[2], HEX);
      Serial.print(F("Decoder adres: "));
      Serial.print(decaddr, HEX);
      Serial.print(F(" found current queue: "));
      Serial.println(hit);
  */
  if (hit < (sizeof(currentSend) - 1) / 2) { // the -1 is due to the extended array and range checking in 1.5.7
    if (quetrc) {
      printDCCMM(decaddr);
      Serial.print(F(" has already cmd executed in queue"));
      Serial.println(i + 1);
    }
    switch (hit) {
      case 0: createQue1Entry(decaddr);
        break;
      case 1: createQue2Entry(decaddr);
        break;
      case 2: createQue3Entry(decaddr);
        break;
    }
  }
  else
  {
    //determine queuenumber if we have already a command queued, while not currently executed
    createQueEntry(decaddr);
  }
}

void showDecPos(int decaddr, byte i) {
  Serial.print(F("decaddr rqst: "));
  Serial.print(decaddr & 0x07FF);
  Serial.print(F(", decaddr found and cleared in list at position: "));
  Serial.println(i);
}

void queryQueue() {
  //  check if we can send a command in respect to the switch command pause
  unsigned long currentTime = millis();
  if ((currentTime - lstqueswtime) < accPauseTime) {
    return;
  }
  // if queue was full, power down the cs after emptying the queue
  if (ArduinoQueueError && queue1.isEmpty() && queue2.isEmpty() && queue3.isEmpty()) {
    ArduinoQueueError = false; //reset the flag
    delay(queuetimeout); // be sure that the power off for coils is issued
    // switch off power from track by the GFP
    powerDown(msHash, lsHash);
    Serial.println(F("Queues empty, Command Station switched off due to queue error"));
    // it should be better to issue a Nothalt first
  }
  // check for elapsed time and if commands are issued to the cs
  i = 0;
  sendfromqueue = 0;
  switch (roundRobin) { // only one message is send in a cycle
    case 0: if (currentTime > pktfromqueue1send) {
        if (!queue1.isEmpty()) {
          //retrieve a command
          message = queue1.dequeue();
          int decaddr = calculateDecAddr();
          //clear in sidelist array
          while ((que1addr[i] != decaddr) && (i < sizeof(que1addr))) {
            i++;
          }
          if (quetrc == true) {
            showDecPos(decaddr, i);
          }
          que1addr[i] = 0;
          sendfromqueue = 1;
        }
        else
        {
          cleanEntry(0);
        }
      }
      break;

    case 1: if (currentTime > pktfromqueue2send) {
        if (!queue2.isEmpty()) {
          //retrieve a command
          message = queue2.dequeue();
          int decaddr = calculateDecAddr();
          //clear in sidelist array
          while ((que2addr[i] != decaddr) && (i < sizeof(que2addr))) {
            i++;
          }
          if (quetrc == true) {
            showDecPos(decaddr, i);
          }
          que2addr[i] = 0;
          sendfromqueue = 2;
        }
        else
        {
          cleanEntry(1);
        }
      }
      break;
    case 2: if (currentTime > pktfromqueue3send) {
        if (!queue3.isEmpty()) {
          //retrieve a command
          message = queue3.dequeue();
          int decaddr = calculateDecAddr();
          //clear in sidelist array
          while ((que3addr[i] != decaddr) && (i < sizeof(que3addr))) {
            i++;
          }
          if (quetrc == true) {
            showDecPos(decaddr, i);
          }
          que3addr[i] = 0;
          sendfromqueue = 3;
        }
        else
        {
          cleanEntry(2); // remove from active send list
        }
      }
      break;
  }
  // pointer to handle queue in next loop
  roundRobin++;
  if (roundRobin > 2) {
    roundRobin = 0;
  }
  if ((quetrc == true) && (sendfromqueue > 0)) {
    Serial.print(F("queue msg retrieved from queue"));
    Serial.print(sendfromqueue);
    Serial.print(F(" : "));
    Serial.println(message);
    printQueStat();
  }
  if (sendfromqueue > 0) {
    int decaddr = calculateDecAddr();
    registerAndSend(decaddr, sendfromqueue);
  }
}

void catchUIDs()
{
  if ((message.data[6] == 0x00) && (message.data[7] == 0x10)) {
    gb_uid_d0 = message.data[0];
    gb_uid_d1 = message.data[1];
    gb_uid_d2 = message.data[2];
    gb_uid_d3 = message.data[3];
  }
  if ((message.data[6] == 0x00) && ((message.data[7] & 0x30) == 0x30)) {
    ms2_uid_d0 = message.data[0];
    ms2_uid_d1 = message.data[1];
    ms2_uid_d2 = message.data[2];
    ms2_uid_d3 = message.data[3];
  }
}

void short_circuit_detection() {
  int unit = ((shortDetect - 1) / 8); // calculate the array index position
  byte bitnr = ((shortDetect - 1) % 8); // calculate the bit position
  mPrevSwitch = mSwitch;
  mSwitch =  digitalRead(pinSCD); //write the new status
  if( mSwitch == 1 && power_ena ) {
    Serial.println(F("Short circuit detected"));
    power_ena = false;    
  }
  byte contacts = mSwitch ^ mPrevSwitch; // contact change
  if (contacts != 0) { //every change should generate a datagram
    message.id1 = 0x00;
    message.id2 = 0x23;
    message.id3 = msHash;
    message.id4 = lsHash;
    message.length  = 8;

    // compute kontaktkennung
    word kk = unit * 8; // base is reporter byte
    kk = kk + bitnr + 1; // first contact is nr 1
    message.data[0] = highByte(gk);
    message.data[1] = lowByte(gk);
    message.data[2] = highByte(kk);
    message.data[3] = lowByte(kk);
    message.data[4] = mPrevSwitch;
    // modified to emulate Maerklin CS2 3.0.1 software
    if (message.data[4] != 0) {
      message.data[4] = 1;
    }
    // modified to emulate Maerklin CS2 3.0.1 software
    message.data[5] = mSwitch;
    if (message.data[5] != 0) {
      message.data[5] = 1;
    }
    message.data[6] = 0;
    message.data[7] = 0;
    writeMessage2Rocrail();
  }
}

void loop() {
  // The CAN controller has different Receive registers which use a common interrupt pin
  // So read until the INT has cleared
  if (ctrl.receiveMessage(message) == true) {
    delayMicroseconds(1300); //wait some time to prevent Catch22 interrupt actions, limit is about 1140
    writeMessage2Rocrail();
    CANrx++;
    // check for a ended accessory command
    if ((message.id2 == 0x17) && (message.data[5] == 0)) {
      cleanAccAddr();
    }
    // check to catch the UIDs from Gleisbox or MS2 (ping + responsebit)
    if ((message.id2 == 0x31) && (message.length == 8)) {
      catchUIDs();
    }

    if (counters) {
      Serial.print(F("CAN Packets Rx: "));
      Serial.println(CANrx);
    }
  }
//  pollGleisboxChannel();
  noInterrupts();
  int packetSize = Udp.parsePacket(); //waiting for message from Rocrail
  interrupts();
  if (IPtrc && packetSize) {
    Serial.print(F("Received Ethpacket of size "));
    Serial.println(packetSize);
    Serial.print(F("From "));
    noInterrupts();
    IPAddress remote = Udp.remoteIP();
    interrupts();
    Serial.print(remote);
    Serial.print(F(", port "));
    noInterrupts();
    Serial.println(Udp.remotePort());
    interrupts();
  }
  if (packetSize == 13)  { // waiting for a valid Rocrail Ethernet CAN bus message via Ethernet
    UDPrx++;
    noInterrupts();
    Udp.read(packetRxBuffer, 13);
    interrupts();
    message.id1 = packetRxBuffer[0];
    message.id2 = packetRxBuffer[1];
    message.id3 = packetRxBuffer[2];
    message.id4 = packetRxBuffer[3];
    message.length = packetRxBuffer[4];
//    for (int i = 0; i < message.length; i++) {
    for (int i = 0; i < 13; i++) {
      message.data[i] = packetRxBuffer[i + 5];
    }
    if ((swCmdtrc == true && message.id2 == 0x16) || UDPintrc) { // show switch commands
      time = millis() - lstswtime;
      lstswtime = millis();
      pptime = lstswtime - lstpptime;
      lstpptime = lstswtime;
      Serial.print(F("UDP -> CAN: "));
      Serial.print(message);
      if (timetrc) {
        showInterval(time, pptime);
      }
      else {
        Serial.println();
      }

    }
    // for accessory commands check the number of outstanding commands or queue command
    if (message.id2 == 0x16) {
      sendByQueue();
    }
    else
    {
      // it is not a accessory command
      ctrl.sendMessage(message);  // Send UDP Rocrail message to CAN bus
      CANsnd++;
    }
  }
  else
  {
    if (packetSize != 0) {  //we received something but it is not our cup of tea, log anytime
      Serial.print(F("Rx Ethpckt<>13 size: "));
      Serial.println(packetSize);
    }
    queryQueue();
  }
  // check for a debug command
  if (Serial.available() > 0) {
    setDebugflags(Serial.read());
  }
  // in case of power command change power status
  if ((message.id2 == 0x00) && (message.length == 5) && (message.data[4] < 0x02)) {
    if (message.data[4] == 0x01) { //on "Go" command
      power_ena = true;
    }
    else {
      power_ena = false;
    }
  }
  if( shortdetection ) {  // check for short circuit detection if enabled
    short_circuit_detection();
  }

  /*
       // this is for development use. It measures the time necesseray for executing the loop
       prevlooptime = looptime;
       looptime = millis();
       if ((looptime - prevlooptime) > 2) {
         Serial.println(looptime - prevlooptime);
       }
  */
}
