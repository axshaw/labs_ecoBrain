#include <EcoSPI.h>
#include <EcoSensors.h>
#include "EcoBlackboard.h"
#include "pins_arduino.h"

#define BRAIN_READY              9

// what to do with incoming data
volatile byte command = 0;
volatile int byteCount = 0;
byte* b;

volatile boolean calculate = false;

//volatile int brainCommandCount = 0;

volatile byte* brainCommand;

byte brainCommandCount = 0;
byte** brainCommands;

//volatile int brainCommandIndex = 0;
//volatile int brainCommandByteIndex = 0;

EcoBlackboard eb;
EcoSensors es;
//SPI_BRAIN_COMMAND* brainCommands;

// start of transaction, no command yet
void ss_falling ()
{
  digitalWrite(BRAIN_READY, HIGH);
  calculate = false;
  
  command = 0;
  byteCount = 0;
//  brainCommandIndex++;
//  brainCommandByteIndex = 0;
  EcoSPI::getBytes(0, b);
}  // end of interrupt service routine (ISR) ss_falling

void setup (void)
{
//  Serial.begin(115200);
  Serial.println("EcoBrain");
  // setup pins
  pinMode(MISO, OUTPUT);
  pinMode(SS, INPUT);

  pinMode(BRAIN_READY, OUTPUT);
  digitalWrite(BRAIN_READY, HIGH);

  //eb.rundiagnostic();

  // turn on SPI in slave mode
  SPCR |= _BV(SPE);

  // turn on interrupts
  SPCR |= _BV(SPIE);

  // interrupt for SS falling edge
  attachInterrupt (0, ss_falling, FALLING);
}  // end of setup

void loop (void)
{
  if (calculate) {
    Serial.println("Let's calculate it");
  
//    es.ultrasonic0 = 10;//random(500);
//    es.ultrasonic45 = random(500);
//    es.ultrasonic90 = 20;//random(500);
//    es.ultrasonic135 = random(500);
//    es.ultrasonic180 = 30;//random(500);
//    es.ultrasonic225 = random(500);
//    es.ultrasonic270 = 40;//random(500);
//    es.ultrasonic315 = random(500);

/*
    eb.rundiagnostic();
    eb.getAction(es);
*/
  
    byte* cmd;
    eb.getAction(es, cmd);

    writeCommand(cmd);
    
    digitalWrite(BRAIN_READY, LOW);
    //readCommand();
  }

//  delay(1000);
}  // end of loop

//void readCommand(void) {
//  Serial.println("Read command");
//  
//  byte* buf;
//  buf[0] = brainCommand[2];
//  buf[1] = brainCommand[3];
//  buf[2] = brainCommand[4];
//  buf[3] = brainCommand[5];
//  
//  Serial.println(brainCommand[0], HEX);
//  Serial.println(brainCommand[1]);
//  Serial.println(brainCommand[2]);
//  Serial.println(brainCommand[3]);
//  Serial.println(brainCommand[4]);
//  Serial.println(brainCommand[5]);
//}

void writeCommand(byte* &cmd) {
  Serial.println("Write command");

  int messageSize = ((uint8_t)cmd[1]) + 2;

  Serial.print("messageSize: ");  
  Serial.println(messageSize);

//  brainCommands[brainCommandCount] = new byte[messageSize];
//
  brainCommand = new byte[messageSize];

  for(int x=0;x<messageSize;x++) {
    brainCommand[x] = cmd[x];
//    brainCommands[brainCommandCount][x] = cmd[x];
  }
//  
//  brainCommandCount++;
}

// SPI interrupt routine
ISR (SPI_STC_vect)
{
  byte c = SPDR;
 
  switch (command)
  {
  // no command? then this is the command
  case 0:
    command = c;
    SPDR = 0;
    break;

//  case RESET_SENSORS:
//    // new ecosensors object;
//    break;
  case BRAIN_COMMAND:
    SPDR = brainCommand[byteCount++];  
    break;
    
  case BRAIN_CALCULATE:
    calculate = true;
    SPDR = 0;
    break;

  case ULTRASONIC_0:
    b[byteCount++] = c;
    if (byteCount == 4) {
      es.ultrasonic0 = EcoSPI::bytesToLong(b);
    }
    SPDR = 0;
    break;
  case ULTRASONIC_45:
    b[byteCount++] = c;
    if (byteCount == 4) {
      es.ultrasonic45 = EcoSPI::bytesToLong(b);
    }
    SPDR = 0;
    break;
  case ULTRASONIC_90:
    b[byteCount++] = c;
    if (byteCount == 4) {
      es.ultrasonic90 = EcoSPI::bytesToLong(b);
    }
    SPDR = 0;
    break;
  case ULTRASONIC_135:
    b[byteCount++] = c;
    if (byteCount == 4) {
      es.ultrasonic135 = EcoSPI::bytesToLong(b);
    }
    SPDR = 0;
    break;
  case ULTRASONIC_180:
    b[byteCount++] = c;
    if (byteCount == 4) {
      es.ultrasonic180 = EcoSPI::bytesToLong(b);
    }
    SPDR = 0;
    break;
  case ULTRASONIC_225:
    b[byteCount++] = c;
    if (byteCount == 4) {
      es.ultrasonic225 = EcoSPI::bytesToLong(b);
    }
    SPDR = 0;
    break;
  case ULTRASONIC_270:
    b[byteCount++] = c;
    if (byteCount == 4) {
      es.ultrasonic270 = EcoSPI::bytesToLong(b);
    }
    SPDR = 0;
    break;
  case ULTRASONIC_315:
    b[byteCount++] = c;
    if (byteCount == 4) {
      es.ultrasonic315 = EcoSPI::bytesToLong(b);
    }
    SPDR = 0;
    break;
    
  case SPI_HANDSHAKE:
    SPDR = DEVICE_BRAIN;
    break;
    
  } // end of switch
}  // end of interrupt service routine (ISR) SPI_STC_vect
