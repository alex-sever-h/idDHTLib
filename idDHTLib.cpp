/*
	FILE: 		idDHTLib.cpp
	VERSION: 	0.0.3
	PURPOSE: 	Interrupt driven Lib for DHT11 and DHT22 with Arduino.
	LICENCE:	GPL v3 (http://www.gnu.org/licenses/gpl.html)
	DATASHEET: http://www.micro4you.com/files/sensor/DHT11.pdf
	DATASHEET: http://www.adafruit.com/datasheets/DHT22.pdf

	Based on idDHT11 library: https://github.com/niesteszeck/idDHT11
	Based on DHTLib library: http://playground.arduino.cc/Main/DHTLib
	Based on code proposed: http://forum.arduino.cc/index.php?PHPSESSID=j6n105kl2h07nbj72ac4vbh4s5&topic=175356.0

	Changelog:
		v 0.0.1
			fork from idDHT11 lib
			change names to idDHTLib
			added DHT22 functionality
		v 0.0.2
			Optimization on shift var (pylon from Arduino Forum)
		v 0.0.3
			Timing correction to finally work properly on DHT22
			(Dessimat0r from Arduino forum)
    v 1.0.0
      autoformat code with Arduino IDE code formatting standards (kcsoft)
      remove the interrupt number from the constructor by using digitalPinToInterrupt (kcsoft)
      fix type for us and timeout when no interrupt is triggered (kcsoft)
      removed the callback parameter from the constructor, added sensor type (DHT11, DHT22) as optional param (kcsoft)
      removed temp/humid calculation from the isr (kcsoft)
      new function acquireFastLoop to remove delay when start acquiring (kcsoft)
      update README.md file (kcsoft)
 */

#include "idDHTLib.h"
#define DEBUG_idDHTLIB


/* Pin to interrupt map:
 * D0-D7 = PCINT 16-23 = PCIR2 = PD = PCIE2 = pcmsk2
 * D8-D13 = PCINT 0-5 = PCIR0 = PB = PCIE0 = pcmsk0
 * A0-A5 (D14-D19) = PCINT 8-13 = PCIR1 = PC = PCIE1 = pcmsk1
 */

volatile uint8_t *port_to_pcmask[] = {
  &PCMSK0,
  &PCMSK1,
  &PCMSK2
};

static int PCintMode[24];

typedef void (*voidFuncPtr)(void);

volatile static voidFuncPtr PCintFunc[24] = {
  NULL };

volatile static uint8_t PCintLast[3];

/*
 * attach an interrupt to a specific pin using pin change interrupts.
 */
 void PCattachInterrupt(uint8_t pin, void (*userFunc)(void), int mode) {
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  uint8_t slot;
  volatile uint8_t *pcmask;

  // map pin to PCIR register
  if (port == NOT_A_PORT) {
    return;
  }
  else {
    port -= 2;
    pcmask = port_to_pcmask[port];
  }

// -- Fix by Baziki. In the original sources it was a little bug, which cause analog ports to work incorrectly.
  if (port == 1) {
     slot = port * 8 + (pin - 14);
  }
  else {
     slot = port * 8 + (pin % 8);
  }
// --Fix end
  PCintMode[slot] = mode;
  PCintFunc[slot] = userFunc;
  // set the mask
  *pcmask |= bit;
  // enable the interrupt
  PCICR |= 0x01 << port;
}

void PCdetachInterrupt(uint8_t pin) {
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *pcmask;

  // map pin to PCIR register
  if (port == NOT_A_PORT) {
    return;
  }
  else {
    port -= 2;
    pcmask = port_to_pcmask[port];
  }

  // disable the mask.
  *pcmask &= ~bit;
  // if that's the last one, disable the interrupt.
  if (*pcmask == 0) {
    PCICR &= ~(0x01 << port);
  }
}

// common code for isr handler. "port" is the PCINT number.
// there isn't really a good way to back-map ports and masks to pins.
static void PCint(uint8_t port) {
  uint8_t bit;
  uint8_t curr;
  uint8_t mask;
  uint8_t pin;

  // get the pin states for the indicated port.
  curr = *portInputRegister(port+2);
  mask = curr ^ PCintLast[port];
  PCintLast[port] = curr;
  // mask is pins that have changed. screen out non pcint pins.
  if ((mask &= *port_to_pcmask[port]) == 0) {
    return;
  }
  // mask is pcint pins that have changed.
  for (uint8_t i=0; i < 8; i++) {
    bit = 0x01 << i;
    if (bit & mask) {
      pin = port * 8 + i;
      // Trigger interrupt if mode is CHANGE, or if mode is RISING and
      // the bit is currently high, or if mode is FALLING and bit is low.
      if ((PCintMode[pin] == CHANGE
          || ((PCintMode[pin] == RISING) && (curr & bit))
          || ((PCintMode[pin] == FALLING) && !(curr & bit)))
          && (PCintFunc[pin] != NULL)) {
        PCintFunc[pin]();
      }
    }
  }
}


SIGNAL(PCINT0_vect) {
  PCint(0);
}
SIGNAL(PCINT1_vect) {
  PCint(1);
}
SIGNAL(PCINT2_vect) {
  PCint(2);
}


const pCallback idDHTLib::pCallbackArray[] = {PFUNC_CALLBACKS};
idDHTLib * idDHTLib::objectAtInt[MAX_INTERRUPT + 1];

idDHTLib::idDHTLib(int pin) {
  init(pin, DHT11);
}

idDHTLib::idDHTLib(int pin, DHTType sensorType) {
  init(pin, sensorType);
}

void idDHTLib::init(int pin, DHTType sensorType) {
  intNumber = 0;
  this->pin = pin;
  this->sensorType = sensorType;
  objectAtInt[intNumber] = this;
  hum = 0;
  temp = 0;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  state = STOPPED;
  status = IDDHTLIB_ERROR_NOTSTARTED;
}

int idDHTLib::startSignal(bool useDelay) {
  if (state == STOPPED || state == ACQUIRED) {

    //set the state machine for interruptions analisis of the signal
    state = useDelay ? RESPONSE : START_SIGNAL;

    // EMPTY BUFFER and vars
    for (byte i = 0; i < 5; i++) bits[i] = 0;
    cnt = 7;
    idx = 0;
    hum = 0;
    temp = 0;

    // REQUEST SAMPLE
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);

    if (useDelay) {
      delay(18);
      digitalWrite(pin, HIGH);
      delayMicroseconds(25);
      pinMode(pin, INPUT);
    }

    us = micros();
    // Analize the data in an interrupt
    if (useDelay)
      PCattachInterrupt(this->pin, dhtCallbackStatic, FALLING);

    return IDDHTLIB_ACQUIRING;
  } else
    return IDDHTLIB_ERROR_ACQUIRING;
}

int idDHTLib::acquire() {
  return startSignal(true);
}

// start acquiring but don't do the start delay here, instead will be done
// in the acquiring() function that has to be called in a loop with < 10ms delay 
int idDHTLib::acquireFastLoop() {
  return startSignal(false);
}

int idDHTLib::acquireAndWait() {
  acquire();
  while (acquiring())
    ;
  return getStatus();
}
void idDHTLib::dhtCallback() {
  unsigned long newUs = micros();
  byte delta;
  if (newUs - us > 255) {
    status = IDDHTLIB_ERROR_TIMEOUT;
    state = STOPPED;
    PCdetachInterrupt(this->pin);
    return;
  }

  delta = newUs - us;
  us = newUs;

  switch (state) {
    case RESPONSE:
      if (delta < 25) {
        us -= delta;
        break; //do nothing, it started the response signal
      } if (125 < delta && delta < 190) {
        state = DATA;
      } else {
        PCdetachInterrupt(this->pin);
        status = IDDHTLIB_ERROR_TIMEOUT;
        state = STOPPED;
      }
      break;
    case DATA:
      if (60 < delta && delta < 145) { //valid in timing
        bits[idx] <<= 1; //shift the data
        if (delta > 100) //is a one
          bits[idx] |= 1;
        if (cnt == 0) {  // when we have fulfilled the byte, go to the next
          cnt = 7;    // restart at MSB
          if (++idx == 5) { // go to next byte; when we have got 5 bytes, stop.
            PCdetachInterrupt(this->pin);
            state = RAW_DATA_READY;
            break;
          }
        } else cnt--;
      } else if (delta < 10) {
        PCdetachInterrupt(this->pin);
        status = IDDHTLIB_ERROR_DELTA;
        state = STOPPED;
      } else {
        PCdetachInterrupt(this->pin);
        status = IDDHTLIB_ERROR_TIMEOUT;
        state = STOPPED;
      }
      break;
    default:
      break;
  }
}
bool idDHTLib::acquiring() {
  unsigned long delta;
  if (state != ACQUIRED && state != STOPPED) {
    if (state == RAW_DATA_READY) {
      // WRITE TO RIGHT VARS
      uint8_t sum;
      if (sensorType == DHT22) {
        hum = word(bits[0], bits[1]) * 0.1;
        temp = (bits[2] & 0x80 ?
                -word(bits[2] & 0x7F, bits[3]) :
                word(bits[2], bits[3]))
               * 0.1;
        sum = bits[0] + bits[1] + bits[2] + bits[3];
      } else {
        hum    = bits[0];
        // as bits[1] and bits[3] are always zero they are omitted in formulas.
        temp = bits[2];
        sum = bits[0] + bits[2];
      }
      if (bits[4] != (sum & 0xFF)) {
        status = IDDHTLIB_ERROR_CHECKSUM;
        state = STOPPED;
      } else {
        status = IDDHTLIB_OK;
        state = ACQUIRED;
      }
    } else {
      cli();
      delta = micros() - us;
      sei();
      if (state == START_SIGNAL) {
        if (delta > 18000) {
          state = RESPONSE;
          digitalWrite(pin, HIGH);
          delayMicroseconds(25);
          pinMode(pin, INPUT);
          us = micros();
          // Analize the data in an interrupt
          PCattachInterrupt(this->pin, dhtCallbackStatic, FALLING);
        }
      } else {
        if (delta > 255) {
          status = IDDHTLIB_ERROR_TIMEOUT;
          state = STOPPED;
          PCdetachInterrupt(this->pin);
          return false;
        }
      }
      return true;
    }
  }
  return false;
}
int idDHTLib::getStatus() {
  return status;
}
float idDHTLib::getCelsius() {
  IDDHTLIB_CHECK_STATE;
  return temp;
}

float idDHTLib::getHumidity() {
  IDDHTLIB_CHECK_STATE;
  return hum;
}

float idDHTLib::getFahrenheit() {
  IDDHTLIB_CHECK_STATE;
  return temp * 1.8 + 32;
}

float idDHTLib::getKelvin() {
  IDDHTLIB_CHECK_STATE;
  return temp + 273.15;
}

// delta max = 0.6544 wrt dewPoint()
// 5x faster than dewPoint()
// reference: http://en.wikipedia.org/wiki/Dew_point
double idDHTLib::getDewPoint() {
  IDDHTLIB_CHECK_STATE;
  double a = 17.271;
  double b = 237.7;
  double temp_ = (a * (double) temp) / (b + (double) temp) + log( (double) hum / 100);
  double Td = (b * temp_) / (a - temp_);
  return Td;

}
// dewPoint function NOAA
// reference: http://wahiduddin.net/calc/density_algorithms.htm
double idDHTLib::getDewPointSlow() {
  IDDHTLIB_CHECK_STATE;
  double A0 = 373.15 / (273.15 + (double) temp);
  double SUM = -7.90298 * (A0 - 1);
  SUM += 5.02808 * log10(A0);
  SUM += -1.3816e-7 * (pow(10, (11.344 * (1 - 1 / A0))) - 1) ;
  SUM += 8.1328e-3 * (pow(10, (-3.49149 * (A0 - 1))) - 1) ;
  SUM += log10(1013.246);
  double VP = pow(10, SUM - 3) * (double) hum;
  double T = log(VP / 0.61078); // temp var
  return (241.88 * T) / (17.558 - T);
}
// EOF
