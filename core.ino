/*
Guinguin MME DAC CV Controller
Alessandro Guttenberg <guttenba@gmail.com>, May 2021
*/

#include <DIO2.h>
#include <SPI.h>

#define KEYS_NUMBER 49

#define NOTE_SF 47.069f // This value can be tuned if CV output isn't exactly 1V/octave

#define KEY_OFF               0
#define KEY_START             1
#define KEY_ON                2
#define KEY_RELEASED          3

#define MIN_TIME_MS   3
#define MAX_TIME_MS   50
#define MAX_TIME_MS_N (MAX_TIME_MS - MIN_TIME_MS)

#define FSR   A0 // Force Sensitive Resistor (ribbon), with pull-down resistor
#define PITCH_WHEEL A1
#define GATE  5
#define DAC1  8
#define DAC2  9

//Lower Ribbon
#define PIN_T1  50
#define PIN_T2  48
#define PIN_T3  46
#define PIN_T4  44
#define PIN_T5  42
#define PIN_T6  40
#define PIN_T7  38
#define PIN_T8  36

//Upper Ribbon
#define PIN_T9  34
#define PIN_T10 32
#define PIN_T11 30
#define PIN_T12 28
#define PIN_T13 26
#define PIN_T14 24
#define PIN_T15 22
#define PIN_T16 20

//Lower Ribbon
#define PIN_B3  47
#define PIN_B4  45
#define PIN_B5  43
#define PIN_B6  41
#define PIN_B7  39
#define PIN_B8  37

//Upper Ribbon
#define PIN_B9  35
#define PIN_B10 33
#define PIN_B11 31
#define PIN_B12 29
#define PIN_B13 27
#define PIN_B14 25
#define PIN_B15 23
#define PIN_B16 21

byte input_pins[] = {
  PIN_T5,
  PIN_T5,
  PIN_T6,
  PIN_T6,
  PIN_T7,
  PIN_T7,
  PIN_T8,
  PIN_T8,
  PIN_T1,
  PIN_T1,
  PIN_T2,
  PIN_T2,
  PIN_T3,
  PIN_T3,
  PIN_T4,
  PIN_T4,
  PIN_T5,
  PIN_T5,
  PIN_T6,
  PIN_T6,
  PIN_T7,
  PIN_T7,
  PIN_T8,
  PIN_T8,
  PIN_T1,
  PIN_T1,
  PIN_T2,
  PIN_T2,
  PIN_T3,
  PIN_T3,
  PIN_T4,
  PIN_T4,
  PIN_T5,
  PIN_T5,
  PIN_T6,
  PIN_T6,
  PIN_T7,
  PIN_T7,
  PIN_T8,
  PIN_T8,
  PIN_T9,
  PIN_T9,
  PIN_T10,
  PIN_T10,
  PIN_T11,
  PIN_T11,
  PIN_T12,
  PIN_T12,
  PIN_T13,
  PIN_T13,
  PIN_T14,
  PIN_T14,
  PIN_T15,
  PIN_T15,
  PIN_T16,
  PIN_T16,
  PIN_T9,
  PIN_T9,
  PIN_T10,
  PIN_T10,
  PIN_T11,
  PIN_T11,
  PIN_T12,
  PIN_T12,
  PIN_T13,
  PIN_T13,
  PIN_T14,
  PIN_T14,
  PIN_T15,
  PIN_T15,
  PIN_T16,
  PIN_T16,
  PIN_T9,
  PIN_T9,
  PIN_T10,
  PIN_T10,
  PIN_T11,
  PIN_T11,
  PIN_T12,
  PIN_T12,
  PIN_T13,
  PIN_T13,
  PIN_T14,
  PIN_T14,
  PIN_T15,
  PIN_T15,
  PIN_T16,
  PIN_T16,
  PIN_T9,
  PIN_T9,
  PIN_T10,
  PIN_T10,
  PIN_T11,
  PIN_T11,
  PIN_T12,
  PIN_T12,
  PIN_T13,
  PIN_T13
};

byte output_pins[] = {
  PIN_B4,
  PIN_B3,
  PIN_B4,
  PIN_B3,
  PIN_B4,
  PIN_B3,
  PIN_B4,
  PIN_B3,
  PIN_B6,
  PIN_B5,
  PIN_B6,
  PIN_B5,
  PIN_B6,
  PIN_B5,
  PIN_B6,
  PIN_B5,
  PIN_B6,
  PIN_B5,
  PIN_B6,
  PIN_B5,
  PIN_B6,
  PIN_B5,
  PIN_B6,
  PIN_B5,
  PIN_B8,
  PIN_B7,
  PIN_B8,
  PIN_B7,
  PIN_B8,
  PIN_B7,
  PIN_B8,
  PIN_B7,
  PIN_B8,
  PIN_B7,
  PIN_B8,
  PIN_B7,
  PIN_B8,
  PIN_B7,
  PIN_B8,
  PIN_B7,
  PIN_B10,
  PIN_B9,
  PIN_B10,
  PIN_B9,
  PIN_B10,
  PIN_B9,
  PIN_B10,
  PIN_B9,
  PIN_B10,
  PIN_B9,
  PIN_B10,
  PIN_B9,
  PIN_B10,
  PIN_B9,
  PIN_B10,
  PIN_B9,
  PIN_B12,
  PIN_B11,
  PIN_B12,
  PIN_B11,
  PIN_B12,
  PIN_B11,
  PIN_B12,
  PIN_B11,
  PIN_B12,
  PIN_B11,
  PIN_B12,
  PIN_B11,
  PIN_B12,
  PIN_B11,
  PIN_B12,
  PIN_B11,
  PIN_B14,
  PIN_B13,
  PIN_B14,
  PIN_B13,
  PIN_B14,
  PIN_B13,
  PIN_B14,
  PIN_B13,
  PIN_B14,
  PIN_B13,
  PIN_B14,
  PIN_B13,
  PIN_B14,
  PIN_B13,
  PIN_B14,
  PIN_B13,
  PIN_B16,
  PIN_B15,
  PIN_B16,
  PIN_B15,
  PIN_B16,
  PIN_B15,
  PIN_B16,
  PIN_B15,
  PIN_B16,
  PIN_B15,
  PIN_B16,
  PIN_B15,
  PIN_B16,
  PIN_B15,
  PIN_B16,
  PIN_B15
};

//uncomment the next line to inspect the number of scans per seconds
//#define DEBUG_SCANS_PER_SECOND

//uncomment the next line to get text midi message at output
#define DEBUG_MIDI_MESSAGE

byte          keys_state[KEYS_NUMBER];
unsigned long keys_time[KEYS_NUMBER];
boolean       signals[KEYS_NUMBER * 2];
byte          fsrReading;
byte          pitchWheelReading;

void setup() {

#ifdef DEBUG_MIDI_MESSAGE
  Serial.begin(115200);
#else
  Serial.begin(31250);
#endif

  pinMode(GATE, OUTPUT);
  pinMode(DAC1, OUTPUT);
  pinMode(DAC2, OUTPUT);
  digitalWrite2(GATE, LOW);
  digitalWrite2(DAC1, HIGH);
  digitalWrite2(DAC2, HIGH);

  pinMode(13, OUTPUT);
  digitalWrite2(13, LOW);
  int i;
  for (i = 0; i < KEYS_NUMBER; i++)
  {
    keys_state[i] = KEY_OFF;
    keys_time[i] = 0;
  }
  for (byte pin = 0; pin < sizeof(output_pins); pin++)
  {
    pinMode(output_pins[pin], OUTPUT);
  }
  for (byte pin = 0; pin < sizeof(input_pins); pin++)
  {
    pinMode(input_pins[pin], INPUT_PULLUP);
  }

  pinMode(FSR, INPUT);
  fsrReading = 0;

  //read pin
  pinMode(PITCH_WHEEL, INPUT);
  pitchWheelReading = 64;

  SPI.begin();

}

void send_midi_note_event(int command, int key_index, unsigned long time)
{
  unsigned long t = time;

  if (t > MAX_TIME_MS)
    t = MAX_TIME_MS;
  if (t < MIN_TIME_MS)
    t = MIN_TIME_MS;
  t -= MIN_TIME_MS;
  unsigned long velocity = 127 - (t * 127 / MAX_TIME_MS_N);
  int vel = (((velocity * velocity) >> 7) * velocity) >> 7;
  int key = 12 + key_index;
#ifdef DEBUG_MIDI_MESSAGE
  char out[32];
  sprintf(out, "%02X %d %03d %d", command, key, vel, time);

  unsigned int mV = (unsigned int) ((float) key * NOTE_SF + 0.5); 
  setVoltage(DAC2, 1, 1, mV);
  setVoltage(DAC1, 1, 1, vel << 5);
  Serial.println(out);

#else
  setVoltage(DAC1, 1, 1, vel << 5);
  Serial.write(command);
  Serial.write(key);
  Serial.write(vel);
#endif
}

void send_midi_pressure_event(int command, int pressure)
{
#ifdef DEBUG_MIDI_MESSAGE
  setVoltage(DAC1, 0, 1, fsrReading << 5); // channel pressure via DAC
  char out[32];
  sprintf(out, "%02X %d", command, pressure);
  Serial.println(out);
#else
  setVoltage(DAC1, 0, 1, fsrReading << 5); // channel pressure via DAC
  Serial.write(command);
  Serial.write(pressure);
#endif
}

void send_midi_cc_event(int command, int value)
{
#ifdef DEBUG_MIDI_MESSAGE
  setVoltage(DAC2, 0, 1, pitchWheelReading << 5);
  char out[32];
  sprintf(out, "%02X %d", command, value);
  Serial.println(out);
#else
  setVoltage(DAC2, 0, 1, pitchWheelReading << 5);
  Serial.write(command);
  Serial.write(pressure);
#endif
}

/*
void send_midi_cc_event(int status_byte, int param1, int param2)
{
#ifdef DEBUG_MIDI_MESSAGE
  char out[32];
  sprintf(out, "%02X %d %d", status_byte, param1, param2);
  Serial.println(out);
#else
  Serial.write(status_byte);
  Serial.write(param1);
  Serial.write(param2);
#endif
}
*/

void setVoltage(int dacpin, bool channel, bool gain, unsigned int mV)
{
  unsigned int command = channel ? 0x9000 : 0x1000;

  command |= gain ? 0x0000 : 0x2000;
  command |= (mV & 0x0FFF);

  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite2(dacpin, LOW);
  SPI.transfer(command >> 8);
  SPI.transfer(command & 0xFF);
  digitalWrite2(dacpin, HIGH);
  SPI.endTransaction();
}


void loop() {

#ifdef DEBUG_SCANS_PER_SECOND
  static unsigned long cycles = 0;
  static unsigned long start = 0;
  static unsigned long current = 0;
  cycles++;
  current = millis();
  if (current - start >= 1000)
  {
    Serial.println(cycles);
    cycles = 0;
    start = current;
  }
#endif

 
  pitchWheelReading = map(analogRead(PITCH_WHEEL), 0, 1023, 0, 127) ;
  if ((pitchWheelReading >=68) && (pitchWheelReading<= 60))
  {
    send_midi_cc_event(176, pitchWheelReading);
  }
  
  fsrReading = map(analogRead(FSR), 0, 1023, 0, 127);

  boolean *s = signals;
  for (byte i = 0; i < KEYS_NUMBER * 2; i++)
  {
    byte output_pin = output_pins[i];
    byte input_pin = input_pins[i];
    digitalWrite2(output_pin, LOW);
    *(s++) = !digitalRead2(input_pin);
    digitalWrite2(output_pin, HIGH);
  }

  byte          *state  = keys_state;
  unsigned long *ktime  = keys_time;
  boolean       *signal = signals;
  for (byte key = 0; key < KEYS_NUMBER; key++)
  {
    for (byte state_index = 0; state_index < 2; state_index++)
    {

      switch (*state)
      {
        case KEY_OFF:

          if (state_index == 0 && *signal)
          {
            *state = KEY_START;
            *ktime = millis();
          }
          break;
        case KEY_START:

          if (state_index == 0 && !*signal)
          {
            *state = KEY_OFF;
            break;
          }

          if (state_index == 1 && *signal)
          {
            *state = KEY_ON;
            send_midi_note_event(144, key, millis() - *ktime);
            //digitalWrite2(TRIG, HIGH);
          }
          break;
        case KEY_ON:
          digitalWrite2(GATE, HIGH); //remove for legato?

          if (fsrReading > 0)
          {
            send_midi_pressure_event(208, fsrReading); // channel pressure via MIDI
          }
          if (state_index == 1 && !*signal)
          {
            //send_midi_pressure_event(208, 0); //ensure 0
            //setVoltage(DAC, 0, 1, 0 << 5); // ensure 0
            *state = KEY_RELEASED;
            *ktime = millis();
          }
          break;
        case KEY_RELEASED:

          if (state_index == 0 && !*signal)
          {
            *state = KEY_OFF;
            send_midi_note_event(128, key, millis() - *ktime);
            digitalWrite2(GATE, LOW);

          }
          break;
      }
      signal++;
    }
    state++;
    ktime++;
  }
}
