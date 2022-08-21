/*
Guinguin MME Keyscanner and MIDI Controller
Alessandro Guttenberg <guttenba@gmail.com>, May 2021
*/

#include <DIO2.h>
#include <SPI.h>
#include <MIDI.h>

MIDI_CREATE_DEFAULT_INSTANCE();

#define chan 1

#define KEYS_NUMBER 49

#define KEY_OFF               0
#define KEY_START             1
#define KEY_ON                2
#define KEY_RELEASED          3

#define MIN_TIME_MS   3
#define MAX_TIME_MS   350
#define MAX_TIME_MS_N (MAX_TIME_MS - MIN_TIME_MS)


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

//pitch pot range: 377 to 653
//mod pot range:   369 to 643

int lastAnalogBendValue;
int lastAnalogModValue;

void setup() {

  MIDI.begin (MIDI_CHANNEL_OMNI); 

  #ifdef DEBUG_MIDI_MESSAGE
    Serial.begin(115200);
  #else
    Serial.begin(31250);
  #endif

  //pinMode(13, OUTPUT);
  //digitalWrite2(13, LOW);
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
  Serial.println(out);
#else
  if(command = 144){
    MIDI.sendNoteOn(key,vel,chan);
  }
  else{
    MIDI.sendNoteOff(key,vel,chan);
  }
#endif
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

  int analogModValue = map(analogRead(A1),369, 643, 0, 127); //invert mapping for opposite direction
  if (abs(analogModValue-lastAnalogModValue) > 1)  { // have we moved enough to avoid analog jitter?
      lastAnalogModValue = analogModValue;
      #ifdef DEBUG_MIDI_MESSAGE
        Serial.println(analogModValue);
      #else
        MIDI.sendControlChange(1, analogModValue, 1);
      #endif
  }
  
  int analogBendValue = analogRead(A3) - 510; // 0 at mid point
  if (abs(analogBendValue-lastAnalogBendValue) > 1)  { // have we moved enough to avoid analog jitter?
    if (abs(analogBendValue) > 4) { // are we out of the central dead zone?
      lastAnalogBendValue = analogBendValue;
      #ifdef DEBUG_MIDI_MESSAGE
        Serial.println(8*analogBendValue);
      #else
        MIDI.sendPitchBend(8*analogBendValue, 1); // or -8 depending which way you want to go up and down 
      #endif    
    }
   }

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
          }
          break;
        case KEY_ON:
          //digitalWrite2(GATE, HIGH); 
          if (state_index == 1 && !*signal)
          {
            *state = KEY_RELEASED;
            *ktime = millis();
          }
          break;
        case KEY_RELEASED:
          if (state_index == 0 && !*signal)
          {
            *state = KEY_OFF;
            send_midi_note_event(128, key, millis() - *ktime);

          }
          break;
      }
      signal++;
    }
    state++;
    ktime++;
  }
}
