#include <Arduino.h>
#include <InputDebounce.h>
#include <RotaryEncoder.h>
#include <EEPROM.h>
#include <SevenSegmentTM1637.h>

#define DEBUG

/*
 * Constants
 */

// Pin numbers for rotary encoder

const uint8_t ROTARY_ENC_SWITCH = 4;
const uint8_t ROTARY_ENC_A = 3;
const uint8_t ROTARY_ENC_B = 2;

// number of pedal keys

const int NUM_KEYS = 13;

#ifdef ARDUINO_AVR_MEGA2560
const uint8_t KEY_PINS[NUM_KEYS] = { A12, A11, A10, A9, A8, A7, A6, A5, A4, A3, A2, A1, A0 };
#endif
#ifdef ARDUINO_AVR_NANO
const uint8_t KEY_PINS[NUM_KEYS] = { A5, A4, A3, A2, A1, A0, 13, 12, 11, 10, 9, 8, 7 };
#endif

// bounce delay

const int KEY_BOUNCE_DELAY = 20;

// MIDI channel min and max

const uint8_t MIDI_CHANNEL_MIN = 0;
const uint8_t MIDI_CHANNEL_MAX = 15;

// MIDI velocity min and max

const uint8_t MIDI_VELOCITY_MIN = 0;
const uint8_t MIDI_VELOCITY_MAX = 127;

// default, min and max base note
// TODO: probably use the MIDI library here?

const uint8_t MIDI_C1 = 24;
const uint8_t MIDI_NOTE_MIN = 0;
const uint8_t MIDI_NOTE_MAX = 127 - NUM_KEYS; // last MIDI note possible with first key

const uint8_t MIDI_NOTE_ON = 0x90;
const uint8_t MIDI_NOTE_OFF = 0x80;

// rotary encoder button press duration to save settings

const unsigned long ROTARY_ENC_SWITCH_SAVE_DUR = 1000;

// magic number for EEPROM saved settings data

const uint16_t EEPROM_MAGIC = 0xba55;

// 7 segment display pins

const uint8_t SEG7_CLK = 5;
const uint8_t SEG7_DIO = 6;

// Note names for 7 segment display
// A # sign cannot be displayed, so we use something else.

const static char SEG_7_NOTE_NAMES[] = R"(C C"D D"E F F"G G"A A"H )";

// Hexadecimal characters

const static char SEG_7_HEX_DIGITS[] = "0123456789AbCdEF";

/*
 * Static global variables
 */

// Debouncers for pedal keys and rotary encoder button

static InputDebounce keyDebouncers[NUM_KEYS];
static InputDebounce rotaryEncDebouncer;

// Rotary encoder movement tracker

static RotaryEncoder *rotaryEnc = nullptr;

// 7 segment display

static SevenSegmentTM1637 seg7Display(SEG7_CLK, SEG7_DIO);

// current duration of rotary encoder button press

static unsigned long rotaryEncSwitchDur = 0;

// modes for the settings adjustable by the rotary encoder

static enum {
    ROTARY_ENC_BASE_KEY,
    ROTARY_ENC_CHANNEL,
    ROTARY_ENC_VELOCITY,
    ROTARY_ENC_SERIAL_SPEED
} rotaryEncMode = ROTARY_ENC_BASE_KEY;

const uint32_t serialSpeeds[] = { 9600, 19200, 31250, 38400, 57600, 115200 };

const uint8_t SERIAL_SPEED_MIN = 0;
const uint8_t SERIAL_SPEED_MAX = sizeof(serialSpeeds) / sizeof(serialSpeeds[0]) - 1;

// settings

static uint8_t midiChannel = 0;
static uint8_t baseKey = MIDI_C1;
static uint8_t midiVelocity = 127;
static uint8_t serialSpeed = SERIAL_SPEED_MAX;

/***
 * Calculate MIDI note number from key pin.
 */

uint8_t keyPinToNote(uint8_t keyPin) {
    uint8_t key;

    for(key = 0; key < NUM_KEYS; key++) {
        if(KEY_PINS[key] == keyPin) {
            break;
        }
    }

    return key + baseKey;
}

/***
 * Write MIDI "note on" event to serial port
 */

void keyPressCallback(uint8_t keyPin) {
    uint8_t midiNote = keyPinToNote(keyPin);

#ifdef DEBUG
    Serial.print("MIDI ");
    Serial.print((uint8_t) MIDI_NOTE_ON | midiChannel);
    Serial.print(' ');
    Serial.print((uint8_t) midiNote);
    Serial.print(' ');
    Serial.print((uint8_t) midiVelocity);
    Serial.println();
#else
    Serial.write((uint8_t) MIDI_NOTE_ON | midiChannel);
    Serial.write((uint8_t) midiNote);
    Serial.write((uint8_t) midiVelocity);
#endif
}

/***
 * Write MIDI "note off" event to serial port
 */

void keyReleaseCallback(uint8_t keyPin) {
    uint8_t midiNote = keyPinToNote(keyPin);

#ifdef DEBUG
    Serial.print("MIDI ");
    Serial.print((uint8_t) MIDI_NOTE_OFF | midiChannel);
    Serial.print(' ');
    Serial.print((uint8_t) midiNote);
    Serial.print(' ');
    Serial.print((uint8_t) midiVelocity);
    Serial.println();
#else
    Serial.write((uint8_t) MIDI_NOTE_OFF | midiChannel);
    Serial.write((uint8_t) midiNote);
    Serial.write((uint8_t) midiVelocity);
#endif
}

/***
 * Write current parameter settings to the 7 segment 4 character display.
 *
 * Base Key: Note name, ' ', octave 0-8
 * MIDI Channel: 'Ch ', MIDI channel in hex 0-f
 * Velocity: 'U ', Velocity in 2 digit hex 00-7f
 * Serial speed: Speed divided by 100 or "MIDI" for 31250
 */

void seg7DisplayParameters() {
    char seg7Str[5] = "    ";

    switch(rotaryEncMode) {
        case ROTARY_ENC_BASE_KEY:
            {
                seg7Str[0] = SEG_7_NOTE_NAMES[(baseKey % 12) * 2];
                seg7Str[1] = SEG_7_NOTE_NAMES[(baseKey % 12) * 2 + 1];

                uint8_t octave = (baseKey / 12) - 1 + '0';

                if(seg7Str[1] == ' ') {
                    seg7Str[1] = octave;
                } else {
                    seg7Str[2] = octave;
                }
            }
            break;

        case ROTARY_ENC_CHANNEL:
            seg7Str[0] = 'C';
            seg7Str[1] = 'h';
            seg7Str[3] = SEG_7_HEX_DIGITS[midiChannel];
            break;

        case ROTARY_ENC_VELOCITY:
            seg7Str[0] = 'U';
            seg7Str[2] = SEG_7_HEX_DIGITS[midiVelocity >> 4];
            seg7Str[3] = SEG_7_HEX_DIGITS[midiVelocity & 0xf];
            break;

        case ROTARY_ENC_SERIAL_SPEED:
             {
                uint32_t speed = serialSpeeds[serialSpeed] / 100;

                if(speed != 312) {
                    for(int i = 0; i < 4 && speed != 0; i++) {
                        seg7Str[3 - i] = speed % 10 + '0';
                        speed /= 10;
                    }
                } else {
                    strcpy(seg7Str, "MIDI");
                }
            }
            break;
    }

    seg7Display.print(seg7Str);

#ifdef DEBUG
    Serial.println(seg7Str);
#endif
}

/***
 * Load base key, MIDI channel and velocity settings from EEPROM.
 *
 * EEPROM_MAGIC ist expected at location 0, otherwise settings are not loaded.
 */

void eepromLoad() {
    uint16_t magic;

    EEPROM.get(0, magic);

#ifdef DEBUG
    Serial.print("EEPROM magic ");
    Serial.print(magic);
    Serial.println();
#endif

    if(magic == EEPROM_MAGIC) {
        EEPROM.get(sizeof(magic), baseKey);
        EEPROM.get(sizeof(magic) + 1, midiChannel);
        EEPROM.get(sizeof(magic) + 2, midiVelocity);
        EEPROM.get(sizeof(magic) + 3, serialSpeed);
    }
}

/***
 * Save base key, MIDI channel and velocity settings from EEPROM.
 *
 * EEPROM_MAGIC ist written at location 0, followed by settings.
 * "SAVE" ist written to the 7 segment display and the flashed a few times.
 */

void eepromSave() {
    seg7Display.print("SAVE");

    EEPROM.put(0, EEPROM_MAGIC);
    EEPROM.put(sizeof(EEPROM_MAGIC), baseKey);
    EEPROM.put(sizeof(EEPROM_MAGIC) + 1, midiChannel);
    EEPROM.put(sizeof(EEPROM_MAGIC) + 2, midiVelocity);
    EEPROM.put(sizeof(EEPROM_MAGIC) + 3, serialSpeed);

    seg7Display.blink();
    seg7DisplayParameters();
}

/***
 * Set position for rotary encoder handling to current settings.
 */

void rotaryEncSetPosition() {
    switch(rotaryEncMode) {
        case ROTARY_ENC_BASE_KEY:
            rotaryEnc->setPosition(baseKey);
            break;

        case ROTARY_ENC_CHANNEL:
            rotaryEnc->setPosition(midiChannel);
            break;

        case ROTARY_ENC_VELOCITY:
            rotaryEnc->setPosition(midiVelocity);
            break;

        case ROTARY_ENC_SERIAL_SPEED:
            rotaryEnc->setPosition(serialSpeed);
            break;
    }
}

/***
 * Handle release of the rotary encoder switch button.
 * If pressed for a short time, cycle through settings.
 * If pressed for ROTARY_ENC_SWITCH_SAVE_DUR msecs, save settings to EEPROM.
 *
 * @param pin ignored, since only one rotary encoder with one button is used
 */

void rotaryEncReleaseCallback(uint8_t pin) {
    if(rotaryEncSwitchDur < ROTARY_ENC_SWITCH_SAVE_DUR) {
        // Duration of press shorter than ROTARY_ENC_SWITCH_SAVE_DUR: switch base key, channel and velocity modes

        switch(rotaryEncMode) {
            case ROTARY_ENC_BASE_KEY:
                rotaryEncMode = ROTARY_ENC_CHANNEL;
                break;

            case ROTARY_ENC_CHANNEL:
                rotaryEncMode = ROTARY_ENC_VELOCITY;
                break;

            case ROTARY_ENC_VELOCITY:
                rotaryEncMode = ROTARY_ENC_SERIAL_SPEED;
                break;

            case ROTARY_ENC_SERIAL_SPEED:
                rotaryEncMode = ROTARY_ENC_BASE_KEY;
                break;
        }

        rotaryEncSetPosition();
        seg7DisplayParameters();
    } else {
        // Duration of press longer than ROTARY_ENC_SWITCH_SAVE_DUR: save base key and channel mode to EEPROM

        eepromSave();
    }
}

/***
 * Update duration of rotary encoder button press. Will be evaluated in rotaryEncReleaseCallback().
 *
 * @param pin ignored
 * @param duration current duration
 */

void rotaryEncPressDurationCallback(uint8_t pin, unsigned long duration) {
    rotaryEncSwitchDur = duration;
}

/***
 * Interrupt routine for rotary encoder.
 * Just update the current position.
 */

void rotaryEncCheckPosition()
{
    rotaryEnc->tick();
}

/***
 * Process rotary encoder position while loop().
 *
 * Update settings according to the currently selected mode.
 * Check for limits.
 * Redisplay if settings have been changed.
 */

void rotaryEncProcess() {
    int pos = rotaryEnc->getPosition();
    int newPos = pos;
    bool setNewPos = false;

    switch(rotaryEncMode) {
        case ROTARY_ENC_BASE_KEY:
            if(pos < MIDI_NOTE_MIN) {
                newPos = MIDI_NOTE_MIN;
            } else if(pos > MIDI_NOTE_MAX) {
                newPos = MIDI_NOTE_MAX;
            }

            if(newPos != baseKey) {
                baseKey = pos;
                setNewPos = true;
            }
            break;

        case ROTARY_ENC_CHANNEL:
            if(pos < MIDI_CHANNEL_MIN) {
                newPos = MIDI_CHANNEL_MIN;
            } else if(pos > MIDI_CHANNEL_MAX) {
                newPos = MIDI_CHANNEL_MAX;
            }

            if(newPos != midiChannel) {
                midiChannel = pos;
                setNewPos = true;
            }
            break;

        case ROTARY_ENC_VELOCITY:
            if(pos < MIDI_VELOCITY_MIN) {
                newPos = MIDI_VELOCITY_MIN;
            } else if(pos > MIDI_VELOCITY_MAX) {
                newPos = MIDI_VELOCITY_MAX;
            }

            if(newPos != midiVelocity) {
                midiVelocity = pos;
                setNewPos = true;
            }
            break;

        case ROTARY_ENC_SERIAL_SPEED:
            if(pos < SERIAL_SPEED_MIN) {
                newPos = SERIAL_SPEED_MIN;
            } else if(pos > SERIAL_SPEED_MAX) {
                newPos = SERIAL_SPEED_MAX;
            }

            if(newPos != serialSpeed) {
                serialSpeed = pos;
                setNewPos = true;

                Serial.flush();
                Serial.begin(serialSpeeds[serialSpeed]);

                while(Serial.available()) {
                    Serial.read();
                }
            }

            break;
    }

    if(newPos != pos) {
        setNewPos = true;
    }

    if(setNewPos) {
        rotaryEnc->setPosition(newPos);
        seg7DisplayParameters();
    }
}

/***
 * Setup
 */

void setup() {
    eepromLoad();

    Serial.begin(serialSpeeds[serialSpeed]);

    Serial.println("oiosa");

    // Setup pins for pedal keys. Use internal pull up resistors and debouncing.

    for(int i = 0; i < NUM_KEYS; ++i) {
        int keyPin = KEY_PINS[i];

        pinMode(keyPin, INPUT_PULLUP);

        keyDebouncers[i].registerCallbacks(keyPressCallback, keyReleaseCallback);
        keyDebouncers[i].setup(keyPin, KEY_BOUNCE_DELAY, InputDebounce::PIM_INT_PULL_UP_RES);
    }

    // Setup rotary encoder button. Use internal pull up resistor and debouncing.

    pinMode(ROTARY_ENC_SWITCH, INPUT_PULLUP);

    rotaryEncDebouncer.registerCallbacks(NULL, rotaryEncReleaseCallback, rotaryEncPressDurationCallback);
    rotaryEncDebouncer.setup(ROTARY_ENC_SWITCH, KEY_BOUNCE_DELAY, InputDebounce::PIM_INT_PULL_UP_RES);

    // Setup rotary encoder. Use interrupts to avoid polling. (We might have used polling, since the other stuff is polled.)
    // Use LatchMode::FOUR3 with KY-040 to avoid double stepping.

    rotaryEnc = new RotaryEncoder(ROTARY_ENC_A , ROTARY_ENC_B, RotaryEncoder::LatchMode::FOUR3);

    rotaryEncSetPosition();

    attachInterrupt(digitalPinToInterrupt(ROTARY_ENC_A), rotaryEncCheckPosition, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_ENC_B), rotaryEncCheckPosition, CHANGE);

    // Setup 7 segment display

    seg7Display.begin();
    seg7Display.setBacklight(100);

    seg7DisplayParameters();
}

/***
 * Loop
 */

void loop() {
    unsigned long now = millis();

    // Handle bass pedal keys.

    for(int i = 0; i < NUM_KEYS; ++i) {
        keyDebouncers[i].process(now);
    }

    // Handle rotary encoder button.

    rotaryEncDebouncer.process(now);

    // Handle rotary encoder movement.

    rotaryEncProcess();
}