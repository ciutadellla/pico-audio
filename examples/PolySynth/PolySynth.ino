#include <Adafruit_TinyUSB.h>
#include <MIDI.h>

// USB MIDI object
Adafruit_USBD_MIDI usb_midi;

// Create a new instance of the Arduino MIDI Library,
// and attach usb_midi as the transport.
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

#define SAMPLE_TYPE int16_t
#define SAMPLE_BITS_DEPTH 16
#define VOICES 20

#include <pico-audio.h>
#include "SynthVoice.h"
#include "Mixer.h"

SynthVoice voice[VOICES];
AudioMixer<VOICES> mixer;
AudioOutputI2S output;

uint8_t currentVoice = 0;

void setup() {
  AudioMemory(200);

  for (int i = 0; i < VOICES; i++) {
    new AudioConnection(*voice[i].getOutput(), 0, mixer, i);
  }
  new AudioConnection(mixer, 0, output, 0);
  new AudioConnection(mixer, 0, output, 1);

  mixer.gain((float)1.0 / VOICES / 2);

  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }

  usb_midi.setStringDescriptor("TinyUSB MIDI");

  // Initialize MIDI, and listen to all MIDI channels
  // This will also call usb_midi's begin()
  MIDI.begin(MIDI_CHANNEL_OMNI);

  // If already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  // Attach the handleNoteOn function to the MIDI Library. It will
  // be called whenever the Bluefruit receives MIDI Note On messages.
  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);

  output.begin();
}


void loop() {

#ifdef TINYUSB_NEED_POLLING_TASK
  // Manual call tud_task since it isn't called by Core's background
  TinyUSBDevice.task();
#endif

  // not enumerated()/mounted() yet: nothing to do
  if (!TinyUSBDevice.mounted()) {
    return;
  }
  // read any new MIDI messages
  MIDI.read();
}


void handleNoteOn(byte channel, byte pitch, byte velocity) {
  Serial.println(currentVoice);
  voice[currentVoice].noteOn(pitch);
  currentVoice++;
  if (currentVoice > VOICES - 1) {
    currentVoice = 0;
  }
}


void handleNoteOff(byte channel, byte pitch, byte velocity) {
  for (int i = 0; i < VOICES; i++) {
    if (voice[i].getNote() == pitch) {
      voice[i].noteOff();
    }
  }
}