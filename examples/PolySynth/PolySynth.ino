#include <Adafruit_TinyUSB.h>
#include <MIDI.h>

// USB MIDI object
Adafruit_USBD_MIDI usb_midi;

// Create a new instance of the Arduino MIDI Library,
// and attach usb_midi as the transport.
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

#define SAMPLE_TYPE int16_t
#define SAMPLE_BITS_DEPTH 16
#define VOICES 16

#include <pico-audio.h>
#include "SynthVoice.h"

SynthVoice voice[VOICES];
AudioMixer4 mixer;
AudioMixer4 mixer2;
AudioMixer4 mixer3;
AudioMixer4 mixer4;
AudioMixer4 mixer5;
AudioOutputI2S output;

// Test test1;
// AudioConnection patch1(sine, 0, envelope, 0);
AudioConnection patch2(*voice[0].getOutput(), 0, mixer, 0);
AudioConnection patch3(*voice[1].getOutput(), 0, mixer, 1);
AudioConnection patch4(*voice[2].getOutput(), 0, mixer, 2);
AudioConnection patch5(*voice[3].getOutput(), 0, mixer, 3);
AudioConnection patch6(*voice[4].getOutput(), 0, mixer2, 0);
AudioConnection patch7(*voice[5].getOutput(), 0, mixer2, 1);
AudioConnection patch8(*voice[6].getOutput(), 0, mixer2, 2);
AudioConnection patch9(*voice[7].getOutput(), 0, mixer2, 3);
AudioConnection patch10(*voice[8].getOutput(), 0, mixer3, 0);
AudioConnection patch11(*voice[9].getOutput(), 0, mixer3, 1);
AudioConnection patch12(*voice[10].getOutput(), 0, mixer3, 2);
AudioConnection patch13(*voice[11].getOutput(), 0, mixer3, 3);
AudioConnection patch14(*voice[12].getOutput(), 0, mixer4, 0);
AudioConnection patch15(*voice[13].getOutput(), 0, mixer4, 1);
AudioConnection patch16(*voice[14].getOutput(), 0, mixer4, 2);
AudioConnection patch17(*voice[15].getOutput(), 0, mixer4, 3);
AudioConnection patch18(mixer, 0, mixer5, 0);
AudioConnection patch19(mixer2, 0, mixer5, 1);
AudioConnection patch20(mixer3, 0, mixer5, 2);
AudioConnection patch21(mixer4, 0, mixer5, 3);
AudioConnection patch22(mixer5, 0, output, 0);
AudioConnection patch23(mixer5, 0, output, 1);

uint8_t currentVoice = 0;

void setup() {
  AudioMemory(80);

  mixer.gain(0, 0.25);
  mixer.gain(1, 0.25);
  mixer.gain(2, 0.25);
  mixer.gain(3, 0.25);
  mixer2.gain(0, 0.25);
  mixer2.gain(1, 0.25);
  mixer2.gain(2, 0.25);
  mixer2.gain(3, 0.25);
  mixer3.gain(0, 0.25);
  mixer3.gain(1, 0.25);
  mixer3.gain(2, 0.25);
  mixer3.gain(3, 0.25);
  mixer4.gain(0, 0.25);
  mixer4.gain(1, 0.25);
  mixer4.gain(2, 0.25);
  mixer4.gain(3, 0.25);
  mixer5.gain(0, 0.25);
  mixer5.gain(1, 0.25);
  mixer5.gain(2, 0.25);
  mixer5.gain(3, 0.25);

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