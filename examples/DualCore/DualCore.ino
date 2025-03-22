#include <Adafruit_TinyUSB.h>
#include <pico-audio.h>

// These are accessible to both cores, because 
// they're sketch-global symbols:
//
// Audio Processing Nodes
AudioSynthWaveform              wav1; //xy=650,165
AudioSynthWaveform              wav1b; //xy=650,210
AudioSynthWaveform              wav2; //xy=645,350
AudioSynthWaveform              wav2b; //xy=645,395
AudioMixer4                     mixerR; //xy=810,210
AudioMixer4                     mixerL; //xy=815,370
AudioOutputI2S  PCM5102; //xy=961,273 // hand-edited!

// Audio Connections (all connections (aka wires or links))
AudioConnection        patchCord1(wav1, 0, mixerR, 0);
AudioConnection        patchCord2(wav1b, 0, mixerR, 1);
AudioConnection        patchCord3(wav2, 0, mixerL, 0);
AudioConnection        patchCord4(wav2b, 0, mixerL, 1);
AudioConnection        patchCord5(mixerR, 0, PCM5102, 0);
AudioConnection        patchCord6(mixerL, 0, PCM5102, 1);

// Control Nodes (all control nodes (no inputs or outputs))

// TeensyAudioDesign: end automatically generated code


void setup() {
  delay(2000);
  Serial.println("\n\nDual core example running");  
  pinMode(LED_BUILTIN,OUTPUT);  

  wav1.begin(0.9,100.0,WAVEFORM_SINE);
  wav2.begin(0.9, 50.05,WAVEFORM_SINE);
  wav2.phase(90);
  
  wav1b.begin(0.05,1001.1,WAVEFORM_SINE);
  wav2b.begin(0.06,1001.1,WAVEFORM_SINE);
}

bool led;
void printStuff(void)
{
  Serial.printf("Usage: CPU %.2f%%; wav1 %.2f%%; I²S %.2f%%\n", 
                AudioProcessorUsage(),
                wav1.processorUsage(),
                PCM5102.processorUsage()
              ); 
  Serial.flush();
  digitalWrite(LED_BUILTIN,led);
  led = !led;
}


void loop() 
{
  static uint32_t next;

  if (millis() >= next)
  {
    next = millis() + 500;
    printStuff();
  }
}
