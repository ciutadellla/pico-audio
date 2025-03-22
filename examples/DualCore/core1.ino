/*
 * Make Core 1 run the audio updates, leaving Core 0
 * free to do other stuff without affecting audio.
 * 
 * NOTE THAT as published this ia actually VERY poorly
 * implemented. The pico-audio library assumes that
 * disabling interrupts will prevent the audio engine
 * from executing updates with partially-modified data,
 * BUT this is NOT the case when the interrupt in question
 * is serviced by the other core.
 * 
 * Thus if Core 0 were to modify an object. e.g. by
 * executing AudioEffectEnvelope::noteOn() due to receiving
 * a MIDI Note On, there is a risk that the envelope will
 * NOT actually trigger.
 */
#include <pico-audio.h>

void setup1(void)
{
  AudioMemory(20); // so we get the Core 1 cycle counter started
  delay(3000);
  PCM5102.begin(); // Core 1 responds to I2S and audio interrupts
}

// Do nothing but run the audio
void loop1() 
{
  __WFI(); // halt until an interrupt occurs
}
