#ifndef AudioOutputI2S_h
#define AudioOutputI2S_h

#include <I2S.h>

#include "AudioStream.h"

class AudioOutputI2S : public AudioStream {
protected:
  static I2S i2s;
  audio_block_t *inputQueueArray[2];

public:
  AudioOutputI2S();
  void begin(uint pBCLK, uint pWS, uint pDOUT);
  void update();
};

I2S AudioOutputI2S::i2s(OUTPUT);

inline AudioOutputI2S::AudioOutputI2S()
  : AudioStream(2, inputQueueArray) {
}

inline void AudioOutputI2S::begin(uint pBCLK = 20, uint pWS = 21, uint pDOUT = 22) {

  // ********** I2S **********
  i2s.setBCLK(pBCLK);
  i2s.setMCLK(pWS);
  i2s.setDATA(pDOUT);
  i2s.setBitsPerSample(SAMPLE_BITS_DEPTH);
  i2s.setFrequency(AUDIO_SAMPLE_RATE);
  i2s.setBuffers(6, AUDIO_BLOCK_SAMPLES * 8 * sizeof(int16_t) / sizeof(uint32_t));

  // start I2S at the sample rate with 16-bits per sample
  if (!i2s.begin()) {
    Serial.println("Failed to initialize I2S!");
    while (1)
      ;  // do nothing
  }
}

inline void AudioOutputI2S::update() {
  audio_block_t *inputLeftBlock = receiveReadOnly(0);
  audio_block_t *inputRightBlock = receiveReadOnly(1);


  for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
    // When sending 0, some DAC will power off causing a hearable jump from silence to floor noise and cracks,
    // sending 1 is a workaround
    // Tested only with the PCM5100A
    SAMPLE_TYPE sampleL = inputLeftBlock == NULL || inputLeftBlock->data[i] == 0 ? 1 : inputLeftBlock->data[i] * 0.1;
    SAMPLE_TYPE sampleR = inputRightBlock == NULL || inputRightBlock->data[i] == 0 ? 1 : inputRightBlock->data[i] * 0.1;
    if (SAMPLE_BITS_DEPTH == 32) {
      i2s.write32(sampleL, sampleR);
    } else if (SAMPLE_BITS_DEPTH == 16) {
      i2s.write16(sampleL, sampleR);
    }
  }
  
  if (inputLeftBlock) {
    release(inputLeftBlock);
  }
  if (inputRightBlock) {
    release(inputRightBlock);
  }
}

#endif