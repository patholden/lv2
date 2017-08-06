#ifndef _SINUSOID_H
#define _SINUSOID_H

#define SAMPLERATE 22000 // Set samples per second
#define FREQUENCY 30	 // Set Frequency of wave

static inline short convert_level_to_signed_16_bit(short sample)
{
  float amplitude = 1.0;    // 0 to 1.0, one being the highest amplitude, set to max DAC amplitude (-10 to +10v).
  double angle;
  float  voltage;
  short  r;

  angle = (2*3.14159) * sample / (SAMPLERATE / FREQUENCY);
  voltage = sin(angle) * amplitude;

  // clamp values:
  if (voltage > 1.0)
    voltage = 1.0;
  else if (voltage < -1.0)
    voltage = -1.0;

    voltage += 1.0;      // remap from [-1.0,1.0] to [0.0,2.0]
    voltage /= 2;        // remap from [0.0,2.0] to [0.0,1.0]
    r = voltage * 65535;  // remap from [0,65535] to [-32768,32767]
    return (r);  // remap from [0,65535] to [-32768,32767]

}
#endif  /*  _SINUSOID_H  */
