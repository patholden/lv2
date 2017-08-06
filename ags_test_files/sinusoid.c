// Function to generate a 30hz sinusoid @ 22K samples per second for 1 second
// Loop on this to provide a continuous wave output

void makesinewave(void)
{
    #define SAMPLERATE 22000 // Set samples per second
    #define FREQUNECY 30	 // Set Frequency of wave
    int sample;  		 // Sample position in wave
    int dacvolts = 0;		 // DAC voltage
    float amplitude = 1.0;  	 // 0 to 1.0, one being the highest amplitude, set to max DAC amplitude (-10 to +10v).

    for ( sample = 0; samples < SAMPLES; samples ++)
    {
        // Map sample position to a wave that cycles at 30 Hz. Each cycle on a sine wave's domain goes from 0
        // to 2 * PI, and we're going to do that 30 times in SAMPLES samples or once in 22000/30 samples:
        float angle = (2*3.14159) * pos / (SAMPLERATE / FREQUENCY);
        float voltage = sin(angle) * amplitude;

        // convert from [-1.0,1.0] level to signed int [-32768,32767]:
        dacvolts = convert_level_to_signed_16_bit(voltage);
    
        //Write dacval to DAC
        function to write to hi, lo and strobe(dacvolts);
    }
}

int convert_level_to_signed_16_bit(float voltage)
{
    long r;

    // clamp values:
    if (v > 1.0) { v = 1.0; }
    else if (v < -1.0) { v = -1.0; }

    v += 1.0;      // remap from [-1.0,1.0] to [0.0,2.0]
    v /= 2;        // remap from [0.0,2.0] to [0.0,1.0]
    r = v * 32767; // remap from [0.0,1.0] to [0,32767]

    return r - 16384;  // remap from [0,32767] to [-16384,16383]
}
