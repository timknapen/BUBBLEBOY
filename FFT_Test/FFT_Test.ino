// Audio FFT (pure)
// outputs FFT to serial

#define ARM_MATH_CM4
#include <arm_math.h>

////////////////////////////////////////////////////////////////////////////////
// CONIFIGURATION
// These values can be changed to alter the behavior of the spectrum display.
////////////////////////////////////////////////////////////////////////////////

int SAMPLE_RATE_HZ = 9000; // Sample rate of the audio in hertz.

const int FFT_SIZE = 256; // Size of the FFT.  Realistically can only be at most 256
// without running out of memory for buffers and other state.
const int AUDIO_INPUT_PIN = A0;        // Input ADC pin for audio data.
const int ANALOG_READ_RESOLUTION = 10; // Bits of resolution for the ADC.
const int ANALOG_READ_AVERAGING = 16;  // Number of samples to average with each ADC reading.

////////////////////////////////////////////////////////////////////////////////
// INTERNAL STATE
// These shouldn't be modified unless you know what you're doing.
////////////////////////////////////////////////////////////////////////////////

IntervalTimer samplingTimer;
float samples[FFT_SIZE * 2];
float magnitudes[FFT_SIZE];
int sampleCounter = 0;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  // Set up serial port.
  while (!Serial)
  { // wait for the serial port, because this thing only works when there's a serial port..
    for (int i = 0; i < 3; i++)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(250);
      digitalWrite(LED_BUILTIN, LOW);
      delay(250);
    }
    delay(500);
  }
  
  Serial.begin(115200);
  Serial.print("Sending FFT data at ");
  Serial.print(SAMPLE_RATE_HZ);
  Serial.println("Hz sample rate");
  delay(1000);

  // Set up ADC and audio input.
  pinMode(AUDIO_INPUT_PIN, INPUT);
  analogReadResolution(ANALOG_READ_RESOLUTION);
  analogReadAveraging(ANALOG_READ_AVERAGING);

  // Begin sampling audio
  samplingBegin();
}

void loop()
{
  // Calculate FFT if a full sample is available.
  if (samplingIsDone())
  {
    // Run FFT on sample data.
    arm_cfft_radix4_instance_f32 fft_inst;
    arm_cfft_radix4_init_f32(&fft_inst, FFT_SIZE, 0, 1);
    arm_cfft_radix4_f32(&fft_inst, samples);
    // Calculate magnitude of complex numbers output by the FFT.
    arm_cmplx_mag_f32(samples, magnitudes, FFT_SIZE);

    outPutData();

    // Restart audio sampling.
    samplingBegin();
  }
}

// Convert intensity to decibels
float intensityDb(float intensity)
{
  return 20 * log10(intensity);
}

void outPutData()
{

  //*

  // run through the magnitudes and send them out over Serial
  for (int i = 1; i < FFT_SIZE / 2; ++i)
  {
    Serial.print(magnitudes[i]);
    Serial.print(" ");
  }
  Serial.println();
  //*/

  // Print out DB scaled values
  for (int i = 1; i < FFT_SIZE / 2; ++i)
  {
    Serial.print(magnitudes[i]);
    Serial.print(" ");
  }
  Serial.println();

  /*
	// FRESH NEW NICOLAS VERSION
	float maxAmplitude = 0;
	int strongestBin = -1;
	for (int i = 1; i < FFT_SIZE / 2; ++i) {
		float db = intensityDb(magnitudes[i]);
		if( db > maxAmplitude){
			strongestBin = i;
			maxAmplitude = intensityDb(magnitudes[i]);
		}
	}
	if(strongestBin >= 0){
		float highestFreq =  (float)strongestBin / (float)(FFT_SIZE) * (float)(SAMPLE_RATE_HZ) ;
		Serial.print(highestFreq);
		Serial.print(" ");
		Serial.print(maxAmplitude);
		Serial.println("");
	}else{
		Serial.println("0 0");
	}
	*/
}

////////////////////////////////////////////////////////////////////////////////
// SAMPLING FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

void samplingCallback()
{
  // Read from the ADC and store the sample data
  samples[sampleCounter] = (float32_t)analogRead(AUDIO_INPUT_PIN);
  // Complex FFT functions require a coefficient for the imaginary part of the input.
  // Since we only have real data, set this coefficient to zero.
  samples[sampleCounter + 1] = 0.0;
  // Update sample buffer position and stop after the buffer is filled
  sampleCounter += 2;
  if (sampleCounter >= FFT_SIZE * 2)
  {
    samplingTimer.end();
  }
}

void samplingBegin()
{
  // Reset sample buffer position and start callback at necessary rate.
  sampleCounter = 0;
  samplingTimer.begin(samplingCallback, 1000000 / SAMPLE_RATE_HZ);
}

boolean samplingIsDone()
{
  return sampleCounter >= FFT_SIZE * 2;
}
