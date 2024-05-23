/*
** Title: uC Code
** Author: Richard Cerrato
** Date: 3/28/2024
**
** Descirption: Interfaces AD7684 differential ADC by Analog Devices with the ESP32 Dev Board.
** The communication protocol used in this block is SPI (Serial Peripheral Interface). This 
** ADC only uses three of the four SPI pins (MISO, SCK and SDA). It does not use MOSI. The 
** primary use of this program is to be able to sample live audio data from a XLR port
** connected to the ADC.
**
*/

#include <Wire.h>
#include <SPI.h> // Arduino IDE SPI library
#include <stdint.h> // Standard C library for int16_t and int8_t
#include <cmath>
#include "arduinoFFT162.h" // Version 1.62 of arduinoFFT by Enrique Condes

// ------------------------------- SPI Values ------------------------------- //

// Define ALTERNATE_PINS to use non-standard GPIO pins for SPI bus
#define VSPI_MISO   19 // Master in (input) Slave out (output)
#define VSPI_MOSI   24 // Changed from 23 since we dont use MOSI in this program and we need GPIO 23 for one of the LEDs
#define VSPI_SCLK   18 // Supplied Clock from uC to ADC
#define VSPI_SS     5 // chip select

#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
#define VSPI
#endif

static const int sampleRate = 44100; // 44,100 Hz or 44.1 kHz (optimal audio sampling rate).

static const int spiClk = sampleRate * 24; // 1,058,400 Hz or 1.0584 MHz
// ADC needs a minimum of 22 clock cycles to load data.
// So, SCK (also known as DCLOCK) must have a frequency at least 22 times larger than the sample rate.
// Scaling by 24 instead of 22 to be safe.

static const float samplePeriod = 22.676; // T = 1/freq = 1/44100 = 22.67573 micro seconds (10^-6), this 
// will be used to set the delay time between flipping CS low and back to high (allowing 24 cycles of 
// DCLOCK to complete and for all 16-bits from the ADC to load).

SPIClass * vspi = NULL; //uninitalised pointer to SPI object

// ------------------------------- LED Values ------------------------------- //

#define REDn        21 // pins for two LEDs on PCB
#define GREENn      22
#define BLUEn       23

#define GREENp      0
#define BLUEp       4
#define REDp        2

const int ADC_max_threshold_90 = 0.9 * 32767; // 90% of 32767
const int ADC_max_threshold_80 = 0.8 * 32767; // 80% of 32767
const int ADC_max_threshold_70 = 0.7 * 32767; // 70% of 32767
const int ADC_max_threshold_60 = 0.6 * 32767; // 60% of 32767
const int ADC_max_threshold_25 = 0.25 * 32767; // 25% of 32767
const int ADC_max_threshold_15 = 0.15 * 32767; // 10% of 32767

const int ADC_min_threshold_90 = -0.9 * 32767; // 90% of -32767
const int ADC_min_threshold_80 = -0.8 * 32767; // 80% of -32767
const int ADC_min_threshold_70 = -0.7 * 32767; // 70% of -32767
const int ADC_min_threshold_60 = -0.6 * 32767; // 60% of -32767
const int ADC_min_threshold_25 = -0.25 * 32767; // 25% of -32767
const int ADC_min_threshold_15 = -0.05 * 32767; // 10% of -32767

// ------------------------------- FFT Values ------------------------------- //

#define samples 1024 // 1024 Number of samples needed to fill one window for the FFT operation. Must be a power of 2.

double vReal[samples]; // Will store real part of frequency spectrum of window after FFT is calculated
double vImag[samples]; // Will store imaginary part of frequency spectrum of window after FFT is calculated
double ADC_amplitudes[samples]; // Array of samples of size equal to the sample window. The ADC will output 16 bit 
// two's compliment numbers (so it can include positive and negative amplitude values). 

const int NUM_BINS = 10; // Number of frequency bins (will split the frequency spectrum into 10 evenly sized bins, effectively filtering it)
double frequencyBins[NUM_BINS];

// ------------------------------- Initialize Function ------------------------------- //

void setup() {

  pinMode(REDp,OUTPUT); 
  pinMode(GREENp,OUTPUT);
  pinMode(BLUEp,OUTPUT);
  
  digitalWrite(REDp,HIGH);
  digitalWrite(GREENp,LOW);
  digitalWrite(BLUEp,HIGH);

  pinMode(REDn,OUTPUT);
  pinMode(GREENn,OUTPUT);
  pinMode(BLUEn,OUTPUT);

  digitalWrite(REDn,HIGH);
  digitalWrite(GREENn,LOW);
  digitalWrite(BLUEn,HIGH);

  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE2);

  vspi = new SPIClass(VSPI); //initialise instance of the SPIClass attached to VSPI

  vspi->begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS); //SCLK, MISO, MOSI, SS

  pinMode(vspi->pinSS(), OUTPUT); //VSPI CS/SS
  // Sets up slave select pins as outputs as the Arduino API
  // Doesn't handle automatically pulling SS low

  Serial.begin(115200); // Sets baud rate to 115200 bauds/sec (per ESP32)

  digitalWrite(vspi->pinSS(), HIGH); //Start with CS high for initial state.

  memset(vReal, 0, samples);
  memset(vImag, 0, samples);

}

// ------------------------------- Loop Function ------------------------------- //

void loop() {

  int count_max_90 = 0;
  int count_max_80 = 0;
  int count_max_70 = 0;
  int count_max_60 = 0;
  int count_max_25 = 0;
  int count_max_15 = 0;

  int count_min_90 = 0;
  int count_min_80 = 0;
  int count_min_70 = 0;
  int count_min_60 = 0;
  int count_min_25 = 0;
  int count_min_15 = 0;

  // ------------------------------- Get, Convert, & Store Sample ------------------------------- //

  for(int i = 0; i < samples; i++){

      vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3)); // Starts new SPI cycle

      digitalWrite(vspi->pinSS(), LOW); //pull CS low to prep other end for transfer.

      uint16_t byteOne = vspi->transfer(0); // LSB (least significant byte, bits 0 - 7) read data in on MISO pin while sending out 0 on MOSI.
      uint16_t byteTwo = vspi->transfer(0); // MSB (most significant byte, bits 8 - 15) read data in on MISO pin while sending out 0 on MOSI.
      uint16_t byteThree = vspi->transfer(0); // MSB (most significant byte, bits 8 - 15) read data in on MISO pin while sending out 0 on MOSI.

      int16_t combined_sample = ((uint16_t)byteOne << 15) | (((uint16_t)byteTwo << 7) & (0b01111111111111111)) | (((uint16_t)byteThree >> 1));
    
    
      vReal[i] = (double)combined_sample; //store in sample window for FFT //was call ADC_amplitudes
      vImag[i] = 0;
  

      delayMicroseconds(samplePeriod);

      digitalWrite(vspi->pinSS(), HIGH); //pull CS high to signify end of data transfer.

      vspi->endTransaction(); // Finishes one sample. (O ne SPI cycle)

      if (vReal[i] >= ADC_max_threshold_80){
          count_max_80++;
      }
      if (vReal[i] <= ADC_min_threshold_80){
          count_min_80++;
      }
      if (vReal[i] >= ADC_max_threshold_60){
          count_max_60++;
      }
      if (vReal[i] <= ADC_min_threshold_60){
          count_min_60++;
      }
      if (vReal[i] <= ADC_max_threshold_25 && vReal[i] >= 0){
          count_max_25++;
      }
      if (vReal[i] >= ADC_min_threshold_25 && vReal[i] < 0){
          count_min_25++;
      }
      if (vReal[i] <= ADC_max_threshold_15 && vReal[i] >= 0){
          count_max_15++;
      }
      if (vReal[i] >= ADC_min_threshold_15 && vReal[i] < 0){
          count_min_15++;
      }

  }

  // ------------------------------- Perform FFT ------------------------------- //
  
  arduinoFFT FFT = arduinoFFT(vReal, vImag, samples, sampleRate);

  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, samples); // Uses the Real and Imaginary part of the FFT to calculate the maginute vs. frequency values. Stored in vReal.
  //Serial.println(vReal[0]); // For testing FFT
  //double peak = (FFT.MajorPeak(vReal, samples, sampleRate) / 3); //For testing FFT (calculates largest frequency present in the window), needs to be divided by three because for some reason it is triple what its supposed to be
  
  double log_count = 2; // initialize as 2^1
  double window_count = 0;
  double maxMagnitude = 0.0;
  double bin_mag_avg = 0.0;

  for (int i = 0; i < samples; i++) { // Sorting the entire frequency spectrum into 10 evenly sized/spaced bins. [ [] [] [] [] [] [] [] [] [] [] ]
    
    maxMagnitude += vReal[i];
    window_count++;

    if (i >= (log_count - 1)){
        
        bin_mag_avg = maxMagnitude / window_count;

        int bindex = log2(log_count) - 1 ;
        frequencyBins[bindex] = bin_mag_avg;  // Store the final 10 (or NUM_BINS) maximum magnitude values in this array.
        
        log_count *= 2; // 2^(old value + 1);
        
        maxMagnitude = 0;
        window_count = 0;
    }
   
  }
  
  // ------------------------------- Change LEDs ------------------------------- //
  if(count_max_15 > 0.6 * samples || count_max_25 > 0.6 * samples || count_min_15 > 0.6 * samples || count_min_25 > 0.6 * samples || count_max_60 > 0.025 * samples || count_min_60 > 0.025 * samples || count_max_80 > 0.025 * samples || count_min_80 > 0.025 * samples) {
      if (count_max_15 > 0.6 * samples) {
        // Turn positive LED red
        digitalWrite(REDp, LOW);
        digitalWrite(GREENp, HIGH);
        digitalWrite(BLUEp, HIGH);
      } 
      else if (count_max_25 > 0.6 * samples) {
        // Turn positive LED red
        digitalWrite(REDp, HIGH);
        digitalWrite(GREENp, LOW);
        digitalWrite(BLUEp, HIGH);
      } 

      if (count_min_15 > 0.6 * samples) {
        // Turn negative LED red
        digitalWrite(REDn, LOW);
        digitalWrite(GREENn, HIGH);
        digitalWrite(BLUEn, HIGH);
      } 
      else if (count_min_25 > 0.6 * samples) {
        // Turn negative LED red
        digitalWrite(REDn, HIGH);
        digitalWrite(GREENn, LOW);
        digitalWrite(BLUEn, HIGH);
      } 

      if (count_max_80 > 0.025 * samples) {
        // Turn positive LED red
        digitalWrite(REDp, HIGH);
        digitalWrite(GREENp, LOW);
        digitalWrite(BLUEp, LOW);
      } 
      else if (count_max_60 > 0.025 * samples) {
        // Turn positive LED red
        digitalWrite(REDp, HIGH);
        digitalWrite(GREENp, HIGH);
        digitalWrite(BLUEp, LOW);
      } 

      if (count_min_80 > 0.025 * samples) {
        // Turn negative LED red
        digitalWrite(REDn, HIGH);
        digitalWrite(GREENn, LOW);
        digitalWrite(BLUEn, LOW);
      } 
      else if (count_min_60 > 0.025 * samples) {
        // Turn negative LED red
        digitalWrite(REDn, HIGH);
        digitalWrite(GREENn, HIGH);
        digitalWrite(BLUEn, LOW);
      } 
  }
  else {
    digitalWrite(REDp, LOW);
    digitalWrite(GREENp, HIGH);
    digitalWrite(BLUEp, LOW);
    digitalWrite(REDn, LOW);
    digitalWrite(GREENn, HIGH);
    digitalWrite(BLUEn, LOW);
  }


  // ------------------------------- Output Audio Spectrum ------------------------------- //



// <------------------------------------- beginning of actual output (audio spectrum bin array)
  for(int i = 0; i < NUM_BINS; i++) {
    Serial.print(frequencyBins[i]);
    
    if(i < (NUM_BINS - 1)) {
      Serial.print(',');
    }
  }

  Serial.println();    
  // <------------------------------------- end of actual output (audio spectrum bin array)

   
  
  /*
  for(int i = 0; i < samples; i++) {  // Uncomment this to plot actual signal in time (audio signal)
    Serial.println(vReal[i]);         // IMPORTANT: if you are going to plot this, you HAVE TO *comment out* the FFT part of the loop(), because it changes the vReal[] array to frequencies instead of samples
  }                                   // So, temporarily comment out lines 157 - 190 and 213 - 223.
  */

  //Serial.println(combined_sample);  // For testing printing out raw sample (no FFT). To test: uncomment this and comment out other serial prints above.
  //Serial.println(peak); // For testing FFT, shows loudest frequency in signal at current time

  //Serial.println(vReal[0]);

}
