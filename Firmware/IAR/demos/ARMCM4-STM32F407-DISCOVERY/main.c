/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#include "ff.h"

#include "shell.h"


#include "file_utils.h"
#include <time.h>



//#define INDICATE_IDLE_ON() palSetPad(GPIOA, GPIOA_PIN5_LED_R)
//#define INDICATE_IDLE_OFF() palClearPad(GPIOA, GPIOA_PIN5_LED_R)
#define INDICATE_IDLE_ON()
#define INDICATE_IDLE_OFF()

uint8_t bButton = 0;
unsigned char bLogging = 0; // if =1 than we logging to SD card

//------------------------------------------------------------------------------
// ADC configurations

#define ADC_NUM_CHANNELS   8
#define ADC_BUF_DEPTH      1

static adcsample_t samples[ADC_NUM_CHANNELS * ADC_BUF_DEPTH];

static uint8_t channel_en[ADC_NUM_CHANNELS];

static float channel_zero[ADC_NUM_CHANNELS];
static float channel_gain[ADC_NUM_CHANNELS];

static float channel_data[ADC_NUM_CHANNELS];
static float channel_fltorder[ADC_NUM_CHANNELS];
//------------------------------------------------------------------------------

/*===========================================================================*/
// data bufferization functions

#define SD_WRITE_BUFFER             (1024*49)//(1024*21)   // 21K
#define SD_WRITE_BUFFER_FLUSH_LIMIT (1024*48)//(1024*20)   // 20K

#include <string.h>
#include "mmcsd.h"

// buffer for collecting data to write
char sd_buffer[SD_WRITE_BUFFER];
WORD sd_buffer_length = 0;

// buffer for storing ready to write data
char sd_buffer_for_write[SD_WRITE_BUFFER];
unsigned char bReqWrite = 0; // write request, the sd_buffer is being copied to sd_buffer_for_write 
WORD sd_buffer_length_for_write = 0;

unsigned char bWriteFault = 0; // in case of overlap or write fault


// fill buffer with spaces (before \r\n) to make it 512 byte size
// return 1 if filled and ready to write
int align_buffer()
{
  int i;
  int len;
  
  if (sd_buffer_length < 2) return 0;
  if (sd_buffer[sd_buffer_length-2] != '\r') return 0;
  if (sd_buffer[sd_buffer_length-1] != '\n') return 0;
  
  len = MMCSD_BLOCK_SIZE - (sd_buffer_length % MMCSD_BLOCK_SIZE);
  for (i = 0; i < len; i++)
    sd_buffer[sd_buffer_length + i - 2] = ' ';
  sd_buffer[sd_buffer_length - 2] = ',';
  sd_buffer[sd_buffer_length + len - 2] = '\r';
  sd_buffer[sd_buffer_length + len - 1] = '\n';
  
  sd_buffer_length += len;
  
  return 1;
}

// copy input buffer into the buffer for flash writing data
void copy_buffer()
{
  // request write operation
  memcpy(sd_buffer_for_write, sd_buffer, sd_buffer_length);
  sd_buffer_length_for_write = sd_buffer_length;
  sd_buffer_length = 0;
}

void request_write()
{
  if (bReqWrite)
    bWriteFault = 1; // buffer overlapping
  
  // request write operation
  align_buffer();
  copy_buffer();
  bReqWrite = 1;
}

int iLastWriteSecond = 0;
static struct tm timp;
  
void fwrite_string(char *pString)
{
  WORD length = strlen(pString);

  // Add string
  memcpy(&sd_buffer[sd_buffer_length], pString, length);
  sd_buffer_length += length;
  
  // Check flush limit
  if(sd_buffer_length >= SD_WRITE_BUFFER_FLUSH_LIMIT)
  {
    request_write();
  }
}





// file writing 
FATFS SDC_FS;
FIL *file;
FRESULT fres;

int i;

#define STRLINE_LENGTH 1024
char sLine[STRLINE_LENGTH];
systime_t stLastWriting;
unsigned char bIncludeTimestamp = 1;
char sTmp[128];
char format_str[128];

void start_log()
{
  // open file and write the begining of the load
  rtcGetTimeTm(&RTCD1, &timp);        
  sprintf(sLine, "%02d-%02d-%02d.csv", timp.tm_hour, timp.tm_min, timp.tm_sec); // making new file

  file = fopen_(sLine, "a");
  
  // write header line
  sLine[0] = 0;
  if (bIncludeTimestamp)
    strcpy(sLine, "Timestamp");
  
  for (i = 0; i < ADC_NUM_CHANNELS; i++)
  {
    if (channel_en[i])
    {
      sprintf(sTmp, ",ch #%d", i+1);
      strcat(sLine, sTmp);
    }
  }
  strcat(sLine, "\r\n");

  fwrite_string(sLine);
  align_buffer();
  fwrite_(sd_buffer, 1, sd_buffer_length, file);
  f_sync(file);

  // reset buffer counters
  sd_buffer_length_for_write = 0;
  sd_buffer_length = 0;

  bWriteFault = 0;

  stLastWriting = chTimeNow(); // record time when we did write

  bLogging = 1;
}



int read_config_file()
{
  float value;
  char name[64];
  char svalue[64];
  float sample_time;
  int res = 0;
  int i;

  bIncludeTimestamp = 1;
  
  for (i = 0; i < ADC_NUM_CHANNELS; i++) channel_en[i] = 0; // all channels disabled by default
  for (i = 0; i < ADC_NUM_CHANNELS; i++) channel_zero[i] = 0;
  for (i = 0; i < ADC_NUM_CHANNELS; i++) channel_gain[i] = 1;
  for (i = 0; i < ADC_NUM_CHANNELS; i++) channel_fltorder[i] = 1;
  strcpy(format_str, "%f");
  
  // read file
  file = fopen_("ADC.txt", "r");
  if (file == 0) 
  {
    return 0;
  }
  
  while( f_gets(sLine, STRLINE_LENGTH, file) )
  {
    if (sscanf(sLine, "%s %f", name, &value) < 2)
    {
      if (sscanf(sLine, "%s %s", name, svalue) < 2)
      {
        continue;
      }        
    }
    
    if (strcmp(name, "sample") == 0)
    {
      sample_time = value; 
      res = 1; // at least we got sample time, config file accepted
    }
    else
    if (strcmp(name, "timestamp")  == 0)
    {
      bIncludeTimestamp = value;
    }
    else
      
    if (strcmp(name, "ch1_en")  == 0)
      channel_en[0] = (int)value; 
    else
    if (strcmp(name, "ch2_en")  == 0)
      channel_en[1] = (int)value; 
    else
    if (strcmp(name, "ch3_en")  == 0)
      channel_en[2] = (int)value; 
    else
    if (strcmp(name, "ch4_en")  == 0)
      channel_en[3] = (int)value; 
    else
    if (strcmp(name, "ch5_en")  == 0)
      channel_en[4] = (int)value; 
    else
    if (strcmp(name, "ch6_en")  == 0)
      channel_en[5] = (int)value; 
    else
    if (strcmp(name, "ch7_en")  == 0)
      channel_en[6] = (int)value; 
    else
    if (strcmp(name, "ch8_en")  == 0)
      channel_en[7] = (int)value;  
    else
      
    if (strcmp(name, "ch1_zero")  == 0)
      channel_zero[0] = value;
    else
    if (strcmp(name, "ch2_zero")  == 0)
      channel_zero[1] = value;
    else
    if (strcmp(name, "ch3_zero")  == 0)
      channel_zero[2] = value;
    else
    if (strcmp(name, "ch4_zero")  == 0)
      channel_zero[3] = value;
    else
    if (strcmp(name, "ch5_zero")  == 0)
      channel_zero[4] = value;
    else
    if (strcmp(name, "ch6_zero")  == 0)
      channel_zero[5] = value;
    else
    if (strcmp(name, "ch7_zero")  == 0)
      channel_zero[6] = value;
    else
    if (strcmp(name, "ch8_zero")  == 0)
      channel_zero[7] = value;
    else
      
    if (strcmp(name, "ch1_gain")  == 0)
      channel_gain[0] = value;
    else
    if (strcmp(name, "ch2_gain")  == 0)
      channel_gain[1] = value;
    else
    if (strcmp(name, "ch3_gain")  == 0)
      channel_gain[2] = value;
    else
    if (strcmp(name, "ch4_gain")  == 0)
      channel_gain[3] = value;
    else
    if (strcmp(name, "ch5_gain")  == 0)
      channel_gain[4] = value;
    else
    if (strcmp(name, "ch6_gain")  == 0)
      channel_gain[5] = value;
    else
    if (strcmp(name, "ch7_gain")  == 0)
      channel_gain[6] = value;
    else
    if (strcmp(name, "ch8_gain")  == 0)
      channel_gain[7] = value;
    else
      
    if (strcmp(name, "ch1_filt")  == 0)
      channel_fltorder[0] = value;
    else
    if (strcmp(name, "ch2_filt")  == 0)
      channel_fltorder[1] = value;
    else
    if (strcmp(name, "ch3_filt")  == 0)
      channel_fltorder[2] = value;
    else
    if (strcmp(name, "ch4_filt")  == 0)
      channel_fltorder[3] = value;
    else
    if (strcmp(name, "ch5_filt")  == 0)
      channel_fltorder[4] = value;
    else
    if (strcmp(name, "ch6_filt")  == 0)
      channel_fltorder[5] = value;
    else
    if (strcmp(name, "ch7_filt")  == 0)
      channel_fltorder[6] = value;
    else
    if (strcmp(name, "ch8_filt")  == 0)
      channel_fltorder[7] = value;
    else
      
    if (strcmp(name, "format_str")  == 0)
      strcpy(format_str, svalue);
    
  }
  
  gptStartContinuous(&GPTD4, sample_time*10);
  
  fclose_(file);
    
  return res;
}

int init_sd()
{
  // initializing SDC interface
  sdcStart(&SDCD1, NULL);
  if (sdcConnect(&SDCD1) == CH_FAILED) 
  {
    return 0;
  }
  
  // mount the file system
  fres = f_mount(0, &SDC_FS);
  if (fres != FR_OK)
  {
    sdcDisconnect(&SDCD1);
    return 0;
  }
  
  // trying just dummy read file
  file = fopen_("Config.txt", "r");
  if (file)
    fclose_(file);

  
  return 1;
}
//------------------------------------------------------------------------------
/*
 * ADC streaming callback.
 */
static void adccallback(ADCDriver *adcp, adcsample_t *buffer, size_t n) 
{
  (void)adcp;
  if (samples == buffer) 
  {
    palSetPad(GPIOB, GPIOB_PIN15_LED_G);
    
    chSysLockFromIsr();
    for (i = 0; i < ADC_NUM_CHANNELS; i++) 
    {
      if (channel_en[i])
      {    
        channel_data[i] = channel_data[i]*((channel_fltorder[i] - 1)/channel_fltorder[i]) + samples[i]/channel_fltorder[i];
      }
    }
    chSysUnlockFromIsr();
    
    palClearPad(GPIOB, GPIOB_PIN15_LED_G);
  }
}

static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) 
{
  (void)adcp;
  (void)err;
}

/*
 * ADC conversion group.
 * Mode:        Continuous, 16 samples of 8 channels, SW triggered.
 * Channels:    IN11, IN12, IN11, IN12, IN11, IN12, Sensor, VRef.
 */
static const ADCConversionGroup adcgrpcfg = {
  TRUE,
  ADC_NUM_CHANNELS,
  adccallback,
  adcerrorcallback,
  0,                        /* CR1 */
  ADC_CR2_SWSTART,          /* CR2 */
  ADC_SMPR1_SMP_AN15(ADC_SAMPLE_480) | ADC_SMPR1_SMP_AN14(ADC_SAMPLE_480),
  ADC_SMPR2_SMP_AN4(ADC_SAMPLE_480) | ADC_SMPR2_SMP_AN5(ADC_SAMPLE_480) | ADC_SMPR2_SMP_AN6(ADC_SAMPLE_480) | ADC_SMPR2_SMP_AN7(ADC_SAMPLE_480) | ADC_SMPR2_SMP_AN8(ADC_SAMPLE_480) | ADC_SMPR2_SMP_AN9(ADC_SAMPLE_480),                        /* SMPR2 */
  ADC_SQR1_NUM_CH(ADC_NUM_CHANNELS),
  ADC_SQR2_SQ8_N(ADC_CHANNEL_IN15) | ADC_SQR2_SQ7_N(ADC_CHANNEL_IN14),
  ADC_SQR3_SQ6_N(ADC_CHANNEL_IN9)   | ADC_SQR3_SQ5_N(ADC_CHANNEL_IN8) |
  ADC_SQR3_SQ4_N(ADC_CHANNEL_IN7)   | ADC_SQR3_SQ3_N(ADC_CHANNEL_IN6) |
  ADC_SQR3_SQ2_N(ADC_CHANNEL_IN5)   | ADC_SQR3_SQ1_N(ADC_CHANNEL_IN4)
};
//------------------------------------------------------------------------------
/* 
 * Configure a GPT object 
 */ 

void gpt_writer_cb (GPTDriver *gpt_ptr) 
{ 
  int i;
  float data;
  
  if (bLogging)
  {
palSetPad(GPIOB, GPIOB_PIN13_LED_R);
    
    // write down data
    sLine[0] = 0;
    
    if (bIncludeTimestamp)
      sprintf(sLine, "%d", chTimeNow());
    
    for (i = 0; i < ADC_NUM_CHANNELS; i++) 
    {
      if (channel_en[i])
      {
        //data = (samples[i]-channel_zero[i])*channel_gain[i];
        
        //channel_data[i] = channel_data[i]*((channel_fltorder[i] - 1)/channel_fltorder[i]) + data/channel_fltorder[i];
        data = (channel_data[i]-channel_zero[i])*channel_gain[i];
        
        sprintf(sTmp, format_str, data);
        strcat(sLine, ",");
        strcat(sLine, sTmp);
      }
    }
    
    strcat(sLine, "\r\n");

    chSysLockFromIsr();
    fwrite_string(sLine);
    chSysUnlockFromIsr();
    
palClearPad(GPIOB, GPIOB_PIN13_LED_R);
  }
}

static GPTConfig gpt_writer_config = 
{ 
     10000,  // timer clock: 1Mhz 
     gpt_writer_cb  // Timer callback function 
};
//------------------------------------------------------------------------------
int iButtonStableCounter = 0;
unsigned char bButtonNew = 0;
unsigned char bButtonPrev = 0;
#define BUTTON_COUNTER_THRESHOLD 50000



/*
 * Application entry point.
 */
int main(void) 
{
  //CANTxFrame txmsg;
  
  halInit();
  chSysInit();
  
  palSetPad(GPIOB, GPIOB_PIN15_LED_G);
 
  gptStart(&GPTD4, &gpt_writer_config); 
  
  for (i = 0; i < ADC_NUM_CHANNELS; i++) channel_en[i] = 0; // all channels disabled by default
  for (i = 0; i < ADC_NUM_CHANNELS; i++) channel_zero[i] = 0;
  for (i = 0; i < ADC_NUM_CHANNELS; i++) channel_gain[i] = 1;
  for (i = 0; i < ADC_NUM_CHANNELS; i++) channel_fltorder[i] = 1;
  
   /*
   * Initializes the ADC driver 1 and enable the thermal sensor.
   * The pin PC1 on the port GPIOC is programmed as analog input.
   */
  adcStart(&ADCD1, NULL);
  adcSTM32EnableTSVREFE();

  adcStartConversion(&ADCD1, &adcgrpcfg, samples, ADC_BUF_DEPTH);
  

  i = 0;
  while (TRUE) 
  {
INDICATE_IDLE_ON();
    
    // maybe we need to write log, because we didnt for long time?
    if (chTimeElapsedSince(stLastWriting) > S2ST(5))
    {
      if (sd_buffer_length > 0) // there is data to write
      {
        // request write operation
        request_write();
      }
    }
    
    if (bReqWrite)
    {
      //palSetPad(GPIOD, GPIOD_PIN_15_BLUELED);
INDICATE_IDLE_OFF();
      if (fwrite_(sd_buffer_for_write, 1, sd_buffer_length_for_write, file) != sd_buffer_length_for_write)
        bWriteFault = 2;
      if (f_sync(file) != FR_OK)
        bWriteFault = 2;
INDICATE_IDLE_ON();        
      bReqWrite = 0;
      
      stLastWriting = chTimeNow(); // record time when we did write
      
      //palClearPad(GPIOD, GPIOD_PIN_15_BLUELED);
    }
    
    // start-stop log button handling
    if (bButton && bButtonPrev == 0)
    {
      if (bLogging)
      {
        bLogging = 0;
        
        // we are in logging state -- should write the rest of log
        request_write();
      }
      else
      {
        palClearPad(GPIOB, GPIOB_PIN13_LED_R);
        palClearPad(GPIOB, GPIOB_PIN14_LED_B); 
        
        // we are not logging -- opening SD card and starting log
        if (init_sd()) // trying to initialize sd card
        {
          if (read_config_file()) // trying to read configuration file
          {
            // all done -- start loging
            start_log();
          }
          else
          {
            palSetPad(GPIOB, GPIOB_PIN13_LED_R);
            palSetPad(GPIOB, GPIOB_PIN14_LED_B);
          }
        }
        else
          palSetPad(GPIOB, GPIOB_PIN13_LED_R);
      }
    }
    bButtonPrev = bButton;  
    
    // this loop is going very fast, so the button filtering is needed
    bButtonNew = palReadPad(GPIOC, GPIOC_PIN6_BTN);
    if (bButtonNew && bButton == 0)
    {
      iButtonStableCounter++;
      if (iButtonStableCounter > BUTTON_COUNTER_THRESHOLD)
      {
        iButtonStableCounter = 0;
        bButton = 1;
      }
    }
    else
    if (bButtonNew == 0 && bButton)
    {
      iButtonStableCounter++;
      if (iButtonStableCounter > BUTTON_COUNTER_THRESHOLD)
      {
        iButtonStableCounter = 0;
        bButton = 0;
      }
    }
    else
      iButtonStableCounter = 0;
    
    if (bWriteFault)
      palSetPad(GPIOB, GPIOB_PIN13_LED_R);

  }
}
//------------------------------------------------------------------------------