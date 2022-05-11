#include <EEPROM.h>

/*
 *  Simple eight channel data recorder based on a Teensy 4.1.
 *  
 *  17.03.2021  B. Ulmann Start of implementation, nothing fancy yet, everything out of the box.
 *  13.04.2021  B. Ulmann Added calibration and floating point output, adapted to Perl library TeensyLogger.
 *  17.09.2021  B. Ulmann Data is written out as a single large chunk to speed things up considerably.
 *  10.05.2022  R. Jansen Update ADC speed and use 2 ADC units in parallel 
 */
 
/*
 *  Do NOT change the output of commands as this will break the interface to the Perl library TeensyLogger!
 *
 * TODO:  - Make use of the additional RAM available on the Teensy board.
 */

#include <ADC.h>
#include <IntervalTimer.h>

#define VERSION 0.2

#define STRING_LENGTH     133
#define MAX_CHANNELS      8
#define DEPTH             16384   // 16 k samples
#define ADC_RESOLUTION    10      // Resolution in bits
#define BENCHMARK_SIZE    1000
#define MAX_OVERSAMPLING  4

#define STATE_IDLE    0
#define STATE_ARMED   1
#define STATE_RUNNING 2

#define ARMED_LED     2
#define RUNNING_LED   3
#define TRIGGER_IN    4

/*
 *  The following arrays contain calibration data. Since the simple data logger does not feature analog
 * switches which could connect each input to 0 V, +10 V, and -10 V at startup, this calibration data
 * was derived manually by measuring the results for the aforementioned three voltages for each channel.
 *  A more refined implementation of this data logger should definitely feature automatic calibration at
 * startup at the expense of 8 * 3 analog switches.
 */

int zero[MAX_CHANNELS] = {507, 508, 508, 507, 511, 508, 510, 509},  // Value at 0 V input
    vpos[MAX_CHANNELS] = {821, 822, 821, 821, 821, 822, 823, 823},  // Value at +10 V input
    vneg[MAX_CHANNELS] = {197, 197, 192, 197, 200, 198, 200, 197};  // Value at -10 V input


String help_text = "Teensy DataRecorder " + String(VERSION) + "\n\
?               Print help\n\
arm             Prepare a data collection run which will be started by a trigger signal or command\n\
benchmark       Perform ADC benchmark\n\
calibrate       Calibrate analog interface\n\
channels=x      Set number of channels to x (0 < x <= )" + String(MAX_CHANNELS) + "\n\
dump            Write data gathered to the USB interface\n\
interval=x      Set the sampling interval to x microseconds\n\
ms=x            Set number of samples for the run\n\
reset           Reset the data logger\n\
sample          Perform a singe sample of n channels\n\
oversampling=x  Set degree of oversampling as 2 ** x\n\
start           Start data acqusition (system must be armed prior to this)\n\
status          Print status information\n\
stop            Stop a running data acquisition\n\
";

volatile unsigned short data[DEPTH][MAX_CHANNELS],  // Data storage (10 bits)
                        next_sample = 0,            // Position for the next sample
                        active_channels = 1,        // Number of currently active channels (up to CHANNELS is possible)
                        state = STATE_IDLE,
                        oversampling = 0;

volatile unsigned int interval = 1000,              // Sampling interval in microseconds
                      max_samples = DEPTH;          // Sample DEPTH samples per default
                      
ADC *adc = new ADC();

IntervalTimer sampling_timer;

/*
** Local variant of strtok, just better. :-) The first call expects the string to be tokenized as its first argument.
** All subsequent calls only require the second argument to be set. If there is nothing left to be tokenized, a zero pointer
** will be returned. In contrast to strtok this routine will not alter the string to be tokenized since it
** operates on a local copy of this string.
*/
char *tokenize(char *string, char *delimiters) {
  static char local_copy[STRING_LENGTH], *position;
  char *token;

  if (string) { /* Initial call, create a copy of the string pointer */
    strcpy(local_copy, string);
    position = local_copy;
  } else { /* Subsequent call, scan local copy until a delimiter character will be found */
    while (*position && strchr(delimiters, *position)) /* Skip delimiters if there are any at the beginning of the string */
      position++;

    token = position; /* Now we are at the beginning of a token (or the end of the string :-) ) */

    if (*position == '\'') { /* Special case: Strings delimited by single quotes won't be split! */
      position++;
      while (*position && *position != '\'')
        position++;
    }

    while (*position) {
      position++;
      if (!*position || strchr(delimiters, *position)) { /* Delimiter found */
        if (*position)
          *position++ = (char) 0; /* Split string copy */
        return token;
      }
    }
  }

  return NULL;
}

/*
 * Load the calibration data from the EEPROM section in Flash
 * but only when the magic number is correct (0xA5, 0x5A in the first two bytes)
 */
void calibration_load() {
  unsigned addr;
  unsigned magic;

  magic = (EEPROM.read(0) << 8) + EEPROM.read(1);

  if(magic != 0xA55A) {
    Serial.println("Warning: Analog interface not calibrated !");
    return;
  }
  addr = 2;
  for (int c = 0; c < MAX_CHANNELS; c++) {
    zero[c] = (EEPROM.read(addr  ) << 8) + EEPROM.read(addr+1);
    vpos[c] = (EEPROM.read(addr+2) << 8) + EEPROM.read(addr+3);
    vneg[c] = (EEPROM.read(addr+4) << 8) + EEPROM.read(addr+5);
    addr += 6;
  }
}

/*
 * Save the calibration data to the EEPROM section in Flash
 * This also writes the magic number if needed
 */
void calibration_save() {
  unsigned addr;
  unsigned magic;

  addr = 2;
  for (int c = 0; c < MAX_CHANNELS; c++) {
    EEPROM.write(addr, zero[c] >> 8);
    EEPROM.write(addr+1, zero[c] & 0xff);
    EEPROM.write(addr+2, vpos[c] >> 8);
    EEPROM.write(addr+3, vpos[c] & 0xff);
    EEPROM.write(addr+4, vneg[c] >> 8);
    EEPROM.write(addr+5, vneg[c] & 0xff);
    addr += 6;
  }
  magic = (EEPROM.read(0) << 8) + EEPROM.read(1);

  if(magic != 0xA55A) {
    EEPROM.write(0, 0xA5);
    EEPROM.write(1, 0x5A);
  }
}

/*
 * Calibrate all channels by connecting them all to -10, 0 and +10 V
 */
void calibration_procedure() {
  char input[STRING_LENGTH];
  unsigned short org_oversampling = oversampling,
                 org_max_samples = max_samples,
                 org_active_channels = active_channels;
  
  Serial.println("Starting calibration procedure, press 'n' at any time to abort");
  Serial.println("Connect all channels to -10 V, then press -");
  while (!Serial.available()) ; // Wait for input
  Serial.readString().toCharArray(input, STRING_LENGTH);
  if (input[0] != '-') {
    Serial.println("Aborting calibration");
    return;
  }

  // Initialize and read all vneg levels
  active_channels = MAX_CHANNELS;
  oversampling = 4;
  max_samples = 3;
  next_sample = 0;
  
  sample();

  Serial.println("Connect all channels to 0 V, then press 0");
  while (!Serial.available()) ; // Wait for input
  Serial.readString().toCharArray(input, STRING_LENGTH);
  if (input[0] != '0') {
    Serial.println("Aborting calibration");
    oversampling = org_oversampling;
    max_samples = org_max_samples;
    active_channels = org_active_channels;
    
    return;
  }

  // Read all zero levels
  sample();
  
  Serial.println("Connect all channels to +10 V, then press +");
  while (!Serial.available()) ; // Wait for input
  Serial.readString().toCharArray(input, STRING_LENGTH);
  if (input[0] != '+') {
    Serial.println("Aborting calibration");
    oversampling = org_oversampling;
    max_samples = org_max_samples;
    active_channels = org_active_channels;
    return;
  }

  // Read all vpos levels
  sample();

  // Calibration done, save all samples in calibration data

  for (int i = 0; i < MAX_CHANNELS; i++) {
    vneg[i] = data[0][i];
    zero[i] = data[1][i];
    vpos[i] = data[2][i];
  }

  oversampling = org_oversampling;
  max_samples = org_max_samples;
  active_channels = org_active_channels;
  
  Serial.println("Save calibration data [y/n] ?");
  while (!Serial.available()) ; // Wait for input
  Serial.readString().toCharArray(input, STRING_LENGTH);
  if (input[0] != 'y') {
    Serial.println("Calibration finished, data NOT saved to EEPROM");
    return;
  }

  calibration_save();

  Serial.println("Calibration finished, data saved");
}

void setup() {
  adc->adc0->setResolution(ADC_RESOLUTION);
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);
  adc->adc1->setResolution(ADC_RESOLUTION);
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);

  pinMode(ARMED_LED,   OUTPUT);
  pinMode(RUNNING_LED, OUTPUT);
  pinMode(TRIGGER_IN,  INPUT);

  calibration_load();
}

void sample() {
  unsigned i;
  
  if (!active_channels)
    return;
    
  // Clear data so we start at 0 when using oversampling
  
  for (unsigned i = 0; i < active_channels; i++) {
    data[next_sample][i] = 0;
  }
  
  for (int j = 0; j < (1 << oversampling); j++) {
    for (i = 0; i < active_channels; i+= 2) {
      adc->startSynchronizedSingleRead(i, i+1);
      while(adc->adc0->isConverting() || adc->adc1->isConverting());
      data[next_sample][i] += adc->adc0->readSingle();
      data[next_sample][i+1] += adc->adc1->readSingle();
    }
  }

  // Average data after oversampling
  for (i = 0; i < active_channels; i++) {
    data[next_sample][i] >>= oversampling;
  }
    
  next_sample++;
  if (next_sample >= max_samples) {
    stop();
    Serial.print("\tSampling automatically stopped after " + String(next_sample) + " samples.\n");
  }
}

void trigger() {
  detachInterrupt(TRIGGER_IN);
  attachInterrupt(TRIGGER_IN, stop, RISING);
  state = STATE_RUNNING;
  digitalWrite(ARMED_LED,   LOW);
  digitalWrite(RUNNING_LED, HIGH);

  sampling_timer.begin(sample, interval);
}

void stop() {
  sampling_timer.end();
  detachInterrupt(TRIGGER_IN);
  state = STATE_IDLE;
  digitalWrite(RUNNING_LED, LOW);
}

void loop() {
  char input[STRING_LENGTH], command[STRING_LENGTH], value[STRING_LENGTH];
  if (Serial.available() > 0) {       // There is something to read and process
    Serial.readString().toCharArray(input, STRING_LENGTH);

    tokenize(input, (char *) 0);
    strcpy(command, tokenize((char *) 0, (char *) " ="));

    if (!strcmp(command, "?")) {
      Serial.print(help_text);
    } else if (!strcmp(command, "arm")) {
      Serial.print("\tArmed\n");
      state = STATE_ARMED;
      digitalWrite(ARMED_LED, HIGH);
      next_sample = 0;
      attachInterrupt(TRIGGER_IN, trigger, FALLING);
    } else if (!strcmp(command, "benchmark")) {
      unsigned int start = millis();
      next_sample = 0;
      for (unsigned i = 0; i < BENCHMARK_SIZE; i++) 
        sample();
      next_sample = 0;
      float conversion_time = (float) (millis() - start) / (float) (BENCHMARK_SIZE) * 1000.;
      Serial.print("\t" + String(conversion_time) + " us per conversion @ " + String(active_channels) + " channels.\n");
    } else if (!strcmp(command, "channels")) {
      strcpy(value, tokenize((char *) 0, (char *) "="));
      unsigned int channels = atoi(value);
      if (channels < 1 || channels > MAX_CHANNELS)
        Serial.print("Value >>>" + String(channels) + "<<< out of bounds!\n");
      else {
        active_channels = channels;
        Serial.print("\tchannels=" + String(active_channels) + "\n");
      }
    } else if (!strcmp(command, "calibrate")) {
      calibration_procedure();
    } else if (!strcmp(command, "dump")) {
      Serial.print(String(next_sample) + " samples\n");
      for (int i = 0; i < next_sample; i++) {
        for (int j = 0; j < active_channels; j++) {
          float value = 2 * (float) (data[i][j] - zero[j]) / (float) (vpos[j] - vneg[j]);
          Serial.print(value, 3);
          if (j < active_channels - 1) 
            Serial.print(",");
        }
        Serial.print(";");
      }
      Serial.print("\n");
    } else if(!strcmp(command, "disp")) {
      /*
       * Display samples on multiple lines with 0 - 3.3V readout
       * Mainly used for testing soft- and hardware.
       */
      Serial.println(String(next_sample) + " samples");
      for (int i = 0; i < next_sample; i++) {
        for (int j = 0; j < active_channels; j++) {
          Serial.print(data[i][j] * 3.3 / 1024, 3);
          if (j < active_channels - 1)
            Serial.print(",");
        }
        Serial.println();
      }
    } else if (!strcmp(command, "interval")) {
      strcpy(value, tokenize((char *) 0, (char *) "="));
      interval = atoi(value);
      Serial.print("\tinterval=" + String(interval) + "\n");
    } else if (!strcmp(command, "ms")) {
      strcpy(value, tokenize((char *) 0, (char *) "="));
      unsigned int i = atoi(value);
      if (i < 1 || i > DEPTH) 
        Serial.print("Value >>>" + String(i) + "<<< out of bounds!\n");
      else {
        max_samples = i;
        Serial.print("\tms=" + String(max_samples) + "\n");
      }
    } else if (!strcmp(command, "oversampling")) {
      strcpy(value, tokenize((char *) 0, (char *) "="));
      unsigned int i = atoi(value);
      if (i < 0 || i > MAX_OVERSAMPLING)
        Serial.print("Value >>>" + String(i) + "<<< out of bounds!\n");
      else {
        oversampling = i;
        Serial.print("\toversampling=" + String(oversampling) + "\n");
      }
       
    } else if (!strcmp(command, "reset")) {
      next_sample = 0;
      active_channels = 1;
      state = STATE_IDLE;
      max_samples = DEPTH;
      interval = 1000;
      Serial.print("\tReset\n");
    } else if (!strcmp(command, "sample")) {
      sample();
      Serial.print("\tSampled\n");
    } else if (!strcmp(command, "start")) {
      if (state != STATE_ARMED) 
        Serial.print("\tNot armed, no acquisition!\n");
      else {
        Serial.print("\tStarted\n");
        trigger();
      }
    } else if (!strcmp(command, "status")) {
      Serial.print("\tchannels=" + String(active_channels) + 
        ", samples="      + String(next_sample - 1) + 
        ", state="        + String(state) + 
        ", interval="     + String(interval) +
        ", max_samples="  + String(max_samples) + 
        ", oversampling=" + String(oversampling) +
        "\n");
    } else if (!strcmp(command, "stop")) {
      if (state == STATE_RUNNING) {
        stop();
        Serial.print("\tStopped\n");
      }
      else
        Serial.print("\tDisarmed\n");
      state = STATE_IDLE;
      digitalWrite(RUNNING_LED, LOW);
      digitalWrite(ARMED_LED,   LOW);
    } else {
      Serial.print("Unknown command >>" + String(command) + "<<\n");
    }
  }
}
