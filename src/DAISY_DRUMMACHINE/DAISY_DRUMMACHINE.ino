#define SERIAL_BUFFER_SIZE 128
#include "DaisyDuino.h"
#include <MIDI.h>

// MUX PINS
// MUX 1 -- S & SIG --
#define MUX1_S0 0
#define MUX1_S1 1
#define MUX1_S2 2
#define MUX1_S3 3
#define MUX1_SIG A11
// MUX 2 -- S & SIG --
#define MUX2_S0 4
#define MUX2_S1 5
#define MUX2_S2 6
#define MUX2_S3 7
#define MUX2_SIG A10

// MUX 1 CHANNEL DEFINITIONS
#define KICK_TUNE 0
#define KICK_DECAY 1
#define KICK_PUNCH 2
#define KICK_MORPH 3
#define KICK_LEVEL 4
#define SNARE_TUNE 5
#define SNARE_DECAY 6
#define SNARE_SNAP 7
#define SNARE_LEVEL 8
#define HATS_TUNE 9
#define HATS_DECAY 10
#define HATS_TONE 11
#define HATS_LEVEL 12
#define FM1_CARRIER_TUNE 13
#define FM1_DECAY 14
#define FM1_MODULATOR_TUNE 15

// MUX 2 CHANNEL DEFINITIONS
#define FM1_MOD_AMOUNT 0
#define FM1_LEVEL 1
#define FM2_CARRIER_TUNE 2
#define FM2_DECAY 3
#define FM2_MODULATOR_TUNE 4
#define FM2_MOD_AMOUNT 5
#define FM2_LEVEL 6
#define MASTER_VOLUME 15

// TRIGGER/LED PINS
#define KICK_LED 16
#define SNARE_LED 17
#define HATS_LED 18
#define FM1_LED 19
#define FM2_LED 20

// MUX READING VARIABLES
const int numMuxChannels = 16;
uint16_t muxValues[numMuxChannels] = { 0 };
uint16_t mux2Values[numMuxChannels] = { 0 };
int muxChannelIndex = 0;
const int muxReadInterval = 5;  // ms between channel reads
unsigned long lastMuxReadTime = 0;

// SEED PINS FOR DIRECT POT READING
#define DISTORTION_POT A0

// *************************************** //
// --- DAISY SEED --- //
DaisyHardware hw;

// *************************************** //
// --- OBJECTS ---
// -- KICK
Oscillator kickSineOsc;  // SINE OSC
Oscillator kickTriOsc;   // TRIANGLE OSC
Adsr kickPitchEnv;       // PITCH ENV
Adsr kickEnv;            // ENV
WhiteNoise kickNoise;    // KICK NOISE
// -- SNARE
Oscillator snareOsc;     // SINE OSC
WhiteNoise snareRattle;  // NOISE
Adsr snarePitchEnv;      // PITCH ENV
Adsr snareEnv;           // ENV
Adsr snareSnappyEnv;     // NOISE ENV
Svf snareRattleHP;       // RATTLE HP
Svf snareRattleLP;       // RATTLE LP
// -- HATS
Oscillator hatsOsc[6];  // OSCILLATORS ARRAY
WhiteNoise hatsNoise;   // NOISE
Adsr hatsEnv;           // HATS ENV
Svf hatsFilter;         // HATS FILTER
// -- FM 1
Oscillator fmCar1;  // FM 1 CARRIER
Oscillator fmMod1;  // FM 1 MODULATOR
Adsr fmEnv1;        // FM1 ENV
Adsr fmPitchEnv1;   // PITCH ENV
// -- FM 1
Oscillator fmCar2;  // FM 2 CARRIER
Oscillator fmMod2;  // FM 2 MODULATOR
Adsr fmEnv2;        // FM2 ENV
Adsr fmPitchEnv2;   // PITCH ENV

// -- FX
Overdrive distortion;  // OUTPUT DISTORTION

// *************************************** //
// --- VARIABLES ---
// -- DAISY
float samplerate;
// -- GATES
volatile bool kickGate = false;
volatile bool snareGate = false;
volatile bool hatsGate = false;
volatile bool fm1Gate = false;
volatile bool fm2Gate = false;
// -- LED / GATE PULSE TIMING
const unsigned long gatePulseTime = 12;  // ms
unsigned long kickGateTime = 0;
unsigned long snareGateTime = 0;
unsigned long hatsGateTime = 0;
unsigned long fm1GateTime = 0;
unsigned long fm2GateTime = 0;
// -- TRIGGERS
volatile bool kickTrigger = false;
volatile bool snareTrigger = false;
volatile bool hatsTrigger = false;
volatile bool fm1Trigger = false;
volatile bool fm2Trigger = false;
// -- KICK
float kickOscTune;                // FREQ CONTROL
float kickPitchEnvDecay = 0.02f;  // FIXED PITCH ENV DEC
float kickEnvDecay;               // DECAY CONTROL
float kickPunch;                  // PITCH ENV AMOUNT CONTROL
float kickMorph;                  // MORPH BETWEEN SINE AND TRIANGLE
float kickLevel;                  // KICK VOL CONTROL
// -- SNARE
float snareTune;                   // FREQ CONTROL
float snareDecay;                  // DECAY CONTROL
float snarePitchEnvDecay = 0.02f;  // FIXED PITCH ENV DEC
float snareSnapDecay;              // SNAP CONTROL
float snareLevel;                  // SNARE VOL CONTROL
// -- HATS
float hatsBaseTune;          // HATS TUNE
float hatsDecay;             // HATS DECAY
float hatsTone;              // HATS TONE
float hatsLevel;             // HATS VOL CONTROL
float hatsDecayMult = 1.0f;  // HATS DECAY COEFF
// -- FM 1
float fm1Tune;                  // FM 1 CARRIER TUNE
float fm1ModTune;               // FM 1 MODULATOR TUNE
float fm1Decay;                 // FM 1 DECAY
float fm1PitchEnvDecay = 0.02;  // FM 1 DECAY
float fm1ModCtrl;               // FM 1 MOD AMOUNT
float fm1Level;                 // FM 1 VOL CONTROL
// -- FM 2
float fm2Tune;                  // FM 1 CARRIER TUNE
float fm2ModTune;               // FM 1 MODULATOR TUNE
float fm2Decay;                 // FM 1 DECAY
float fm2PitchEnvDecay = 0.02;  // FM 1 DECAY
float fm2ModCtrl;               // FM 1 MOD AMOUNT
float fm2Level;                 // FM 1 VOL CONTROL
// -- FX
float distortionGain;  // DISTORTION CONTROL
float distortionMix;   // DISTORTION CONTROL
// -- MASTER
float masterVol;  // OUTPUT CONTROL

// -- POT HANDLING VARIABLES
uint16_t distortionPotValue = 0;
const int potReadInterval = 0;  // ms between direct pot reads
unsigned long lastPotReadTime = 0;

// *************************************** //
// --- CREATE MIDI INSTANCE ---
MIDI_CREATE_DEFAULT_INSTANCE();

// *************************************** //
// --- AUDIO HANDLING ---
static void AudioCallback(float **in, float **out, size_t size) {

  // -- HANDLE TRIGGERS
  if (kickTrigger) {
    kickPitchEnv.Retrigger(false);
    kickEnv.Retrigger(false);
    kickTrigger = false;
  }
  if (snareTrigger) {
    snareEnv.Retrigger(false);
    snarePitchEnv.Retrigger(false);
    snareSnappyEnv.Retrigger(false);
    snareTrigger = false;
  }
  if (hatsTrigger) {
    hatsEnv.Retrigger(false);
    hatsTrigger = false;
  }
  if (fm1Trigger) {
    fmEnv1.Retrigger(false);
    fmPitchEnv1.Retrigger(false);
    fm1Trigger = false;
  }
  if (fm2Trigger) {
    fmEnv2.Retrigger(false);
    fmPitchEnv2.Retrigger(false);
    fm2Trigger = false;
  }

  // -- UPDATE PARAMETERS
  // -- KICK
  kickPitchEnv.SetDecayTime(kickPitchEnvDecay * 0.5f);    // PITCH ENV
  kickPitchEnv.SetReleaseTime(kickPitchEnvDecay * 0.5f);  // PITCH ENV
  kickEnv.SetDecayTime(kickEnvDecay * 0.5f);              // KICK ENV
  kickEnv.SetReleaseTime(kickEnvDecay * 0.5f);            // KICK ENV

  // -- SNARE
  snareEnv.SetDecayTime(snareDecay * 0.5);                 // SNARE ENV
  snareEnv.SetReleaseTime(snareDecay * 0.5);               // SNARE ENV
  snarePitchEnv.SetDecayTime(snarePitchEnvDecay * 0.5);    // PITCH ENV
  snarePitchEnv.SetReleaseTime(snarePitchEnvDecay * 0.5);  // PITCH ENV
  snareSnappyEnv.SetDecayTime(snareSnapDecay * 0.5);       // SNAP ENV
  snareSnappyEnv.SetReleaseTime(snareSnapDecay * 0.5);     // SNAP ENV

  // -- HATS
  hatsEnv.SetDecayTime(hatsDecay * hatsDecayMult * 0.5);    // HATS ENV
  hatsEnv.SetReleaseTime(hatsDecay * hatsDecayMult * 0.5);  // HATS ENV
  hatsFilter.SetFreq(4000.0f + (hatsTone * 7000.0f));       // SET FILTER

  // -- FM1
  fmEnv1.SetDecayTime(fm1Decay * 0.5f);
  fmEnv1.SetReleaseTime(fm1Decay * 0.5f);
  fmPitchEnv1.SetDecayTime(fm1PitchEnvDecay * 0.5f);
  fmPitchEnv1.SetReleaseTime(fm1PitchEnvDecay * 0.5f);
  // -- FM2
  fmEnv2.SetDecayTime(fm2Decay * 0.5f);
  fmEnv2.SetReleaseTime(fm2Decay * 0.5f);
  fmPitchEnv2.SetDecayTime(fm2PitchEnvDecay * 0.5f);
  fmPitchEnv2.SetReleaseTime(fm2PitchEnvDecay * 0.5f);


  for (int i = 0; i < size; i++) {

    // -- KICK
    // PROCESS ENVELOPES
    float kickPitchEnvSig = kickPitchEnv.Process(kickGate) * kickPunch;  // PITCH ENV SIG
    float kickEnvSig = kickEnv.Process(kickGate);                        // KICK ENV SIG
    // UPDATE OSCS
    kickSineOsc.SetFreq(kickOscTune + (kickPitchEnvSig * 400.0f));  // SINE OSC TUNE
    kickTriOsc.SetFreq(kickOscTune + (kickPitchEnvSig * 400.0f));   // TRI OSC TUNE
    // PROCESS OSCILLATORS & MIX
    float kickOscSig = (kickSineOsc.Process() * (1.0f - kickMorph)) + (kickTriOsc.Process() * kickMorph);  // KICK SIG
    float kickOut = (kickOscSig * kickEnvSig) * 1.4f;                                                      // KICK OUT // ENV APPLIED // UNITY SCALING
    // KICK OUTPUT
    kickOut = kickOut * kickLevel;

    // -- SNARE
    // PROCESS ENVELOPES
    float snareEnvSig = snareEnv.Process(snareGate);
    float snarePitchEnvSig = snarePitchEnv.Process(snareGate);
    float snareSnapEnvSig = snareSnappyEnv.Process(snareGate);
    // SET OSCILLATORS
    snareOsc.SetFreq(snareTune + (snarePitchEnvSig * 200.0f));
    // PROCESS OSCILLATORS & MIX
    float snareOscSig = snareOsc.Process() * snareEnvSig;
    float rawSnareSnapSig = snareRattle.Process();
    snareRattleHP.Process(rawSnareSnapSig);
    float hpSnareSnapSig = snareRattleHP.High();
    snareRattleLP.Process(hpSnareSnapSig);
    float snareSnapSig = snareRattleLP.Low() * snareSnapEnvSig;
    float snareOut = (snareOscSig + snareSnapSig);
    // SNARE OUTPUT
    snareOut = (snareOut * snareLevel) * 1.4f;

    // -- HATS
    // HATS TUNE & PROCESS
    float hatSum = 0;
    float hatsOscRatios[6] = { 1.0f, 1.24f, 1.51f, 1.77f, 2.44f, 3.34f };

    for (int j = 0; j < 6; j++) {
      hatsOsc[j].SetFreq(hatsBaseTune * hatsOscRatios[j]);
      hatSum += hatsOsc[j].Process();
    }

    hatSum += hatsNoise.Process();
    hatsFilter.Process(hatSum);
    float hatSig = hatsFilter.Band() * hatsEnv.Process(hatsGate);
    // HATS OUTPUT
    float hatsOut = hatSig * hatsLevel;

    // -- FM 1
    // PROCESS ENVELOPES
    float fm1EnvSig = fmEnv1.Process(fm1Gate);
    float fm1PitchEnvSig = fmPitchEnv1.Process(fm1Gate);
    // PITCH CALCULATIONS
    float fm1CarFreq = fm1Tune + (fm1PitchEnvSig * 300.0f);
    float fm1ModFreq = fm1ModTune + (fm1PitchEnvSig * 300.0f);
    // SET MOD OSC
    fmMod1.SetFreq(fm1ModFreq);
    float fm1ModSig = fmMod1.Process();
    // SET CAR OSC
    fmCar1.SetFreq(fm1CarFreq + (fm1ModSig * 500 * fm1ModCtrl));
    float fm1CarSig = fmCar1.Process() * fm1EnvSig;
    float fm1Out = fm1CarSig * fm1Level;

    // -- FM 2
    // PROCESS ENVELOPES
    float fm2EnvSig = fmEnv2.Process(fm2Gate);
    float fm2PitchEnvSig = fmPitchEnv2.Process(fm2Gate);
    // PITCH CALCULATIONS
    float fm2CarFreq = fm2Tune + (fm2PitchEnvSig * 300.0f);
    float fm2ModFreq = fm2ModTune + (fm2PitchEnvSig * 300.0f);
    // SET MOD OSC
    fmMod2.SetFreq(fm2ModFreq);
    float fm2ModSig = fmMod2.Process();
    // SET CAR OSC
    fmCar2.SetFreq(fm2CarFreq + (fm2ModSig * 500 * fm2ModCtrl));
    float fm2CarSig = fmCar2.Process() * fm2EnvSig;
    float fm2Out = fm2CarSig * fm2Level;

    // DRUM MIXER
    float mixer = (kickOut + snareOut + hatsOut + fm1Out + fm2Out) * 1.0f;

    // FX
    // DISTORTION
    distortion.SetDrive(distortionGain);  // SET DISTORTION AMOUNT
    float dryMix = mixer;
    float distortionSig = (distortion.Process(mixer) * 0.5f);  // PROCESS DISTORTION ONTO MIXER
    float mixDistortion = (dryMix + (distortionSig * distortionMix)) * masterVol;

    // OUTPUT
    out[0][i] = mixDistortion;
    out[1][i] = mixDistortion;
  }
}

// *************************************** //
// --- MIDI HANDLING ---
// -- NOTE ON
void handleNoteOn(byte inChannel, byte inNote, byte inVelocity) {

  // OP-XY sends Note On velocity 0 instead of Note Off
  if (inVelocity == 0) {
    handleNoteOff(inChannel, inNote, inVelocity);
    return;
  }

  // CHECK FOR KICK NOTE
  // 35/36 == Standard MIDI
  // 53/54 == OP-XY KEYBOARD
  if (inNote == 35 || inNote == 36 || inNote == 53 || inNote == 54) {
    kickGate = true;
    kickTrigger = true;
    digitalWrite(KICK_LED, HIGH);
    kickGateTime = millis();
    digitalWrite(KICK_LED, HIGH);
  }

  // CHECK FOR SNARE NOTE
  // 38/39 == Standard MIDI
  // 55/56 == OP-XY KEYBOARD
  if (inNote == 38 || inNote == 39 || inNote == 55 || inNote == 56) {
    snareGate = true;
    snareTrigger = true;
    digitalWrite(SNARE_LED, HIGH);
    snareGateTime = millis();
    digitalWrite(SNARE_LED, HIGH);
  }

  // CHECK FOR CLOSED HAT NOTE
  // 42 == Standard MIDI
  // 60/61 == OP-XY KEYBOARD
  if (inNote == 42 || inNote == 60 || inNote == 61) {
    hatsGate = true;
    hatsTrigger = true;
    hatsDecayMult = 1.0f;
    digitalWrite(HATS_LED, HIGH);
    hatsGateTime = millis();
    digitalWrite(HATS_LED, HIGH);
  }

  // CHECK FOR OPEN HAT NOTE
  // 46 == Standard MIDI
  // 62/63 == OP-XY KEYBOARD
  if (inNote == 46 || inNote == 62 || inNote == 63) {
    hatsGate = true;
    hatsTrigger = true;
    hatsDecayMult = 3.0f;
    digitalWrite(HATS_LED, HIGH);
    hatsGateTime = millis();
    digitalWrite(HATS_LED, HIGH);
  }

  // CHECK FOR FM1 NOTE
  // 45/47 == Standard MIDI Low Tom/Low-Mid Tom
  // 65/67 == OP-XY KEYBOARD
  if (inNote == 45 || inNote == 47 || inNote == 65 || inNote == 67) {
    fm1Gate = true;
    fm1Trigger = true;
    digitalWrite(FM1_LED, HIGH);
    fm1GateTime = millis();
    digitalWrite(FM1_LED, HIGH);
  }

  // CHECK FOR FM2 NOTE
  // 48/50 == Standard MIDI High-Mid Tom/High Tom
  // 69/71 == OP-XY KEYBOARD
  if (inNote == 48 || inNote == 50 || inNote == 69 || inNote == 71) {
    fm2Gate = true;
    fm2Trigger = true;
    digitalWrite(FM2_LED, HIGH);
    fm2GateTime = millis();
    digitalWrite(FM2_LED, HIGH);
  }
}
// -- NOTE OFF
void handleNoteOff(byte inChannel, byte inNote, byte inVelocity) {

  // CHECK FOR KICK NOTE
  // 35/36 == Standard MIDI
  // 53/54 == OP-XY F3 / Lowest note on keyboard
  if (inNote == 35 || inNote == 36 || inNote == 53 || inNote == 54) {
    kickGate = false;
    digitalWrite(KICK_LED, LOW);
  }

  // CHECK FOR SNARE NOTE
  // 38/39 == Standard MIDI
  // 55/56 == OP-XY KEYBOARD
  if (inNote == 38 || inNote == 39 || inNote == 55 || inNote == 56) {
    snareGate = false;
    digitalWrite(SNARE_LED, LOW);
  }

  // CHECK FOR HAT NOTE
  // 42/46 == Standard MIDI
  // 60-63 == OP-XY KEYBOARD
  if (inNote == 42 || inNote == 46 || inNote == 60 || inNote == 61 || inNote == 62 || inNote == 63) {
    hatsGate = false;
    digitalWrite(HATS_LED, LOW);
  }

  // CHECK FOR FM1 NOTE
  // 45/47 == Standard MIDI Low Tom/Low-Mid Tomx
  // 65/67 == OP-XY KEYBOARD
  if (inNote == 45 || inNote == 47 || inNote == 65 || inNote == 67) {
    fm1Gate = false;
    digitalWrite(FM1_LED, LOW);
  }

  // CHECK FOR FM2 NOTE
  // 48/50 == Standard MIDI High-Mid Tom/High Tom
  // 69/71 == OP-XY KEYBOARD
  if (inNote == 48 || inNote == 50 || inNote == 69 || inNote == 71) {
    fm2Gate = false;
    digitalWrite(FM2_LED, LOW);
  }
}

// -- GATES --
void updateGates() {
  unsigned long now = millis();

  if (kickGate && (now - kickGateTime >= gatePulseTime)) {
    kickGate = false;
    digitalWrite(KICK_LED, LOW);
  }
  if (snareGate && (now - snareGateTime >= gatePulseTime)) {
    snareGate = false;
    digitalWrite(SNARE_LED, LOW);
  }
  if (hatsGate && (now - hatsGateTime >= gatePulseTime)) {
    hatsGate = false;
    digitalWrite(HATS_LED, LOW);
  }
  if (fm1Gate && (now - fm1GateTime >= gatePulseTime)) {
    fm1Gate = false;
    digitalWrite(FM1_LED, LOW);
  }
  if (fm2Gate && (now - fm2GateTime >= gatePulseTime)) {
    fm2Gate = false;
    digitalWrite(FM2_LED, LOW);
  }
}

// *************************************** //
// --- SETUP ---
void setup() {

  // Initialize hardware
  hw = DAISY.init(DAISY_SEED, AUDIO_SR_48K);
  samplerate = DAISY.get_samplerate();

  //Serial.begin(115200);

  analogReadResolution(16);

  // Setup MUX control pins as outputs
  // MUX 1
  pinMode(MUX1_S0, OUTPUT);
  pinMode(MUX1_S1, OUTPUT);
  pinMode(MUX1_S2, OUTPUT);
  pinMode(MUX1_S3, OUTPUT);
  pinMode(MUX1_SIG, INPUT);
  // MUX 2
  pinMode(MUX2_S0, OUTPUT);
  pinMode(MUX2_S1, OUTPUT);
  pinMode(MUX2_S2, OUTPUT);
  pinMode(MUX2_S3, OUTPUT);
  pinMode(MUX2_SIG, INPUT);

  // Setup direct pot pin
  pinMode(DISTORTION_POT, INPUT);

  // INIT LEDS
  pinMode(KICK_LED, OUTPUT);
  pinMode(SNARE_LED, OUTPUT);
  pinMode(HATS_LED, OUTPUT);
  pinMode(FM1_LED, OUTPUT);
  pinMode(FM2_LED, OUTPUT);

  // -- LED START
  runStartupAnimation();

  // INIT DRUM VOICES
  initKick();
  initSnare();
  initHats();
  initFM1();
  initFM2();
  initFX();

  // Initialize LEDs
  hw.leds[0].Set(0, 0, 0);
  hw.leds[1].Set(0, 0, 0);

  // Initialize Serial1 for MIDI (31250 baud)
  Serial1.begin(31250);
  Serial1.setRx(14);

  // Set up MIDI handlers
  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);

  // Begin listening for MIDI messages
  MIDI.begin(MIDI_CHANNEL_OMNI);

  // Start audio
  DAISY.begin(AudioCallback);
}

// *************************************** //
// --- LOOP ---
void loop() {

  while (MIDI.read()) {}
  updateGates();
  readMuxChannelNonBlocking();
  readDirectPotNonBlocking();
  processValues();
}

// *************************************** //
// --- MUX HANDLING ---
// Non-blocking MUX channel reading
void readMuxChannelNonBlocking() {
  unsigned long currentTime = millis();

  if (currentTime - lastMuxReadTime >= muxReadInterval) {
    lastMuxReadTime = currentTime;

    // Select the current MUX channel
    selectMuxChannel(muxChannelIndex);

    // Read MUX 1 from A11
    muxValues[muxChannelIndex] = analogRead(MUX1_SIG);
    // Read MUX 2 from A10
    mux2Values[muxChannelIndex] = analogRead(MUX2_SIG);

    // Move to next channel
    muxChannelIndex++;
    if (muxChannelIndex >= numMuxChannels) {
      muxChannelIndex = 0;
    }
  }
}

// Non-blocking direct pot reading
void readDirectPotNonBlocking() {
  unsigned long currentTime = millis();

  if (currentTime - lastPotReadTime >= potReadInterval) {
    lastPotReadTime = currentTime;

    // Read the direct pot on A0
    distortionPotValue = analogRead(DISTORTION_POT);
  }
}

// Select MUX channel using control pins
void selectMuxChannel(int channel) {
  digitalWrite(MUX1_S0, channel & 0x01);
  digitalWrite(MUX1_S1, (channel >> 1) & 0x01);
  digitalWrite(MUX1_S2, (channel >> 2) & 0x01);
  digitalWrite(MUX1_S3, (channel >> 3) & 0x01);
  digitalWrite(MUX2_S0, channel & 0x01);
  digitalWrite(MUX2_S1, (channel >> 1) & 0x01);
  digitalWrite(MUX2_S2, (channel >> 2) & 0x01);
  digitalWrite(MUX2_S3, (channel >> 3) & 0x01);
}

// -- PROCESS POTS
void processValues() {

  // KICK POTS
  kickOscTune = fmap(muxValues[KICK_TUNE] / 65535.0f, 30.0, 120.0, Mapping::EXP);
  kickEnvDecay = fmap(muxValues[KICK_DECAY] / 65535.0f, 0.01, 1.5, Mapping::EXP);
  kickPunch = muxValues[KICK_PUNCH] / 65535.0f;  // 0.0 TO 1.0f
  kickMorph = muxValues[KICK_MORPH] / 65535.0f;  // 0.0 TO 1.0f
  kickLevel = fmap(muxValues[KICK_LEVEL] / 65535.0f, 0.0, 1.0, Mapping::LINEAR);

  // SNARE POTS
  snareTune = fmap(muxValues[SNARE_TUNE] / 65535.0f, 180.0, 350.0, Mapping::EXP);
  snareDecay = fmap(muxValues[SNARE_DECAY] / 65535.0f, 0.05, 1.5, Mapping::EXP);
  snareSnapDecay = fmap(muxValues[SNARE_SNAP] / 65535.0f, 0.05, 1.8, Mapping::EXP);
  snareLevel = fmap(muxValues[SNARE_LEVEL] / 65535.0f, 0.0, 1.0, Mapping::LINEAR);

  // HATS POTS
  hatsBaseTune = fmap(muxValues[HATS_TUNE] / 65535.0f, 250.0, 500.0, Mapping::EXP);
  hatsDecay = fmap(muxValues[HATS_DECAY] / 65535.0f, 0.01, 0.8, Mapping::EXP);
  hatsTone = fmap(muxValues[HATS_TONE] / 65535.0f, 0.0, 1.0, Mapping::LINEAR);
  hatsLevel = fmap(muxValues[HATS_LEVEL] / 65535.0f, 0.0, 1.0, Mapping::LINEAR);

  // FM1 POTS
  fm1Tune = fmap(muxValues[FM1_CARRIER_TUNE] / 65535.0f, 100.0, 600.0, Mapping::EXP);
  fm1Decay = fmap(muxValues[FM1_DECAY] / 65535.0f, 0.05, 1.0, Mapping::EXP);
  fm1ModTune = fmap(muxValues[FM1_MODULATOR_TUNE] / 65535.0f, 120.0, 1000.0, Mapping::EXP);
  fm1Level = fmap(mux2Values[FM1_LEVEL] / 65535.0f, 0.0, 1.0, Mapping::LINEAR);
  fm1ModCtrl = mux2Values[FM1_MOD_AMOUNT] / 65535.0f;

  if (fm1ModCtrl < 0.01f) {
    fm1ModCtrl = 0.0f;
  } else {
    fm1ModCtrl = fmap(fm1ModCtrl, 0.0f, 1.0f, Mapping::LINEAR);
  }

  // FM2 POTS
  fm2Tune = fmap(mux2Values[FM2_CARRIER_TUNE] / 65535.0f, 100.0, 800.0, Mapping::EXP);
  fm2Decay = fmap(mux2Values[FM2_DECAY] / 65535.0f, 0.05, 1.5, Mapping::EXP);
  fm2ModTune = fmap(mux2Values[FM2_MODULATOR_TUNE] / 65535.0f, 120.0, 1000.0, Mapping::EXP);
  fm2Level = fmap(mux2Values[FM2_LEVEL] / 65535.0f, 0.0, 1.0, Mapping::LINEAR);
  fm2ModCtrl = mux2Values[FM2_MOD_AMOUNT] / 65535.0f;

  if (fm2ModCtrl < 0.01f) {
    fm2ModCtrl = 0.0f;
  } else {
    fm2ModCtrl = fmap(fm2ModCtrl, 0.0f, 1.0f, Mapping::LINEAR);
  }


  // DISTORTION
  // Process direct pot value for distortion parameters
  // Using the same pot to control both gain and mix with different mappings
  float potNormalized = distortionPotValue / 65535.0f;

  // Map to different ranges for gain and mix
  distortionGain = fmap(potNormalized, 0.0, 1.0, Mapping::LINEAR);  // Gain: 0.0 to 0.5
  distortionMix = potNormalized;                                    // Mix: 0.0 to 1.0 (full range)

  // MASTER
  masterVol = fmap(mux2Values[MASTER_VOLUME] / 65535.0f, 0.0, 1.3, Mapping::LINEAR);
}

// *************************************** //
// --- INITIALISE VOICES & FX ---
// -- INIT KICK
void initKick() {
  // SINE OSC
  kickSineOsc.Init(samplerate);
  kickSineOsc.SetWaveform(Oscillator::WAVE_SIN);
  kickSineOsc.SetFreq(kickOscTune);
  kickSineOsc.SetAmp(1.0);

  // TRI OSC
  kickTriOsc.Init(samplerate);
  kickTriOsc.SetWaveform(Oscillator::WAVE_TRI);
  kickTriOsc.SetFreq(kickOscTune);
  kickTriOsc.SetAmp(1.0);

  // PITCH ENV
  kickPitchEnv.Init(samplerate);
  kickPitchEnv.SetAttackTime(0.005f);
  kickPitchEnv.SetDecayTime(kickPitchEnvDecay * 0.5f);
  kickPitchEnv.SetSustainLevel(0.0f);
  kickPitchEnv.SetReleaseTime(kickPitchEnvDecay * 0.5f);

  // ENV
  kickEnv.Init(samplerate);
  kickEnv.SetAttackTime(0.005f);
  kickEnv.SetDecayTime(kickEnvDecay * 0.5f);
  kickEnv.SetSustainLevel(0.0f);
  kickEnv.SetReleaseTime(kickEnvDecay * 0.5f);

  kickNoise.Init();
  kickNoise.SetAmp(kickPunch);
}

// -- INIT SNARE
void initSnare() {

  // SINE
  snareOsc.Init(samplerate);
  snareOsc.SetWaveform(Oscillator::WAVE_SIN);
  snareOsc.SetFreq(snareTune);
  snareOsc.SetAmp(0.707f);

  // NOISE RATTLE
  snareRattle.Init();
  snareRattle.SetAmp(0.707f);

  // ENV
  snareEnv.Init(samplerate);
  snareEnv.SetAttackTime(0.01f);
  snareEnv.SetDecayTime(snareDecay * 0.5f);
  snareEnv.SetSustainLevel(0.0f);
  snareEnv.SetReleaseTime(snareDecay * 0.5f);

  // PITCH ENV
  snarePitchEnv.Init(samplerate);
  snarePitchEnv.SetAttackTime(0.005f);
  snarePitchEnv.SetDecayTime(snarePitchEnvDecay * 0.5f);
  snarePitchEnv.SetSustainLevel(0.0f);
  snarePitchEnv.SetReleaseTime(snarePitchEnvDecay * 0.5f);

  // SNAP ENV
  snareSnappyEnv.Init(samplerate);
  snareSnappyEnv.SetAttackTime(0.002f);
  snareSnappyEnv.SetDecayTime(snareSnapDecay * 0.5f);
  snareSnappyEnv.SetSustainLevel(0.0f);
  snareSnappyEnv.SetReleaseTime(snareSnapDecay * 0.5f);

  // HP FOR RATTLE
  snareRattleHP.Init(samplerate);
  snareRattleHP.SetRes(0.1f);
  snareRattleHP.SetDrive(0.0f);
  snareRattleHP.SetFreq(2500.0f);
  // LP FOR RATTLE
  snareRattleLP.Init(samplerate);
  snareRattleLP.SetRes(0.1f);
  snareRattleLP.SetDrive(0.0f);
  snareRattleLP.SetFreq(10000.0f);
}

// -- INIT HATS
void initHats() {

  // OSCS
  float hatFreqs[6] = { hatsBaseTune, (hatsBaseTune + 60.0f), (hatsBaseTune + 115.0f), (hatsBaseTune + 190.0f), (hatsBaseTune + 355.0f), (hatsBaseTune + 575.0f) };
  for (int i = 0; i < 6; i++) {
    hatsOsc[i].Init(samplerate);
    hatsOsc[i].SetWaveform(Oscillator::WAVE_SQUARE);
    hatsOsc[i].SetFreq(hatFreqs[i]);
    hatsOsc[i].SetAmp(0.15f);
  }

  // NOISE
  hatsNoise.Init();
  hatsNoise.SetAmp(0.3f);

  // ENV
  hatsEnv.Init(samplerate);
  hatsEnv.SetAttackTime(0.002f);
  hatsEnv.SetDecayTime(hatsDecay * 0.5f);
  hatsEnv.SetSustainLevel(0.0f);
  hatsEnv.SetReleaseTime(hatsDecay * 0.5f);

  // FILTER
  hatsFilter.Init(samplerate);
  hatsFilter.SetRes(5.0f);
  hatsFilter.SetDrive(0.5f);
  hatsFilter.SetFreq(7000.0f);
}

// -- FM DRUMS
// -- INIT FM1
void initFM1() {

  // CARRIER OSC
  fmCar1.Init(samplerate);
  fmCar1.SetWaveform(Oscillator::WAVE_SIN);
  fmCar1.SetFreq(fm1Tune);
  fmCar1.SetAmp(1.0);

  // MOD OSC
  fmMod1.Init(samplerate);
  fmMod1.SetWaveform(Oscillator::WAVE_SIN);
  fmMod1.SetFreq(fm1ModTune);
  fmMod1.SetAmp(1.0f);

  // ENV
  fmEnv1.Init(samplerate);
  fmEnv1.SetAttackTime(0.002f);
  fmEnv1.SetDecayTime(fm1Decay * 0.5f);
  fmEnv1.SetSustainLevel(0.0f);
  fmEnv1.SetReleaseTime(fm1Decay * 0.5f);

  // PITCH ENV
  fmPitchEnv1.Init(samplerate);
  fmPitchEnv1.SetAttackTime(0.002f);
  fmPitchEnv1.SetDecayTime(fm1PitchEnvDecay * 0.5f);
  fmPitchEnv1.SetSustainLevel(0.0f);
  fmPitchEnv1.SetReleaseTime(fm1PitchEnvDecay * 0.5f);
}
// -- INIT FM2
void initFM2() {

  // CARRIER OSC
  fmCar2.Init(samplerate);
  fmCar2.SetWaveform(Oscillator::WAVE_TRI);
  fmCar2.SetFreq(fm2Tune);
  fmCar2.SetAmp(1.0);

  // MOD OSC
  fmMod2.Init(samplerate);
  fmMod2.SetWaveform(Oscillator::WAVE_TRI);
  fmMod2.SetFreq(fm2ModTune);
  fmMod2.SetAmp(1.0f);

  // ENV
  fmEnv2.Init(samplerate);
  fmEnv2.SetAttackTime(0.002f);
  fmEnv2.SetDecayTime(fm2Decay * 0.5f);
  fmEnv2.SetSustainLevel(0.0f);
  fmEnv2.SetReleaseTime(fm2Decay * 0.5f);

  // PITCH ENV
  fmPitchEnv2.Init(samplerate);
  fmPitchEnv2.SetAttackTime(0.002f);
  fmPitchEnv2.SetDecayTime(fm2PitchEnvDecay * 0.5f);
  fmPitchEnv2.SetSustainLevel(0.0f);
  fmPitchEnv2.SetReleaseTime(fm2PitchEnvDecay * 0.5f);
}
// INIT FX
void initFX() {
  // DISTORTION
  distortion.Init();
  distortion.SetDrive(distortionGain);
}

// -- INIT LEDS
void runStartupAnimation() {
  int leds[] = { KICK_LED, SNARE_LED, HATS_LED, FM1_LED, FM2_LED };
  int numLeds = 5;
  int delayTime = 50;  // Speed of the animation in ms

  // Forward: Left to Right
  for (int i = 0; i < numLeds; i++) {
    digitalWrite(leds[i], HIGH);
    delay(delayTime);
    digitalWrite(leds[i], LOW);
  }

  // Backward: Right to Left
  for (int i = numLeds - 2; i >= 0; i--) {  // Start at FM1 to avoid double-hitting FM2
    digitalWrite(leds[i], HIGH);
    delay(delayTime);
    digitalWrite(leds[i], LOW);
  }

  // Quick blink all at once to signal "Ready"
  for (int i = 0; i < numLeds; i++) digitalWrite(leds[i], HIGH);
  delay(200);
  for (int i = 0; i < numLeds; i++) digitalWrite(leds[i], LOW);
  delay(200);
  for (int i = 0; i < numLeds; i++) digitalWrite(leds[i], HIGH);
  delay(200);
  for (int i = 0; i < numLeds; i++) digitalWrite(leds[i], LOW);
  delay(200);
}