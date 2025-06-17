//
// Copyright (c) 2025, CullenLab, Johns Hopkins University
// All rights reserved.
//
// This source code is licensed under the MIT-style license found in the
// LICENSE file in the root directory of this source tree. 
//

//
// This code uses register-level access to the Teensy 4 Quad Timers (TMRx) to
// generate a 30KHz sample clock which is phase-locked to the Neuropixels
// 11.7MHz head-stage pixel clock. The Teensy timers are clocked at 150MHz,
// giving 6.67nSec timing resolution.
//
// Please refer to the IMXRT1060 (the microcontroller chip used by the Teensy
// 4.0) reference manual, Chapter 54, for details on programming the registers
// of the "TMR" Quad Timer peripheral.
//
//   https://www.pjrc.com/teensy/IMXRT1060RM_rev3_annotations.pdf
//

#define TRIG_IN_PIN 12


// Some useful ANSI escape sequences.
#define CLREOL "\033[K"
#define CURSOR_HOME "\033[H"

// This clears from the cursor to end of screen.
#define CLREOSCR "\033[J"

char spbuf[1000];
void sprintf(char const *Format, ...)
{
  va_list ap;
  va_start(ap, Format);
  vsnprintf(spbuf, sizeof(spbuf), Format, ap);
  va_end(ap);

  Serial.print(spbuf);
}


void ShowTMRCounts(IMXRT_TMR_t &T) {
  int Tnum = 1 + (&T - &IMXRT_TMR1) / (&IMXRT_TMR2 - &IMXRT_TMR1);  // Calculate timer number from address.
  Serial.printf("TMR%d(%5d|%5d|%5d|%5d) ", Tnum, T.CH[0].CNTR, T.CH[1].CNTR, T.CH[2].CNTR, T.CH[3].CNTR);
}

void ShowAllTMRCounts() {
  //  ShowTMRCounts(IMXRT_TMR1);
  //  ShowTMRCounts(IMXRT_TMR2);
  ShowTMRCounts(IMXRT_TMR3);
  //  ShowTMRCounts(IMXRT_TMR4);
}

void ResetTMR(IMXRT_TMR_t &T) {
  T.ENBL = 0;

  for (int i = 0; i < 4; ++i) {
    T.CH[i].CTRL = 0;
    T.CH[i].SCTRL = 0;
    T.CH[i].CSCTRL = 0;
    T.CH[i].CNTR = 0;
    T.CH[i].COMP1 = 0xffff;
    T.CH[i].COMP2 = 0;
    T.CH[i].LOAD = 0;
    T.CH[i].CMPLD1 = 0xffff;
    T.CH[i].CMPLD2 = 0;
    T.CH[i].FILT = 0;
    T.CH[i].DMA = 0;
  }
}

void EnableTMRPins()
{
  // ============== The Quad Timer pins ==================
  // TMR1
  //  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 1;    // QT1 CHAN 0 on pin 10
  //  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01 = 1;    // QT1 CHAN 1 on pin 12
  //  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02 = 1;    // QT1 CHAN 2 on pin 11

  // TMR3
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_00 = 1;  // QT3 CHAN 0 on pin 19
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_01 = 1;  // QT3 CHAN 1 on pin 18
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 1;  // QT3 CHAN 2 on pin 14
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_03 = 1;  // QT3 CHAN 3 on pin 15

  // TMR4
  // IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_10 = 1;    // QT4 CHAN 1 on pin 6
  // IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_11 = 1;    // QT4 CHAN 2 on pin 9
}

void ResetTMRS()
{
  // Enable the peripheral clocks for all the Quad timers.
  CCM_CCGR6 |= CCM_CCGR6_QTIMER1(CCM_CCGR_ON);  // enable QTMR1
  CCM_CCGR6 |= CCM_CCGR6_QTIMER2(CCM_CCGR_ON);  // enable QTMR2
  CCM_CCGR6 |= CCM_CCGR6_QTIMER3(CCM_CCGR_ON);  // enable QTMR3
  CCM_CCGR6 |= CCM_CCGR6_QTIMER4(CCM_CCGR_ON);  // enable QTMR4

  // Completely reset all timer registers, so they do not misbehave.
  ResetTMR(IMXRT_TMR1);
  ResetTMR(IMXRT_TMR2);
  ResetTMR(IMXRT_TMR3);
  ResetTMR(IMXRT_TMR4);

  EnableTMRPins();
}


//==========================================================================================
// Internal pulse train generator, uses the 30KHz timer channel with TMR_IRQ()
// to drive the timing. The timer output pulses are generated DIRECTLY by the
// timer hardware, to avoid software timing jitter.
//
int trig_interval = 60; // Trigger interval, in multiples of 30KHz clock interval ~33.3uSec.
int trig_count = 10; // Number of trigger pulses to put out.

// The "live" counters manipulated by the IRQ routine.
int trig_count_30k = 0;
int trig_count_down = 0;

// Set up counters.
void InitTrig(float TrigFreq_Hz, int NTrig)
{
  trig_interval = round(30000.0 / TrigFreq_Hz);
  trig_count = NTrig;

  trig_count_30k = trig_interval;
  trig_count_down = NTrig;
}

// This convenience function takes a duration in mSec, instead of a Number of
// triggers.
void InitTrig_dur_ms(float TrigFreq_Hz, float Dur_ms)
{
  InitTrig(TrigFreq_Hz, round(TrigFreq_Hz * (Dur_ms+0.001) / 1000.0));
}

// Called from IRQ routine to handle trig pulse outputs.
void HandleTrig()
{
  // If the trig count down is 0, the mechanism is disabled.
  if(trig_count_down <= 0)
    return;

  // The output trigger will just be a square wave at the desired TrigFreq. We
  // toggle the signal down halfway through the 30KHz count.
  // 
  // NOTE WELL, this code does NOT actually set the output directly!! It
  // instructs the timer hardware which way to set the output on the NEXT
  // compare. That way, the output triggers have NO SOFTWARE JITTER, since the
  // timing is determed only by the counter hardware.
  if(trig_count_30k > (trig_interval/2)) {
    TMR3_CTRL3 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(1) | TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE(2);
  } else {
    TMR3_CTRL3 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(1) | TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE(1);
  }

  // When trig_count_down becomes zero, the mechanism automatically shuts itself
  // off.
  if(--trig_count_30k <= 0) {
    if(--trig_count_down > 0) {
      trig_count_30k = trig_interval;
    }
  }
}

volatile int IRQcnt = 0;
void TMR_IRQ()
{
  digitalWriteFast(10, HIGH);
  ++IRQcnt;

  // Just to be sure, we clear the "all" compare IRQ flag in the SCTRL register,
  // and also each of the two separate Compare 1 and Compare 2 flags in the
  // CSCTRL register.
  TMR3_SCTRL0 &= ~TMR_SCTRL_TCF; // Clear TCF IRQ flag.

  // Clear out both Compare 1 and Compare 2 flags.
  TMR3_CSCTRL0 &= ~(TMR_CSCTRL_TCF1 | TMR_CSCTRL_TCF2);

  HandleTrig();

  // MUST do this "barrier" operation, else we can get double-calls to the IRQ.
  asm volatile("dsb");  // avoid double calls due to possible bus sync issues
  digitalWriteFast(10, LOW);
  return;
}


// -------------------------------------------------
// This routine sets up the Quad Timers to divide the 11.7MHz "pixel clock" to
// generate the 30KHz sample clock.
//
// Keep track of timer and external pin usage for the 11.7MHz scheme. The Quad
// Timer pins have limited flexibility, so we keep track of their allocation
// here.
//
// TMR3
//   CHAN    Teensy_Pin         Timer_purpose
//   QT3 Ch0  19 - 30kHz out    Divide 11.7MHz by 390, to get 30KHz
//   QT3 Ch1  18 - 11.7MHz in
//   QT3 Ch2  14 - stim out     Divide Ch3 to make fake stim pulse
//   QT3 Ch3  15 - 

void SetupQuadTimers_11p7MHz()
{
  ResetTMRS();

  // Set lower than the default level of 128, so that the GPIO interrupt above gets priority.
  NVIC_SET_PRIORITY(IRQ_QTIMER3, 150);
  attachInterruptVector(IRQ_QTIMER3, TMR_IRQ);


  // =================== Ch3 Divide by 390 =======================
  //
  // Divide the 11.7MHz master clock input by 390 to get a 30KHz output.

  // Count mode 1 (rising edges primary src), Chan1 input pin (Teensy Pin 18),
  // Mode 4 means use alternating COMPare registers.

  TMR3_CTRL0 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(1) | TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE(4);
  TMR3_SCTRL0 = TMR_SCTRL_OEN | TMR_SCTRL_MSTR; // Enable output, Teensy Pin 19.

  // Enable interrupts ONLY for Compare 1, so we only get one interrupt for each
  // 30KHz interval.
  TMR3_CSCTRL0 = TMR_CSCTRL_TCF1EN;

  // The TOTAL count should be 390, BUT, you MUST SUBTRACT 1!! And if you use
  // "Mode 4" for alternate compare registers (so you can get one pulse per
  // 30KHz sample), then you MUST SUBTRACT 1 **FOR EACH** counter!! So subtract
  // a total of 2!!
  TMR3_COMP10 = 390-30-1;  // All but 30 ticks.
  TMR3_COMP20 = 30-1;      // Give a pulse that is 30 ticks wide.

  // ================== Ch2 Internally-timed Stim Out, clocked from CH0 (30KHz) =========
  //
  // Ch2's input will be the Ch0 30KHz output, and we will count some multiple
  // of the 30KHz sample clock, to generate internally timed stim pulse blocks.
  // We will use the same "trick" above, with alternating compare registers, so
  // that we can set an output pulse of about 5 samples (about 150uSec) to
  // emulate a stim pulse.
  
  // Mode 1 (rising edges), Primary input is Ch0 output, use alternate CMP registers.
  TMR3_CTRL2 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0x04+0) | TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE(4);
  TMR3_SCTRL2 = TMR_SCTRL_OEN; // Enable output, Teensy Pin 14.

  int ch3_total = 30*10;  // 10mSec stim pulse interval.
  TMR3_COMP22 = 4; // The stim pulse width, in 30KHz sample periods
  TMR3_COMP12 = ch3_total - TMR3_COMP22 - 2;


  // ================== Ch3 Triggered Stim Out, clocked from CH0  (30KHz) =========
  //
  // Ch3's input will be the NP phase-locked 30KHz Ch0 output. We use the GPIO
  // interrupt to set OUTMODE to set the timer output HIGH or LOW based on the
  // state of the GPIO input. This way, the timer output becomes a version of
  // the trigger with each trigger pulse edge re-synchronized to the 30KHz Ch0
  // phase-locked sample clock.
  
  // Mode 1 (rising edges), Primary input is Ch0 output, use alternate CMP registers.
  // Initially set OUTMODE to 1, to set output low.
  TMR3_CTRL3 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(1) | TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE(1);
  TMR3_SCTRL3 = TMR_SCTRL_OEN; // Enable output, Teensy Pin 15.

  TMR3_COMP13 = 390-1;     // 390 ticks of the 11.7MHz clock.

  NVIC_ENABLE_IRQ(IRQ_QTIMER3);
  IMXRT_TMR3.ENBL = 0xf;
}

// This is the IRQ handler for the trigger input. It gets called whenever the
// Trigger input changes state, either low-to-high, or high-to-low. This will
// serve to "re-time" the input triggers to be in phase with the 30KHz
// Neuropixels sample clock.
int TrigIn_cnt = 0;
void GPIOTrigIn()
{
  ++TrigIn_cnt;

  if(digitalRead(TRIG_IN_PIN)) {
    // If the trigger input has gone from low to high, then on the next 30KHz
    // timer tick, set the output trigger to HIGH.
    TMR3_CTRL3 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(1) | TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE(2);
  } else {
    // If the trigger input has gone from high to low, then on the next 30KHz
    // timer tick, set the output trigger to LOW.
    TMR3_CTRL3 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(1) | TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE(1);
  }
}


// =================================================================================

void setup()
{
  Serial.begin(1);

  // Enable the DMA peripheral clock.
  CCM_CCGR5 |= CCM_CCGR5_DMA(CCM_CCGR_ON);

  SetupQuadTimers_11p7MHz();

  // Set up GPIO interrupt for Teensy Pin 12.
  attachInterrupt(TRIG_IN_PIN, GPIOTrigIn, CHANGE);

  Serial.printf("Hello from Teensy!\n");
  delay(1);
}

int countup = 0;
bool EnableLoopPrint = true;

void loop()
{
  delay(100);
  digitalToggleFast(13);

  //
  // Use the Arduino Serial Monitor to send individual key presses to the
  // Teensy, to evoke the desired actions in the switch() block.
  //
  while(Serial.available()) {
    char ch = Serial.read();
    switch (ch) {
      case ' ': EnableLoopPrint = !EnableLoopPrint; break;
      case '\\': Serial.printf(CURSOR_HOME CLREOSCR); break;  // Clear the terminal screen.

      // Some manual trigger outputs.
      case '1': InitTrig(200, 10); break; // 200Hz for 10 pulses.
      case '2': InitTrig(500, 30); break; // 500Hz for 30 pulses.
      case '3': InitTrig_dur_ms(500, 20); break; // 500Hz for 20mSec
    }
  }

  if(EnableLoopPrint) {
    
    // Cursor to top/left:
    sprintf(CURSOR_HOME);

    // Show debugging information.    
    //
    // * countup just counts up on each loop() iteration, so that we know the
    //   Teensy is alive and well.
    //
    // * IRQcnt counts the number of entries into the Timer IRQ interrupt
    //   function. If the Timer IRQ count is not moving up continuously, then
    //   the 11.7MHz signal from the head-stage is not reaching us on GPIO pin
    //   18. Check all wiring.
    //
    // * If input triggers are being applied, and the TrigIn IRQ cnt is not
    //   increasing, then the input triggers are not reaching us on GPIO pin 12.
    //   Check all wiring and signals.
    // 
    sprintf("countup: %d  -- #Timer IRQ: %d\n", ++countup, IRQcnt);
    sprintf("TrigIn IRQ cnt %d (GPIO in: %d) %s\n", TrigIn_cnt, digitalReadFast(12), CLREOL);
    sprintf("\n" CLREOSCR);
  }
}
