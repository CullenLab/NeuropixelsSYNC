//
// Copyright (c) 2025, CullenLab, Johns Hopkins University
// All rights reserved.
//
// This source code is licensed under the MIT-style license found in the
// LICENSE file in the root directory of this source tree. 
//


// Using the Teensy 4 Quad Timers (TMRx) to phase-lock to the NeuroPixels SYNC signal.
//
// 2023_1218 - Fractional frequency implemented for TMR3 CH0. IRQ dithers
// randomly between 2 Compare values.
//
// 2023_1220 - DMA from two registers (CAPTURE and SCTRL) with MLOFF is working.
//           - Demonstrated synchronizing a TMR CH2 with "Master" CH0.
//

// Some useful ANSI escape sequences.
#define CLREOL "\033[K"
#define CURSOR_HOME "\033[H"

// This clears from the cursor to end of screen.
#define CLREOSCR "\033[J"

char spbuf[1000];
char spbuf2[1000];
void sprintf(char const *Format, ...)
{
  va_list ap;
  va_start(ap, Format);
  vsnprintf(spbuf, sizeof(spbuf), Format, ap);
  va_end(ap);

  // Add in the "clear to EOL" sequence wherever there is a newline.
  int o=0;
  for(int i=0; spbuf[i] != 0; ++i) {
    if(spbuf[i] != '\n') {
      spbuf2[o++] = spbuf[i];
      continue;
    }

    // Add in the clear-to-EOL sequence
    spbuf2[o++] = '\033';
    spbuf2[o++] = '[';
    spbuf2[o++] = 'K';
    spbuf2[o++] = '\n';
  }

  spbuf[o] = 0;
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

void TMRSimpleTest(IMXRT_TMR_t &T) {
  for (int i = 0; i < 4; ++i) {
    // Count up, busclk 150MHz, use COMP1, toggle output.
    T.CH[i].CTRL = (1 << 13) | (0x8 << 9) | (1 << 5) | (3 << 0);
    T.CH[i].SCTRL = 1;    // output pin enable
    T.CH[i].COMP1 = 150;  // Count up to this value.
  }
}


// =========================== Parse and Evaluate SYNC signal ===========================
//
// After each 30KHz timer interrupt, we re-evaluate how well we are locked to
// the incoming sync edges.


// This struct holds both the CAPT register, and the SCTRL register, so we can
// more easily get the current signal level (low or high) after each edge.
typedef struct {
  uint16_t tm, stat;
} capt_struct;

capt_struct cs[100];

// Pull the Signal bit out of the SCTRL status value.
#define SIG(stat) ((stat & 0x100) ? 1 : 0)

// Our "test" SYNC signal has 16 "edges" per 30KHz cycle.
const int N_CAPT_VALS = 16*100;
//volatile uint16_t capt_vals[N_CAPT_VALS];
volatile capt_struct capt_vals[N_CAPT_VALS];

// Holds our "parsed" array of edges.
//
//  hilo - 1 or 0 for the signal state after this edge.
//  tm - the time stamp copied out of capt_struct
//  dur_tick - duration in timer ticks
//  dur_N - try to discern how long this is in underlying SYNC "bit" times.

typedef struct {
  uint16_t hilo, tm, dur_tick, dur_N;
} parse_struct;

const int N_ps = 20;
parse_struct ps[N_ps];

// This parses our test sync signal from the Teensy. Currently it has two sets
// of double-pulses per 30KHz period. Baseline is low. One pair of pulses is
// short-short, and one is short-long. The durations come out to about 150 and
// 300 clock ticks.
//
// We will try to "lock" to the beginning of the short-long pulse as the "phase
// zero" position, and line our 30KHz rising edge with that rising edge.
//
int ParseEdges_testsig()
{
  // Create array of edges, giving 1/0 status, and pulse durations.
  for(int i=0; i < N_ps; ++i) {
    ps[i].tm = capt_vals[i].tm;
    ps[i].hilo = SIG(capt_vals[i].stat);

    int dur = (int)capt_vals[i+1].tm - (int)ps[i].tm;
    if(dur < 0)
      dur += 4798;
    
    ps[i].dur_tick = dur;
    ps[i].dur_N = (dur+10) / 150;
  }

  // Use a small state machine to "parse" the array to find the short-long
  // pattern: 1 0 1 1 0.
  int state = 0, stime = -1; //, idx = -1;
  for(int i=0; i < N_ps; ++i) {
    parse_struct &P = ps[i];

    switch(state) {
      case 0:  // look for initial 1, and save index and time-stamp.
        if((P.hilo == 1) && (P.dur_N == 1)) {
          stime = P.tm;
          //idx = i;
          state = 1;
        }
        break;

      case 1: if((P.hilo == 0) && (P.dur_N == 1)) {state=2;} else {state=0;} break; // look for 0
      case 2: if((P.hilo == 1) && (P.dur_N == 2)) {goto DONE;} else {state=0;} break; // look for 1 with dur_N 2
    }
  }
  stime = -1;
  //idx = -1;
DONE:

  // Turn stime into a signed number.
  if(stime > 4798/2)
    stime = stime - 4798;

  // printf("stime %d, at index %d\n", stime, idx);
  // for(int i=0; i<10; ++i) {
  //   Serial.printf("(%d %d %d %d) ",
  //       ps[i].hilo, ps[i].dur_N, ps[i].dur_tick, ps[i].tm);
  // }
  // Serial.printf("\n");

  return stime;
}


//==========================================================================================
// Internal pulse train generator, uses the 30KHz timer channel with TMR_IRQ()
// to drive the timing.
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

// The setup code should initialize this value to enable whatever interrupts are
// desired, then the IRQ routine will just write this value into the SCTRL
// register to clear all interrupt requests.
volatile int SCTRL_set = 0;

volatile int IRQcnt = 0;
volatile int cmp_irq_cnt, edge_irq_cnt;

// IRQ automatically splits this into the integer portion, and a fraction
// portion which becomes randomly "dithered" to sort-of achieve fractional
// frequency resolution.
//float TMR_cmp_flt = 4998.924;  // Desired timer COMP1. We "dither" the fractional part.
float TMR_cmp_flt = 4798.973;  // This is the freq for our Teensy FlexIO test signal.

FASTRUN void TMR_IRQ()  // Not sure "FASTRUN" actually does anything.
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



  //int phase = ParseEdges_testsig();
  //float addition = 0;
  int addition = 0;

  // const int PHASE0 = 40; // time-stamp where we want our clock edge.
  // if(phase > PHASE0 + 2) {
  //   addition = 1;
  // } else if (phase < PHASE0 - 2) {
  //   addition = -1;
  // }

  // Generate random val from 0.0 to 1.0.
  float randval = (float)random(65535) / 65535.0;

  // // Split counter COMPare value into integer, and fractional part.
  uint16_t cmp_int = (uint16_t)TMR_cmp_flt;
  float cmp_fract = TMR_cmp_flt - cmp_int; // + addition;
  
  // // Store integer part, and do proportional random dither between integer part,
  // // and one number higher.
  if(cmp_fract > randval)
    ++cmp_int;

  TMR3_COMP10 = cmp_int + addition; // This will be used at the NEXT compare/re-load time.

  //uint16_t val = TMR3_SCTRL0 & ~TMR_SCTRL_TCF; // Read and clear TCF IRQ flag.
  uint16_t val = TMR3_SCTRL0 & ~TMR_SCTRL_TCF; // Read and clear TCF IRQ flag.

  delayMicroseconds(2);
  TMR3_SCTRL0 = (val & ~TMR_SCTRL_VAL) | TMR_SCTRL_FORCE;; // Force output low.
  TMR3_SCTRL0 = val; // stop forcing.

  // Do same for Chan 2..
  val = TMR3_SCTRL2;
  TMR3_SCTRL2 = (val & ~TMR_SCTRL_VAL) | TMR_SCTRL_FORCE;
  TMR3_SCTRL2 = val;  // Restore register to original state.

  // MUST do this "barrier" operation, else we can get double-calls to the IRQ.
  asm volatile("dsb");  // avoid double calls due to possible bus sync issues
  digitalWriteFast(10, LOW);
}

// Show status of the main DMA controller registers (these are outside the TCD).
void ShowDMAStatus()
{
  Serial.printf("CR %08x  ES %08x  ERQ %08x  INT 0x%08x  ERR 0x%08x, HRS 0x%08x\n",
    DMA_CR, DMA_ES, DMA_ERQ, DMA_INT, DMA_ERR, DMA_HRS);
}


//
// ==========================  DMA  ===========================
//

// Use DMA to store LOTS of "edges" for analysis and phase-lock.

const int DMA_CHAN = 10;
volatile uint32_t &DMAMUX_CHCFG_reg = *(&DMAMUX_CHCFG0 + DMA_CHAN);
const IRQ_NUMBER_t DMA_IRQ_num = (IRQ_NUMBER_t)(IRQ_DMA_CH0 + (DMA_CHAN & 0xf));  // only 16 DMA IRQs, CH0 and CH16 share, etc.

IMXRT_DMA_TCD_t &TCD = IMXRT_DMA_TCD[DMA_CHAN]; // TCD for our DMA channel.


void ShowTCD()
{
  // Show the 8 32-bit words of the TCD
  Serial.printf("DMA TCD registers (at 0x%08x)", &TCD);
  for (int i = 0; i < 8; ++i)
    Serial.printf("  %d  0x%08x\n", i, ((uint32_t *)&TCD)[i]);
}


// Show a few of the time-stamp differences, for debugging.
uint16_t TS_diff[16];

// Save some DADDR values.
int dbgidx = 0;
uint32_t DADDR_save[10];

//volatile uint32_t ticks_sum = 0;
volatile double ticks_sum = 0;
int N_summed;

int dma_irq_cnt=0;
void DMA_irq()
{
  ++dma_irq_cnt;
  DMA_CINT = DMA_CHAN; // Clear IRQ request bit.

  // See if we are in the first half or second half of the buffer. Convert DADDR
  // to a buffer index. The index will be near (but maybe not exactly) zero if
  // the second half of the buffer was just filled, or it will be >= NVALS/2 if
  // the first half of the buffer was just filled.
  int idx = ((uint32_t)TCD.DADDR - (uint32_t)capt_vals) / sizeof(capt_struct);

  DADDR_save[dbgidx] = (uint32_t)TCD.DADDR;
  if(++dbgidx >= 10) dbgidx=0;

  // Try to average over a bunch of time-stamp intervals, to see if we get a
  // good estimate of the SYNC interval.
  int base = (idx >= N_CAPT_VALS/2) ? 0 : N_CAPT_VALS/2;

  double sum=0;
  N_summed = N_CAPT_VALS / 2 -16;

  for(int i=0; i < N_summed; ++i) {
    uint16_t diff = capt_vals[base+i+16].tm - capt_vals[base+i].tm;
    if(i < 16)
      TS_diff[i] = diff;
    //sum += diff + 0.5;
    sum += diff;
  }

  ticks_sum = sum;

  asm volatile("dsb");  // avoid double calls due to possible bus sync issues
}

// ========================================================================================
// ========================================================================================
//
// This sets up a circular DMA buf to any number of 16-bit items.
//
void InitDMA(void volatile *BUF, int NVALS)
{  
  CCM_CCGR5 |= CCM_CCGR5_DMA(CCM_CCGR_ON);  // Enable DMA peripheral clock.
  NVIC_DISABLE_IRQ(DMA_IRQ_num);

  // Make sure DMA channel is deactivated.
  DMA_CERQ = DMA_CHAN;  // Clear the Enable Request bit.
  DMA_CERR = DMA_CHAN;  // Clear any previous DMA errors.

  // Clear out the WHOLE 32-byte TCD first.
  for (int i = 0; i < 8; ++i)
    ((uint32_t *)&TCD)[i] = 0;

  TCD.SADDR = &TMR3_CAPT0;
  TCD.ATTR_SRC = 1;  // 16-bit transfer SIZE
  TCD.SOFF = 0;      // keep reading from same address
  TCD.SLAST = 0;     // no change after buffer wraps, since we just read from same addr always

  TCD.DADDR = BUF;
  TCD.ATTR_DST = 1;  // 16-bit transfer SIZE
  TCD.DOFF = 2;      // increment 2 bytes per transfer

  TCD.NBYTES = 2;  // Transfer 2 bytes (16-bit integer) for each DMA request.
  //TCD.CSR = 0;
  TCD.CSR = (3<<1);     // IRQ for half and full buffer.

  // This will be the buffer size, and then at the end we will add a negative offset
  // to go back to the start of our "circular" buffer.
  TCD.CITER = NVALS;
  TCD.BITER = NVALS;

  TCD.DLASTSGA = -NVALS*2; // Reset to start of buffer.

  // DMAMUX_CHCFG_reg = DMAMUX_CHCFG_ENBL | DMAMUX_SOURCE_QTIMER3_READ0;  // Enable QTMR3 trigger in DMA MUX.
  DMAMUX_CHCFG_reg = DMAMUX_CHCFG_ENBL | DMAMUX_SOURCE_QTIMER3_READ1;  // Enable QTMR3 trigger in DMA MUX.
  NVIC_ENABLE_IRQ(DMA_IRQ_num);

}

// This version saves both the CAPT register, and the SCTRL register. MUST make
// use of the DMA "minor loop maping" facility, so we can reset the source
// address back to the CAPT0 register after each transfer.
void InitDMA_two_regs(void volatile *BUF, int NVALS)
{  
  NVIC_DISABLE_IRQ(DMA_IRQ_num);

  // Set timer priority lower, so that our "sync jitter" IRQ takes precedence.
  NVIC_SET_PRIORITY(DMA_IRQ_num, 151);
  attachInterruptVector(DMA_IRQ_num, DMA_irq);

  // Make sure DMA channel is deactivated.
  DMA_CERQ = DMA_CHAN;  // Clear the Enable Request bit.
  DMA_CERR = DMA_CHAN;  // Clear any previous DMA errors.

  // Strange this is a global option, and not specific to each DMA channel.
  // hmmmm. Enable "minor loop mapping" mode, so that we can read from two
  // registers that are not adjacent, and then reset the source address each
  // time.
  DMA_CR |= DMA_CR_EMLM;

  // Clear out the WHOLE 32-byte TCD first.
  for (int i = 0; i < 8; ++i)
    ((uint32_t *)&TCD)[i] = 0;

  // We want to read TWO 16-bit registers. CAPT0 is at offset 0x04, and SCTRL0
  // is at offset 0x0E. So we use SOFF to "jump" 10 bytes.
  TCD.SADDR = &TMR3_CAPT1;
  TCD.DADDR = BUF;
  TCD.ATTR_SRC = 1;  // 16-bit transfer SIZE
  TCD.ATTR_DST = 1;  // 16-bit transfer SIZE

  TCD.SOFF = 10;     // jump 10 bytes to the SCTRL0 register
  TCD.DOFF = 2;      // increment 2 bytes per transfer at destination

  int nbytes = 4;    // copy two 16-bit values for each DMA request

  // In "minor loop mapping" mode, the NBYTES register gets split into NBYTES,
  // then an offset (which can be negative, but MUST be limited to that bit
  // field) that gets applies after each DMA request. We use this to subtract
  // from the source address, and reset the source pointer to TMR3_CAPT0 each
  // time.
  //
  // Enable Source loop (1<<31).
  // The offset is 20 bits long, so we must truncate it.
  int mloff = -20;   // Must subtract out SOFF twice
  TCD.NBYTES = (1<<31) | ((mloff & 0xfffff) << 10) | nbytes;

  // The CITER determines how long the Major loop is. This will be the
  // destination buffer size, and then at the end we will add a negative offset
  // to go back to the start of our "circular" buffer.
  TCD.CITER = NVALS;
  TCD.BITER = NVALS;

  // When the major loop ends, this offset is applied to the destination
  // address. We use this to make a "circular" buffer by setting the destination
  // address back to the beginning of the destination buffer.
  int dst_off = (TCD.NBYTES & 0x3ff) * TCD.CITER; // subtract however much we incremented.
  TCD.DLASTSGA = -dst_off; // Reset to start of buffer.

  // When Minor Loop Offset is enabled, you MUST still provide the "Major" loop
  // offset also!! Apparently the minor loop offset is NOT applied on the last
  // iteration as the major loop finishes.
  TCD.SLAST = mloff;

  //TCD.CSR = 0;+
  TCD.CSR = (3<<1);     // IRQ for half and full buffer.


  // DMAMUX_CHCFG_reg = DMAMUX_CHCFG_ENBL | DMAMUX_SOURCE_QTIMER3_READ0;  // Enable QTMR3 trigger in DMA MUX.
  DMAMUX_CHCFG_reg = DMAMUX_CHCFG_ENBL | DMAMUX_SOURCE_QTIMER3_READ1;  // Enable QTMR3 trigger in DMA MUX.
  NVIC_ENABLE_IRQ(DMA_IRQ_num);
}


volatile int pinirqcnt=0;
void pinIRQ()
{
  ++pinirqcnt;
  delayNanoseconds(25 * random(16));   // Random delay to create some "jitter".
  digitalWriteFast(21, digitalReadFast(20));
}

int COMP13_orig = 0; // This is set as the COMP13 register is loaded.

// Set up the timers to monitor the incoming SYNC signal "edges".
//
// We will get back to this, but for now, we are ignoring this function, and
// just using the 11.7MHz clock for a quick demo of "perfect artifact zapping",
// in the function just below.
//
void SetupQuadTimers_for_sync_pattern_decoding()
{
  ResetTMRS();

  // Set lower than the default level of 128, so that the GPIO interrupt above gets priority.
  NVIC_SET_PRIORITY(IRQ_QTIMER3, 150);

  // Test to see if we can get interrupt for each rising edge.
//  NVIC_SET_PRIORITY(IRQ_QTIMER3, 0);  // Highest priority.
  attachInterruptVector(IRQ_QTIMER3, TMR_IRQ);

  // Set up TMR3 Chan 1 for 150MHz primary source, SYNC secondary source,
  // wrap around to 0xffff, and use DMA to save SYNC edge time-stamps.

  // Count up, 150MHz clock, CH1 input pin is SYNC input and secondary source. 
  TMR3_CTRL1 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(8) | TMR_CTRL_SCS(1) ;
  TMR3_SCTRL1 = TMR_SCTRL_CAPTURE_MODE(3);
  TMR3_DMA1 = TMR_DMA_IEFDE;                // Enable edge-capture DMA

  // For Teensy Test signal, just capture roughly four complete cycles. There are 8 edges per
  // 30KHz cycle.
  InitDMA_two_regs(capt_vals, N_CAPT_VALS);

  // For PLL, we want the clock period to be about 30KHz, and output enabled,
  // so we can observe behavior on scope.
  
  // simple count up, 150MHz clock, Pin1 as Secondary Count Source (edge
  // detect), reload on COMP1 ("LENGTH")
  TMR3_CTRL0 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(8) | TMR_CTRL_SCS(1) | TMR_CTRL_LENGTH | 3;

  // Enable "Master" broadcast of compare signal, so we can keep CH2 in sync.
  TMR3_SCTRL0 = TMR_SCTRL_MSTR;  // | 1;  -- Do not enable output, so we can use pin as 11.7MHz input.
  TMR3_SCTRL0 |= TMR_SCTRL_TCFIE; // Enable compare interrupt

  //TMR_cmp_flt = 150000 / 30;  // 30KHz period
  TMR3_COMP10 = (uint16_t)TMR_cmp_flt; // 30KHz period
  //TMR3_COMP20 = 150000 / 30 / 2; // Use CMP2 to toggle mid-phase.
//  TMR3_CMPLD10 = 150000 / 30 / 2; // 60KHz period, and toggle output for 30KHz output.

  // ========================= Sync CH2 to CH0 ==============================
  //
  // This works VERY well, in case we need it for capturing rising and falling
  // edges separately. OR, obviously, for generating phase-locked trigger output
  // pulses.
  //
  // See if TMR3 chan 0 can "broadcast" its compare to CH2, and keep them in
  // absolute sync. Enable CO-Init, so CH0 can "broadcast" its compare to this
  // counter. We do NOT need this counter's "LENGTH" bit set. The master's
  // compare will reset this counter as long as the COINIT bit is set.
  TMR3_CTRL2 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(8) | TMR_CTRL_SCS(1) /*| TMR_CTRL_LENGTH*/ | TMR_CTRL_COINIT | 2;

  // EEOF enables "external" force of output flag from the CH0 "master".
  // VAL=1 sets the force to '1', to match Chan 0, which goes high on compare.


  // !!!!!!   !!!!!!   !!!!!!
  // For now, leave output dissabled, so we can tinker with 11.7MHz clock.

  TMR3_SCTRL2 = TMR_SCTRL_CAPTURE_MODE(3) | TMR_SCTRL_EEOF | TMR_SCTRL_VAL; // | 1;  // capture BOTH edges, OUTPUT enable.
  TMR3_COMP12 = 0xffff; // Make it high, so it does not interfere.


  // =================== Ch3 Divide by 390 =======================
  //
  // Just for a quick test, simply divide the 11.7MHz master clock input by 390
  // to get a 30KHz output.

  // Count mode 1 (rising edges primary src), Chan1 input pin (Teensy Pin 18),
  // Mode 4 means use alternating COMPare registers.
  TMR3_CTRL3 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(1) | TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE(4);
  TMR3_SCTRL3 = TMR_SCTRL_OEN; // Enable output, Teensy Pin 15.

  // The TOTAL count should be 390, BUT, you MUST SUBTRACT 1!! And if you use
  // "Mode 4" for alternate compare registers (so you can get one pulse per
  // 30KHz sample), then you MUST SUBTRACT 1 **FOR EACH** counter!! So subtract
  // a total of 2!!
  TMR3_COMP13 = 390-30-1;  // All but 30 ticks.
  TMR3_COMP23 = 30-1;      // Give a pulse that is 30 ticks wide.

  // Used in the "nudge" routine, to be sure we set back correctly to original
  // value.
  COMP13_orig = TMR3_COMP13;

  // ==================================================
  DMA_SERQ = DMA_CHAN;  // Enable the DMA channel
  NVIC_ENABLE_IRQ(IRQ_QTIMER3);
  IMXRT_TMR3.ENBL = 0xf;
}


// -------------------------------------------------
// This is supposed to be a "quick and dirty" test, "cheating" and using the
// 11.7MHz "pixel clock" directly off the head-stage.
//
// Keep track of timer and external pin usage for the 11.7MHz scheme. Note we
// only have luck getting an *INPUT* signal from Pin 18, so we'll use that here.
//
// TMR3
//   CHAN    Teensy_Pin         Timer_purpose
//   QT3 Ch0  19 - 30kHz out    Divide 11.7MHz by 390, to get 30KHz
//   QT3 Ch1  18 - 11.7MHz in
//   QT3 Ch2  14 - stim out     Divide Ch3 to make fake stim pulse
//   QT3 Ch3  15 - 


void SetupQuadTimers_quick_demo_11p7MHz()
{
  ResetTMRS();

  // Set lower than the default level of 128, so that the GPIO interrupt above gets priority.
  NVIC_SET_PRIORITY(IRQ_QTIMER3, 150);

  // Test to see if we can get interrupt for each rising edge.
//  NVIC_SET_PRIORITY(IRQ_QTIMER3, 0);  // Highest priority.
  attachInterruptVector(IRQ_QTIMER3, TMR_IRQ);


  // =================== Ch3 Divide by 390 =======================
  //
  // Just for a quick test, simply divide the 11.7MHz master clock input by 390
  // to get a 30KHz output.

  // Count mode 1 (rising edges primary src), Chan1 input pin (Teensy Pin 18),
  // Mode 4 means use alternating COMPare registers.

  // // See if Pin 19 input works... For some reason, it does not.
  // //TMR3_CTRL3 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0) | TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE(4);
  // TMR3_CTRL3 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(1) | TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE(4);
  
  // //TMR3_SCTRL3 = TMR_SCTRL_OEN; // Enable output, Teensy Pin 15.

  // // The TOTAL count should be 390, BUT, you MUST SUBTRACT 1!! And if you use
  // // "Mode 4" for alternate compare registers (so you can get one pulse per
  // // 30KHz sample), then you MUST SUBTRACT 1 **FOR EACH** counter!! So subtract
  // // a total of 2!!
  // TMR3_COMP13 = 390-30-1;  // All but 30 ticks.
  // TMR3_COMP23 = 30-1;      // Give a pulse that is 30 ticks wide.

  TMR3_CTRL0 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(1) | TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE(4);
  TMR3_SCTRL0 = TMR_SCTRL_OEN | TMR_SCTRL_MSTR; // Enable output, Teensy Pin 19.

  // This would enable interrupts for BOTH Compare registers, which we don't
  // want. We only want one interrupt per 30KHz cycle. TMR3_SCTRL0 =
  // TMR_SCTRL_OEN | TMR_SCTRL_MSTR | TMR_SCTRL_TCFIE; // Enable output, Teensy
  // Pin 19.

  // Enable interrupts ONLY for Compare 1, so we only get one interrupt for each
  // 30KHz interval.
  TMR3_CSCTRL0 = TMR_CSCTRL_TCF1EN;

  // The TOTAL count should be 390, BUT, you MUST SUBTRACT 1!! And if you use
  // "Mode 4" for alternate compare registers (so you can get one pulse per
  // 30KHz sample), then you MUST SUBTRACT 1 **FOR EACH** counter!! So subtract
  // a total of 2!!
  TMR3_COMP10 = 390-30-1;  // All but 30 ticks.
  TMR3_COMP20 = 30-1;      // Give a pulse that is 30 ticks wide.


  // Used in the "nudge" routine, to be sure we set back correctly to original
  // value.
  COMP13_orig = TMR3_COMP13;


  // // An alternate version, to demonstrate when our stim pulses are NOT phase
  // // locked. Here, instead of using the external 11.7MHz head-stage clock, we
  // // use the internal 150MHz clock, and divide by 5000.

  // TMR3_CTRL3 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(8) | TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE(4);
  // TMR3_SCTRL3 = TMR_SCTRL_OEN; // Enable output, Teensy Pin 15.

  // // The TOTAL count should be 390, BUT, you MUST SUBTRACT 1!! And if you use
  // // "Mode 4" for alternate compare registers (so you can get one pulse per
  // // 30KHz sample), then you MUST SUBTRACT 1 **FOR EACH** counter!! So subtract
  // // a total of 2!!
  // TMR3_COMP13 = 5003-300-1;  // All but 30 ticks.
  // TMR3_COMP23 = 300-1;      // Give a pulse that is 30 ticks wide.

  // // Used in the "nudge" routine, to be sure we set back correctly to original
  // // value.
  // COMP13_orig = TMR3_COMP13;



  // ================== Ch2 Fake Stim Out, clocked from CH0 (30KHz) =========
  //
  // Ch2's input will be Ch0 output, and we will count some multiple of the
  // 30KHz sample clock, to generate some fake stim pulses. We will use the same
  // "trick" above, with alternating compare registers, so that we can set an
  // output pulse of about 5 samples (about 150uSec) to emulate a stim pulse.
  
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
  // the trigger that is re-synchronized to the 30KHz Ch0 phase-locked sample
  // clock.
  
  // Mode 1 (rising edges), Primary input is Ch0 output, use alternate CMP registers.
  // Initially set OUTMODE to 1, to set output low.
  TMR3_CTRL3 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(1) | TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE(1);
  TMR3_SCTRL3 = TMR_SCTRL_OEN; // Enable output, Teensy Pin 15.

  TMR3_COMP13 = 390-1;     // 390 ticks of the 11.7MHz clock.

  // This timer will NOT COUNT AT ALL. We simply react to Chan 0, which is set
  // to MASTER, and the GPIO IRQ routine decides wether to SET or CLEAR the
  // output in sync with the 30KHz Ch0.

  // TMR3_CTRL3 = TMR_CTRL_CM(0) | TMR_CTRL_PCS(0) | TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE(1);
  // TMR3_SCTRL3 = TMR_SCTRL_OEN | TMR_SCTRL_EEOF; // Enable output, Teensy Pin 15.

  // TMR3_COMP13 = 999;



/**************

  // Set up TMR3 Chan 1 for 150MHz primary source, SYNC secondary source,
  // wrap around to 0xffff, and use DMA to save SYNC edge time-stamps.

  // Count up, 150MHz clock, CH1 input pin is SYNC input and secondary source. 
  TMR3_CTRL1 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(8) | TMR_CTRL_SCS(1) ;
  TMR3_SCTRL1 = TMR_SCTRL_CAPTURE_MODE(3);
  TMR3_DMA1 = TMR_DMA_IEFDE;                // Enable edge-capture DMA

  // For Teensy Test signal, just capture roughly four complete cycles. There are 8 edges per
  // 30KHz cycle.
  InitDMA_two_regs(capt_vals, N_CAPT_VALS);

  // For PLL, we want the clock period to be about 30KHz, and output enabled,
  // so we can observe behavior on scope.
  
  // simple count up, 150MHz clock, Pin1 as Secondary Count Source (edge
  // detect), reload on COMP1 ("LENGTH")
  TMR3_CTRL0 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(8) | TMR_CTRL_SCS(1) | TMR_CTRL_LENGTH | 3;

  // Enable "Master" broadcast of compare signal, so we can keep CH2 in sync.
  TMR3_SCTRL0 = TMR_SCTRL_MSTR;  // | 1;  -- Do not enable output, so we can use pin as 11.7MHz input.
  TMR3_SCTRL0 |= TMR_SCTRL_TCFIE; // Enable compare interrupt

  //TMR_cmp_flt = 150000 / 30;  // 30KHz period
  TMR3_COMP10 = (uint16_t)TMR_cmp_flt; // 30KHz period
  //TMR3_COMP20 = 150000 / 30 / 2; // Use CMP2 to toggle mid-phase.
//  TMR3_CMPLD10 = 150000 / 30 / 2; // 60KHz period, and toggle output for 30KHz output.

  // ========================= Sync CH2 to CH0 ==============================
  //
  // This works VERY well, in case we need it for capturing rising and falling
  // edges separately. OR, obviously, for generating phase-locked trigger output
  // pulses.
  //
  // See if TMR3 chan 0 can "broadcast" its compare to CH2, and keep them in
  // absolute sync. Enable CO-Init, so CH0 can "broadcast" its compare to this
  // counter. We do NOT need this counter's "LENGTH" bit set. The master's
  // compare will reset this counter as long as the COINIT bit is set.
  TMR3_CTRL2 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(8) | TMR_CTRL_SCS(1) | TMR_CTRL_COINIT | 2;

  // EEOF enables "external" force of output flag from the CH0 "master".
  // VAL=1 sets the force to '1', to match Chan 0, which goes high on compare.


  // !!!!!!   !!!!!!   !!!!!!
  // For now, leave output dissabled, so we can tinker with 11.7MHz clock.

  TMR3_SCTRL2 = TMR_SCTRL_CAPTURE_MODE(3) | TMR_SCTRL_EEOF | TMR_SCTRL_VAL; // | 1;  // capture BOTH edges, OUTPUT enable.
  TMR3_COMP12 = 0xffff; // Make it high, so it does not interfere.

************/


  // ==================================================
  //DMA_SERQ = DMA_CHAN;  // Enable the DMA channel

  NVIC_ENABLE_IRQ(IRQ_QTIMER3);
  IMXRT_TMR3.ENBL = 0xf;
}





// "Nudge" the count temporarily of Chan 3, so (while looking on scope) we can
// nudge the phase of our 30KHz output clock relative to the SYNC pattern from
// the headstage.
void NudgeCh3(int amount)
{
  TMR3_CMPLD13 = COMP13_orig + amount;

  // Set the bit to enable loading the CMP1 register from CMPLD1, and this also
  // clears out the flag that tells us when a successful compare has occurred.
  TMR3_CSCTRL3 = TMR_CSCTRL_CL1(1); // This also clears the TCF1 "sticky" compare flag.
  while(!(TMR3_CSCTRL3 & TMR_CSCTRL_TCF1))
    ;

  // Now put the original value back.  
  TMR3_CMPLD13 = COMP13_orig;

  // while(!(TMR3_CSCTRL3 & TMR_CSCTRL_TCF1))
  //   ;
  
  // TMR3_CSCTRL3 = 0; // Disable the compar load feature.
  // TMR3_COMP13 = curcmp;
}


// This is the IRQ handler for the trigger input.
int TrigIn_cnt = 0;
void GPIOTrigIn()
{
  ++TrigIn_cnt;


  // for testing, just alternate the output.
//  if(TrigIn_cnt & 1) {
  // if(digitalReadFast(12) == HIGH) {
  //   TMR3_SCTRL3 = TMR_SCTRL_OEN | TMR_SCTRL_EEOF | TMR_SCTRL_VAL; // Enable output, Teensy Pin 15.
  // } else {
  //   TMR3_SCTRL3 = TMR_SCTRL_OEN | TMR_SCTRL_EEOF; // Enable output, Teensy Pin 15.
  // }

  if(digitalRead(12)) {
    TMR3_CTRL3 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(1) | TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE(2);
  } else {
    TMR3_CTRL3 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(1) | TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE(1);
  }
}



// =================================================================================

void setup()
{
  Serial.begin(1);

  // Enable the DMA peripheral clock.
  CCM_CCGR5 |= CCM_CCGR5_DMA(CCM_CCGR_ON);

  // Set up some scope signals.
  pinMode(10, OUTPUT);
  //pinMode(11, OUTPUT);  // NEED THIS for LPSPI4 output.
  pinMode(12, OUTPUT);

  pinMode(13, OUTPUT);

  // An input and an output pin, so we can add some "jitter" to the incoming
  // SYNC signal, to be more realistic.
  pinMode(20, INPUT);
  attachInterrupt(20, pinIRQ, CHANGE);
  pinMode(21, OUTPUT);
  //NVIC_SET_PRIORITY(IRQ_GPIO1_16_31, 5); // DOES NOT WORK.
  // This DOES NOT work.
  //NVIC_SET_PRIORITY(IRQ_GPIO6789, 0); // All "fast" GPIO's mapped to one IRQ.

  //SetupQuadTimers();
  //SetupQuadTimers_for_sync_pattern_decoding();
  SetupQuadTimers_quick_demo_11p7MHz();

  // Set up GPIO interrupt for Teensy Pin 12.
  //attachInterrupt(12, GPIOTrigIn, RISING);
  attachInterrupt(12, GPIOTrigIn, CHANGE);

  Serial.printf("Hello from Teensy!\n");
  delay(1);
}

int countup = 0;
int frq;
bool EnableLoopPrint = true;

uint8_t spicount=0;


uint32_t bitpos = 1;

// Copy and display some of the DMA buffer values, so we can get an idea whether this is working correctly.

//capt_struct cs[100];
void ShowBufVals()
{
  // Copy them "quickly" so hopefully DMA will not overwrite while we are working with them.
  for(int i=0; i<100; ++i) {
    cs[i].tm = capt_vals[i].tm;
    cs[i].stat = capt_vals[i].stat;
  }

  // Now show the 0/1 signal value (from the status reg) and the delta time to the NEXT value
  for(int i=0; i < 20; ++i) {
    int sig = SIG(cs[i].stat);
    int duration = cs[i+1].tm - cs[i].tm;
    Serial.printf("(%d %3d) ", sig, duration);
  }
  Serial.printf("\n");
}

void loop()
{
  delay(100);
  digitalToggleFast(13);
  // Serial.printf("%d \n", ++countup);

  while(Serial.available()) {
    char ch = Serial.read();
    switch (ch) {
      case ' ': EnableLoopPrint = !EnableLoopPrint; break;
      case '\\': Serial.printf(CURSOR_HOME CLREOSCR); break;
      case 'D': delay(10); TCD.CSR = 1; delay(10); break;
      case 'd': ShowDMAStatus(); break;
      case 'E': DMA_ES=0; DMA_CERR=DMA_CHAN; Serial.printf("Cleared DMA errors\n"); break;
      case '-': DMA_CERQ = DMA_CHAN; break; // Disable our DMA channel.
      case '+': DMA_SERQ = DMA_CHAN; break; // Enable our DMA channel.
      case 't': ShowTCD(); break;

      // case ',': if(bitpos < 0x80000000) bitpos <<= 1; FLEXIO3_SHIFTBUFBIS0 = 0x80000000 | bitpos; break;
      // case '.': if(bitpos > 1) bitpos >>= 1; FLEXIO3_SHIFTBUFBIS0 = 0x80000000 | bitpos; break;

      // case ',': --TMR3_COMP10; break;
      // case '.': ++TMR3_COMP10; break;
      // case ',': --TMR_cmpval; break; // The register is loaded during the IRQ.
      // case '.': ++TMR_cmpval; break;

      case ',': --TMR_cmp_flt; break; // The register is loaded during the IRQ.
      case '.': ++TMR_cmp_flt; break;
      // case '>': if(TMR_cmp_flt < 1.0) TMR_cmp_flt += 0.01; break;
      // case '<': if(TMR_cmp_flt > 0.0) TMR_cmp_flt -= 0.01; break;
      // case 'k': if(TMR_cmp_flt > 0.0) TMR_cmp_flt -= 0.001; break;
      // case 'l': if(TMR_cmp_flt < 1.0) TMR_cmp_flt += 0.001; break;
      case '>': TMR_cmp_flt += 0.01; break;
      case '<': TMR_cmp_flt -= 0.01; break;
      case 'k': TMR_cmp_flt -= 0.001; break;
      case 'l': TMR_cmp_flt += 0.001; break;

      case 'b': ShowBufVals(); break;
      case 'p': ParseEdges_testsig(); break;

      case '[': NudgeCh3(1); break;
      case ']': NudgeCh3(-1); break;

      // Some manual trigger outputs.
      case '1': InitTrig(200, 10); break; // 200Hz for 10 pulses.
      case '2': InitTrig(500, 30); break; // 500Hz for 30 pulses.
      case '3': InitTrig_dur_ms(500, 20); break; // 500Hz for 20mSec

      case '|':
        // Note this nice feature of the quad timers. You can read one counter, then
        // ALL four channels count values are simultaneously stored in their HOLD
        // registers. This was used to verify that CH0 and Ch2 are absolutely in
        // sync (they are).
        int cval0 = TMR3_CNTR0;
        int cval2 = TMR3_HOLD2;
        Serial.printf("Ch0 counter val: %d, Ch2 counter val: %d\n", cval0, cval2);
        break;
    }
  }

  if(EnableLoopPrint) {
    // Serial.printf("%d IRQ %d DMA#%d DMAerr 0x%x  CAPT(%3d %3d %3d %3d)  ",
    //             ++countup, IRQcnt, TCD.NBYTES, DMA_ES, DMABuf[0], DMABuf[1], DMABuf[2], DMABuf[3]);
    // Serial.printf("%d %08lx CMP:%.3f IRQ %d DMA#%4d DMAirq# %d  CAPT(%3d %3d %3d %3d)  ",
    //             ++countup, myval, TMR_cmp_flt, IRQcnt, TCD.CITER, dma_irq_cnt, capt_vals[0], capt_vals[1], capt_vals[2], capt_vals[3]);

    // Cursor to top/left:
    sprintf(CURSOR_HOME);
    sprintf("%d CMP:%.3f IRQ %d CITER%4d DMAirq# %d  CAPT(%3d %3d %3d %3d)\n"
            "  bits (%d %d %d %d %d %d) stat: 0x%04x  ",
                ++countup,
                TMR_cmp_flt, IRQcnt, TCD.CITER, dma_irq_cnt,
                capt_vals[0].tm, capt_vals[1].tm, capt_vals[2].tm, capt_vals[3].tm,
                SIG(capt_vals[0].stat), SIG(capt_vals[1].stat), SIG(capt_vals[2].stat),
                SIG(capt_vals[3].stat), SIG(capt_vals[4].stat), SIG(capt_vals[5].stat),
                capt_vals[0].stat);


    ShowAllTMRCounts();
    //ShowAllGPTCounts();
    Serial.printf(CLREOL "\n");

    sprintf("TrigIn IRQ cnt %d (GPIO in: %d) %s\n", TrigIn_cnt, digitalReadFast(12), CLREOL);

    Serial.printf("DADDR idx's: ");
    for(int i=0; i<10; ++i)
      Serial.printf(" %2d", ((uint32_t)DADDR_save[i] - (uint32_t)&capt_vals[0]) / sizeof(capt_struct));
    Serial.printf(CLREOL "\n");

    Serial.printf("average interval, in ticks: %.3f" CLREOL, (double)ticks_sum / N_summed);
    for(int i=0; i < 16; ++i)
      Serial.printf(" %4d", TS_diff[i]);
    
    //Serial.printf(CLREOL"\n");
    sprintf("\n" CLREOSCR);
  }
}
