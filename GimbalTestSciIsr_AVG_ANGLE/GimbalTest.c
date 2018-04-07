///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Connect GPIO33 (Header J1, Pin 9) to GPIO5 (Header J4, Pin 9) and uncomment a GPIO33 toggle to self test eCAP capture PWM
// Connect Encoder signal to GPIO5 (Header J4, Pin 9) to test eCAP capture PWM
// Serial  GPIO29 (Header J1, Pin 4) transmits every 10th capture

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// http://www.ti.com/lit/ug/spruh18g/spruh18g.pdf Table 1-119. PIE MUXed Peripheral Interrupt Vector Table

// http://www.my-ftm.com/2015/01/future-electronics-the-magnetic-encoder-a-robust-device-for-rotary-measurements/

#include "DSP28x_Project.h"
#include "stdbool.h"
#include <stdlib.h>

///////////////////////////////////////////////////////////
// AS5048 absolute magnetic position encoder PWM interface

// The US Digital Ma3 is 250 Hz, where as the AMS AS5048 is 1 Khz

// 1 Khz, 4119 'tick' pwm packet spec

// 12   ticks pwm high => init
// 4    ticks pwm    ? => error_n
// 4095 ticks pwm    ? => angle     1 ~ 4095, on error data section set to zero
// 8    ticks pwm low  => exit

// I think the 0 angle is (12+4)/4119 and the 360 angle is (12+4+4095)/4119
// I think this means the period of a single tick is 4119 slices of each 1/1000 sec complete reading = 244 nano second per tick

Uint32 loop_count = 0;

Uint32 ecap_on_ticks     = 0;
Uint32 ecap_on_avg_ticks  = 0;

Uint32 ecap_angle        = 0;
Uint32 ecap_avg_angle    = 0;

Uint32 ecap_isr_count    = 0;

Uint16 ecap_sci_tx;
Uint16 ecap_sci_tx_buff[4];
Uint16 ecap_sci_crc;

struct RingBuff
{
  volatile Uint16 * const buff_ptr;
  volatile const size_t   buff_size;
  volatile Uint16         push_index;
  volatile Uint16         pop_index;
  volatile Uint16         count;
};

volatile Uint16   sci_buffer[256];
struct   RingBuff SciRing = {sci_buffer, sizeof(sci_buffer), 0, 0, 0};

void   ringPush(   struct RingBuff* Ring, Uint16 push_byte);
Uint16 ringPop(    struct RingBuff* Ring);
bool   ringIsEmpty(struct RingBuff* Ring);
bool   ringIsFull( struct RingBuff* Ring);
size_t ringBytes(  struct RingBuff* Ring);
Uint16 poluluCRC(Uint16 *buffer, Uint16 size);

void ecapInit(void);
void sciAInit(void);

__interrupt void cpuTimer0Isr(void);
__interrupt void ecapIsr(void);
__interrupt void sciATxIsr(void);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void main(void)
{
  InitSysCtrl();                          // PLL, WatchDog, Periph Clocks, F2806x_SysCtrl.c

  DINT;                                   // init PIE vector table
  InitPieCtrl();                          // F2806x_PieCtrl.c
  IER = 0x0000;                           // Disable CPU isr and clear all CPU isr flags
  IFR = 0x0000;
  InitPieVectTable();                     // F2806x_PieVect.c

  InitCpuTimers();                              // F2806x_CpuTimers.c
  ConfigCpuTimer(&CpuTimer0, 1, 90000000/1000); // ConfigCpuTimer(*Timer, float Freq, float Period) => 11.11 nanosec clock_ticks
  CpuTimer0Regs.TCR.all  = 0x4001;              // write-only instructions set TSS bit = 0

  EALLOW;                                       // enable write to EALLOW protected registers

  PieVectTable.TINT0     = &cpuTimer0Isr;
  PieVectTable.ECAP1_INT = &ecapIsr;
  PieVectTable.SCITXINTA = &sciATxIsr;

  GpioCtrlRegs.GPAPUD.bit.GPIO5   = 0;    // Enable pull-up on GPIO5 (CAP1)
  GpioCtrlRegs.GPAQSEL1.bit.GPIO5 = 0;    // Synch to SYSCLKOUT GPIO5 (CAP1)
  GpioCtrlRegs.GPAMUX1.bit.GPIO5  = 3;    // Configure GPIO5 as CAP1;

  GpioCtrlRegs.GPAPUD.bit.GPIO29  = 0;    // Enable pull-up for GPIO29 (SCITXDA)
  GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;    // Configure GPIO29 for SCITXDA operation


  GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;    // launch-xl user led D9
  GpioCtrlRegs.GPBDIR.bit.GPIO34  = 1;    // launch-xl user led D9

  GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 0;    // launch-xl user led D10
  GpioCtrlRegs.GPBDIR.bit.GPIO39  = 1;    // launch-xl  user led D10

  GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 0;    // self write test capture pin
  GpioCtrlRegs.GPBDIR.bit.GPIO33  = 1;    // self write test capture pin

  EDIS;                                   // disable write to EALLOW protected registers

  PieCtrlRegs.PIEIER1.bit.INTx7 = 1;      // Enable TINT0 in the PIE: Group 1 interrupt 7
  IER |= M_INT1;                          // Enable CPU INT1 which is connected to CPU-Timer 0:

  ecapInit();
  PieCtrlRegs.PIEIER4.bit.INTx1 = 1;      // Enable eCAP INTn in PIE Group 3 interrupt 1-6
  IER |= M_INT4;                          // Enable CPU INT4 which is connected to ECAP1-4 INT

  sciAInit();
  PieCtrlRegs.PIEIER9.bit.INTx2 = 1;      // PIE Group 9, INT2. Sci0 tx
  IER |= M_INT9;                          // Enable CPU INT

  EINT;                                   // Enable Global interrupt INTM
  ERTM;                                   // Enable Global realtime interrupt DBGM

  GpioDataRegs.GPBSET.bit.GPIO34 = 1;     // set to clear active low leds
  GpioDataRegs.GPBSET.bit.GPIO39 = 1;     // set to clear active active low leds

  ///////////////////////////////////////////////////////////////
  while(1) // IDLE loop. Just sit and loop forever (optional):
  {
    loop_count = (loop_count + 1) % 90000000;

    if (!(loop_count % (90000)))
    {
      //GpioDataRegs.GPBTOGGLE.bit.GPIO33 = 1; // generate capture trigger test
      //GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // blink launch-xl RED led D9
      //GpioDataRegs.GPBTOGGLE.bit.GPIO39 = 1; // blink launch-xl BLUE led D10
    };
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

__interrupt void ecapIsr(void)
{
  ecap_on_ticks     = ((ECap1Regs.CAP1 * 100) + (ECap1Regs.CAP3 * 100)) / 2;

  ecap_on_avg_ticks = (ecap_on_ticks + ecap_on_avg_ticks - (ecap_on_avg_ticks/4096)) / 4096;
  ecap_angle        = (ecap_on_avg_ticks * 36000) / 2048;
  ecap_avg_angle    = (ecap_angle + 35*ecap_avg_angle) / 36;
  ecap_sci_tx       = (ecap_avg_angle / 10);

  ecap_isr_count = (ecap_isr_count + 1) % 10000000;

  if (!(ecap_isr_count % 10))
  {
    ecap_sci_tx_buff[0] = (ecap_sci_tx & 0xFF000000) >> 24;
    ecap_sci_tx_buff[1] = (ecap_sci_tx & 0x00FF0000) >> 16;
    ecap_sci_tx_buff[2] = (ecap_sci_tx & 0x0000FF00) >> 8;
    ecap_sci_tx_buff[3] =  ecap_sci_tx & 0x000000FF;
    ecap_sci_crc    = poluluCRC(ecap_sci_tx_buff, 4) & 0x00FF;

    ringPush(&SciRing, ecap_sci_tx_buff[0]);
    ringPush(&SciRing, ecap_sci_tx_buff[1]);
    ringPush(&SciRing, ecap_sci_tx_buff[2]);
    ringPush(&SciRing, ecap_sci_tx_buff[3]);
    ringPush(&SciRing, ecap_sci_crc);

    GpioDataRegs.GPBTOGGLE.bit.GPIO39 = 1; // blink launch-xl BLUE led D10

    SciaRegs.SCICTL2.bit.TXINTENA = 1;     // enable SCIA TX interrupt
    PieCtrlRegs.PIEIFR9.bit.INTx2 = 1;     // trigger SCIA TX interrupt
  }

  GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;   // blink launch-xl RED led D9

  ECap1Regs.ECCLR.bit.CEVT4  = 1;
  ECap1Regs.ECCLR.bit.INT    = 1;
  ECap1Regs.ECCTL2.bit.REARM = 1;
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;   // acknowledge this interrupt to receive more interrupts from group 4
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
__interrupt void cpuTimer0Isr(void)
{
  CpuTimer0.InterruptCount++;
  //GpioDataRegs.GPBTOGGLE.bit.GPIO33 = 1; // generate capture trigger test
  //GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // blink launch-xl RED led D9
  //GpioDataRegs.GPBTOGGLE.bit.GPIO39 = 1; // blink launch-xl BLUE led D10
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;  // Acknowledge this interrupt to receive more interrupts from group 1
}



///////////////////////////////////////////////////////////////////////////////
__interrupt void sciATxIsr()
{
  //GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // blink launch-xl RED led D9
  //GpioDataRegs.GPBTOGGLE.bit.GPIO39 = 1; // blink launch-xl BLUE led D10

  if(ringIsEmpty(&SciRing))
  {
    SciaRegs.SCICTL2.bit.TXINTENA = 0;

  } else
  {
    SciaRegs.SCITXBUF = ringPop(&SciRing) & 0x00FF;
  }

  SciaRegs.SCIFFTX.bit.TXFFINTCLR=1;  // Clear SCI Interrupt flag
  PieCtrlRegs.PIEACK.all|=0x100;      // Issue PIE ACK
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void sciAInit(void)
{
   SciaRegs.SCICCR.all  = 0x0007;         // 1 stop bit,  No loopback, No parity, 8 char bits, async mode, idle-line protocol
   SciaRegs.SCICTL1.all = 0x0003;         // enable TX, RX, internal SCICLK, Disable RX ERR, SLEEP, TXWAKE

   SciaRegs.SCICTL1.bit.SWRESET = 1;      // Release module from reset so it can start up

   SciaRegs.SCIFFTX.bit.TXFIFOXRESET = 0; //disable fifos, we'll use ring buffers instead for extra space
   SciaRegs.SCIFFRX.bit.RXFIFORESET  = 0;

   SciaRegs.SCIHBAUD = 0x0001;            //set up baud rate. Register value = (low speed clock freq / (baud rate * 8)) - 1. Max is about 2.8 mb/s
   SciaRegs.SCILBAUD = 0x0024;
}

/////////////////////////////////////////////////////////////////////////////////
void ecapInit(void)
{
   ECap1Regs.ECEINT.all = 0x0000; // Disable all capture interrupts
   ECap1Regs.ECCLR.all  = 0xFFFF; // Clear all CAP interrupt flags

   ECap1Regs.ECCTL1.bit.CAPLDEN     = 0;      // Disable CAP1-CAP4 register loads
   ECap1Regs.ECCTL2.bit.TSCTRSTOP   = 0;      // Make sure the counter is stopped
   ECap1Regs.ECCTL2.bit.CONT_ONESHT = 1;      // One-shot
   ECap1Regs.ECCTL2.bit.STOP_WRAP   = 3;      // Stop at 4 events

   ECap1Regs.ECCTL1.bit.CAP1POL = 1;          // Falling edge
   ECap1Regs.ECCTL1.bit.CAP2POL = 0;          // Rising edge
   ECap1Regs.ECCTL1.bit.CAP3POL = 1;          // Falling edge
   ECap1Regs.ECCTL1.bit.CAP4POL = 0;          // Rising edge

   ECap1Regs.ECCTL1.bit.CTRRST1 = 1;          // Difference operation         
   ECap1Regs.ECCTL1.bit.CTRRST2 = 1;          // Difference operation         
   ECap1Regs.ECCTL1.bit.CTRRST3 = 1;          // Difference operation         
   ECap1Regs.ECCTL1.bit.CTRRST4 = 1;          // Difference operation

   ECap1Regs.ECCTL2.bit.SYNCI_EN  = 0;        // Enable sync in
   ECap1Regs.ECCTL2.bit.SYNCO_SEL = 0;        // Pass through
   ECap1Regs.ECCTL1.bit.CAPLDEN   = 1;        // Enable capture units

   ECap1Regs.ECCTL2.bit.TSCTRSTOP = 1;        // Start Counter
   ECap1Regs.ECCTL2.bit.REARM     = 1;        // arm one-shot
   ECap1Regs.ECCTL1.bit.CAPLDEN   = 1;        // Enable CAP1-CAP4 register loads
   ECap1Regs.ECEINT.bit.CEVT4     = 1;        // 4 events = interrupt
}


////////////////////////////////////////////////////////////////////////

bool ringIsEmpty(struct RingBuff* Ring)
{
  return (Ring->count == 0);
}

//////////////////////////////////////////////////////////////////////

bool ringIsFull(struct RingBuff* Ring)
{
  return ((Ring->push_index == Ring->pop_index) && (Ring->count != 0));
}

/////////////////////////////////////////////////

size_t ringBytes(struct RingBuff* Ring)
{
  return Ring->count;
}

/////////////////////////////////////////////////////////////////////

void ringPush(struct RingBuff* Ring, Uint16 push_byte)
{
  if(ringIsFull(Ring))
  {
    return;

  } else {

    Ring->buff_ptr[Ring->push_index] = push_byte;
    Ring->push_index = (Ring->push_index + 1) % Ring->buff_size;
    Ring->count++;
    return;
  }
}

///////////////////////////////////////////////////////////////

Uint16 ringPop(struct RingBuff* Ring)
{
  if(ringIsEmpty(Ring))
  {
    return 0;

  } else {

    Uint16 pop_byte = Ring->buff_ptr[Ring->pop_index];
    Ring->pop_index = (Ring->pop_index + 1) % Ring->buff_size;
    Ring->count--;
    return pop_byte;
  }
}

///////////////////////////////////////////////////////////////////

//https://www.pololu.com/docs/0J44/6.7.6

Uint16 poluluCRC(Uint16 *buffer, Uint16 size)
{
  Uint16 crc7 = 0x0000;
  Uint16 i    = 0x0000;
  Uint16 j    = 0x0000;

  for (i = 0; i < size; i++)
  {
      crc7 = crc7 ^ (buffer[i] & 0x00FF);

    for (j = 0; j < 8; j++)
    {
      if (crc7 & 0x0001)
      {
        crc7 = crc7 ^ 0x0091;
      }

      crc7 = crc7 >> 1;
    }
  }
  return crc7;
}
