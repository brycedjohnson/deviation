/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Deviation is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Deviation.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifdef MODULAR
  //Allows the linker to properly relocate
  #define REDPINE_Cmds PROTO_Cmds
  #pragma long_calls
#endif
#include "common.h"
#include "interface.h"
#include "mixer.h"
#include "config/model.h"
#include "config/tx.h"
#include "telemetry.h"

#ifdef MODULAR
  //Some versions of gcc applythis to definitions, others to calls
  //So just use long_calls everywhere
  //#pragma long_calls_off
  extern unsigned _data_loadaddr;
  const unsigned long protocol_type = (unsigned long)&_data_loadaddr;
#endif

#ifdef PROTO_HAS_CC2500

#include "iface_cc2500.h"

static const char * const redpine_opts[] = {
  _tr_noop("AD2GAIN"),  "0", "2000", "655361", NULL,       // big step 10, little step 1
  _tr_noop("Freq-Fine"),  "-127", "127", NULL,
  _tr_noop("Format"),  "Fast", "Slow", NULL,
  _tr_noop("LoopIn.1ms"),  "1", "100", NULL,
  _tr_noop("PacketSize"),  "14", "100", NULL,
  NULL
};
enum {
    PROTO_OPTS_AD2GAIN,
    PROTO_OPTS_FREQFINE,
    PROTO_OPTS_FORMAT,
    PROTO_OPTS_LOOPTIME,
    PROTO_OPTS_PACKETSIZE,
    LAST_PROTO_OPT,
};
ctassert(LAST_PROTO_OPT <= NUM_PROTO_OPTS, too_many_protocol_opts);

#define MAX_PACKET_SIZE 33

// Statics are not initialized on 7e so in initialize() if necessary
static u8 calData[48][3];
static u8 channr;
static u8 ctr;
static s8 fine;
static u8 packet_size;

static enum {
  REDPINE_BIND,
#ifndef EMULATOR
  REDPINE_BIND_DONE = 1000,
#else
  REDPINE_BIND_DONE = 50,
#endif
  REDPINE_DATAM,
  REDPINE_DATA1
} state;

static u16 fixed_id;
static u8 packet[MAX_PACKET_SIZE];
static u16 mixer_runtime;

static const u8 hop_data[] = {
  0x02, 0xD4, 0xBB, 0xA2, 0x89,
  0x70, 0x57, 0x3E, 0x25, 0x0C,
  0xDE, 0xC5, 0xAC, 0x93, 0x7A,
  0x61, 0x48, 0x2F, 0x16, 0xE8,
  0xCF, 0xB6, 0x9D, 0x84, 0x6B,
  0x52, 0x39, 0x20, 0x07, 0xD9,
  0xC0, 0xA7, 0x8E, 0x75, 0x5C,
  0x43, 0x2A, 0x11, 0xE3, 0xCA,
  0xB1, 0x98, 0x7F, 0x66, 0x4D,
  0x34, 0x1B, 0x00, 0x1D, 0x03
};


extern u16 frsky_CRCTable[];
u16 frsky_crc(u8 *data, u8 len);

static void initialize_data(u8 adr)
{
  CC2500_WriteReg(CC2500_0C_FSCTRL0, fine);  // Frequency offset hack
  CC2500_WriteReg(CC2500_18_MCSM0,    0x8);
  CC2500_WriteReg(CC2500_09_ADDR, adr ? 0x03 : (fixed_id & 0xff));
  CC2500_WriteReg(CC2500_07_PKTCTRL1,0x05);
}

static void set_start(u8 ch)
{
  CC2500_Strobe(CC2500_SIDLE);
  CC2500_WriteReg(CC2500_23_FSCAL3, calData[ch][0]);
  CC2500_WriteReg(CC2500_24_FSCAL2, calData[ch][1]);
  CC2500_WriteReg(CC2500_25_FSCAL1, calData[ch][2]);
  CC2500_WriteReg(CC2500_0A_CHANNR, ch == 47 ? 0 : hop_data[ch]);
}

#define RXNUM 16
static void redpine_build_bind_packet()
{
    const u8 packet_size_bind = 30;

    packet[0] = 29;
    packet[1] = 0x03;
    packet[2] = 0x01;
    packet[3] = fixed_id;
    packet[4] = fixed_id >> 8;
    int idx = ((state - REDPINE_BIND) % 10) * 5;
    packet[5] = idx;
    packet[6] = hop_data[idx++];
    packet[7] = hop_data[idx++];
    packet[8] = hop_data[idx++];
    packet[9] = hop_data[idx++];
    packet[10] = hop_data[idx++];
    packet[11] = 0x02;
    packet[12] = RXNUM;

    memset(&packet[13], 0, packet_size_bind-15);

    u16 lcrc = frsky_crc(&packet[3], packet_size_bind-5);
    packet[packet_size_bind-2] = lcrc >> 8;
    packet[packet_size_bind-1] = lcrc;
}

#define STICK_SCALE    751 
static u16 scaleForPXX(u8 chan)
{ 
//mapped 860,2140(125%) range to 64,1984(PXX values);
//  return (u16)(((Servo_data[i]-PPM_MIN)*3)>>1)+64;
// 0-2047, 0 = 817, 1024 = 1500, 2047 = 2182
    s32 chan_val;

    chan_val = Channels[chan];
    chan_val = chan_val * STICK_SCALE / CHAN_MAX_VALUE + 1024;

    if (chan_val > 2046)   chan_val = 2046;
    else if (chan_val < 1) chan_val = 1;

    return chan_val;
}


#define GET_FLAG(ch, mask) (Channels[ch] > 0 ? mask : 0)

// For code readability
enum {
    CHANNEL1 = 0,
    CHANNEL2,
    CHANNEL3,
    CHANNEL4,
    CHANNEL5,
    CHANNEL6,
    CHANNEL7,
    CHANNEL8,
    CHANNEL9,
    CHANNEL10,
    CHANNEL11,
    CHANNEL12,
    CHANNEL13,
    CHANNEL14,
    CHANNEL15,    
    CHANNEL16
};


static void redpine_data_frame() {
    u16 chan[4];
 
    packet_size = Model.proto_opts[PROTO_OPTS_PACKETSIZE];

    memset(&packet[0], 0, packet_size);

    packet[0] = packet_size-1;
    packet[1] = fixed_id;
    packet[2] = fixed_id >> 8;

    chan[0] = scaleForPXX(0);
    chan[1] = scaleForPXX(1);
    chan[2] = scaleForPXX(2);
    chan[3] = scaleForPXX(3);

    packet[3] = chan[0];
    packet[4] = (((chan[0] >> 8) & 0x07) | (chan[1] << 4)) | GET_FLAG(CHANNEL5, 0x08);
    packet[5] = ((chan[1] >> 4) & 0x7F) | GET_FLAG(CHANNEL6, 0x80);
    packet[6] = chan[2];
    packet[7] = (((chan[2] >> 8) & 0x07) | (chan[3] << 4))  | GET_FLAG(CHANNEL7, 0x08); 
    packet[8] = ((chan[3] >> 4) & 0x7F) | GET_FLAG(CHANNEL8, 0x80);
    packet[9] = GET_FLAG(CHANNEL9, 0x01)
            | GET_FLAG(CHANNEL10, 0x02)
            | GET_FLAG(CHANNEL11, 0x04)
            | GET_FLAG(CHANNEL12, 0x08)
            | GET_FLAG(CHANNEL13, 0x10)
            | GET_FLAG(CHANNEL14, 0x20)
            | GET_FLAG(CHANNEL15, 0x40)
            | GET_FLAG(CHANNEL16, 0x80);
            
    packet[10] = Model.proto_opts[PROTO_OPTS_LOOPTIME]; 
    packet[11] = mixer_runtime/10;

    u16 lcrc = frsky_crc(&packet[0], 12);
    packet[12] = lcrc >> 8;
    packet[13] = lcrc;

}

static u16 redpine_cb() {

  switch(state) {
    default:
        set_start(47);
        CC2500_SetPower(Model.tx_power);
        CC2500_Strobe(CC2500_SFRX);
        redpine_build_bind_packet();
        CC2500_Strobe(CC2500_SIDLE);
        CC2500_WriteData(packet, packet[0]+1);
        state++;
#ifndef EMULATOR
        return 9000;
#else
        return 90;
#endif
    case REDPINE_BIND_DONE:
        PROTOCOL_SetBindState(0);
        initialize_data(0);
        channr = 0;
        state++;
break;

    case REDPINE_DATAM:
#ifndef EMULATOR
        CLOCK_RunMixer();    // clears mixer_sync, which is then set when mixer update complete
        state = REDPINE_DATA1;
        return mixer_runtime;
#else
        return 5;
#endif
          
    case REDPINE_DATA1:
        if (fine != (s8)Model.proto_opts[PROTO_OPTS_FREQFINE]) {
            fine = (s8)Model.proto_opts[PROTO_OPTS_FREQFINE];
            CC2500_WriteReg(CC2500_0C_FSCTRL0, fine);
        }
        CC2500_SetTxRxMode(TX_EN);
        set_start(channr);
        CC2500_SetPower(Model.tx_power);
        CC2500_Strobe(CC2500_SFRX);
        if (mixer_sync != MIX_DONE && mixer_runtime < 2000) { mixer_runtime += 1; }
        redpine_data_frame();
        CC2500_Strobe(CC2500_SIDLE);
        CC2500_WriteData(packet, packet[0]+1);
        channr = (channr + 1) % 47;
        state = REDPINE_DATAM;
#ifndef EMULATOR
      return (Model.proto_opts[PROTO_OPTS_LOOPTIME]*100 - mixer_runtime);
#else
      return Model.proto_opts[PROTO_OPTS_LOOPTIME];
#endif


  }
  return 1;
}

// register, fast 250k, slow
static const u8 init_data[][3] = {
    {CC2500_00_IOCFG2,    0x06,  0x06},
    {CC2500_02_IOCFG0,    0x06,  0x06},    
    {CC2500_06_PKTLEN,    0x1E,  0x23},
    {CC2500_07_PKTCTRL1,  0x04,  0x04},
    {CC2500_08_PKTCTRL0,  0x05,  0x01},
    {CC2500_0B_FSCTRL1,   0x0A,  0x08},
    {CC2500_0C_FSCTRL0,   0x00,  0x00},
    {CC2500_0D_FREQ2,     0x5D,  0x5c},
    {CC2500_0E_FREQ1,     0x93,  0x80},
    {CC2500_0F_FREQ0,     0xB1,  0x00},
    {CC2500_10_MDMCFG4,   0x2D,  0x7B},
    {CC2500_11_MDMCFG3,   0x3B,  0xF8},
    {CC2500_12_MDMCFG2,   0x73,  0x03},
    {CC2500_13_MDMCFG1,   0x22,  0x23},
    {CC2500_14_MDMCFG0,   0xF8,  0x7a},
    {CC2500_15_DEVIATN,   0x00,  0x53},
    {CC2500_17_MCSM1,     0x0c,  0x0E},
    {CC2500_18_MCSM0,     0x18,  0x18},
    {CC2500_3E_PATABLE,   0xff,  0xff},
};

// register, value
static const u8 init_data_shared[][2] = {
    {CC2500_03_FIFOTHR,   0x07},
    {CC2500_09_ADDR,      0x00},
    {CC2500_19_FOCCFG,    0x1D},
    {CC2500_1A_BSCFG,     0x1C},
    {CC2500_1B_AGCCTRL2,  0xC7},
    {CC2500_1C_AGCCTRL1,  0x00},
    {CC2500_1D_AGCCTRL0,  0xB0},
    {CC2500_21_FREND1,    0xB6},
    {CC2500_22_FREND0,    0x10},
    {CC2500_23_FSCAL3,    0xa9},
    {CC2500_24_FSCAL2,    0x0A},
    {CC2500_25_FSCAL1,    0x00},
    {CC2500_26_FSCAL0,    0x11},
    {CC2500_29_FSTEST,    0x59},
    {CC2500_2C_TEST2,     0x88},
    {CC2500_2D_TEST1,     0x31},
    {CC2500_2E_TEST0,     0x0B},
};

static void redpine_init() {
  CC2500_Reset();

  unsigned format = Model.proto_opts[PROTO_OPTS_FORMAT] + 1;

  for (unsigned i=0; i < ((sizeof init_data) / (sizeof init_data[0])); i++)
      CC2500_WriteReg(init_data[i][0], init_data[i][format]);
  for (unsigned i=0; i < ((sizeof init_data_shared) / (sizeof init_data_shared[0])); i++)
      CC2500_WriteReg(init_data_shared[i][0], init_data_shared[i][1]);

  CC2500_WriteReg(CC2500_0C_FSCTRL0, fine);
  CC2500_Strobe(CC2500_SIDLE);

  //calibrate hop channels
  for (u8 c = 0; c < 47; c++) {
      CC2500_Strobe(CC2500_SIDLE);
      CC2500_WriteReg(CC2500_0A_CHANNR, hop_data[c]);
      CC2500_Strobe(CC2500_SCAL);
      usleep(900);
      calData[c][0] = CC2500_ReadReg(CC2500_23_FSCAL3);
      calData[c][1] = CC2500_ReadReg(CC2500_24_FSCAL2);
      calData[c][2] = CC2500_ReadReg(CC2500_25_FSCAL1);
  }
  CC2500_Strobe(CC2500_SIDLE);
  CC2500_WriteReg(CC2500_0A_CHANNR, 0x00);
  CC2500_Strobe(CC2500_SCAL);
  usleep(900);
  calData[47][0] = CC2500_ReadReg(CC2500_23_FSCAL3);
  calData[47][1] = CC2500_ReadReg(CC2500_24_FSCAL2);
  calData[47][2] = CC2500_ReadReg(CC2500_25_FSCAL1);
}



// Generate internal id from TX id and manufacturer id (STM32 unique id)
static int get_tx_id()
{
    u32 lfsr = 0x7649eca9ul;

    u8 var[12];
    MCU_SerialNumber(var, 12);
    for (int i = 0; i < 12; ++i) {
        rand32_r(&lfsr, var[i]);
    }
    for (u8 i = 0, j = 0; i < sizeof(Model.fixed_id); ++i, j += 8)
        rand32_r(&lfsr, (Model.fixed_id >> j) & 0xff);
    return rand32_r(&lfsr, 0);
}

static void initialize(int bind)
{
    CLOCK_StopTimer();
    mixer_runtime = 50;

    // initialize statics since 7e modules don't initialize
    fine = Model.proto_opts[PROTO_OPTS_FREQFINE];
    fixed_id = (u16) get_tx_id();
    channr = 0;
    ctr = 0;
    u32 seed = get_tx_id();

    redpine_init();
    CC2500_SetTxRxMode(TX_EN);  // enable PA

    if (bind) {
        PROTOCOL_SetBindState(0xFFFFFFFF);
        state = REDPINE_BIND;
        initialize_data(1);
    } else {
        state = REDPINE_DATA1;
        initialize_data(0);
    }

#ifndef EMULATOR
    CLOCK_StartTimer(10000, redpine_cb);
#else
    CLOCK_StartTimer(100, redpine_cb);
#endif
}

const void *REDPINE_Cmds(enum ProtoCmds cmd)
{
    switch(cmd) {
        case PROTOCMD_INIT: initialize(0); return 0;
        case PROTOCMD_CHECK_AUTOBIND: return 0; //Never Autobind
        case PROTOCMD_BIND:  initialize(1); return 0;
        case PROTOCMD_NUMCHAN: return (void *)16L;
        case PROTOCMD_DEFAULT_NUMCHAN: return (void *)16L;
        case PROTOCMD_CURRENT_ID: return Model.fixed_id ? (void *)((unsigned long)Model.fixed_id) : 0;
        case PROTOCMD_GETOPTIONS:
            if (!Model.proto_opts[PROTO_OPTS_AD2GAIN]) Model.proto_opts[PROTO_OPTS_AD2GAIN] = 100;  // if not set, default to no gain
            if (!Model.proto_opts[PROTO_OPTS_LOOPTIME]) Model.proto_opts[PROTO_OPTS_LOOPTIME] = 15;  // if not set, default to no gain
            return redpine_opts;
        case PROTOCMD_RESET:
        case PROTOCMD_DEINIT:
            CLOCK_StopTimer();
            return (void *)(CC2500_Reset() ? 1L : -1L);
        case PROTOCMD_TELEMETRYSTATE: return (void *) PROTO_TELEM_UNSUPPORTED; 
        case PROTOCMD_CHANNELMAP: return AETRG;
        default: break;
    }
    return 0;
}
#endif
