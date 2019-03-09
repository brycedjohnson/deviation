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

#include "common.h"
#include "interface.h"
#include "mixer.h"
#include "config/model.h"
#include "config/tx.h"
#include "telemetry.h"

#ifdef PROTO_HAS_CC2500

static void initialize(int bind);
static void redpine_init(unsigned int format);

static const char * const redpine_opts[] = {
  _tr_noop("Freq-Fine"),  "-127", "127", NULL,
  _tr_noop("Format"),  "Fast", "Slow", NULL,
  _tr_noop("Fast .1ms"),  "1", "250", NULL,
  _tr_noop("Slow 1ms"),  "1", "250", NULL,
  _tr_noop("PacketSize"),  "14", "100", NULL,
  NULL
};
enum {
    PROTO_OPTS_FREQFINE,
    PROTO_OPTS_FORMAT,
    PROTO_OPTS_LOOPTIME_FAST,
    PROTO_OPTS_LOOPTIME_SLOW,
    PROTO_OPTS_PACKETSIZE,
    LAST_PROTO_OPT,
};
ctassert(LAST_PROTO_OPT <= NUM_PROTO_OPTS, too_many_protocol_opts);

#define MAX_PACKET_SIZE 33

#define NUM_HOPS 50
// Statics are not initialized on 7e so in initialize() if necessary
static u8 calData[NUM_HOPS][3];
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

static u8 hop_data[NUM_HOPS];

unsigned format = 1;  // todo: use enum

static const u16 CRCTable[] = {
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
  0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
  0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
  0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
  0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
  0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
  0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
  0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
  0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
  0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
  0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
  0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
  0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
  0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
  0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
  0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
  0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
  0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
  0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
  0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
  0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
  0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

static u16 crc(u8 *data, u8 len)
{
    u16 crc = 0;
    for (int i = 0; i < len; i++)
        crc = (crc << 8) ^ CRCTable[((u8)(crc >> 8) ^ *data++) & 0xFF];
    return crc;
}

static void initialize_data(u8 adr)
{
    CC2500_WriteReg(CC2500_0C_FSCTRL0, fine);  // Frequency offset hack
    CC2500_WriteReg(CC2500_18_MCSM0, 0x8);
    CC2500_WriteReg(CC2500_09_ADDR, adr ? 0x03 : (fixed_id & 0xff));
    CC2500_WriteReg(CC2500_07_PKTCTRL1, 0x05);
}

static void set_start(u8 ch)
{
    CC2500_Strobe(CC2500_SIDLE);
    CC2500_WriteReg(CC2500_23_FSCAL3, calData[ch][0]);
    CC2500_WriteReg(CC2500_24_FSCAL2, calData[ch][1]);
    CC2500_WriteReg(CC2500_25_FSCAL1, calData[ch][2]);
    CC2500_WriteReg(CC2500_0A_CHANNR, hop_data[ch]);
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

    u16 lcrc = crc(&packet[3], packet_size_bind-5);
    packet[packet_size_bind-2] = lcrc >> 8;
    packet[packet_size_bind-1] = lcrc;
}

static u16 scaleForRedpine(u8 chan)
{
    s32 chan_val;

    chan_val = Channels[chan] * 15 * 100 / (2 * CHAN_MAX_VALUE) + 1024;

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

    chan[0] = scaleForRedpine(0);
    chan[1] = scaleForRedpine(1);
    chan[2] = scaleForRedpine(2);
    chan[3] = scaleForRedpine(3);

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

    if (Model.proto_opts[PROTO_OPTS_FORMAT] == 0) {
        packet[10] = Model.proto_opts[PROTO_OPTS_LOOPTIME_FAST];
    } else {
        packet[10] = Model.proto_opts[PROTO_OPTS_LOOPTIME_SLOW];
    }

    packet[11] = mixer_runtime/10;

    u16 lcrc = crc(&packet[0], 12);
    packet[12] = lcrc >> 8;
    packet[13] = lcrc;
}

static u16 redpine_cb() {
  switch (state) {
    default:
        if (state == REDPINE_BIND) {
            redpine_init(0);
        }
        if (state == REDPINE_BIND_DONE/2) {
           redpine_init(1);
        }
        set_start(49);
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
        if (format != (unsigned)Model.proto_opts[PROTO_OPTS_FORMAT]) {
            format = (unsigned)Model.proto_opts[PROTO_OPTS_FORMAT];
            redpine_init(format);
            mixer_runtime = 50;
            return 5000;
        }

        CC2500_SetTxRxMode(TX_EN);
        set_start(channr);
        CC2500_SetPower(Model.tx_power);
        CC2500_Strobe(CC2500_SFRX);
        if (mixer_sync != MIX_DONE && mixer_runtime < 2000) {
            mixer_runtime += 1;
        }
        redpine_data_frame();
        CC2500_Strobe(CC2500_SIDLE);
        CC2500_WriteData(packet, packet[0]+1);
        channr = (channr + 1) % 49;
        state = REDPINE_DATAM;
#ifndef EMULATOR
        if (Model.proto_opts[PROTO_OPTS_FORMAT] == 0) {
            return (Model.proto_opts[PROTO_OPTS_LOOPTIME_FAST]*100 - mixer_runtime);
        } else {
            return (Model.proto_opts[PROTO_OPTS_LOOPTIME_SLOW]*1000 - mixer_runtime);
        }
#else
        if (Model.proto_opts[PROTO_OPTS_FORMAT] == 0) {
            return (Model.proto_opts[PROTO_OPTS_LOOPTIME_FAST]);
        } else {
            return (Model.proto_opts[PROTO_OPTS_LOOPTIME_SLOW]);
        }
#endif
  }
  return 1;
}

// register, fast 250k, slow
static const u8 init_data[][3] = {
    {CC2500_00_IOCFG2,    0x06, 0x06},
    {CC2500_02_IOCFG0,    0x06, 0x06},
    {CC2500_03_FIFOTHR,   0x07, 0x07},
    {CC2500_06_PKTLEN,    0x1E, 0x1E},
    {CC2500_07_PKTCTRL1,  0x04, 0x04},
    {CC2500_08_PKTCTRL0,  0x01, 0x01},
    {CC2500_09_ADDR,      0x00, 0x00},
    {CC2500_0B_FSCTRL1,   0x0A, 0x0A},
    {CC2500_0C_FSCTRL0,   0x00, 0x00},
    {CC2500_0D_FREQ2,     0x5D, 0x5c},
    {CC2500_0E_FREQ1,     0x93, 0x76},
    {CC2500_0F_FREQ0,     0xB1, 0x27},
    {CC2500_10_MDMCFG4,   0x2D, 0x7B},
    {CC2500_11_MDMCFG3,   0x3B, 0x61},
    {CC2500_12_MDMCFG2,   0x73, 0x13},
    {CC2500_13_MDMCFG1,   0x23, 0x23},
    {CC2500_14_MDMCFG0,   0x56, 0x7a},  // Chan space
    {CC2500_15_DEVIATN,   0x00, 0x51},
    {CC2500_17_MCSM1,     0x0c, 0x0c},
    {CC2500_18_MCSM0,     0x18, 0x18},
    {CC2500_19_FOCCFG,    0x1D, 0x16},
    {CC2500_1A_BSCFG,     0x1C, 0x6c},
    {CC2500_1B_AGCCTRL2,  0xC7, 0x43},
    {CC2500_1C_AGCCTRL1,  0x00, 0x40},
    {CC2500_1D_AGCCTRL0,  0xB0, 0x91},
    {CC2500_21_FREND1,    0xB6, 0x56},
    {CC2500_22_FREND0,    0x10, 0x10},
    {CC2500_23_FSCAL3,    0xEA, 0xA9},
    {CC2500_24_FSCAL2,    0x0A, 0x0A},
    {CC2500_25_FSCAL1,    0x00, 0x00},
    {CC2500_26_FSCAL0,    0x11, 0x11},
    {CC2500_29_FSTEST,    0x59, 0x59},
    {CC2500_2C_TEST2,     0x88, 0x88},
    {CC2500_2D_TEST1,     0x31, 0x31},
    {CC2500_2E_TEST0,     0x0B, 0x0B},
};

static const u8 init_data_shared[][2] = {
    {CC2500_3E_PATABLE,   0xff}
};


static void redpine_init(unsigned int format) {
  CC2500_Reset();

  for (unsigned i=0; i < ((sizeof init_data) / (sizeof init_data[0])); i++) {
      CC2500_WriteReg(init_data[i][0], init_data[i][format+1]);
  }
  for (unsigned i=0; i < ((sizeof init_data_shared) / (sizeof init_data_shared[0])); i++) {
      CC2500_WriteReg(init_data_shared[i][0], init_data_shared[i][1]);
  }

  CC2500_WriteReg(CC2500_0C_FSCTRL0, fine);
  CC2500_Strobe(CC2500_SIDLE);

  // calibrate hop channels
  for (u8 c = 0; c < sizeof(hop_data); c++) {
      CC2500_Strobe(CC2500_SIDLE);
      CC2500_WriteReg(CC2500_0A_CHANNR, hop_data[c]);
      CC2500_Strobe(CC2500_SCAL);
      usleep(900);
      calData[c][0] = CC2500_ReadReg(CC2500_23_FSCAL3);
      calData[c][1] = CC2500_ReadReg(CC2500_24_FSCAL2);
      calData[c][2] = CC2500_ReadReg(CC2500_25_FSCAL1);
  }
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
    for (u8 i = 0, j = 0; i < sizeof(Model.fixed_id); ++i, j += 8) {
        rand32_r(&lfsr, (Model.fixed_id >> j) & 0xff);
    }
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

    // Used from kn_nrf24l01.c : kn_calculate_freqency_hopping_channels
    u32 idx = 0;
    u32 rnd = get_tx_id();
    #define MAX_RF_CHANNEL 255
    hop_data[idx++] = 1;
    while (idx < sizeof(hop_data)-1) {
        u32 i;
        rnd = rnd * 0x0019660D + 0x3C6EF35F;  // Randomization
        // Drop least-significant byte for better randomization. Start from 1
        u8 next_ch = (rnd >> 8) % MAX_RF_CHANNEL + 1;
        // Check that it's not duplicate nor adjacent nor channel 0 or 1
        for (i = 0; i < idx; i++) {
            u8 ch = hop_data[i];
            if ((ch <= next_ch + 1) && (ch >= next_ch - 1) && (ch > 1)) {
                break;
            }
        }
        if (i != idx) {
            continue;
        }
        hop_data[idx++] = next_ch;
    }
    hop_data[49] = 0;  // Last channel is the bind channel at hop 0

    redpine_init(format);

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

uintptr_t REDPINE_Cmds(enum ProtoCmds cmd)
{
    switch (cmd) {
        case PROTOCMD_INIT: initialize(0); return 0;
        case PROTOCMD_CHECK_AUTOBIND: return 0;  // Never Autobind
        case PROTOCMD_BIND:  initialize(1); return 0;
        case PROTOCMD_NUMCHAN: return 16L;
        case PROTOCMD_DEFAULT_NUMCHAN: return 16L;
        case PROTOCMD_CURRENT_ID: return Model.fixed_id;
        case PROTOCMD_GETOPTIONS:
            if (!Model.proto_opts[PROTO_OPTS_LOOPTIME_FAST]) {
                Model.proto_opts[PROTO_OPTS_LOOPTIME_FAST] = 20;  // if not set, default to no gain
            }
            if (!Model.proto_opts[PROTO_OPTS_LOOPTIME_SLOW]) {
                Model.proto_opts[PROTO_OPTS_LOOPTIME_SLOW] = 9;  // if not set, default to no gain
            }
            return (uintptr_t)redpine_opts;
        case PROTOCMD_RESET:
        case PROTOCMD_DEINIT:
            CLOCK_StopTimer();
            return (CC2500_Reset() ? 1 : -1);
        case PROTOCMD_TELEMETRYSTATE: return PROTO_TELEM_UNSUPPORTED;
        case PROTOCMD_CHANNELMAP: return AETRG;
        default: break;
    }
    return 0;
}
#endif
