/****************************************************************************
 * apps/examples/hello/hello_main.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>

static void a64_uart_send(int ch);
static void test_backlight(void);
static void test_led(void);
static void test_display(void);
void test_zig(void);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * hello_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  printf("Hello, World!!\n");

  // Make sure we can write to A64 I/O Registers  
  a64_uart_send('A');

  // Turn on the PinePhone Backlight
  test_backlight();

  // Turn on the PinePhone Red, Green and Blue LEDs
  test_led();

  // Make sure we can still write to A64 I/O Registers  
  a64_uart_send('B');

  // Test the display
  test_display();

  // Test Zig App
  test_zig();
  return 0;
}

// UART0 Base Address for PinePhone Allwinner A64 UART
#define UART_BASE_ADDRESS 0x01C28000

// Send one byte to PinePhone Allwinner A64 UART
static void a64_uart_send(int ch)
{
  // Write to UART Transmit Holding Register (UART_THR)
  // Offset: 0x0000
  uint8_t *uart_thr = (uint8_t *) (UART_BASE_ADDRESS + 0x0);

  // Bits 7 to 0: Transmit Holding Register (THR)
  // Data to be transmitted on the serial output port . Data should only be
  // written to the THR when the THR Empty (THRE) bit (UART_LSR[5]) is set.

  // If in FIFO mode and FIFOs are enabled (UART_FCR[0] = 1) and THRE is set,
  // 16 number of characters of data may be written to the THR before the
  // FIFO is full. Any attempt to write data when the FIFO is full results in the
  // write data being lost.
  *uart_thr = ch;
}

// PIO Base Address for PinePhone Allwinner A64 Port Controller (GPIO)
#define PIO_BASE_ADDRESS 0x01C20800

// Turn on the PinePhone Backlight
static void test_backlight(void)
{
  // From PinePhone Schematic: https://files.pine64.org/doc/PinePhone/PinePhone%20v1.2b%20Released%20Schematic.pdf
  // - Backlight Enable: GPIO PH10 (PH10-LCD-BL-EN)
  // - Backlight PWM:    PWM  PL10 (PL10-LCD-PWM)
  // We won't handle the PWM yet

  // Write to PH Configure Register 1 (PH_CFG1_REG)
  // Offset: 0x100
  uint32_t *ph_cfg1_reg = (uint32_t *)
    (PIO_BASE_ADDRESS + 0x100);

  // Bits 10 to 8: PH10_SELECT (Default 0x7)
  // 000: Input     001: Output
  // 010: MIC_CLK   011: Reserved
  // 100: Reserved  101: Reserved
  // 110: PH_EINT10 111: IO Disable
  *ph_cfg1_reg = 
    (*ph_cfg1_reg & ~(0b111 << 8))  // Clear the bits
    | (0b001 << 8);                 // Set the bits for Output
  printf("ph_cfg1_reg=0x%x\n", *ph_cfg1_reg);

  // Write to PH Data Register (PH_DATA_REG)
  // Offset: 0x10C
  uint32_t *ph_data_reg = (uint32_t *)
    (PIO_BASE_ADDRESS + 0x10C);

  // Bits 11 to 0: PH_DAT (Default 0)
  // If the port is configured as input, the corresponding bit is the pin state. If
  // the port is configured as output, the pin state is the same as the
  // corresponding bit. The read bit value is the value setup by software.
  // If the port is configured as functional pin, the undefined value will
  // be read.
  *ph_data_reg |= (1 << 10);  // Set Bit 10 for PH10
  printf("ph_data_reg=0x%x\n", *ph_data_reg);
}

// Turn on the PinePhone Red, Green and Blue LEDs
static void test_led(void)
{
  // From PinePhone Schematic: https://files.pine64.org/doc/PinePhone/PinePhone%20v1.2b%20Released%20Schematic.pdf
  // - Red LED:   GPIO PD18 (PD18-LED-R)
  // - Green LED: GPIO PD19 (PD19-LED-G)
  // - Blue LED:  GPIO PD20 (PD20-LED-B)

  // Write to PD Configure Register 2 (PD_CFG2_REG)
  // Offset: 0x74
  uint32_t *pd_cfg2_reg = (uint32_t *)
    (PIO_BASE_ADDRESS + 0x74);

  // Bits 10 to 8: PD18_SELECT (Default 0x7)
  // 000: Input    001: Output
  // 010: LCD_CLK  011: LVDS_VPC
  // 100: RGMII_TXD0/MII_TXD0/RMII_TXD0 101: Reserved
  // 110: Reserved 111: IO Disable
  *pd_cfg2_reg = 
    (*pd_cfg2_reg & ~(0b111 << 8))  // Clear the bits
    | (0b001 << 8);                 // Set the bits for Output

  // Bits 14 to 12: PD19_SELECT (Default 0x7)
  // 000: Input    001: Output
  // 010: LCD_DE   011: LVDS_VNC
  // 100: RGMII_TXCK/MII_TXCK/RMII_TXCK 101: Reserved
  // 110: Reserved 111: IO Disable
  *pd_cfg2_reg = 
    (*pd_cfg2_reg & ~(0b111 << 12))  // Clear the bits
    | (0b001 << 12);                 // Set the bits for Output

  // Bits 18 to 16: PD20_SELECT (Default 0x7)
  // 000: Input     001: Output
  // 010: LCD_HSYNC 011: LVDS_VP3
  // 100: RGMII_TXCTL/MII_TXEN/RMII_TXEN 101: Reserved
  // 110: Reserved  111: IO Disable
  *pd_cfg2_reg = 
    (*pd_cfg2_reg & ~(0b111 << 16))  // Clear the bits
    | (0b001 << 16);                 // Set the bits for Output
  printf("pd_cfg2_reg=0x%x\n", *pd_cfg2_reg);

  // Write to PD Data Register (PD_DATA_REG)
  // Offset: 0x7C
  uint32_t *pd_data_reg = (uint32_t *)
    (PIO_BASE_ADDRESS + 0x7C);

  // Bits 24 to 0: PD_DAT (Default 0)
  // If the port is configured as input, the corresponding bit is the pin state. If
  // the port is configured as output, the pin state is the same as the
  // corresponding bit. The read bit value is the value setup by software. If the
  // port is configured as functional pin, the undefined value will be read.
  *pd_data_reg |= (1 << 18);  // Set Bit 18 for PD18
  *pd_data_reg |= (1 << 19);  // Set Bit 19 for PD19
  *pd_data_reg |= (1 << 20);  // Set Bit 20 for PD20
  printf("pd_data_reg=0x%x\n", *pd_data_reg);
}

// Test NuttX MIPI DSI
#define TEST_NUTTX_MIPI_DSI

// NuttX MIPI DSI Driver defined in pinephone-nuttx/display.zig
// This MIPI DSI Interface is compatible with Zephyr MIPI DSI: https://github.com/zephyrproject-rtos/zephyr/blob/main/include/zephyr/drivers/mipi_dsi.h
struct device {};
ssize_t nuttx_mipi_dsi_dcs_write(const struct device *dev, uint8_t channel,
  uint8_t cmd, const void *buf, size_t len);

// Test PinePhone Display:
// https://github.com/lupyuen/pinephone-nuttx/blob/main/test_display.c
#include "../../pinephone-nuttx/test_display.c"

// Dump the buffer
void dump_buffer(const u8 *data, size_t len)
{
	for (int i = 0; i < len; i++) {
		printf("%02x ", data[i]);
		if ((i + 1) % 8 == 0) { printf("\n"); }
	}
	printf("\n");
}

#ifdef NOTUSED
/// From CRC-16-CCITT (x^16 + x^12 + x^5 + 1)
static const uint16_t crc16ccitt_tab[256] = 
{
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
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78,
};

/************************************************************************************************
 * Name: crc16ccitt_part
 *
 * Description:
 *   Continue CRC-CCITT calculation on a part of the buffer.
 *
 ************************************************************************************************/
/// Based on nuttx/libs/libc/misc/lib_crc16.c
uint16_t crc16ccitt_part(FAR const uint8_t *src, size_t len, uint16_t crc16val)
{
  size_t i;

  for (i = 0; i < len; i++)
    {
      crc16val = (crc16val >> 8)
        ^ crc16ccitt_tab[(crc16val ^ src[i]) & 0xff];
    }

  return crc16val;
}

/************************************************************************************************
 * Name: crc16ccitt
 *
 * Description:
 *   Return a 16-bit CRC-CCITT of the contents of the 'src' buffer, length 'len'
 *
 ************************************************************************************************/
/// Based on nuttx/libs/libc/misc/lib_crc16.c
uint16_t crc16ccitt(FAR const uint8_t *src, size_t len)
{
  return crc16ccitt_part(src, len, 0xffff);
}
#endif  //  NOTUSED

#ifdef NOTUSED
// TCON0 Base Address for PinePhone Allwinner A64 Display Timing Controller
#define TCON0_BASE_ADDRESS 0x01C0C000

// Test TCON0 PinePhone Allwinner A64 Display Timing Controller
static void test_tcon0(void)
{
  // TODO: Switch on Backlight PWM
  {
    // Write to TCON Global Control Register (TCON_GCTL_REG)
    // Offset: 0x0000
    uint32_t *tcon_gctl_reg = (uint32_t *)
      (TCON0_BASE_ADDRESS + 0x0000);

    // Bit 31: TCON_En
    // 0: disable 
    // 1: enable 
    // When it’s disabled, the module will be reset to idle state. 
    *tcon_gctl_reg |= (1 << 31);
    printf("tcon_gctl_reg=0x%x\n", *tcon_gctl_reg);
  }
  {
#ifdef NOTUSED
// Write to TCON FRM Control Register0 (TCON0_FRM_CTL_REG)
// Offset: 0x0010
    uint32_t *aaaa = (uint32_t *)
      (TCON0_BASE_ADDRESS + aaaa);

// Bit 31: TCON0_Frm_En
// 0:disable
// 1:enable    
    *aaaa |= (1 << 31);
    printf("aaaa=0x%x\n", *aaaa);
#endif  //  NOTUSED
  }
  {
    // Write to TCON 3D FIFO Register0 (TCON0_3D_FIFO_REG)
    // Offset: 0x003C
    uint32_t *tcon0_3d_fifo_reg = (uint32_t *) 
      (TCON0_BASE_ADDRESS + 0x003C);

    // Bit 31: 3D_FIFO_BIST_EN
    // 0: disable 
    // 1: enable 
    *tcon0_3d_fifo_reg |= (1 << 31);

    // (Might not be needed)
    // Bits 13 to 4: 3D_FIFO_HALF_LINE_SIZE
    // Note: The number of data in half line=3D_FIFO_HALF_LINE_SIZE+1 only valid when 3D_FIFO_SETTING set as 2 
    *tcon0_3d_fifo_reg =
      (*tcon0_3d_fifo_reg & ~(0b1111111111 << 4))
      | (99 << 4);

    // Bits 1 to 0: 3D_FIFO_SETTING
    // 0: by pass 
    // 1: used as normal FIFO 
    // 2: used as 3D interlace FIFO 
    // 3: reserved 
    *tcon0_3d_fifo_reg = 
      (*tcon0_3d_fifo_reg & ~(0b11 << 0))
      | (1 << 0);
    printf("tcon0_3d_fifo_reg=0x%x\n", *tcon0_3d_fifo_reg);
  }
  {
    // Write to TCON0 Control Register (TCON0_CTL_REG)
    // Offset: 0x040
    uint32_t *tcon0_ctl_reg = (uint32_t *) 
      (TCON0_BASE_ADDRESS + 0x040);

    // Bit 31: TCON0_En
    // 0: disable 
    // 1: enable 
    // Note: It executes at the beginning of the first blank line of TCON0 timin
    *tcon0_ctl_reg |= (1 << 31);
    printf("tcon0_ctl_reg=0x%x\n", *tcon0_ctl_reg);
  }
  {
#ifdef NOTUSED
// Write to TCON0 Data Clock Register (TCON0_DCLK_REG)
// Offset: 0x044
    uint32_t *aaaa = (uint32_t *)
      (TCON0_BASE_ADDRESS + aaaa);

// Bits 31 to 28: TCON0_Dclk_En
// LCLK_EN[3:0] :TCON0 clock enable    
    *aaaa = 
      (*aaaa & ~(aaaa << 28))
      | (aaaa << 28);

// Bits 6 to 0: TCON0_Dclk_Div
// Tdclk = Tsclk * DCLKDIV
// Note:
// 1.if dclk1&dclk2 used，DCLKDIV >=6
// 2.if dclk only，DCLKDIV >=1
    *aaaa = 
      (*aaaa & ~(aaaa << 0))
      | (aaaa << 0);
    printf("aaaa=0x%x\n", *aaaa);
#endif  //  NOTUSED
  }
  {
    // Write to TCON0 Basic0 Register (TCON0_BASIC0_REG)
    // Offset: 0x048
    uint32_t *tcon0_basic0_reg = (uint32_t *) 
      (TCON0_BASE_ADDRESS + 0x048);

    // Bits 27 to 16: TCON0_X
    // Panel width is X+1
    *tcon0_basic0_reg = 
      (*tcon0_basic0_reg & ~(0b111111111111 << 16))
      | (99 << 16);

    // Bits 11 to 0: TCON0_Y
    // Panel height is Y+1
    *tcon0_basic0_reg = 
      (*tcon0_basic0_reg & ~(0b111111111111 << 0))
      | (99 << 0);
    printf("tcon0_basic0_reg=0x%x\n", *tcon0_basic0_reg);
  }
  {
#ifdef NOTUSED
// Write to TCON0 CPU Panel Interface Register (TCON0_CPU_IF_REG)
// Offset: 0x060
    uint32_t *aaaa = (uint32_t *)
      (TCON0_BASE_ADDRESS + aaaa);

// Bits 31 to 28: CPU_Mode
// 0000: 18bit/256K mode
// 0010: 16bit mode0
// 0100: 16bit mode1
// 0110: 16bit mode2
// 1000: 16bit mode3
// 1010: 9bit mode
// 1100: 8bit 256K mode
// 1110: 8bit 65K mode
// xxx1: 24bit for DSI    
    *aaaa = 
      (*aaaa & ~(aaaa << 28))
      | (aaaa << 28);

// Bit 3: Trigger_FIFO_Bist_En
// 0: disable
// 1: enable
// Entry addr is 0xFF8
    *aaaa = 
      (*aaaa & ~(aaaa << 3))
      | (aaaa << 3);

// Bit 2: Trigger_FIFO_En
// 0:enable
// 1:disable
    *aaaa = 
      (*aaaa & ~(aaaa << 2))
      | (aaaa << 2);

// Bit 1: Trigger_Start
// write ‘1’ to start a frame flush, write’0’ has no effect.
// this flag indicated frame flush is running
// sofeware must make sure write ‘1’ only when this flag is ‘0’.
    *aaaa = 
      (*aaaa & ~(aaaa << 1))
      | (aaaa << 1);

// Bit 0: Trigger_En
// 0: trigger mode disable
// 1: trigger mode enable
    *aaaa = 
      (*aaaa & ~(aaaa << 0))
      | (aaaa << 0);
    printf("aaaa=0x%x\n", *aaaa);
#endif  //  NOTUSED
  }
  {
    // Write to TCON0 LVDS Panel Interface Register (TCON0_LVDS_IF_REG)
    // Offset: 0x084
    uint32_t *tcon0_lvds_if_reg = (uint32_t *) 
      (TCON0_BASE_ADDRESS + 0x084);

    // Bit 31: TCON0_LVDS_En
    // 0: disable
    // 1: enable
    *tcon0_lvds_if_reg |= (1 << 31);
    printf("tcon0_lvds_if_reg=0x%x\n", *tcon0_lvds_if_reg);
  }

  for (int i = 0; i < 1000 * 1000; i++) {
    // Write to TCON0 CPU Panel Write Data Register (TCON0_CPU_WR_REG)
    // Offset: 0x064
    uint32_t *tcon0_cpu_wr_reg = (uint32_t *) 
      (TCON0_BASE_ADDRESS + 0x064);

    // Bits 23 to 0: Data_Wr
    // data write on 8080 bus, launch a write operation on 8080 bus
    *tcon0_cpu_wr_reg =
      (i << 16)
      | (i << 8)
      | i;
  }
}
#endif  //  NOTUSED
