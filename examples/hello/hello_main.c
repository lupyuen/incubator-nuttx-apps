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

////#warning Test Display ////
#include "../../test_display.c" ////
