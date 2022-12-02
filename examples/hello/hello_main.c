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

void test_render(int channels);  //  From https://github.com/lupyuen/pinephone-nuttx/blob/main/render.zig
void test_zig(void);  //  From https://github.com/lupyuen/pinephone-nuttx/blob/main/display.zig
static void a64_uart_send(int ch);
static void test_led(void);

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

  // Turn on the PinePhone Red, Green and Blue LEDs
  test_led();

  // Make sure we can still write to A64 I/O Registers  
  a64_uart_send('B');

  // Test rendering in Zig: https://github.com/lupyuen/pinephone-nuttx/blob/main/render.zig
  if (argc == 2 && argv[1][0] == '0') {  // "hello 0"
      // Render 3 UI Channels, no backlight
      test_render(0);
  } else if (argc == 2 && argv[1][0] == '1') {  // "hello 1"
      // Render 1 UI Channel
      test_render(1);
  } else if (argc == 2 && argv[1][0] == '3')  {  // "hello 3"
      // Render 3 UI Channels
      test_render(3);
  } else {
      puts("Argument must be 1 or 3");
  }

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
#include "test_display.c"

// Dump the buffer
void dump_buffer(const u8 *data, size_t len)
{
	for (int i = 0; i < len; i++) {
		printf("%02x ", data[i]);
		if ((i + 1) % 8 == 0) { printf("\n"); }
	}
	printf("\n");
}
