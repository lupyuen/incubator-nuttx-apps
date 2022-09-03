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
static void test_tcon0(void);

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

  // Test TCON0 PinePhone Allwinner A64 Display Timing Controller
  test_tcon0();

  // Make sure we can still write to A64 I/O Registers  
  a64_uart_send('B');
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

// TCON0 Base Address for PinePhone Allwinner A64 Display Timing Controller
#define TCON0_BASE_ADDRESS 0x01C0C000

// Test TCON0 PinePhone Allwinner A64 Display Timing Controller
static void test_tcon0(void)
{
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
