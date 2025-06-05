/*  imds_sys.c: multibus system interface

    Copyright (c) 2017, William A. Beech

    Permission is hereby granted, free of charge, to any person obtaining a
    copy of this software and associated documentation files (the "Software"),
    to deal in the Software without restriction, including without limitation
    the rights to use, copy, modify, merge, publish, distribute, sublicense,
    and/or sell copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
    William A. Beech BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
    IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
    CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

    Except as contained in this notice, the name of William A. Beech shall not be
    used in advertising or otherwise to promote the sale, use or other dealings
    in this Software without prior written authorization from William A. Beech.

    28 Oct 17 - Original file.
    
    18 May 19 - Equipment Emulated:
    Model 210 chassis.
    Integrated processor board (IPB).
    Parallel I/O board (PIO).
    ROM-resident system monitor.
    Auxiliary ROM board with MCS-80/MCS-85 assembler
    and text editor.
    
*/

#include "system_defs.h"

extern DEVICE i8080_dev;
extern REG i8080_reg[];
extern DEVICE aes103disk_dev;
extern DEVICE RAM_dev;
extern DEVICE aes103mem_dev;
extern DEVICE port_dev;
extern DEVICE irq_dev;
extern DEVICE aes103vid_dev;
extern DEVICE aes103timer_dev;
extern DEVICE aes103flags_dev;
extern DEVICE aes103keyboard_dev;
extern int32 saved_PC;




/* SCP data structures

   sim_name             simulator name string
   sim_PC               pointer to saved PC register descriptor
   sim_emax             number of words needed for examine
   sim_devices          array of pointers to simulated devices
   sim_stop_messages    array of pointers to stop messages
*/

char sim_name[] = "AES 103";

REG *sim_PC = &i8080_reg[0];

int32 sim_emax = 4;

DEVICE *sim_devices[] = {
    &i8080_dev,
    &port_dev,
    &irq_dev,
    &aes103disk_dev,
    &RAM_dev,
    &aes103mem_dev,
    &aes103vid_dev,
    &aes103timer_dev,
    & aes103flags_dev,
    & aes103keyboard_dev,
    NULL
};

const char *sim_stop_messages[SCPE_BASE] = {
    "Unknown error",
    "Reserved Instruction",
    "HALT instruction",
    "Breakpoint",
    "Invalid Opcode",
    "Unknown I/O Instruction",
    "Invalid Memory",
    "XACK Error"
};




//#define MAXMEMSIZE 0x8000
//unsigned char M[MAXMEMSIZE];
//
//uint8 get_mbyte(uint16 addr)
//{
//    return M[addr];
//}
//
//uint16 get_mword(uint32 addr)
//{
//    uint16 val;
//    if (addr >= MAXMEMSIZE)
//        return 0;
//    val = get_mbyte(addr);
//    val |= (get_mbyte(addr + 1) << 8);
//    return val;
//}
//
//void put_mbyte(uint32 addr, uint8 val)
//{
//    if (addr >= MAXMEMSIZE)
//        return ;
//    M[addr] = val;
//}
//
//void put_mword(uint32 addr, uint16 val)
//{
//    put_mbyte(addr, val & 0xff);
//    put_mbyte(addr + 1, val >> 8);
//}
