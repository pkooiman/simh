/*  irq.c: Intel Interrupt simulator

    Copyright (c) 2010, William A. Beech

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
        WILLIAM A. BEECH BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
        IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
        CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

        Except as contained in this notice, the name of William A. Beech shall not be
        used in advertising or otherwise to promote the sale, use or other dealings
        in this Software without prior written authorization from William A. Beech.

    MODIFICATIONS:

        20 Sep 20 - Original file.

    NOTES:

*/

#include "system_defs.h"

#define irq_NAME   "AES103 Interrupt Simulator"

#define IRQCOM2 7
#define IRQCOM1 6
#define IRQOPTION2 5
#define IRQOPTION1 4
#define IRQKB 3
#define IRQDISK 2
#define IRQPRINTER 1
#define IRQTIMER 0

/* function prototypes */

t_stat irq_svc(UNIT *uptr);
t_stat irq_reset(DEVICE *dptr);


/* external function prototypes */

//extern t_stat SBC_reset(DEVICE *dptr);  /* reset the iSBC80/10 emulator */
extern void set_cpuint(int32 irq_num);



/* local globals */

int32   mbirq = 0;                      /* set no multibus interrupts */
static const char* irq_desc(DEVICE *dptr) {
    return irq_NAME;
}

/* external globals */

uint8_t irqstat; /* status register */
uint8_t irqreqs; /* request lines */
uint8_t irqdisff; /* irq disable flip flop */
uint8_t irqvector; /* vector for current irq */
uint8_t latchedirq; /* last accepted irq */


/* multibus Standard SIMH Device Data Structures */

UNIT irq_unit = { 
    UDATA (&irq_svc, 0, 0), 1
};

REG irq_reg[] = { 
    { HRDATA (IRQSTAT, irqstat, 8) },
    { HRDATA (IRQREQS, irqreqs, 8) },
    { HRDATA (IRQDISFF, irqdisff, 8) },
    { HRDATA(IRQVECTOR, irqvector, 8) },
    { NULL }
};

DEBTAB irq_debug[] = {
    { "ALL", DEBUG_all },
    { "FLOW", DEBUG_flow },
    { "READ", DEBUG_read },
    { "WRITE", DEBUG_write },
    { "XACK", DEBUG_xack },
    { NULL }
};

DEVICE irq_dev = {
    "IRQ",              //name 
    &irq_unit,          //units 
    irq_reg,            //registers 
    NULL,               //modifiers
    1,                  //numunits 
    16,                 //aradix  
    16,                 //awidth  
    1,                  //aincr  
    16,                 //dradix  
    8,                  //dwidth
    NULL,               //examine  
    NULL,               //deposit  
    &irq_reset,         //reset 
    NULL,               //boot
    NULL,               //attach  
    NULL,               //detach
    NULL,               //ctxt     
    DEV_DEBUG,          //flags 
    0,                  //dctrl 
    irq_debug,          //debflags
    NULL,               //msize
    NULL,               //lname
    NULL,               //help routine
    NULL,               //attach help routine
    NULL,               //help context
    &irq_desc           //device description
};

/* Service routines to handle simulator functions */

/* Reset routine */

t_stat irq_reset(DEVICE *dptr)
{
    irqdisff = 0;
    irqreqs = 0xFF;
    irqstat = 0;
    irqvector = 0;

    
    sim_activate(&irq_unit, irq_unit.wait);
    return SCPE_OK;
}

uint8 aes103irq_r0(t_bool io, uint8 data, uint8 devnum)
{
    if (io == 0)
        return 0;
    else
    {
        //sim_printf("Set IRQ stat %x\n", data);
        set_cpuint(0);
        irqstat = data;
        irqdisff = 0;
        return 0;
    }
}

uint8 get_irqvec()
{
    return irqvector;
}

void triggerirq(int level)
{
    irqdisff = 1;
    irqvector = (7 - level) * 8;
    set_cpuint(1);
    //sim_printf("Trigger IRQ level %d\n", level);
}

/* service routine - actually does the simulated interrupts */

t_stat irq_svc(UNIT *uptr)
{
    sim_activate(&irq_unit, irq_unit.wait); /* continue poll */

    
    if (!irqdisff)
    {
        //Disable flip flop not set

        //Check in order of priority
        for (int i = 7; i >=0; i--)
        {
            if (!(irqreqs & (1 << i)))
            {
                // Found highest asserted irq
                // 
                //Current priority check enabled?
                if (!(irqstat & 8))
                {
                    //Current status compare enabled
                    if (i > ((~irqstat) & 7))
                    {
                        triggerirq(i);
                        break;
                    }
                }
                else
                {
                    triggerirq(i);
                    break;
                }
            }
        }


    }


    
    return SCPE_OK;
}

void irq_set(uint8 irqlevel)
{
    //sim_printf("Assert IRQ level %d\n", irqlevel);
    irqreqs &= ~(1 << irqlevel);
}

void irq_clear(uint8 irqlevel)
{
    //sim_printf("Desassert IRQ level %d\n", irqlevel);
    irqreqs |= (1 << irqlevel);
}

/* end of irq.c */
