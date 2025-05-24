
#include "system_defs.h"

#define aes103timer_NAME   "AES103 Timer Simulator"

#define TIMERPERIOD 5000
#define IRQTIMER 0
#define IOFLAG_MASK_TIMERENABLE 0x2

/* function prototypes */

t_stat aes103timer_svc(UNIT *uptr);
t_stat aes103timer_reset(DEVICE *dptr);


/* external function prototypes */

//extern t_stat SBC_reset(DEVICE *dptr);  /* reset the iSBC80/10 emulator */
extern void set_cpuint(int32 irq_num);


/* local globals */


static const char* aes103timer_desc(DEVICE *dptr) {
    return aes103timer_NAME;
}



/* external globals */


UNIT aes103timer_unit = {
    UDATA (&aes103timer_svc, 0, 0), 1
};

uint8 aes103timer_enabled;


REG aes103timer_reg[] = {
    { HRDATA(TIMERENABLED, aes103timer_enabled, 8) },
    { NULL }
};

DEBTAB aes103timer_debug[] = {
    { "ALL", DEBUG_all },
    { "FLOW", DEBUG_flow },
    { "READ", DEBUG_read },
    { "WRITE", DEBUG_write },
    { "XACK", DEBUG_xack },
    { NULL }
};

DEVICE aes103timer_dev = {
    "TIMER",              //name 
    & aes103timer_unit,          //units 
    aes103timer_reg,            //registers 
    NULL,               //modifiers
    1,                  //numunits 
    16,                 //aradix  
    16,                 //awidth  
    1,                  //aincr  
    16,                 //dradix  
    8,                  //dwidth
    NULL,               //examine  
    NULL,               //deposit  
    & aes103timer_reset,         //reset 
    NULL,               //boot
    NULL,               //attach  
    NULL,               //detach
    NULL,               //ctxt     
    DEV_DEBUG,          //flags 
    0,                  //dctrl 
    aes103timer_debug,          //debflags
    NULL,               //msize
    NULL,               //lname
    NULL,               //help routine
    NULL,               //attach help routine
    NULL,               //help context
    & aes103timer_desc           //device description
};

/* Service routines to handle simulator functions */

/* Reset routine */

t_stat aes103timer_reset(DEVICE *dptr)
{
    
    
    aes103timer_enabled = 0;
    return SCPE_OK;
}

uint8 aes103timer_r2(t_bool io, uint8 data, uint8 devnum)
{
    if (io == 0)
    {
        irq_clear(IRQTIMER);
        //sim_printf("Clear timer irq\n");
    }

    return 0;
    
}

void enable_timer(uint8 enable)
{
    if (enable)
    {
        if (!aes103timer_enabled)
        {
            sim_printf("Enable timer\n");
            sim_activate(&aes103timer_unit, TIMERPERIOD);
        }
    }
    else
    {
        if (aes103timer_enabled)
        {
            sim_cancel(&aes103timer_unit);
            sim_printf("Disable timer\n");
        }
    }

    aes103timer_enabled = enable;
}

/* service routine*/

t_stat aes103timer_svc(UNIT *uptr)
{
    sim_activate(&aes103timer_unit, TIMERPERIOD);

    //sim_printf("Timer IRQ\n");
    irq_set(IRQTIMER);

    
    return SCPE_OK;
}

