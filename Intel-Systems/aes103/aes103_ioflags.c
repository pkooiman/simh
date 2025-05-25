
#include "system_defs.h"

#define aes103flags_NAME   "AES103 IO Flags Simulator"
#define IOFLAG_MASK_TIMERENABLE 0x2


/* function prototypes */

t_stat aes103flags_svc(UNIT *uptr);
t_stat aes103flags_reset(DEVICE *dptr);
uint8 aes103flags_r4(t_bool io, uint8 data, uint8 devnum);

/* external function prototypes */



/* local globals */


static const char* aes103flags_desc(DEVICE *dptr) {
    return aes103flags_NAME;
}

/* external globals */

uint8_t aes103flags_ioflags; /* flags register */



UNIT aes103flags_unit = {
    UDATA (&aes103flags_svc, 0, 0), 1
};

REG aes103flags_reg[] = {
    { HRDATA (IOFLAGS, aes103flags_ioflags, 8) },
    { NULL }
};

DEBTAB aes103flags_debug[] = {
    { "ALL", DEBUG_all },
    { "FLOW", DEBUG_flow },
    { "READ", DEBUG_read },
    { "WRITE", DEBUG_write },
    { "XACK", DEBUG_xack },
    { NULL }
};

DEVICE aes103flags_dev = {
    "IOFLAGS",              //name 
    &aes103flags_unit,          //units 
    aes103flags_reg,            //registers 
    NULL,               //modifiers
    1,                  //numunits 
    16,                 //aradix  
    16,                 //awidth  
    1,                  //aincr  
    16,                 //dradix  
    8,                  //dwidth
    NULL,               //examine  
    NULL,               //deposit  
    & aes103flags_reset,         //reset 
    NULL,               //boot
    NULL,               //attach  
    NULL,               //detach
    NULL,               //ctxt     
    DEV_DEBUG,          //flags 
    0,                  //dctrl 
    aes103flags_debug,          //debflags
    NULL,               //msize
    NULL,               //lname
    NULL,               //help routine
    NULL,               //attach help routine
    NULL,               //help context
    & aes103flags_desc           //device description
};

/* Service routines to handle simulator functions */

/* Reset routine */

t_stat aes103flags_reset(DEVICE *dptr)
{
    aes103flags_ioflags = 0;
    
    
    return SCPE_OK;
}

void handle_flag_change()
{
    enable_timer(aes103flags_ioflags & IOFLAG_MASK_TIMERENABLE);
    
}

uint8 aes103flags_r4(t_bool io, uint8 data, uint8 devnum)
{
    if (io == 0)
        return 0;
    else
    {
        if (data & 0x8)
        {
            aes103flags_ioflags |= 1 << (data & 7);
            //sim_printf("Set ioflag %d\n", (data & 7));
        }
        else
        {
            aes103flags_ioflags &= ~(1 << (data & 7));
            //sim_printf("Clear ioflag %d\n", (data & 7));
        }


        handle_flag_change();
        return 0;
    }
}

uint8 get_ioflags()
{
    return aes103flags_ioflags;
}


/* service routine -  */

t_stat aes103flags_svc(UNIT *uptr)
{
    //Probably handle bell and tick here
    //sim_activate(&irq_unit, irq_unit.wait); /* continue poll */

    
    
    
    return SCPE_OK;
}


/* end of irq.c */
