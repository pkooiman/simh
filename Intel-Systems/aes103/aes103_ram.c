#include "system_defs.h"


/* function prototypes */

t_stat RAM_cfg(uint16 base, uint16 size, uint8 dummy);
t_stat RAM_clr(void);
t_stat RAM_reset (DEVICE *dptr);
t_stat RAM_show_param (FILE *st, UNIT *uptr, int32 val, CONST void *desc);
uint8 RAM_get_mbyte(uint16 addr);
void RAM_put_mbyte(uint16 addr, uint8 val);

/* external function prototypes */

/* external globals */

// globals

static const char* iRAM_desc(DEVICE *dptr) {
    return "AES103 PROGRAM MEMORY";
}

/* SIMH RAM Standard I/O Data Structures */

UNIT RAM_unit = { UDATA (NULL, UNIT_BINK, 0) };

MTAB RAM_mod[] = {
    { MTAB_XTD | MTAB_VDV, 0, "PARAM", NULL, NULL, RAM_show_param, NULL, 
        "show configured parameters for RAM" },
    { 0 }
};

DEBTAB RAM_debug[] = {
    { "ALL", DEBUG_all },
    { "FLOW", DEBUG_flow },
    { "READ", DEBUG_read },
    { "WRITE", DEBUG_write },
    { "XACK", DEBUG_xack },
    { NULL }
};

DEVICE RAM_dev = {
    "RAM",              //name
    &RAM_unit,          //units
    NULL,               //registers
    RAM_mod,            //modifiers
    1,                  //numunits
    16,                 //aradix
    16,                 //awidth
    1,                  //aincr
    16,                 //dradix
    8,                  //dwidth
    NULL,               //examine
    NULL,               //deposit
    RAM_reset,          //reset
    NULL,               //boot
    NULL,               //attach
    NULL,               //detach
    NULL,               //ctxt
    DEV_DEBUG+DEV_DISABLE, //flags 
    0,                  //dctrl
    RAM_debug,          //debflags
    NULL,               //msize
    NULL,               //lname
    NULL,               //help routine
    NULL,               //attach help routine
    NULL,               //help context
    &iRAM_desc          //device description
};

/* RAM functions */

// RAM configuration

t_stat RAM_cfg(uint16 base, uint16 size, uint8 dummy)
{
    RAM_unit.capac = size;              /* set RAM size */
    RAM_unit.u3 = base;                 /* set RAM base */
    RAM_unit.filebuf = (uint8 *)calloc(size, sizeof(uint8));
    if (RAM_unit.filebuf == NULL) {
        sim_printf ("    RAM: Calloc error\n");
        return SCPE_MEM;
    }
    sim_printf("    RAM: 0%04XH bytes at base address 0%04XH\n",
        size, base);
    return SCPE_OK;
}

t_stat RAM_clr(void)
{
    RAM_unit.capac = 0;
    RAM_unit.u3 = 0;
    free(RAM_unit.filebuf);
    return SCPE_OK;
}

/* RAM reset */

t_stat RAM_reset (DEVICE *dptr)
{
    //RAM_cfg(0x0, 0x8000, 0);
    return SCPE_OK;
}


// show configuration parameters

t_stat RAM_show_param (FILE *st, UNIT *uptr, int32 val, CONST void *desc)
{
    if (uptr == NULL)
        return SCPE_ARG;
    fprintf(st, "%s at Base Address 0%04XH (%dD) for 0%04XH (%dD) Bytes ", 
        ((RAM_dev.flags & DEV_DIS) == 0) ? "Enabled" : "Disabled", 
        RAM_unit.u3, RAM_unit.u3, RAM_unit.capac, RAM_unit.capac);
    return SCPE_OK;
}

/*  get a byte from memory */

uint8 RAM_get_mbyte(uint16 addr)
{
    uint8 val;

    val = *((uint8 *)RAM_unit.filebuf + (addr - RAM_unit.u3));
    return (val & BYTEMASK);
}

/*  put a byte into memory */

void RAM_put_mbyte(uint16 addr, uint8 val)
{
    
    *((uint8 *)RAM_unit.filebuf + (addr - RAM_unit.u3)) = val & BYTEMASK;
    return;
}


