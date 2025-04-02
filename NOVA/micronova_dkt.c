/* micronova_dkt.c: MICRONOVA floppy (6038/6039) simulator

   Copyright (c) 1993-2008, Robert M. Supnik

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
   ROBERT M SUPNIK BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
   IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

   Except as contained in this notice, the name of Robert M Supnik shall not be
   used in advertising or otherwise to promote the sale, use or other dealings
   in this Software without prior written authorization from Robert M Supnik.

   dkt          Micronova 6038/6039

   04-Apr-25    PKO     Module created based on nova_dkp.c
   27-Apr-12    RMS     Changed ??? string digraphs to ?, per C rules
   04-Jul-04    BKR     device name changed to DG's DKP from DEC's DP,
                        DEV_SET/CLR/INTR macro use started,
                        fixed 'P' pulse code and secret quirks,
                        added 6097 diag and size support,
                        fixed losing unit drive type during unit change,
                        tightened sector size determination calculations,
                        controller DONE flag handling fixed,
                        fixed cylinder overflow test error,
                        seek error code fixed,
                        restructured dkp_go() and dkp_svc() routines
                        (for known future fixes needed),
                        fixed DIA status calculation,
                        fixed DKP read/write loop to properly emulate DG cylinder and sector overflows,
                        added trace facility,
                        changed 'stime' calculation to force delay time if no cylinders are crossed
                        (this fixes some DG code that assumes disk seek takes some time),
                        fixed boot code to match DG hardware standard
   04-Jan-04    RMS     Changed attach routine to use sim_fsize
   28-Nov-03    CEO     Boot from DP now puts device address in SR
   24-Nov-03    CEO     Added support for disk sizing on 6099/6103
   19-Nov-03    CEO     Corrected major DMA Mapping bug
   25-Apr-03    RMS     Revised autosizing
   08-Oct-02    RMS     Added DIB
   06-Jan-02    RMS     Revised enable/disable support
   30-Nov-01    RMS     Added read only unit, extended SET/SHOW support
   24-Nov-01    RMS     Changed FLG, CAPAC to arrays
   26-Apr-01    RMS     Added device enable/disable support
   12-Dec-00    RMS     Added Eclipse support from Charles Owen
   15-Oct-00    RMS     Editorial changes
   14-Apr-99    RMS     Changed t_addr to unsigned
   15-Sep-97    RMS     Fixed bug in DIB/DOB for new disks
   15-Sep-97    RMS     Fixed bug in cylinder extraction (found by Charles Owen)
   10-Sep-97    RMS     Fixed bug in error reporting (found by Charles Owen)
   25-Nov-96    RMS     Defaulted to autosize
   29-Jun-96    RMS     Added unit disable support
*/

/* The Micronova 6038 floppy controller is fundamentally incompatible with the NOVA "DKP" family of controllers
   The 6038 can only read or write whatever sector is coming up next under the head. The CPU needs to
   issue a "read current address (preamble)" command which tells it the sector currently under the head; it can then issue a
   read for the sector it needs when the sector just prior to the wanted sector is under the head.

   A number of Micronova programs speed up reading and writing by skipping the "read current address" command
   when reading or writing a number of contiguous sectors. Depending on the programs' timimng, after reading sector N,
   they will issue a read/write for either sector N+1 or N+2.

   For the simulator, a timer is used that fires every time a new sector needs to be under the head.
   Read/write and read premable commands are cued and scheduled when the sector timer fires.
   
*/

#include "nova_defs.h"

#define DKT_NUMDR       2                               /* #drives */
#define DKT_NUMWD       256                             /* words/sector */
#define UNIT_V_DTYPE    (UNIT_V_UF + 0)                 /* disk type */
#define UNIT_M_DTYPE    017
#define UNIT_V_AUTO     (UNIT_V_UF + 5)                 /* autosize */
#define UNIT_DTYPE      (UNIT_M_DTYPE << UNIT_V_DTYPE)
#define UNIT_AUTO       (1 << UNIT_V_AUTO)
#define GET_DTYPE(x)    (((x) >> UNIT_V_DTYPE) & UNIT_M_DTYPE)
#define FUNC            u3                              /* function */
#define CYL             u4                              /* on cylinder */
#define SECT             u5                              /* on sector */
#define DKT_TIMER        (DKT_NUMDR)

#define GET_SA(cy,sf,sc,t) (((((cy)*drv_tab[t].surf)+(sf))* \
    drv_tab[t].sect)+(sc))



/* 6038 specify command register
*/

#define SC_6038_V_CMD      0                               /* command */
#define SC_6038_M_CMD      0377
#define SC_6038_V_SECTOR   8                               /* sector nr */
#define SC_6038_M_SECTOR   07
#define SC_6038_V_HITRK    14                              /* Track > 43 */
#define SC_6038_M_HITRK    01
#define SC_6038_V_UNIT     15                              /* unit */
#define SC_6038_M_UNIT     01

#define SC_6038_GET_SECT(x)      (((x) >> SC_6038_V_SECTOR) & SC_6038_M_SECTOR)
#define SC_6038_GET_UNIT(x)      (((x) >> SC_6038_V_UNIT) & SC_6038_M_UNIT)
#define SC_6038_GET_CMD(x)       (((x) >> SC_6038_V_CMD) & SC_6038_M_CMD)
#define SC_6038_GET_HITRK(x)     (((x) >> SC_6038_V_HITRK) & SC_6038_M_HITRK)
#define SC_6038_SETTLE           000
#define SC_6038_STEPOUT          001
#define SC_6038_STEPIN           002
#define SC_6038_READPREAMB       010
#define SC_6038_READNEXT         020
#define SC_6038_WRITENEXT        040
#define SC_6038_FORMAT0          240
#define SC_6038_FORMATNEXT       241

/* 6038 Memory address counter register
*/
#define MA_6038_V_ADDR      0                               /* Address */
#define MA_6038_M_ADDR      077777
#define MA_6038_GET_ADDR(x) (((x) >> MA_6038_V_ADDR) & MA_6038_M_ADDR)
#define MA_6038_SET_ADDR(x) (((x) & MA_6038_M_ADDR) << MA_6038_V_ADDR)

/* 6038 Current address register
*/
#define CA_6038_V_SECTOR      2                               /* Sector */
#define CA_6038_M_SECTOR      07
#define CA_6038_V_TRACK       8                               /* Track */
#define CA_6038_M_TRACK      0177
#define CA_6038_SETADDR(trk, sect) dkt_6038_ca = (((trk & CA_6038_M_TRACK) << CA_6038_V_TRACK) | ((sect & CA_6038_M_SECTOR) << CA_6038_V_SECTOR))

/* 6038 Status */
#define STA_6038_NOTREADY        0100000                        /* Drive not ready */
#define STA_6038_TRACK0          0040000                        /* Head on track 0 */
#define STA_6038_HEADON          0020000                        /* Head loaded */
#define STA_6038_RES_BIT3        0010000                        /* reserved */
#define STA_6038_RES_BIT4        0004000                        /* reserved */
#define STA_6038_RES_BIT5        0002000                        /* reserved */
#define STA_6038_WRITEPROT       0001000                        /* Disk write protected */
#define STA_6038_UNIT            0000400                        /* Drive unit number */
#define STA_6038_DRIVESTAT       0000200                        /* Selected drive not ready at some point since last Clear */
#define STA_6038_RES_BIT9        0000100                        /* reserved */
#define STA_6038_ILLEGAL         0000040                        /* Illegal command */
#define STA_6038_SECTORERR       0000020                        /* Sector address mismatch */
#define STA_6038_CHECKWORDERR    0000010                        /* Checkword mismatch */
#define STA_6038_DATALATE        0000004                        /* Data channel did not repsond in time */
#define STA_6038_WRITEFAULT      0000002                        /* Error during write */
#define STA_6038_ERROR           0000001                        /* Any of bits illegal to writefault is 1 or the Validity (??) flag is set to 1. */


#define STA_EFLGS       (STA_6038_ERROR | STA_6038_DATALATE | STA_6038_CHECKWORDERR | STA_6038_SECTORERR | \
                         STA_6038_WRITEFAULT | STA_6038_ILLEGAL | STA_6038_DRIVESTAT)             /* All error flags */

#define STA_FLGS_SCLR       (STA_6038_ERROR | STA_6038_DATALATE | STA_6038_CHECKWORDERR | STA_6038_SECTORERR | \
                         STA_6038_WRITEFAULT | STA_6038_ILLEGAL )             /* Error flags cleared by set pulse*/

#define STA_FLGS_GENERAL_ERROR (STA_6038_ILLEGAL | STA_6038_SECTORERR | STA_6038_CHECKWORDERR | STA_6038_DATALATE | STA_6038_WRITEFAULT)

#define STA_6038_V_UNIT 8
#define STA_6038_M_UNIT 01
#define STA_6038_SET_UNIT(x) (dkt_sta = ((dkt_sta & ~(STA_6038_M_UNIT << STA_6038_V_UNIT)) | (((x) & STA_6038_M_UNIT) << STA_6038_V_UNIT)))


/* This controller supports one device type

   type         #sectors/       #surfaces/      #cylinders/     new format?
                 surface         cylinder        drive

   6038         8               1               77              no
   
*/


#define TYPE_6038        0
#define SECT_6038        8
#define SURF_6038        1
#define CYL_6038         77
#define SIZE_6038        (SECT_6038 * SURF_6038 * CYL_6038 * DKT_NUMWD)
#define NFMT_6038        FALSE


struct drvtyp {
    int32       sect;                                   /* sectors */
    int32       surf;                                   /* surfaces */
    int32       cyl;                                    /* cylinders */
    int32       size;                                   /* #blocks */
    int32       newf;                                   /* new format flag */
    };

struct drvtyp drv_tab[] = {
    { SECT_6038, SURF_6038, CYL_6038, SIZE_6038, NFMT_6038},
    { 0 }
    };

#define DKT_TRACE(x)    (dkt_trace & (1<<(x)))
#define DKT_TRACE_FP    stderr
/*  current trace bit use (bit 0 = LSB)
    0   I/O instructions
    1   pre-seek/read/write event setup
    2   seek events
    3   read/write events
    4   post read/write events
 */

extern uint16 M[];
extern UNIT cpu_unit;
extern int32 int_req, dev_busy, dev_done, dev_disable;
extern int32 saved_PC, SR, AMASK;
extern cpu_boot(unitno, dptr);


int32 dkt_trace = 0x0;

int32 dkt_6038_sc = 0;      /* Specify command register */
int32 dkt_6038_ma = 0;      /* Memory address counter register */
int32 dkt_6038_ca = 0;      /* Current address register */
int32 dkt_6038_progload_romaddr = 0;      /* Current rom read address for program load */
int32 dkt_sta = 0;                                      /* status register */
int32 dkt_swait = 100;                                  /* seek latency */
int32 dkt_rwait = 100;                                  /* rotate latency */
int32 dkt_settle = 10;                                  /* Settle time */
int32 dkt_step = 1;                                  /* Head step time */
int32 dkt_sectwait = 16;                             /* Sector preamble read latency */


static uint16 tbuf[DKT_NUMWD];                          /* transfer buffer */


int32 dkt (int32 pulse, int32 code, int32 AC);
t_stat dkt_svc (UNIT *uptr);
t_stat dkt_reset (DEVICE *dptr);
t_stat dkt_boot (int32 unitno, DEVICE *dptr);
t_stat dkt_attach (UNIT *uptr, CONST char *cptr);
t_stat dkt_go ( int32 pulse );
t_stat dkt_set_size (UNIT *uptr, int32 val, CONST char *cptr, void *desc);
t_stat dkt_tmrsvc(UNIT* uptr);

/* DKT data structures

   dkt_dev      DKT device descriptor
   dkt_unit     DKT unit list
   dkt_reg      DKT register list
   dkt_mod      DKT modifier list
*/

DIB dkt_dib = { DEV_DKT, INT_DKT, PI_DKT, &dkt };

UNIT dkt_unit[] = {
    { UDATA (&dkt_svc, UNIT_FIX+UNIT_ATTABLE+UNIT_DISABLE+UNIT_AUTO+
             UNIT_ROABLE+(TYPE_6038 << UNIT_V_DTYPE), SIZE_6038) },
    { UDATA(&dkt_svc, UNIT_FIX + UNIT_ATTABLE + UNIT_DISABLE + UNIT_AUTO +
             UNIT_ROABLE + (TYPE_6038 << UNIT_V_DTYPE), SIZE_6038) },
    { UDATA(&dkt_tmrsvc, UNIT_IDLE | UNIT_DIS, 0) },
    };

REG dkt_reg[] = {
        /* 6038/6039 registers */
    { ORDATA(SC_6038, dkt_6038_sc, 16) },
    { ORDATA(MA_6038, dkt_6038_ma, 16) },
    { ORDATA(CA_6038, dkt_6038_ca, 16) },
    { ORDATA(ROMADDR_6038, dkt_6038_progload_romaddr, 16) },

    { FLDATA (INT, int_req, INT_V_DKT) },
    { FLDATA (BUSY, dev_busy, INT_V_DKT) },
    { FLDATA (DONE, dev_done, INT_V_DKT) },
    { FLDATA (DISABLE, dev_disable, INT_V_DKT) },
    
    { DRDATA (TRACE, dkt_trace,   32) },
    
    { DRDATA (STIME, dkt_swait, 24), PV_LEFT },
    { DRDATA (RTIME, dkt_rwait, 24), PV_LEFT },
    { URDATA (CAPAC, dkt_unit[0].capac, 10, T_ADDR_W, 0,
              DKT_NUMDR, PV_LEFT | REG_HRO) },
    { NULL }
    };

MTAB dkt_mod[] = {
    { MTAB_XTD|MTAB_VUN, 0, "write enabled", "WRITEENABLED", 
        &set_writelock, &show_writelock,   NULL, "Write enable drive" },
    { MTAB_XTD|MTAB_VUN, 1, NULL, "LOCKED", 
        &set_writelock, NULL,   NULL, "Write lock drive" },
    
    { (UNIT_AUTO + UNIT_DTYPE + UNIT_ATT), (TYPE_6038 << UNIT_V_DTYPE),
      "6038/6039 Floppy", NULL, NULL },
    
    { (UNIT_AUTO + UNIT_DTYPE), (TYPE_6038 << UNIT_V_DTYPE),
      NULL, "6038", &dkt_set_size },
    { 0 }
    };

DEVICE dkt_dev = {
    "DKT", dkt_unit, dkt_reg, dkt_mod,
    DKT_NUMDR, 8, 30, 1, 8, 16,
    NULL, NULL, &dkt_reset,
    &dkt_boot, &dkt_attach, NULL,
    &dkt_dib, DEV_DISABLE
    };

/* The Micronova treats the 6038 as a low speed device and reads the 6038's loader
   via programmed IO. */

static const uint8 progload_rom_6038[] = {
    0x00, 0x07, 0xFF, 0xC0, 0x00, 0x3F, 0xFF, 0xD9, 0x00, 0x59, 0x28, 0x58, 0x21, 0xFC, 0x8F, 0x00,
    0x49, 0x38, 0xA8, 0x00, 0x11, 0x29, 0x11, 0x29, 0x11, 0x2A, 0x11, 0x2B, 0xAB, 0x04, 0x01, 0xFB,
    0x21, 0xF3, 0x31, 0xF3, 0x2A, 0x00, 0xDA, 0xC0, 0xFB, 0xC0, 0x4B, 0x00, 0xD3, 0x00, 0x83, 0x04,
    0x01, 0xFA, 0x03, 0xDB, 0x62, 0xBF, 0x65, 0xBF, 0x09, 0x15, 0x01, 0x04, 0x01, 0xFE, 0x85, 0x50,
    0x09, 0x11, 0xAA, 0x4B, 0x01, 0xFD, 0x85, 0x00, 0x09, 0x0E, 0xAE, 0x4B, 0x01, 0xFE, 0x21, 0x18,
    0x09, 0x0A, 0xC4, 0x1C, 0x01, 0xFE, 0x82, 0x50, 0x09, 0x06, 0x01, 0x02, 0x01, 0xEC, 0x21, 0x11,
    0x00, 0xFF, 0xD4, 0x91, 0xD4, 0x00, 0x62, 0x3F, 0x67, 0x7F, 0x01, 0xFF, 0x68, 0xFF, 0xCF, 0x00,
    0x75, 0x7F, 0xD2, 0x90, 0xD2, 0x90, 0xAA, 0x8B, 0xAA, 0x52, 0x03, 0x01, 0x03, 0x00, 0x00, 0x08,
    0x00, 0x44, 0x00
};


    

/* From 014-000073-03: 
The only valid method of executing a command is by appending the Start
pulse to the SPECIFY COMMAND instruction (DOA).The mnemonic for this command,
DOAS, is used throughout the following description of programming.A Start command
issued in any other manner will initiate a bootstrap loading procedure.*/

int32 dkt(int32 pulse, int32 code, int32 AC)
{
    UNIT* uptr;
    int32 u, rval, newu;

    rval = 0;

    if (DKT_TRACE(0))
    {
        static const char* f[8] =
        { "NIO", "DIA", "DOA", "DIB", "DOB", "DIC", "DOC", "SKP" };
        static const char* s[4] =
        { " ", "S", "C", "P" };

        fprintf(DKT_TRACE_FP, "  [DKT  %s%s %06o ] ", f[code & 0x07], s[pulse & 0x03], (AC & 0xFFFF));
    }


    u = SC_6038_GET_UNIT(dkt_6038_sc);
    uptr = dkt_dev.units + u;             /* last selected unit */

    switch (pulse) {                                        /* decode IR<8:9> */
    case iopS:
        if (sim_is_active(uptr)) { //Busy!
            dkt_sta = dkt_sta | STA_6038_ERROR;
            break;
        }

        //Clear error flags, set busy, clear done
        dkt_sta &= ~(STA_FLGS_SCLR);
        DEV_SET_BUSY(INT_DKT);
        DEV_CLR_DONE(INT_DKT);
        switch (code) {
        case ioNIO:
            DEV_CLR_BUSY(INT_DKT);                       /*  clear busy  */
            DEV_SET_DONE(INT_DKT);                       /*  set done    */
            DEV_UPDATE_INTR;
            dkt_6038_progload_romaddr = 0; /* Reset address for program load ROM read*/
            break;
        case ioDIA:                                         /* Read program load ROM next address */
            rval = progload_rom_6038[dkt_6038_progload_romaddr];
            dkt_6038_progload_romaddr++;
            DEV_CLR_BUSY(INT_DKT);                       /*  clear busy  */
            DEV_SET_DONE(INT_DKT);                       /*  set done    */
            DEV_UPDATE_INTR;
            break;
        case ioDOA:
            /* Select drive */
            newu = SC_6038_GET_UNIT(AC);


            if (newu != u)
            {
                /* Different drive selected */
                STA_6038_SET_UNIT(newu);
                u = newu;
                /* restore last known current address */
                uptr = dkt_dev.units + u;
                CA_6038_SETADDR(uptr->CYL, uptr->SECT);
            }
            
            dkt_6038_sc = AC;
            
            if (dkt_go(pulse))                         /* do command    */
                break;                                 /* break if no error  */

            
            DEV_CLR_BUSY(INT_DKT);                       /*  clear busy  */
            DEV_SET_DONE(INT_DKT);                       /*  set done    */
            DEV_UPDATE_INTR;                               /*  update ints */            
            break;

        }
        break;
    case iopC:
    default:
        switch (code) {
        case ioDIA:
            //Read status
            rval = dkt_sta;
            break;
        case ioDIB:
            //Read memory address register
            rval = dkt_6038_ma & 077777;
            break;
        case ioDOB:
            dkt_6038_ma = AC & 077777;
            break;
        case ioDIC:
            if (DKT_TRACE(0))
                    fprintf(DKT_TRACE_FP, "  [Read cur address: %06o trk %2d sector %2d  ] ", dkt_6038_ca, (dkt_6038_ca >> CA_6038_V_TRACK) & CA_6038_M_TRACK,(dkt_6038_ca >> CA_6038_V_SECTOR) & CA_6038_M_SECTOR);
            rval = dkt_6038_ca;
            break;
        default:
            ;
        }

        if (pulse == iopC)
        {

            //Clear all error flags, clear busy, clear done
            DEV_CLR_BUSY(INT_DKT);                       /*  clear busy  */
            DEV_CLR_DONE(INT_DKT);                       /*  clear done    */
            DEV_UPDATE_INTR;                               /*  update ints */
            u = SC_6038_GET_UNIT(dkt_6038_sc);
            dkt_sta = dkt_sta & ~(STA_EFLGS);   /*  clear controller flags  */
            sim_cancel(&dkt_unit[u]);                  /*  cancel any r/w op  */
        }
        
        break;
    }
    if (dkt_sta & STA_FLGS_GENERAL_ERROR)
        dkt_sta |= STA_6038_ERROR;

    if (DKT_TRACE(0))
    {
        if (code & 1)
            fprintf(DKT_TRACE_FP, "ret  [%06o]  ", (rval & 0xFFFF));
        fprintf(DKT_TRACE_FP, "]  \n");
    }

    return rval;
}

t_stat dkt_go(int32 pulse)
{
    UNIT* uptr;
    int32 u;

    dkt_sta = dkt_sta & ~STA_EFLGS;                         /* clear errors */
    u = SC_6038_GET_UNIT(dkt_6038_sc);                                /* get unit number */
    uptr = dkt_dev.units + u;                               /* get unit */
    if (((uptr->flags & UNIT_ATT) == 0) || sim_is_active(uptr)) {
        dkt_sta = dkt_sta | STA_6038_ERROR;                        /* attached or busy? */
        return FALSE;
    }

    uptr->FUNC = SC_6038_GET_CMD(dkt_6038_sc);


    if (DKT_TRACE(1))
    {
        int32        xSect;
        int32        xUnit;
        int32        xHi;
        

        xSect = SC_6038_GET_SECT(dkt_6038_sc);
        xUnit = SC_6038_GET_UNIT(dkt_6038_sc);
        xHi = SC_6038_GET_HITRK(dkt_6038_sc);

        fprintf(DKT_TRACE_FP,
            "  [%s:%c  %-5s:  %1d / %2d  %06o ] ",
            "DKT",
            (char)(u + '0'),
            ((uptr->FUNC == SC_6038_SETTLE) ?
                "settle"
                : ((uptr->FUNC == SC_6038_STEPIN) ?
                    "stepin"
                    : ((uptr->FUNC == SC_6038_STEPOUT) ?
                        "stepout"
                        : ((uptr->FUNC == SC_6038_READPREAMB) ?
                            "preamble"
                            : ((uptr->FUNC == SC_6038_WRITENEXT) ?
                                "writenext"
                                : ((uptr->FUNC == SC_6038_READNEXT) ?
                                    "readnext"
                                    : ((uptr->FUNC == SC_6038_FORMAT0) ?
                                        "format0"
                                        : ((uptr->FUNC == SC_6038_FORMATNEXT) ?
                                            "formatnext"
                                            : "<?>"
                                            )
                                        )
                                    )
                                )
                            )
                        )
                    )
                ),
            (unsigned) xUnit,
            (unsigned)xSect,
            (unsigned)(dkt_6038_ma & 0xFFFF) /* show all 16-bits in case DCH B */
        );
    }

    switch (uptr->FUNC)
    {
    case SC_6038_SETTLE:
        sim_activate(uptr, dkt_settle);                 /* schedule head settle */
        break;
    case SC_6038_STEPIN:
    case SC_6038_STEPOUT:
        sim_activate(uptr, dkt_step);                 /* schedule head step */
        break;
    //SC_6038_READPREAMB, SC_6038_WRITENEXT and SC_6038_READNEXT are scheduled when the sector timer handler fires

    case SC_6038_FORMAT0:
    case SC_6038_FORMATNEXT:
        if (uptr->flags & UNIT_WPRT) {
            dkt_sta = dkt_sta & STA_6038_WRITEPROT;
            return FALSE;
        }
        sim_activate(uptr, dkt_rwait);
        break;



    }

    
    return TRUE;
}


t_stat dkt_svc(UNIT* uptr)
{
    t_stat rval = SCPE_OK;
    int32 sa, bda, awc, err, dx, pa;
    int32 u = uptr - dkt_dev.units;                           /* get unit number */
    
    int32 dtype = GET_DTYPE(uptr->flags);

    switch (uptr->FUNC) {
    case SC_6038_SETTLE:
        dkt_sta = dkt_sta | STA_6038_HEADON;
        break;
    case SC_6038_STEPIN:
        uptr->CYL++;
        if (DKT_TRACE(2))
            fprintf(DKT_TRACE_FP, "  [Stepin, track now %2d  ] \r\n", uptr->CYL);
        dkt_sta &= ~STA_6038_TRACK0;
        break;
    case SC_6038_STEPOUT:
        if (uptr->CYL != 0)
            uptr->CYL--;
        if (DKT_TRACE(2))
            fprintf(DKT_TRACE_FP, "  [Stepout, track now %2d  ] \r\n", uptr->CYL);
        if (uptr->CYL == 0)
            dkt_sta |= STA_6038_TRACK0;
        break;

    case SC_6038_READPREAMB:
        
        if (DKT_TRACE(3))
            fprintf(DKT_TRACE_FP, "  [Preamble, next up: track %2d sect %2d  ] \r\n", uptr->CYL, uptr->SECT);
        CA_6038_SETADDR(uptr->CYL, uptr->SECT);
        break;
    case SC_6038_READNEXT:
    case SC_6038_WRITENEXT:
        
        if (DKT_TRACE(3))
            fprintf(DKT_TRACE_FP, "  [%s, next up: track %2d sect %2d, requested sector %d  ] \r\n", (uptr->FUNC == SC_6038_READNEXT ? "readnext" : "writenext"), uptr->CYL, uptr->SECT, SC_6038_GET_SECT(dkt_6038_sc));
        CA_6038_SETADDR(uptr->CYL, uptr->SECT);

        if (uptr->SECT != SC_6038_GET_SECT(dkt_6038_sc))
        {
            if (DKT_TRACE(3))
                fprintf(DKT_TRACE_FP, "  [%s, wrong sector %d, drive at sector %d ] \r\n", (uptr->FUNC == SC_6038_READNEXT ? "readnext" : "writenext"), SC_6038_GET_SECT(dkt_6038_sc), uptr->SECT);
            dkt_sta |= STA_6038_SECTORERR;
            break;
        }

        sa = GET_SA(uptr->CYL, 0, uptr->SECT, dtype);            /* get disk block */
        bda = sa * DKT_NUMWD * sizeof(uint16);             /* to words, bytes */
        err = fseek(uptr->fileref, bda, SEEK_SET);         /* position drive */

        if (DKT_TRACE(3))
            fprintf(DKT_TRACE_FP, "  [%s: seek to 0x%08X, dest mem address is %06o  ] \r\n", (uptr->FUNC == SC_6038_READNEXT ? "readnext" : "writenext"), bda, dkt_6038_ma);

        if (!err)
        {
            if (uptr->FUNC == SC_6038_READNEXT)
            {
                awc = fxread(tbuf, sizeof(uint16), DKT_NUMWD, uptr->fileref);
                for (; awc < DKT_NUMWD; awc++) tbuf[awc] = 0;
                if ((err = ferror(uptr->fileref)) == 0)
                {
                    for (dx = 0; dx < DKT_NUMWD; dx++) {            /* loop thru buffer */
                        pa = MapAddr(0, (dkt_6038_ma & AMASK));
                        if (MEM_ADDR_OK(pa))
                            M[pa] = tbuf[dx];
                        dkt_6038_ma = (dkt_6038_ma + 1) & AMASK;
                    }
                }
            }
            else
            {
                for (dx = 0; dx < DKT_NUMWD; dx++) {        /* loop into buffer */
                    pa = MapAddr(0, (dkt_6038_ma & AMASK));
                    tbuf[dx] = M[pa];
                    dkt_6038_ma = (dkt_6038_ma + 1) & AMASK;
                }
                fxwrite(tbuf, sizeof(int16), DKT_NUMWD, uptr->fileref);
                err = ferror(uptr->fileref);
                

            }
        }      
        
        
        if (err != 0) {
            sim_perror("DKT I/O error");
            clearerr(uptr->fileref);
            rval = SCPE_IOERR;
        }
        break;

    case SC_6038_FORMAT0:
    case SC_6038_FORMATNEXT:
        break;
    default:
        dkt_sta = dkt_sta | STA_6038_ILLEGAL | STA_6038_ERROR;
    }

    if (uptr->CYL == 0)
        dkt_sta |= STA_6038_TRACK0;
    else
        dkt_sta &= ~STA_6038_TRACK0;

    if(dkt_sta & STA_FLGS_GENERAL_ERROR)
        dkt_sta |= STA_6038_ERROR;
    DEV_CLR_BUSY(INT_DKT);
    DEV_SET_DONE(INT_DKT);
    DEV_UPDATE_INTR;
    return rval;
}



#define DKT_SIM_SECTORTIME 500
#define DKT_SIM_ADDRTIME 5
#define DKT_SIM_DATATIME 490


t_stat dkt_tmrsvc(UNIT* tmrptr)
{
    //Simulate next sector coming up under the head
    int32 u;
    
    t_stat rval = SCPE_OK;
    UNIT* uptr;
    
    
    for (u = 0; u < DKT_NUMDR; u++) {                       /* loop thru units */
        uptr = dkt_dev.units + u;
        uptr->SECT = (uptr->SECT + 1) & 7;
    }

    if (DEV_IS_BUSY(INT_DKT))
    {
        //A read preamble, read or write command may be pending, schedule it if so
        u = SC_6038_GET_UNIT(dkt_6038_sc);                                /* get unit number */
        uptr = dkt_dev.units + u;                               /* get unit */
        if (!sim_is_active(uptr))
        {
            switch (uptr->FUNC)
            {
            case SC_6038_READPREAMB:
                sim_activate_after(uptr, DKT_SIM_ADDRTIME);
                break;
            case SC_6038_READNEXT:
                sim_activate_after(uptr, DKT_SIM_ADDRTIME + DKT_SIM_DATATIME);
                break;
            case SC_6038_WRITENEXT:
                sim_activate_after(uptr, DKT_SIM_ADDRTIME + DKT_SIM_DATATIME);
                break;
            }
        }
    }
    
    sim_activate_after(tmrptr, DKT_SIM_SECTORTIME);                     /* reactivate */
    
    
    return rval;
}



/* Reset routine */

t_stat dkt_reset (DEVICE *dptr)
{
int32 u;
UNIT *uptr;

DEV_CLR_BUSY( INT_DKT ) ;                               /*  clear busy    */
DEV_CLR_DONE( INT_DKT ) ;                               /*  clear done    */
DEV_UPDATE_INTR ;                                       /*  update ints    */
dkt_6038_sc = dkt_6038_ma = dkt_6038_ca = dkt_6038_progload_romaddr = dkt_sta = 0;

for (u = 0; u < DKT_NUMDR; u++) {                       /* loop thru units */
    uptr = dkt_dev.units + u;
    sim_cancel (uptr);                                  /* cancel activity */
    uptr->CYL = uptr->SECT = uptr->FUNC = 0;
    }
sim_cancel(&dkt_dev.units[DKT_TIMER]);
sim_activate_after(&dkt_dev.units[DKT_TIMER], DKT_SIM_SECTORTIME);

return SCPE_OK;
}

/* Attach routine (with optional autosizing) */

t_stat dkt_attach (UNIT *uptr, CONST char *cptr)
{
int32 i, p;
t_stat   r;

uptr->capac = drv_tab[GET_DTYPE (uptr->flags)].size;    /* restore capac */
r = attach_unit (uptr, cptr);                           /* attach */
if ((r != SCPE_OK) || !(uptr->flags & UNIT_AUTO))
    return r;
if ((p = sim_fsize (uptr->fileref)) == 0)               /* get file size */
    return SCPE_OK;
for (i = 0; drv_tab[i].sect != 0; i++) {
    if (p <= (drv_tab[i].size * (int32) sizeof (uint16))) {
        uptr->flags = (uptr->flags & ~UNIT_DTYPE) | (i << UNIT_V_DTYPE);
        uptr->capac = drv_tab[i].size;
        return SCPE_OK;
        }
    }
return SCPE_OK;
}

/* Set size command validation routine */

t_stat dkt_set_size (UNIT *uptr, int32 val, CONST char *cptr, void *desc)
{
if (uptr->flags & UNIT_ATT)
    return SCPE_ALATT;
uptr->capac = drv_tab[GET_DTYPE (val)].size;
return SCPE_OK;
}

/* Bootstrap routine */

t_stat dkt_boot (int32 unitno, DEVICE *dptr)
{

UNIT* uptr;



uptr = dkt_dev.units + unitno;             /* select unit */
cpu_boot(unitno, dptr);
// Low speed for 6038, do not set bit 0
SR = DEV_DKT;

return SCPE_OK;
}
