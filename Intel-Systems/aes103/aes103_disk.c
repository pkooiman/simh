/*  aes103_disk.c: AES 103 floppy disk adapter
    05-03-15 version 
 
    
        Port addressing is as follows:
 
        Port    Mode    Command Function

        05      Read        Status
        10      Write       Set write sync byte
        11      Write       Set read sync byte
        12
        13
        14      Write       Step head
        15      Write       Set sector #
        16      Write       Command
        17      Write       Start sector read
        20      Write       Load DMAC Channel 0 Base and Current Address Regsiters
                Read        Read DMAC Channel 0 Current Address Register
        21      Write       Load DMAC Channel 0 Base and Current Word Count Registers
                Read        Read DMAC Channel 0 Current Word Count Register
        22      Write       Load DMAC Channel 1 Base and Current Address Regsiters
                Read        Read DMAC Channel 1 Current Address Register
        23      Write       Load DMAC Channel 1 Base and Current Word Count Registers
                Read        Read DMAC Channel 1 Current Word Count Register
        24      Write       Load DMAC Channel 2 Base and Current Address Regsiters
                Read        Read DMAC Channel 2 Current Address Register
        25      Write       Load DMAC Channel 2 Base and Current Word Count Registers
                Read        Read DMAC Channel 2 Current Word Count Register
        26      Write       Load DMAC Channel 3 Base and Current Address Regsiters
                Read        Read DMAC Channel 3 Current Address Register
        27      Write       Load DMAC Channel 3 Base and Current Word Count Registers
                Read        Read DMAC Channel 3 Current Word Count Register
        28      Write       DMAC status set
                Read        Read DMAC Status Register
        
        Register usage is defined in the following paragraphs.
 
        Read/Write DMAC Address Registers
 
            Used to simultaneously load a channel's current-address register and baseport-address 
            register with the memory address of the first byte to be transferred. (The Channel 
            0 current/baseport address register must be loaded prior to initiating a diskette read 
            or write operation.)  Since each channel's address registers are 16 bits in length
            (64K address range), two "write address register" commands must be executed in 
            order to load the complete current/baseport address registers for any channel.
 
        Read/Write DMAC Word Count Registers
 
            The Write DMAC Word Count Register command is used to simultaneously load a 
            channel's current and baseport word-count registers with the number of bytes 
            to be transferred during a subsequent DMA operation.  Since the word-count 
            registers are 16-bits in length, two commands must be executed to load both 
            halves of the registers.
 
        Write DMAC Command Register
 
            The Write DMAC Command Register command loads an 8-bit byte into the 
            DMAC's command register to define the operating characteristics of the 
            DMAC. 

            Read DMAC Status Register Command
 
            The Read DMAC Status Register command accesses an 8-bit status byte that 
            identifies the DMA channels that have reached terminal count or that 
            have a pending DMA request.
 
            
        Simulated Floppy Disk Drives
 
            The units in this device simulate a 5 1/4 drive. 
 
        uptr->u3 - 
        uptr->u4 - 
        uptr->u5 - current track number
        uptr->u6 - unit number (0-FDD_NUM)
*/
 
#include "system_defs.h"
 
#define UNIT_V_WPMODE   (UNIT_V_UF)     /* Write protect */
#define UNIT_WPMODE     (1 << UNIT_V_WPMODE)
 
/* Status register definitions */
#define ACTIVE             0x80  
#define SPARE1             0x40  
#define SPARE2             0x20  
#define TRACK0             0x10  
#define WRPROT2            0x08  
#define WRPROT1            0x04  
#define READY2             0x02  
#define READY1             0x01

/* Command register definitions */
#define DRIVE2SEL         0x80  
#define WRITE             0x40  
#define DIR               0x20  
#define ACTIVEN           0x10  
#define MOTOR2            0x08  
#define MOTOR1            0x04  
#define LOAD2             0x02  
#define LOAD1             0x01  

//High bits of sector count control leds
#define SECTORMASK 0x3F
 
#define FDD_NUM          2
 
#define aes103disk_NAME    "AES 103 Floppy Disk Controller Board"

#define AES103DISK_IRQ      2



#define AES103NUMCYL 35
#define AES103NUMSEC 16
#define AES103BYTESPERSEC 155
#define AES103BYTESPERTRACK (AES103BYTESPERSEC * AES103NUMSEC)
#define AES103DISKSIZE (AES103BYTESPERTRACK * AES103NUMCYL)
/* internal function prototypes */
 
t_stat aes103disk_cfg();
t_stat aes103disk_clr(void);
t_stat aes103disk_reset (DEVICE *dptr);
void aes103disk_reset1 (void);
t_stat aes103disk_attach (UNIT *uptr, const char *cptr);
t_stat aes103disk_set_mode (UNIT *uptr, int32 val, const char *cptr, void *desc);
t_stat aes103disk_svc (UNIT *uptr);
 
/* external function prototypes */
 
extern void set_irq(int32 int_num);
extern void clr_irq(int32 int_num);
extern uint8 reg_dev(uint8 (*routine)(t_bool, uint8, uint8), uint16, uint16, uint8);
extern uint8 unreg_dev(uint16);
extern void put_mbyte(uint16 addr, uint8 val);
extern uint8 get_mbyte(uint16 addr);
 
/* external globals */

extern uint16   PCX;

/* globals */

int isbc208_onetime = 1;
static const char* aes103disk_desc(DEVICE *dptr) {
    return aes103disk_NAME;
}

/* 8257 physical register definitions */
uint16 i8257_r0;                        // 8257 ch 0 address register
uint16 i8257_r1;                        // 8257 ch 0 count register
uint16 i8257_r2;                        // 8257 ch 1 address register
uint16 i8257_r3;                        // 8257 ch 1 count register
uint16 i8257_r4;                        // 8257 ch 2 address register
uint16 i8257_r5;                        // 8257 ch 2 count register
uint16 i8257_r6;                        // 8257 ch 3 address register
uint16 i8257_r7;                        // 8257 ch 3 count register
uint8 i8257_r8;                         // 8257 status register
uint8 i8257_r9;                         // 8257 mode register
uint8 i8257_ff;                         // 8257 first/last ff
 
 
/* AES103 physical register definitions */
uint8 aes103disk_status;                   // status register
uint8 aes103disk_cmd;                   // command register
uint8 aes103disk_writesync;
uint8 aes103disk_readsync;
uint8 aes103disk_step;
uint8 aes103disk_sectorno;
uint8 aes103disk_setactive;
 


int32 cyl;                              // current cylinder

int32 sec;                              // current sector
int32 drv;                              // current drive
uint8 cmd, pcmd;                        // current command

int32 spt;                              // sectors per track
int32 ssize;                            // sector size
 
 
/* isbc208 Standard SIMH Device Data Structures - 4 units */
UNIT aes103disk_unit[] = {
    { UDATA (&aes103disk_svc, UNIT_ATTABLE|UNIT_BUFABLE|UNIT_MUSTBUF|UNIT_FIX, AES103DISKSIZE), 200 },
    { UDATA(&aes103disk_svc, UNIT_ATTABLE | UNIT_BUFABLE | UNIT_MUSTBUF | UNIT_FIX, AES103DISKSIZE), 200 }
};
 
REG aes103disk_reg[] = {
    { HRDATA (CH0ADR, i8257_r0, 16) },
    { HRDATA (CH0CNT, i8257_r1, 16) },
    { HRDATA (CH1ADR, i8257_r2, 16) },
    { HRDATA (CH1CNT, i8257_r3, 16) },
    { HRDATA (CH2ADR, i8257_r4, 16) },
    { HRDATA (CH2CNT, i8257_r5, 16) },
    { HRDATA (CH3ADR, i8257_r6, 16) },
    { HRDATA (CH3CNT, i8257_r7, 16) },
    { HRDATA (STAT37, i8257_r8, 8) },
    { HRDATA (MODE37, i8257_r9, 8) },
    { HRDATA(FF, i8257_ff, 8) },
    { HRDATA(WRSYNC, aes103disk_writesync, 8) },
    { HRDATA(RDSYNC, aes103disk_readsync, 8) },
    { HRDATA(STEP, aes103disk_step, 8) },
    { HRDATA(SECTORNO, aes103disk_sectorno, 8) },
    { HRDATA(CMD, aes103disk_cmd, 8) },
    { HRDATA(SETACTIVE, aes103disk_setactive, 8) },
    { HRDATA(STATUS, aes103disk_status, 8) },

    
    { NULL }
};
 
MTAB aes103disk_mod[] = {
    { UNIT_WPMODE, 0, "RW", "RW", &aes103disk_set_mode },
    { UNIT_WPMODE, UNIT_WPMODE, "WP", "WP", &aes103disk_set_mode },
    { 0 }
};
 
DEBTAB aes103disk_debug[] = {
    { "ALL", DEBUG_all },
    { "FLOW", DEBUG_flow },
    { "READ", DEBUG_read },
    { "WRITE", DEBUG_write },
    { "XACK", DEBUG_xack },
    { NULL }
};
 
DEVICE aes103disk_dev = {
    "DSK",                   //name 
    aes103disk_unit,               //units 
    aes103disk_reg,                //registers 
    aes103disk_mod,                //modifiers
    FDD_NUM,                    //numunits 
    16,                         //aradix  
    32,                         //awidth  
    1,                          //aincr  
    16,                         //dradix  
    8,                          //dwidth
    NULL,                       //examine  
    NULL,                       //deposit  
    & aes103disk_reset,             //reset
    NULL,                       //boot
    & aes103disk_attach,            //attach  
    NULL,                       //detach
    NULL,                       //ctxt     
    DEV_DEBUG+DEV_DISABLE, //flags 
    0,                          //dctrl 
    aes103disk_debug,              //debflags
    NULL,                       //msize
    NULL,               //lname
    NULL,               //help routine
    NULL,               //attach help routine
    NULL,               //help context
    & aes103disk_desc       //device description
};
 
/* Service routines to handle simulator functions */
 
// configuration routine

t_stat aes103disk_cfg()
{
    int32 i;
    UNIT *uptr;

    

    // one-time initialization for all FDDs
    for (i = 0; i < FDD_NUM; i++) { 
        uptr = aes103disk_dev.units + i;
        uptr->u3 = 0;
        uptr->u4 = 0;
        uptr->u5 = 20;               //current track
        uptr->u6 = i;               //fdd unit number
        
        //uptr->flags |= UNIT_WPMODE; //set WP in unit flags
    }
    return SCPE_OK;
}

t_stat aes103disk_clr(void)
{
    sim_printf("    aes103 disk: Disabled\n");
    return SCPE_OK;
}

/* Reset routine */
 
t_stat aes103disk_reset (DEVICE *dptr)
{
    aes103disk_cfg();
    aes103disk_reset1();
    return SCPE_OK;
}
 
void aes103disk_reset1 (void)
{
    int32 i;
    UNIT *uptr;

    for (i = 0; i < FDD_NUM; i++) {     /* handle all units */
        uptr = aes103disk_dev.units + i;
        if ((uptr->flags & UNIT_ATT) == 0) { // unattached
//            sim_printf("         SBC208: FDD %d - Configured, Status=%02X, No disk image attached\n", 
//                i, fddst[i]);
        } else {                        /* attached */
        
//            sim_printf("         SBC208: FDD %d - Configured, Status=%02X, Attached to disk image %s\n",
//                i, fddst[i], uptr->filename);
            sim_activate (&aes103disk_unit[uptr->u6], aes103disk_unit[uptr->u6].wait);
        }
    }
    i8257_r8 = 0;                       /* status */
    i8257_r9 = 0;                       /* mode */
    i8257_ff = 0;                       /* firs/last byte flipflop */

    aes103disk_status = 0;  
    aes103disk_cmd = 0;     
    aes103disk_setactive = 0;

    

}

 
t_stat aes103disk_attach (UNIT *uptr, const char *cptr)
{
    t_stat r;
    int32 c = 0;
    long len;
    uint8 fddnum;

    if ((r = attach_unit (uptr, cptr)) != SCPE_OK) { 
        sim_printf("   aes103disk_attach: Attach error %d\n", r);
        return r;
    }
    len = sim_fsize (uptr->fileref);

 
    fddnum = uptr->u6;
    
    sim_printf("   AES103: FDD %d - %ld bytes of disk image %s loaded\n", 
        fddnum, len, uptr->filename);
    //sim_activate (&aes103disk_unit[fddnum], aes103disk_unit[fddnum].wait);
    sim_printf( "   aes103disk_attach: Done\n");
    return SCPE_OK;
}
 
 
t_stat aes103disk_set_mode (UNIT *uptr, int32 val, const char *cptr, void *desc)
{
    if (uptr->flags & UNIT_ATT)
        return sim_messagef (SCPE_ALATT, "%s is already attached to %s\n", sim_uname(uptr), uptr->filename);
    if (val & UNIT_WPMODE) {            /* write protect */
        //fddst[uptr->u6] |= WP;
        uptr->flags |= val;
    } else {                            /* read write */
        //fddst[uptr->u6] &= ~WP;
        uptr->flags &= ~val;
    }
    return SCPE_OK;
}
 
/* service routine - actually does the simulated disk I/O */
 
t_stat aes103disk_svc (UNIT *uptr)
{
    int unitno = (aes103disk_cmd & DRIVE2SEL) ? 1 : 0;
    int isWrite = (aes103disk_cmd & WRITE);
    sim_printf("AES103 disk service, %s disk %d, track %d, sector %d\n", isWrite ? "write" : "read", unitno, aes103disk_unit[unitno].u5, aes103disk_sectorno);
    //set_irq(SBC208_INT);    /* set interrupt */

    uint8* fbuf = (uint8*)uptr->filebuf;
    int imgadr = (aes103disk_unit[unitno].u5 * AES103BYTESPERTRACK) + (aes103disk_sectorno * AES103BYTESPERSEC);
    if (!isWrite)
    {
        if ((i8257_r3 & 0xC000) != 0x4000)
        {
            sim_printf("Disk read: incorrect DMA direction ch1\n");
            return SCPE_IOERR;
        }
        for (int i = 0; i < (i8257_r3 & 0xFF) + 1; i++) { /* copy selected sector to memory, count + 1 for sync byte */
            uint8 data = *(fbuf + (imgadr + i));
            put_mbyte(i8257_r2 + i, data);
        }

        i8257_r8 |= (1 << 1); //Channel 1 count reached
    }
    else
    {
        if ((i8257_r1 & 0xC000) != 0x8000)
        {
            sim_printf("Disk write: incorrect DMA direction ch0\n");
            return SCPE_IOERR;
        }
        // Write includes 16 0x00 pre-bytes, skip them
        uint8 syncseen = 0;
        uint8 syncoffset = 0;
        for (int i = 0; i < (i8257_r1 & 0xFF) + 1; i++) { /* copy selected memory to image, count + 1 for sync byte */
            uint8 data = get_mbyte(i8257_r0 + i);
            if (syncseen)
            {
                *(fbuf + (imgadr + i - syncoffset)) = data;
            }
            else
            {
                if (data == 0xDB)
                {
                    syncseen = 1;
                    syncoffset = i;
                    *(fbuf + imgadr) = 0xDB;
                }
            }
        }
        i8257_r8 |= (1 << 0); //Channel 0 count reached
    }

    aes103disk_status &= ~ACTIVE;
    irq_set(AES103DISK_IRQ);
    
    return SCPE_OK;
}
 
void aes103disk_set_ready_and_track()
{
    //Ready if motor is running and file attached, on hw set by sector pulses
    if ((aes103disk_unit[0].flags & UNIT_ATT) && (aes103disk_cmd & MOTOR1))
        aes103disk_status |= READY1;
    else
        aes103disk_status &= ~READY1;

    if ((aes103disk_unit[1].flags & UNIT_ATT) && (aes103disk_cmd & MOTOR2))
        aes103disk_status |= READY2;
    else
        aes103disk_status &= ~READY2;


    int unitno = (aes103disk_cmd & DRIVE2SEL) ? 1 : 0;
    if (aes103disk_unit[unitno].u5 == 0)
        aes103disk_status |= TRACK0;
    else
        aes103disk_status &= ~TRACK0;
}

/*  I/O instruction handlers, called from the CPU module when an
    IN or OUT instruction is issued.
 
    Each function is passed an 'io' flag, where 0 means a read from
    the port, and 1 means a write to the port.  On input, the actual
    input is passed as the return value, on output, 'data' is written
    to the device.
*/

/* Disk status read */
uint8 aes103disk_r5(t_bool io, uint8 data, uint8 devnum)
{
    if (io == 0)
    {
        irq_clear(AES103DISK_IRQ);
        return aes103disk_status;
    }
    else
        return 0;
}

//Sync byte for disk write
uint8 aes103disk_r10(t_bool io, uint8 data, uint8 devnum)
{
    if (io == 0)
        return 0;
    else
    {
        aes103disk_writesync = data;
        return 0;
    }
}


//Sync byte for disk read
uint8 aes103disk_r11(t_bool io, uint8 data, uint8 devnum)
{
    if (io == 0)
        return 0;
    else
    {
        aes103disk_readsync = data;
        return 0;
    }
}

//Step
uint8 aes103disk_r14(t_bool io, uint8 data, uint8 devnum)
{
    if (io == 0)
        return 0;
    else
    {
        int unitno = (aes103disk_cmd & DRIVE2SEL) ? 1 : 0;
        UNIT* u = &aes103disk_unit[unitno];
        if (aes103disk_cmd & DIR)
        {
            if (u->u5 + 1 < AES103NUMCYL)
                u->u5++;
        }
        else
        {
            if (u->u5 > 0)
                u->u5--;
        }

        if (u->u5 == 0)
            aes103disk_status |= TRACK0;
        else
            aes103disk_status &= ~TRACK0;
        irq_set(AES103DISK_IRQ);
        return 0;
    }
}


//Sector #
uint8 aes103disk_r15(t_bool io, uint8 data, uint8 devnum)
{
    if (io == 0)
        return 0;
    else
    {
        aes103disk_sectorno = data & SECTORMASK;
        return 0;
    }
}

//Command
uint8 aes103disk_r16(t_bool io, uint8 data, uint8 devnum)
{
    if (io == 0)
        return 0;
    else
    {
        aes103disk_cmd = data;
        aes103disk_set_ready_and_track();
        return 0;
    }
}

//Activate, start transaction
uint8 aes103disk_r17(t_bool io, uint8 data, uint8 devnum)
{
    if (io == 0)
        return 0;
    else
    {
        if (aes103disk_cmd & ACTIVEN)
        {
            int unitno = (aes103disk_cmd & DRIVE2SEL) ? 1 : 0;
            aes103disk_status |= ACTIVE;
            sim_activate(&aes103disk_unit[unitno], aes103disk_unit[unitno].wait);
        }
        return 0;
    }
}


//8257 DMA controller registers
 
uint8 aes103disk_i8257_reg16_r(uint16 reg, uint8 data)
{
    if (i8257_ff) {                 /* high byte */
        i8257_ff = 0;
        return (reg >> 8);
    } else {                        /* low byte */
        i8257_ff++;
        return (reg & BYTEMASK);
    }
}

uint8 aes103disk_i8257_reg16_w(uint16 *reg, uint8 data)
{
    
    if (i8257_ff) {                 /* high byte */
        i8257_ff = 0;
        *reg |= (data << 8);
    }
    else {                        /* low byte */
        i8257_ff++;
        *reg = data & BYTEMASK;
    }
    return 0;
}


uint8 aes103disk_r20(t_bool io, uint8 data, uint8 devnum)
{
    if (io == 0) {                      /* read current address CH 0 */
        return aes103disk_i8257_reg16_r(i8257_r0, data);
    }
    else {                            /* write base & current address CH 0 */
        return aes103disk_i8257_reg16_w(&i8257_r0, data);
    }
}

uint8 aes103disk_r21(t_bool io, uint8 data, uint8 devnum)
{
    if (io == 0) {                      /* read current count CH 0 */
        return aes103disk_i8257_reg16_r(i8257_r1, data);
    }
    else {                            /* write base & current count CH 0 */
        return aes103disk_i8257_reg16_w(&i8257_r1, data);
    }
}


uint8 aes103disk_r22(t_bool io, uint8 data, uint8 devnum)
{
    if (io == 0) {                      /* read current address CH 1 */
        return aes103disk_i8257_reg16_r(i8257_r2, data);
    }
    else {                            /* write base & current address CH 1 */
        return aes103disk_i8257_reg16_w(&i8257_r2, data);
    }
}

uint8 aes103disk_r23(t_bool io, uint8 data, uint8 devnum)
{
    if (io == 0) {                      /* read current count CH 1 */
        return aes103disk_i8257_reg16_r(i8257_r3, data);
    }
    else {                            /* write base & current count CH 1 */
        return aes103disk_i8257_reg16_w(&i8257_r3, data);
    }
}


uint8 aes103disk_r24(t_bool io, uint8 data, uint8 devnum)
{
    if (io == 0) {                      /* read current address CH 2 */
        return aes103disk_i8257_reg16_r(i8257_r4, data);
    }
    else {                            /* write base & current address CH 2 */
        return aes103disk_i8257_reg16_w(&i8257_r4, data);
    }
}

uint8 aes103disk_r25(t_bool io, uint8 data, uint8 devnum)
{
    if (io == 0) {                      /* read current count CH 2 */
        return aes103disk_i8257_reg16_r(i8257_r5, data);
    }
    else {                            /* write base & current count CH 2 */
        return aes103disk_i8257_reg16_w(&i8257_r5, data);
    }
}

uint8 aes103disk_r26(t_bool io, uint8 data, uint8 devnum)
{
    if (io == 0) {                      /* read current address CH 3 */
        return aes103disk_i8257_reg16_r(i8257_r6, data);
    }
    else {                            /* write base & current address CH 3 */
        return aes103disk_i8257_reg16_w(&i8257_r6, data);
    }
}

uint8 aes103disk_r27(t_bool io, uint8 data, uint8 devnum)
{
    if (io == 0) {                      /* read current count CH 3 */
        return aes103disk_i8257_reg16_r(i8257_r7, data);
    }
    else {                            /* write base & current count CH 3 */
        return aes103disk_i8257_reg16_w(&i8257_r7, data);
    }
}

uint8 aes103disk_r28(t_bool io, uint8 data, uint8 devnum)
{
    if (io == 0) {                      /* read status register */
        uint8 stat = i8257_r8;
        i8257_r8 = 0;
        return (stat);
    }
    else {                            /* write mode register */
        i8257_r9 = data & BYTEMASK;
        return 0;
    }
}

/* end of aes103_disk.c */
