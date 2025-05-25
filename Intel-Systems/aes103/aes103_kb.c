
#include "system_defs.h"
#include "sim_video.h"

#define aes103keyboard_NAME   "AES103 Keyboard Simulator"


#define IRQKEYBOARD 3




/* function prototypes */

t_stat aes103keyboard_svc(UNIT *uptr);
t_stat aes103keyboard_reset(DEVICE *dptr);


/* external function prototypes */

//extern t_stat SBC_reset(DEVICE *dptr);  /* reset the iSBC80/10 emulator */
extern t_stat(*aes103vid_kb_callback)(SIM_KEY_EVENT* kev);


/* local globals */


static const char* aes103keyboard_desc(DEVICE *dptr) {
    return aes103keyboard_NAME;
}

uint8 translate_key(SIM_KEY_EVENT* kev);

/* external globals */


UNIT aes103keyboard_unit = {
    UDATA (&aes103keyboard_svc, 0, 0), 1
};

uint8 aes103keyboard_pressedkey;
uint8 aes103keyboard_keywaiting;


REG aes103keyboard_reg[] = {
    { HRDATA(PRESSEDKEY, aes103keyboard_pressedkey, 8) },
    { HRDATA(KEYWAITING, aes103keyboard_keywaiting, 8) },
    { NULL }
};

DEBTAB aes103keyboard_debug[] = {
    { "ALL", DEBUG_all },
    { "FLOW", DEBUG_flow },
    { "READ", DEBUG_read },
    { "WRITE", DEBUG_write },
    { "XACK", DEBUG_xack },
    { NULL }
};

DEVICE aes103keyboard_dev = {
    "KEYBOARD",              //name 
    & aes103keyboard_unit,          //units 
    aes103keyboard_reg,            //registers 
    NULL,               //modifiers
    1,                  //numunits 
    16,                 //aradix  
    16,                 //awidth  
    1,                  //aincr  
    16,                 //dradix  
    8,                  //dwidth
    NULL,               //examine  
    NULL,               //deposit  
    & aes103keyboard_reset,         //reset 
    NULL,               //boot
    NULL,               //attach  
    NULL,               //detach
    NULL,               //ctxt     
    DEV_DEBUG,          //flags 
    0,                  //dctrl 
    aes103keyboard_debug,          //debflags
    NULL,               //msize
    NULL,               //lname
    NULL,               //help routine
    NULL,               //attach help routine
    NULL,               //help context
    & aes103keyboard_desc           //device description
};


t_stat aes103keyboard_callback(SIM_KEY_EVENT* kev)
{
    aes103keyboard_pressedkey = translate_key(kev);
    if (aes103keyboard_pressedkey != 0)
    {
        aes103keyboard_keywaiting = 1;
        irq_set(IRQKEYBOARD);
    }
    return SCPE_OK;

}

/* Service routines to handle simulator functions */

/* Reset routine */

t_stat aes103keyboard_reset(DEVICE *dptr)
{
    
    
    aes103keyboard_keywaiting = 0;
    aes103keyboard_pressedkey = 0;
    aes103vid_kb_callback = aes103keyboard_callback;
    return SCPE_OK;
}

uint8 aes103keyboard_r6(t_bool io, uint8 data, uint8 devnum)
{
    if (io == 0)
    {
        irq_clear(IRQKEYBOARD);
        //sim_printf("Clear keyboard irq\n");
        aes103keyboard_keywaiting = 0;
        return aes103keyboard_pressedkey;
    }

    return 0;
    
}


/* service routine*/

t_stat aes103keyboard_svc(UNIT *uptr)
{
    sim_activate(&aes103keyboard_unit, 100);

    
    return SCPE_OK;
}

#define KEY_WITH_SHIFT(NORMAL, SHIFTED) ((shifted ? (SHIFTED) : (NORMAL)) | (control ? 0x00: 0x80))
#define KEY(NORMAL) ((NORMAL) | (control ? 0x00: 0x80))

static uint8 translate_key(SIM_KEY_EVENT* kev)
{
    static t_bool shifted = FALSE;
    static t_bool caps = FALSE;
    static t_bool control = FALSE;
    static t_bool erase = FALSE;

    if (kev->key != SIM_KEY_F3) {  /* Clear erase cassette flag */
        erase = FALSE;
    }

    if (kev->state == SIM_KEYPRESS_UP) {
        switch (kev->key) {
        case SIM_KEY_SHIFT_L:
        case SIM_KEY_SHIFT_R:
            shifted = FALSE;
            break;

        case SIM_KEY_CTRL_L:
        case SIM_KEY_CTRL_R:
            control = FALSE;
            break;
        }
    }
    else { /* SIM_KEYPRESS_DOWN */
        switch (kev->key) {
        case SIM_KEY_SHIFT_L:
        case SIM_KEY_SHIFT_R:
            shifted = TRUE;
            break;

        case SIM_KEY_CAPS_LOCK:
            caps = !caps;
            break;

        case SIM_KEY_CTRL_L:
        case SIM_KEY_CTRL_R:
            control = TRUE;
            break;

        case SIM_KEY_0:
            return KEY_WITH_SHIFT(0x30, 0x29);

        case SIM_KEY_1:
            return KEY_WITH_SHIFT(0x31, 0x21);

        case SIM_KEY_2:
            return KEY_WITH_SHIFT(0x32, 0x40);

        case SIM_KEY_3:
            return KEY_WITH_SHIFT(0x33, 0x23);

        case SIM_KEY_4:
            return KEY_WITH_SHIFT(0x34, 0x24);

        case SIM_KEY_5:
            return KEY_WITH_SHIFT(0x35, 0x25);

        case SIM_KEY_6:
            return KEY_WITH_SHIFT(0x36, 0x5E);

        case SIM_KEY_7:
            return KEY_WITH_SHIFT(0x37, 0x26);

        case SIM_KEY_8:
            return KEY_WITH_SHIFT(0x38, 0x2A);

        case SIM_KEY_9:
            return KEY_WITH_SHIFT(0x39, 0x28);

        case SIM_KEY_A:
            return KEY_WITH_SHIFT(0x61, 0x41);

        case SIM_KEY_B:
            return KEY_WITH_SHIFT(0x62, 0x42);

        case SIM_KEY_C:
            return KEY_WITH_SHIFT(0x63, 0x43);

        case SIM_KEY_D:
            return KEY_WITH_SHIFT(0x64, 0x44);

        case SIM_KEY_E:
            return KEY_WITH_SHIFT(0x65, 0x45);

        case SIM_KEY_F:
            return KEY_WITH_SHIFT(0x66, 0x46);

        case SIM_KEY_G:
            return KEY_WITH_SHIFT(0x67, 0x47);

        case SIM_KEY_H:
            return KEY_WITH_SHIFT(0x68, 0x48);

        case SIM_KEY_I:
            return KEY_WITH_SHIFT(0x69, 0x49);

        case SIM_KEY_J:
            return KEY_WITH_SHIFT(0x6A, 0x4A);

        case SIM_KEY_K:
            return KEY_WITH_SHIFT(0x6B, 0x4B);

        case SIM_KEY_L:
            return KEY_WITH_SHIFT(0x6C, 0x4C);

        case SIM_KEY_M:
            return KEY_WITH_SHIFT(0x6D, 0x4D);

        case SIM_KEY_N:
            return KEY_WITH_SHIFT(0x6E, 0x4E);

        case SIM_KEY_O:
            return KEY_WITH_SHIFT(0x6F, 0x4F);

        case SIM_KEY_P:
            return KEY_WITH_SHIFT(0x70, 0x50);

        case SIM_KEY_Q:
            return KEY_WITH_SHIFT(0x71, 0x51);

        case SIM_KEY_R:
            return KEY_WITH_SHIFT(0x72, 0x52);

        case SIM_KEY_S:
            return KEY_WITH_SHIFT(0x73, 0x53);

        case SIM_KEY_T:
            return KEY_WITH_SHIFT(0x74, 0x54);

        case SIM_KEY_U:
            return KEY_WITH_SHIFT(0x75, 0x55);

        case SIM_KEY_V:
            return KEY_WITH_SHIFT(0x76, 0x56);

        case SIM_KEY_W:
            return KEY_WITH_SHIFT(0x77, 0x57);

        case SIM_KEY_X:
            return KEY_WITH_SHIFT(0x78, 0x58);

        case SIM_KEY_Y:
            return KEY_WITH_SHIFT(0x79, 0x59);

        case SIM_KEY_Z:
            return KEY_WITH_SHIFT(0x7A, 0x5A);


        case SIM_KEY_INSERT:
            return KEY(0x1B);

        case SIM_KEY_MINUS:
            return KEY_WITH_SHIFT(0x2D, 0x5F);

        case SIM_KEY_EQUALS:
            return KEY_WITH_SHIFT(0x3D, 0x2B);

        case SIM_KEY_LEFT_BRACKET:
            return KEY_WITH_SHIFT(0x5B, 0x5D);

        
        case SIM_KEY_SEMICOLON:
            return KEY_WITH_SHIFT(0x3B, 0x3A);

        case SIM_KEY_SINGLE_QUOTE:
            return KEY_WITH_SHIFT(0x27, 0x22);

        
        case SIM_KEY_COMMA:
            return KEY_WITH_SHIFT(0x2C, 0x3C);

        case SIM_KEY_PERIOD:
            return KEY_WITH_SHIFT(0x2E, 0x3E);

        case SIM_KEY_SLASH:
            return KEY_WITH_SHIFT(0x2F, 0x3F);

        case SIM_KEY_BACKSPACE:
            return KEY(0x08);

        case SIM_KEY_DELETE:
            return KEY(0x05);

        case SIM_KEY_TAB:
            return KEY(0x09);

        case SIM_KEY_ENTER:
        case SIM_KEY_KP_ENTER:
            return KEY(0x0d);

        case SIM_KEY_SPACE:
            return KEY(0x20);

        case SIM_KEY_UP:
            return KEY(0x10);

        case SIM_KEY_DOWN:
            return KEY(0x00);

        case SIM_KEY_LEFT:
            return KEY(0x18);

        case SIM_KEY_RIGHT:
            return KEY(0x04);

        case SIM_KEY_HOME:
            return KEY(0x17);

        case SIM_KEY_F1:
            return KEY(0x01); //FORMAT
        case SIM_KEY_F2:
            return KEY(0x03); //STOP CONTINUE
        case SIM_KEY_F3:
            return KEY(0x02); //RED
        case SIM_KEY_F4:
            return KEY(0x19); //SCREEN
        case SIM_KEY_F5:
            return KEY(0x1A); //GREEN
        case SIM_KEY_F6:
            return KEY(0x0C); //FILE

        
        default:
            sim_messagef(SCPE_OK, "Unmapped key = %02X\n", kev->key);
            break;
        }
    }

    return 0;
}
