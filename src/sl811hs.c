/*
 * Copyright (c) 2013, Jason S. McMullan <jason.mcmullan@gmail.com>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met: 
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer. 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <aros/debug.h>
#include <aros/macros.h>

#include <exec/errors.h>
#include <exec/lists.h>

#include <devices/timer.h>
#include <devices/usbhardware.h>

#include <devices/usb_hub.h>

#include <proto/exec.h>
#include <proto/utility.h>

#include "sl811hs.h"
#include "sl811hs_sim.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)   ((sizeof(x)/sizeof((x)[0])))
#endif

#undef D2
#if DEBUG >= 2
#define D2(x)   x
#else
#define D2(x)
#endif

#if DEBUG >= 2
#define ebug(fmt, args...) do { bug("%s:%d [%s/%d] ", __func__, __LINE__, sl->sl_Node.ln_Name, sl->sl_Node.ln_Pri); bug(fmt ,##args); } while (0)
#else
#define ebug(fmt, args...) do { bug("[%s/%d] ", sl->sl_Node.ln_Name, sl->sl_Node.ln_Pri); bug(fmt ,##args); } while (0)
#endif

#define SL811HS_ERRATA_1_2  0x10
#define SL811HS_ERRATA_1_5  0x20

#define DEFAULT_INTERVAL        32      /* 32x125us frames */

#define C_HUB_LOCAL_POWER       0
#define C_HUB_OVER_CURRENT      1
#define PORT_CONNECTION         0
#define PORT_ENABLE             1
#define PORT_SUSPEND            2
#define PORT_OVER_CURRENT       3
#define PORT_RESET              4
#define PORT_POWER              8
#define PORT_LOW_SPEED          9
#define C_PORT_CONNECTION       16
#define C_PORT_ENABLE           17
#define C_PORT_SUSPEND          18
#define C_PORT_OVER_CURRENT     19
#define C_PORT_RESET            20

#define IOF_ABORT               (1 << 7)

struct sl811hs {
    struct Node sl_Node;        /* For public use by that which allocates us */

    /* Clockport Interface */
    int sl_Irq;
    volatile UBYTE *sl_Addr;
    volatile UBYTE *sl_Data;

    struct Interrupt sl_Interrupt;
    BYTE  sl_SigDone;
    struct timerequest *sl_TimeRequest;

    /* Internal state */
    UBYTE sl_State;

    UBYTE sl_CurrAddr;

    BOOL  sl_PortScanned;
    UWORD sl_PortStatus;
    UWORD sl_PortChange;

    BYTE  sl_Errata;

    UBYTE sl_PacketStatus;

    struct Task *sl_CommandTask;
    struct MsgPort *sl_CommandPort;

    ULONG sl_DevEP_Toggle[128];         /* Upper 16-bits are OUT toggle,
                                         * lower 16-bits are IN toggle
                                         */

    UBYTE sl_RootDevAddr;
    UBYTE sl_RootConfiguration;

    struct MinList sl_PacketsActive;     /* Packets waiting for a transaction */
    struct MinList sl_XfersFree;         /* Xfers available */
    struct MinList sl_XfersActive;       /* Xfers in-flight */
    struct MinList sl_XfersCompleted;    /* Xfers holding done packets */

    struct sl811hs_Xfer {
        struct MinNode node;
        int ab;         /* 0 for A, 8 for B */
        UBYTE ctl;      /* HOSTCONTROL value */
        UBYTE base;     /* Location in FIFO */
        UBYTE maxlen;
        UBYTE len;      /* Length of current transaction */
#define SL811HS_PID_NONE 0
        UBYTE pidep;    /* USB PID & Endpoint */
        UBYTE dev;
        UBYTE *data;
        struct IOUsbHWReq *iou;
    } sl_Xfer[2];
#if DEBUG
    struct sl811hs_sim sl_Sim;
#endif
};

#if DEBUG > 1
static const char const *PidNames[] = {
    "PID_0",
    "OUT",
    "PID_2",
    "DATA0",
    "PID_4",
    "SOF",
    "PID_6",
    "PID_7",
    "PID_8",
    "IN",
    "NAK",
    "DATA1",
    "PREAMBLE",
    "SETUP",
    "STALL",
    "PID_F",
};
#define PIDNAME(pid)    PidNames[pid & 0xf]
static const char const *CmdNames[] = {
    "CMD_INVALID",
    "CMD_RESET",
    "CMD_READ",
    "CMD_WRITE",
    "CMD_UPDATE",
    "CMD_CLEAR",
    "UHCMD_USBSUSPEND",
    "UHCMD_USBOPER",
    "CMD_FLUSH",
    "UHCMD_QUERYDEVICE",
    "UHCMD_USBRESET",
    "UHCMD_USBRESUME",
    "UHCMD_CONTROLXFER",
    "UHCMD_ISOXFER",
    "UHCMD_INTXFER",
    "UHCMD_BULKXFER"
};
#define CMDNAME(cmd)    CmdNames[cmd & 0xf]
#endif


static inline void resume(struct sl811hs *sl)
{
#if DEBUG
    if (sl->sl_Addr == NULL) {
        /* Resume is a no-op for the sim */
    } else
#endif
    *(sl->sl_Data) = 0;

    D2(ebug("\n"));
}

static inline UBYTE rb(struct sl811hs *sl, UBYTE addr)
{
    UBYTE val;

    sl->sl_CurrAddr = addr;

#if DEBUG
    if (sl->sl_Addr == NULL) {
        sl811hs_sim_Write(&sl->sl_Sim, 0, addr);
        val = sl811hs_sim_Read(&sl->sl_Sim, 1);
    } else
#endif
    {
        *(sl->sl_Addr) = addr;
        val = *(sl->sl_Data);
    }

    D2(ebug("%02x = %02x\n", sl->sl_CurrAddr, val));
    return val;
}

static inline void wb(struct sl811hs *sl, UBYTE addr, UBYTE val)
{
    sl->sl_CurrAddr = addr;
    D2(ebug("%02x = %02x\n", sl->sl_CurrAddr, val));

#if DEBUG
    if (sl->sl_Addr == NULL) {
        sl811hs_sim_Write(&sl->sl_Sim, 0, addr);
        sl811hs_sim_Write(&sl->sl_Sim, 1, val);
        return;
    }
#endif

    *(sl->sl_Addr) = addr;
    *(sl->sl_Data) = val;
}

static inline UBYTE rn(struct sl811hs *sl)
{
    UBYTE val;
    sl->sl_CurrAddr++;

#if DEBUG
    if (sl->sl_Addr == NULL) {
        val = sl811hs_sim_Read(&sl->sl_Sim, 1);
    } else
#endif
    {
        /* SL811HS < 1.5 has a broken
         * autoincrement under certain conditions.
         */
        if (sl->sl_Errata <= SL811HS_ERRATA_1_5) {
           *(sl->sl_Addr) = sl->sl_CurrAddr;
        }
        val = *(sl->sl_Data);
    }

    D2(ebug("%02x = %02x\n", sl->sl_CurrAddr, val));
    return val;
}

static inline void wn(struct sl811hs *sl, UBYTE val)
{
    sl->sl_CurrAddr++;

    D2(ebug("%02x = %02x\n", sl->sl_CurrAddr, val));

#if DEBUG
    if (sl->sl_Addr == NULL) {
        sl811hs_sim_Write(&sl->sl_Sim, 1, val);
        return;
    }
#endif

    /* SL811HS < 1.5 has a broken
     * autoincrement under certain conditions.
     */
    if (sl->sl_Errata <= SL811HS_ERRATA_1_5) {
        *(sl->sl_Addr) = sl->sl_CurrAddr;
    }
    *(sl->sl_Data) = val;
}

static inline BOOL iouIsOut(struct IOUsbHWReq *iou)
{
    if (iou->iouh_Dir == UHDIR_SETUP)
        return (iou->iouh_SetupData.bmRequestType & 0x80) ? FALSE : TRUE;
    
    return (iou->iouh_Dir == UHDIR_OUT) ? TRUE : FALSE;
}

static inline BOOL sl811hs_ToggleState(struct sl811hs *sl, struct IOUsbHWReq *iou)
{
    int dev = iou->iouh_DevAddr & 127;
    int ep  = iou->iouh_Endpoint & 0xf;
    if (iouIsOut(iou))
        ep += 16;

    return (sl->sl_DevEP_Toggle[dev] & (1 << ep)) ? TRUE : FALSE;
}

static inline void sl811hs_ToggleFlip(struct sl811hs *sl, struct IOUsbHWReq *iou)
{
    int dev = iou->iouh_DevAddr & 127;
    int ep  = iou->iouh_Endpoint & 0xf;
    if (iouIsOut(iou))
        ep += 16;

   sl->sl_DevEP_Toggle[dev] ^= (1 << ep);
}

static inline void sl811hs_ToggleClear(struct sl811hs *sl, struct IOUsbHWReq *iou)
{
    int dev = iou->iouh_DevAddr & 127;
    int ep  = iou->iouh_Endpoint & 0xf;
    if (iouIsOut(iou))
        ep += 16;


    sl->sl_DevEP_Toggle[dev] &= ~(1 << ep);
}

static inline void sl811hs_ToggleSet(struct sl811hs *sl, struct IOUsbHWReq *iou)
{
    int dev = iou->iouh_DevAddr & 127;
    int ep  = iou->iouh_Endpoint & 0xf;
    if (iouIsOut(iou))
        ep += 16;

    sl->sl_DevEP_Toggle[dev] |= 1 << ep;
}

static void sl811hs_XferIssue(struct sl811hs *sl, struct sl811hs_Xfer *xfer)
{
    UBYTE ctl, *data, len;
    
    ctl = xfer->ctl;
    data = xfer->data;
    len = xfer->len;

    if (sl->sl_PortStatus & (1 << PORT_LOW_SPEED))
        ctl |= SL811HS_HOSTCTRL_PREAMBLE;

    if (sl811hs_ToggleState(sl, xfer->iou))
        ctl |= SL811HS_HOSTCTRL_DATA1;
    else
        ctl |= SL811HS_HOSTCTRL_DATA0;

    ctl |= SL811HS_HOSTCTRL_ENABLE | SL811HS_HOSTCTRL_ARM;

    if (((ctl & SL811HS_HOSTCTRL_DIR) == SL811HS_HOSTCTRL_DIR_OUT) && (len > 0)) {
        wb(sl, xfer->base, *(data++));
        for (len--; len > 0; len--, data++)
            wn(sl, *data);
    }

   
    AddTail((struct List *)&sl->sl_XfersActive, (struct Node *)xfer);

    wb(sl, xfer->ab + SL811HS_HOSTBASE, xfer->base);
    wn(sl, xfer->len);
    wn(sl, xfer->pidep);
    wn(sl, xfer->dev);

    D2(ebug("%p DATA%d %s\n", xfer->iou, (ctl & SL811HS_HOSTCTRL_DATA) ? 1 : 0, PIDNAME(SL811HS_HOSTID_PID_of(xfer->pidep))));

#ifndef DEBUG
    /* Errata 1.5, section 2 */
    if (!(sl->sl_PortStatus & (1 << PORT_LOW_SPEED))) {
        int ticks = (xfer->len >> 3) + 3;
        if (rb(sl, SL811HS_SOFHIGH) <= ticks)
            ctl |= SL811HS_HOSTCTRL_SYNCSOF;
    }
#endif

    xfer->ctl = ctl;
    wb(sl, xfer->ab + SL811HS_HOSTCTRL, ctl);
}

AROS_INTH1(sl811hs_IntServer, struct sl811hs *, sl)
{
    AROS_INTFUNC_INIT

    UBYTE status;
    UBYTE curraddr;

    curraddr = sl->sl_CurrAddr;
    status = rb(sl, SL811HS_INTSTATUS);

    D2(ebug("IntStatus %02x\n", status));
    if (status & SL811HS_INTMASK_CHANGED) {
        sl->sl_PortScanned = FALSE;
    }

    if (status & SL811HS_INTMASK_USB_A) {
        Remove((struct Node *)&sl->sl_Xfer[0]);
        AddTail((struct List *)&sl->sl_XfersCompleted, (struct Node *)&sl->sl_Xfer[0]);
    }

#ifdef ENABLE_B
    if (status & SL811HS_INTMASK_USB_B) {
        Remove((struct Node *)&sl->sl_Xfer[1]);
        AddTail((struct List *)&sl->sl_XfersCompleted, (struct Node *)&sl->sl_Xfer[1]);
    }
#endif

    wb(sl, SL811HS_INTSTATUS, status);

    /* Reset the address pointer */
    sl->sl_CurrAddr = curraddr;
    if (sl->sl_Addr != NULL)
        *(sl->sl_Addr) = curraddr;

    /* Mask out anything we care about */
    status &= SL811HS_INTMASK_CHANGED |
              SL811HS_INTMASK_USB_A |
              SL811HS_INTMASK_USB_B;

    if (status) {
        Signal(sl->sl_CommandTask, (1 << sl->sl_SigDone));
        D2(RawPutChar('!'));
    }

    return status ? TRUE : FALSE;

    AROS_INTFUNC_EXIT
}

#define DRV1_STATE_DONE             ((IPTR)0)

#define DRV1_STATE_SETUP_START      ((IPTR)1)
#define DRV1_STATE_SETUP_IN         ((IPTR)2)
#define DRV1_STATE_SETUP_OUT        ((IPTR)3)
#define DRV1_STATE_SETUP_STATUS     ((IPTR)4)

#define DRV1_STATE_BULK_IN          ((IPTR)10)
#define DRV1_STATE_BULK_OUT         ((IPTR)11)

#define DRV1_STATE_INT_IN           ((IPTR)20)
#define DRV1_STATE_INT_OUT          ((IPTR)21)

#define DRV1_STATE_ISO_IN           ((IPTR)30)
#define DRV1_STATE_ISO_OUT          ((IPTR)31)

BYTE sl811hs_ControlXfer(struct sl811hs *sl, struct IOUsbHWReq *iou)
{
    struct UsbSetupData *sd = &iou->iouh_SetupData;

    if (!(sl->sl_PortStatus & (1 << PORT_ENABLE)))
        return UHIOERR_USBOFFLINE;

    if ((iou->iouh_Length != AROS_LE2WORD(sd->wLength)) ||
        (iou->iouh_Endpoint != 0))
        return UHIOERR_BADPARAMS;

    iou->iouh_DriverPrivate1 = (APTR)DRV1_STATE_SETUP_START;

    return IOERR_UNITBUSY;
} 

BYTE sl811hs_BulkXfer(struct sl811hs *sl, struct IOUsbHWReq *iou)
{
    LONG len = iou->iouh_Length;

    if (!(sl->sl_PortStatus & (1 << PORT_ENABLE)))
        return UHIOERR_USBOFFLINE;

    if (len > 64)
        len = 64;

    if (len > iou->iouh_MaxPktSize)
        len = iou->iouh_MaxPktSize;

    iou->iouh_Actual = 0;

    /* Mark as bulk transfer phase */
    switch (iou->iouh_Dir) {
    case UHDIR_IN:
        iou->iouh_DriverPrivate1 = (APTR)DRV1_STATE_BULK_IN;
        break;
    case UHDIR_OUT:
        iou->iouh_DriverPrivate1 = (APTR)DRV1_STATE_BULK_OUT;
        break;
    default:
        return UHIOERR_BADPARAMS;
    }

    return IOERR_UNITBUSY;
} 

BYTE sl811hs_InterruptXfer(struct sl811hs *sl, struct IOUsbHWReq *iou)
{
    iou->iouh_Actual = 0;

    if (!(sl->sl_PortStatus & (1 << PORT_ENABLE)))
        return UHIOERR_USBOFFLINE;

    /* Mark as setup transfer phase */
    switch (iou->iouh_Dir) {
    case UHDIR_IN:
        iou->iouh_DriverPrivate1 = (APTR)DRV1_STATE_INT_IN;
        break;
    case UHDIR_OUT:
        iou->iouh_DriverPrivate1 = (APTR)DRV1_STATE_INT_OUT;
        break;
    default:
        return UHIOERR_BADPARAMS;
    }

    return IOERR_UNITBUSY;
} 

BYTE sl811hs_IsoXfer(struct sl811hs *sl, struct IOUsbHWReq *iou)
{
    iou->iouh_Actual = 0;

    if (!(sl->sl_PortStatus & (1 << PORT_ENABLE)))
        return UHIOERR_USBOFFLINE;

    /* Mark as setup transfer phase */
    switch (iou->iouh_Dir) {
    case UHDIR_IN:
        iou->iouh_DriverPrivate1 = (APTR)DRV1_STATE_INT_IN;
        break;
    case UHDIR_OUT:
        iou->iouh_DriverPrivate1 = (APTR)DRV1_STATE_INT_OUT;
        break;
    default:
        return UHIOERR_BADPARAMS;
    }

    return IOERR_UNITBUSY;
}

static BYTE sl811hs_XferStatus(struct sl811hs *sl, struct sl811hs_Xfer *xfer)
{
    LONG err = 0;
    int ab = xfer->ab;
    struct IOUsbHWReq *iou = xfer->iou;
    struct IORequest *io = &iou->iouh_Req;
    UBYTE status;
    D(int data);

    status = rb(sl, SL811HS_HOSTSTATUS + ab);
    D(data = (xfer->ctl & SL811HS_HOSTCTRL_DATA) ? 1 : 0);

    D2(ebug("%p DATA%d PID_%s Status %02x\n", iou, data, PIDNAME(SL811HS_HOSTID_PID_of(xfer->pidep)), status));
   
    if (io->io_Flags & IOF_ABORT) {
        D(ebug("%p DATA%d ABORT\n", iou, data));
        err = IOERR_ABORTED;
    }
   
    if (status & SL811HS_HOSTSTATUS_ERROR) {
        D(ebug("%p DATA%d ERROR\n", iou, data));
        err = UHIOERR_HOSTERROR;
    } else if (status & SL811HS_HOSTSTATUS_STALL) {
        D(ebug("%p DATA%d STALL\n", iou));
    } else if (status & SL811HS_HOSTSTATUS_OVERFLOW) {
        D(ebug("%p DATA%d OVERFLOW\n", iou, data));
        err = UHIOERR_OVERFLOW;
    } else if (status & SL811HS_HOSTSTATUS_TIMEOUT) {
        D(ebug("%p DATA%d TIMEOUT\n", iou, data));
        err  = UHIOERR_TIMEOUT;
    } else if (status & SL811HS_HOSTSTATUS_NAK) {
        D(ebug("%p DATA%d NAK %d.%d\n", iou, data, xfer->dev, SL811HS_HOSTID_EP_of(xfer->pidep)));
        err = UHIOERR_NAK;
    } else if (status & SL811HS_HOSTSTATUS_ACK) {
        if (!(xfer->ctl & SL811HS_HOSTCTRL_ISO))
            sl811hs_ToggleFlip(sl, xfer->iou);
        err = 0;
        if ((xfer->ctl & SL811HS_HOSTCTRL_DIR) && (status & SL811HS_HOSTSTATUS_SEQ)) {
            D(ebug("%p DATA%d SEQ %d.%d\n", iou, data, xfer->dev, SL811HS_HOSTID_EP_of(xfer->pidep)));
            D2(for (;;));
        } else {
            D2(ebug("%p DATA%d ACK %d.%d\n", iou, data, xfer->dev, SL811HS_HOSTID_EP_of(xfer->pidep)));
        }
    } else {
        D(ebug("%p DATA%d HOSTSTATUS %02x?!\n", iou, data, status));
        err = UHIOERR_HOSTERROR;
    }

    if (err) {
        D(ebug("Unsent/recvd: %d\n", rb(sl, SL811HS_HOSTTXLEFT)));
#if DEBUG > 4
        Forbid();
        for (;;);
#endif
    }

    io->io_Error = err;
    if (err)
        D(ebug("%p Error set as %d\n", iou, io->io_Error));

    return err;
}


static BYTE sl811hs_XferComplete(struct sl811hs *sl, struct sl811hs_Xfer *xfer)
{
    UBYTE *data;
    BYTE err;
    struct IOUsbHWReq *iou = xfer->iou;

    if (xfer->pidep==0 || iou == NULL)
        return 0;

    err = sl811hs_XferStatus(sl, xfer);

    if (err == IOERR_UNITBUSY) {
        D(ebug("Requeue %p (DATA SEQ error)\n", iou));
        return IOERR_UNITBUSY;
    }

    if (!(sl->sl_PortStatus & (1 << PORT_ENABLE))) {
        err = iou->iouh_Req.io_Error = UHIOERR_USBOFFLINE;
    }

    if (iou->iouh_Req.io_Error) {
        iou->iouh_DriverPrivate1 = DRV1_STATE_DONE;
    } else {
        int i;

        switch (SL811HS_HOSTID_PID_of(xfer->pidep)) {
        case SL811HS_PID_SETUP:
            D2(ebug("SETUP\n"));
            err = 0;
            break;
        case SL811HS_PID_IN:
            data = xfer->data;
            D2(ebug("IN  %d bytes @%p+%d from %02x\n", xfer->len, iou->iouh_Data, iou->iouh_Actual, xfer->base));
            *(data++) = rb(sl, xfer->base);
            for (i = 1; i < xfer->len; i++, data++) {
                *data = rn(sl);
            }
            iou->iouh_Actual += i;
            err = 0;
            break;
        case SL811HS_PID_OUT:
            D2(ebug("OUT %d bytes (of %d) @%p+%d\n", xfer->len, iou->iouh_Length - iou->iouh_Actual, iou->iouh_Data, iou->iouh_Actual));
            iou->iouh_Actual += xfer->len;
            err = 0;
            break;
        default:
            D2(ebug("%s\n", PIDNAME(SL811HS_HOSTID_PID_of(xfer->pidep))));
            break;
        }
    }

    D2(ebug("%p Error %d\n", iou, err));

    xfer->iou = NULL;
    xfer->pidep = 0;

    return err;
}

enum sl811hs_Perform_e {
    PERFORM_BUSY = -1,
    PERFORM_DONE = 0,
    PERFORM_ACTIVE = 1
};

enum sl811hs_Perform_e sl811hs_Perform(struct sl811hs *sl, struct IOUsbHWReq *iou)
{
    int pid;
    UBYTE ctl, *data, dev, ep;
    int len;
    IPTR nstate;
    struct sl811hs_Xfer *xfer;

    D2(ebug("%p on %d.%d\n", iou, iou->iouh_DevAddr, iou->iouh_Endpoint));

    /* Port gone? */
    if (!(sl->sl_PortStatus & (1 << PORT_ENABLE))) {
        iou->iouh_Req.io_Error = UHIOERR_USBOFFLINE;
        return PERFORM_DONE;
    }

    /* Allocate a transfer
     */
    xfer = (struct sl811hs_Xfer *)RemHead((struct List *)&sl->sl_XfersFree);

    if (xfer == NULL) {
        D(ebug("No Xfers free\n"));
        return PERFORM_BUSY;
    }

    D2(ebug("%p => Xfer[%d]\n", iou, xfer->ab/8));

    /* Reasonable defaults */
    len = iou->iouh_Length - iou->iouh_Actual;
    data = (UBYTE *)iou->iouh_Data + iou->iouh_Actual;
    dev = iou->iouh_DevAddr;
    ep  = iou->iouh_Endpoint;
    
    D2(ebug("State %d\n", (int)(IPTR)(APTR)iou->iouh_DriverPrivate1));
    switch ((IPTR)iou->iouh_DriverPrivate1) {
    case DRV1_STATE_SETUP_START:
        sl811hs_ToggleClear(sl, iou);
        ctl = SL811HS_HOSTCTRL_DIR_OUT;
        pid = SL811HS_PID_SETUP;
        len = sizeof(iou->iouh_SetupData);
        data = (UBYTE *)&iou->iouh_SetupData;
        if (iou->iouh_Length == 0)
            nstate = DRV1_STATE_SETUP_STATUS;
        else {
            if (iou->iouh_SetupData.bmRequestType & 0x80)
                nstate = DRV1_STATE_SETUP_IN;
            else
                nstate = DRV1_STATE_SETUP_OUT;
        }
        break;
    case DRV1_STATE_SETUP_IN:
        if (len == 0)
            goto setup_status;
        ctl = SL811HS_HOSTCTRL_DIR_IN;
        pid = SL811HS_PID_IN;
        nstate = DRV1_STATE_SETUP_IN;
        break;
    case DRV1_STATE_SETUP_OUT:
        if (len == 0)
            goto setup_status;
        ctl = SL811HS_HOSTCTRL_DIR_OUT;
        pid = SL811HS_PID_OUT;
        nstate = DRV1_STATE_SETUP_OUT;
        break;
    case DRV1_STATE_SETUP_STATUS:
setup_status:
        if (iou->iouh_Length && (iou->iouh_SetupData.bmRequestType & 0x80)) {
            ctl = SL811HS_HOSTCTRL_DIR_OUT;
            pid = SL811HS_PID_OUT;
        } else {
            ctl = SL811HS_HOSTCTRL_DIR_IN;
            pid = SL811HS_PID_IN;
        }
        len = 0;
        data = NULL;
        nstate = DRV1_STATE_DONE;
        break;
    case DRV1_STATE_BULK_IN:
        if (len == 0)
            goto state_done;

        if (len > 64)
            len = 64;

        ctl = SL811HS_HOSTCTRL_DIR_IN;
        pid = SL811HS_PID_IN;
        nstate = DRV1_STATE_BULK_IN;
        break;
    case DRV1_STATE_BULK_OUT:
        if (len == 0)
            goto state_done;

        if (len > 64)
            len = 64;

        ctl = SL811HS_HOSTCTRL_DIR_OUT;
        pid = SL811HS_PID_OUT;
        nstate = DRV1_STATE_BULK_OUT;
        break;
    case DRV1_STATE_INT_IN:
        ctl = SL811HS_HOSTCTRL_DIR_IN;
        pid = SL811HS_PID_IN;
        nstate = DRV1_STATE_DONE;
        break;
    case DRV1_STATE_INT_OUT:
        ctl = SL811HS_HOSTCTRL_DIR_OUT;
        pid = SL811HS_PID_OUT;
        nstate = DRV1_STATE_DONE;
        break;
    case DRV1_STATE_ISO_IN:
        ctl = SL811HS_HOSTCTRL_DIR_IN | SL811HS_HOSTCTRL_ISO;
        pid = SL811HS_PID_IN;
        nstate = DRV1_STATE_DONE;
        break;
    case DRV1_STATE_ISO_OUT:
        ctl = SL811HS_HOSTCTRL_DIR_OUT | SL811HS_HOSTCTRL_ISO;
        pid = SL811HS_PID_OUT;
        nstate = DRV1_STATE_DONE;
        break;
    default:
        D(ebug("Unexpected DriverPrivate1 state %p\n", iou->iouh_DriverPrivate1));
        iou->iouh_Req.io_Error = IOERR_NOCMD;
        /* FALLTHROUGH */
    case DRV1_STATE_DONE:
state_done:
        /* Release the xfer */
        xfer->iou = NULL;
        AddTail((struct List *)&sl->sl_XfersFree, (struct Node *)xfer);
        D2(ebug("DONE: err = %d\n", iou->iouh_Req.io_Error));
        return PERFORM_DONE;
    }

    if (len > xfer->maxlen) 
        len = xfer->maxlen;
    if (len > iou->iouh_MaxPktSize)
        len = iou->iouh_MaxPktSize;

    xfer->pidep = SL811HS_HOSTID_PIDEP(pid, ep);
    xfer->ctl = ctl;
    xfer->len = len;
    xfer->iou = iou;
    xfer->data = data;
    xfer->dev = dev;

    iou->iouh_DriverPrivate1 = (APTR)nstate;

    sl811hs_XferIssue(sl, xfer);
    return PERFORM_ACTIVE;
}

static void sl811hs_msSleep(struct sl811hs *sl, int ms)
{
    struct timerequest *tr = sl->sl_TimeRequest;

    tr->tr_node.io_Command = TR_ADDREQUEST;
    tr->tr_time.tv_secs = 0;
    tr->tr_time.tv_micro = ms * 1000;
    DoIO((struct IORequest *)tr);
}
 
static void sl811hs_PortScan(struct sl811hs *sl)
{
    UBYTE state;
    UWORD portstatus, portchange;

    if (sl->sl_PortScanned)
        return;

    portstatus = sl->sl_PortStatus;
    portchange = sl->sl_PortChange;
 
    wb(sl, SL811HS_INTSTATUS, 0xff);
    state = rb(sl, SL811HS_INTSTATUS);

    D(ebug("Port changed %04x: %02x\n", portstatus, state));

    if (state & SL811HS_INTMASK_DETECT) {
        portstatus &= ~((1 << PORT_CONNECTION) |
                        (1 << PORT_ENABLE));
        portchange |= (1 << PORT_CONNECTION) |
                      (1 << PORT_ENABLE);

        portstatus &= ~(1 << PORT_LOW_SPEED);

        wb(sl, SL811HS_INTSTATUS, SL811HS_INTMASK_DETECT);
        if (rb(sl, SL811HS_INTSTATUS) & SL811HS_INTMASK_DETECT)
            wb(sl, SL811HS_INTSTATUS, 0xff);
    } else {
        UBYTE ctrl1 = 0;
        UBYTE ctrl2 = 0;

        portstatus |= (1 << PORT_CONNECTION);
        portchange |= (1 << PORT_CONNECTION);

        if (state & SL811HS_INTMASK_FULLSPEED) {
            portstatus &= ~(1 << PORT_LOW_SPEED);
        } else {
            portstatus |= (1 << PORT_LOW_SPEED);
        }

        /* Update control registers for low or full speed connection */
        if (portstatus & (1 << PORT_LOW_SPEED)) {
            ctrl1 |= SL811HS_CONTROL1_LOW_SPEED;
            ctrl2 |= SL811HS_CONTROL2_LOW_SPEED;
        }

        wb(sl, SL811HS_CONTROL2, ctrl2 | SL811HS_CONTROL2_MASTER | SL811HS_CONTROL2_SOF_HIGH(0x2e));
        wb(sl, SL811HS_SOFLOW, 0xe0);
        wb(sl, SL811HS_CONTROL1, ctrl1 | SL811HS_CONTROL1_SOF_ENABLE);

        portstatus |= (1 << PORT_ENABLE);
        portchange |= (1 << PORT_ENABLE);
    }

    D(ebug("Port changed %04x: %sonnected, %s speed\n", portstatus,
                (portstatus & (1 << PORT_CONNECTION)) ? "C" : "Disc",
                (portstatus & (1 << PORT_LOW_SPEED)) ? "Low" : "Full"));

    /* Update port status */
    sl->sl_PortChange = portchange;
    sl->sl_PortStatus = portstatus;

    sl->sl_PortScanned = TRUE;
}


BYTE sl811hs_ResetUSB(struct sl811hs *sl, BOOL inReset)
{
    D(ebug("%s\n", inReset ? "TRUE" : "FALSE"));
    if (inReset) {
        struct sl811hs_Xfer *xfer;

        /* USB bus reset */
        wb(sl, SL811HS_INTENABLE, 0);
        wb(sl, SL811HS_CONTROL1, SL811HS_CONTROL1_USB_RESET);
        sl811hs_msSleep(sl, 50);

        sl->sl_PortStatus |= (1 << PORT_RESET);
        sl->sl_PortStatus &= ~(1 << PORT_ENABLE);
        sl->sl_PortChange |= (1 << PORT_RESET);
        sl->sl_PortStatus |= (1 << PORT_ENABLE);

        /* Kill any in-flight transfers */
        while ((xfer = (struct sl811hs_Xfer *)RemHead((struct List *)&sl->sl_XfersActive))) {
            xfer->iou->iouh_Req.io_Error = UHIOERR_USBOFFLINE;
            ReplyMsg((struct Message *)xfer->iou);
            xfer->iou = NULL;
            AddTail((struct List *)&sl->sl_XfersFree, (struct Node *)xfer);
        }

        /* Reset all endpoint's toggles */
        memset(&sl->sl_DevEP_Toggle[0], 0, sizeof(sl->sl_DevEP_Toggle));
    } else {
        /* NOTE - interrupts are still disabled! */
        wb(sl, SL811HS_CONTROL1, 0);

        sl->sl_PortScanned = FALSE;
        sl811hs_PortScan(sl);

        if (sl->sl_PortStatus & (1 << PORT_CONNECTION)) {
            /* Re-init host port registers */
            wb(sl, SL811HS_HOSTBASE, sl->sl_Xfer[0].base);
            wb(sl, SL811HS_HOSTLEN, 0x00);
            wb(sl, SL811HS_HOSTID, SL811HS_HOSTID_PIDEP(SL811HS_PID_SOF, 0));   /* SL811HS_HOSTID */
            wb(sl, SL811HS_HOSTDEVICEADDR, 0x00);
            wb(sl, SL811HS_HOSTCTRL, SL811HS_HOSTCTRL_ARM);

            /* Initialize B */
            wb(sl, SL811HS_HOSTBASE + 8, sl->sl_Xfer[1].base);
            wb(sl, SL811HS_HOSTLEN + 8, 0);
            wb(sl, SL811HS_HOSTID + 8, 0);
            wb(sl, SL811HS_HOSTDEVICEADDR + 8, 0x00);
            wb(sl, SL811HS_HOSTCTRL + 8, 0);
        }

        wb(sl, SL811HS_INTENABLE, SL811HS_INTMASK_CHANGED |
                                  SL811HS_INTMASK_USB_B |
                                  SL811HS_INTMASK_USB_A);

        sl->sl_PortStatus &= ~(1 << PORT_RESET);
        sl->sl_PortChange |= (1 << PORT_RESET);

    }

    return 0;
}

BYTE sl811hs_ResetHW(struct sl811hs *sl)
{
    /* Hardware reset */
    sl->sl_State = UHSF_RESET;

    sl->sl_Errata = rb(sl, SL811HS_HWREVISION);

    if (sl->sl_Errata != SL811HS_ERRATA_1_5) {
        D(ebug("SL811HS revision 1.5 expected\n"));
        return UHIOERR_HOSTERROR;
    }

    /* Reset */
    sl->sl_RootDevAddr = 0;

    /* Init controller */
    wb(sl, SL811HS_CONTROL2, SL811HS_CONTROL2_MASTER | SL811HS_CONTROL2_SOF_HIGH(0x2e));
    wb(sl, SL811HS_INTSTATUS, 0xff);

    /* Disable interrupts, wait 40ms */
    wb(sl, SL811HS_INTENABLE, 0);
    sl811hs_msSleep(sl, 40);

    /* Reset USB */
    sl811hs_ResetUSB(sl, TRUE);
    sl811hs_ResetUSB(sl, FALSE);

    sl->sl_State = UHSF_OPERATIONAL;
    return 0;
}

BYTE sl811hs_Suspend(struct sl811hs *sl)
{
    if (sl->sl_State != UHSF_OPERATIONAL)
        return IOERR_UNITBUSY;

    D(bug("%s:\n"));
    sl->sl_State = UHSF_SUSPENDED;
    sl->sl_PortStatus |= (1 << PORT_SUSPEND);
    sl->sl_PortChange |= (1 << PORT_SUSPEND);
    wb(sl, SL811HS_CONTROL1, SL811HS_CONTROL1_USB_RESET | SL811HS_CONTROL1_SUSPEND);

    return 0;
}

BYTE sl811hs_Resume(struct sl811hs *sl)
{
    if (sl->sl_State != UHSF_SUSPENDED)
        return IOERR_UNITBUSY;

    D(bug("%s:\n"));
    /* Single write to Data port to wake up */
    sl->sl_State = UHSF_RESUMING;
    resume(sl);

    sl->sl_PortStatus &= ~(1 << PORT_SUSPEND);
    sl->sl_PortChange |= (1 << PORT_SUSPEND);
    sl->sl_State = UHSF_OPERATIONAL;

    return 0;
}


#ifdef AROS_BIG_ENDIAN
#define CONST_WORD2LE(x) ((((x) & 0x00ff) <<  8) | \
                         (((x) & 0xff00) >>  8))
#else
#define CONST_WORD2LE(x) (x)
#endif

struct UsbStdDevDesc const sl811hs_DevDesc = {
    .bLength = sizeof(struct UsbStdDevDesc),
    .bDescriptorType = UDT_DEVICE,
    .bcdUSB = CONST_WORD2LE(0x0200),
    .bDeviceClass = HUB_CLASSCODE, /* HUB */
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0, /* Full speed hub */
    .bMaxPacketSize0 = 64,
    .idVendor = CONST_WORD2LE(0x04b4), /* Cypress */
    .idProduct = CONST_WORD2LE(0x2050), /* Cypress Hub */
    .bcdDevice = CONST_WORD2LE(0x0100),        /* Version 1.0 */
    .iManufacturer = 1,         /* String index 1 */
    .iProduct = 2,              /* String index 2 */
    .iSerialNumber = 0,
    .bNumConfigurations = 1
};

struct UsbStdCfgDesc const sl811hs_CfgDesc = {
    .bLength = sizeof(struct UsbStdCfgDesc),
    .bDescriptorType = UDT_CONFIGURATION,
    .wTotalLength = CONST_WORD2LE(sizeof(struct UsbStdCfgDesc) + sizeof(struct UsbStdIfDesc) + sizeof(struct UsbStdEPDesc) + sizeof(struct UsbHubDesc)),
    .bNumInterfaces = 1,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = USCAF_ONE | USCAF_SELF_POWERED | USCAF_REMOTE_WAKEUP,
    .bMaxPower = 0,     /* Self-powered */
};

struct UsbStdIfDesc const sl811hs_IntDesc = {
    .bLength = sizeof(struct UsbStdIfDesc),
    .bDescriptorType = UDT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 1,
    .bInterfaceClass = HUB_CLASSCODE,       /* Hub */
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,    /* Fullspeed */
    .iInterface = 0,
};

struct UsbStdEPDesc const sl811hs_EPDesc = {
    .bLength= sizeof(struct UsbStdEPDesc),
    .bDescriptorType = UDT_ENDPOINT,
    .bEndpointAddress = 0x81,
    .bmAttributes = 3,
    .wMaxPacketSize = CONST_WORD2LE(2),
    .bInterval = 255
};

struct slUsbStdStrDesc
{
    UBYTE bLength;             /* Size of this descriptor in bytes */
    UBYTE bDescriptorType;     /* UDT_STRING Descriptor Type */
    UWORD bString[8];          /* UNICODE encoded string */
};

struct slUsbStdStrDesc const sl811hs_StrDesc[] = {
    {
        .bLength = sizeof(struct UsbStdStrDesc) + (1-1) * sizeof(UWORD),
        .bDescriptorType = UDT_STRING,
        .bString = { CONST_WORD2LE(0x0409) }, /* English, United States */
    }, {
        .bLength = sizeof(struct UsbStdStrDesc) + (7-1) * sizeof(UWORD),
        .bDescriptorType = UDT_STRING,
        .bString = { CONST_WORD2LE('C'), CONST_WORD2LE('y'),
                     CONST_WORD2LE('p'), CONST_WORD2LE('r'),
                     CONST_WORD2LE('e'), CONST_WORD2LE('s'),
                     CONST_WORD2LE('s') },
    }, {
        .bLength = sizeof(struct UsbStdStrDesc) + (7-1) * sizeof(UWORD),
        .bDescriptorType = UDT_STRING,
        .bString = { CONST_WORD2LE('S'), CONST_WORD2LE('L'),
                     CONST_WORD2LE('8'), CONST_WORD2LE('1'),
                     CONST_WORD2LE('1'), CONST_WORD2LE('H'),
                     CONST_WORD2LE('S') },
    }
};

struct UsbHubDesc const sl811hs_HubDesc = {
    .bLength = sizeof(struct UsbHubDesc),
    .bDescriptorType = UDT_HUB,
    .bNbrPorts = 1,
    .wHubCharacteristics = 0,
    .bPwrOn2PwrGood = 50/2,     /* 50ms */
    .bHubContrCurrent = 25,     /* 25ma */
    .DeviceRemovable = 0,
    .PortPwrCtrlMask = 0xff,
};

static int sl811hs_AppendData(struct IOUsbHWReq *iou, UWORD *lengthp, int desc_len, CONST_APTR desc)
{
    int err;
    int length = *lengthp;
    int len = iou->iouh_Length - iou->iouh_Actual;
    UBYTE *data = (UBYTE *)iou->iouh_Data + iou->iouh_Actual;

    D2(ebug("Append %d bytes to buffer (%d left of %d), want to send %d\n", desc_len, len, iou->iouh_Length, length));

    if (len != length) {
        D2(ebug("Setup length %d, expected %d\n", length, len));
    }
    if (length > len) {
        err = UHIOERR_OVERFLOW;
    } else if (length < len) {
        len = length;
        err = UHIOERR_RUNTPACKET;
    } else {
        err = 0;
    }

    if (len > desc_len)
        len = desc_len;

    CopyMem(desc, data, len);
    length -= len;
    iou->iouh_Actual += len;

    *lengthp = length;

    return err;
}

#define CTLREQ(type,req)        (((type) << 8) | (req))

static BYTE sl811hs_ControlXferRoot(struct sl811hs *sl, struct IOUsbHWReq *iou)
{
    struct UsbSetupData *setup = &iou->iouh_SetupData;
    BYTE err = UHIOERR_NAK;
    UWORD value, index, length;
    UBYTE buff[4];

    value = AROS_LE2WORD(setup->wValue);
    index = AROS_LE2WORD(setup->wIndex);
    length = AROS_LE2WORD(setup->wLength);

    iou->iouh_Actual = 0;

    D2(ebug("%p value=$%04x, index=$%04x, length=$%04x\n", iou, value, index, length));
    D2(ebug("%p bmRequestType=$%02x, bRequest=$%02x\n", iou, setup->bmRequestType, setup->bRequest));
    switch (CTLREQ(setup->bmRequestType, setup->bRequest)) {
    case CTLREQ(URTF_OUT | URTF_STANDARD | URTF_DEVICE, USR_SET_ADDRESS):
        D2(ebug("SetAddress: %d\n", value));
        sl->sl_RootDevAddr = value;
        err = 0;
        break;
    case CTLREQ(URTF_IN | URTF_STANDARD | URTF_DEVICE, USR_GET_DESCRIPTOR):
        D2(ebug("GetDescriptor: %d [%d]\n", (value>>8) & 0xff, index));
        switch ((value>>8) & 0xff) {
        case UDT_DEVICE:
            err = sl811hs_AppendData(iou, &length, sizeof(sl811hs_DevDesc), &sl811hs_DevDesc);
            break;
        case UDT_CONFIGURATION:
            err = sl811hs_AppendData(iou, &length, sizeof(sl811hs_CfgDesc), &sl811hs_CfgDesc);
            if (err == 0 && length > 0)
                err = sl811hs_AppendData(iou, &length, sizeof(sl811hs_IntDesc), &sl811hs_IntDesc);
            if (err == 0 && length > 0)
                err = sl811hs_AppendData(iou, &length, sizeof(sl811hs_EPDesc), &sl811hs_EPDesc);
            if (err == 0 && length > 0)
                err = sl811hs_AppendData(iou, &length, sizeof(sl811hs_HubDesc), &sl811hs_HubDesc);
            break;
        case UDT_INTERFACE:
            err = sl811hs_AppendData(iou, &length, sizeof(sl811hs_IntDesc), &sl811hs_IntDesc);
            break;
        case UDT_ENDPOINT:
            err = sl811hs_AppendData(iou, &length, sizeof(sl811hs_EPDesc), &sl811hs_EPDesc);
            break;
        case UDT_HUB:
            err = sl811hs_AppendData(iou, &length, sizeof(sl811hs_HubDesc), &sl811hs_HubDesc);
            break;
        case UDT_STRING:
            if ((value & 0xff) < 3) {
                err = sl811hs_AppendData(iou, &length, sl811hs_StrDesc[value & 0xff].bLength, &sl811hs_StrDesc[value & 0xff]);
            }
            break;
        default:
            break;
        }

        break;
    case CTLREQ(URTF_IN | URTF_STANDARD | URTF_DEVICE, USR_GET_CONFIGURATION):
        D2(ebug("GetConfiguration: %d [%d]\n", value, index));
        buff[0] = sl->sl_RootConfiguration;
        err = sl811hs_AppendData(iou, &length, 1, buff);
        break;
    case CTLREQ(URTF_OUT | URTF_STANDARD | URTF_DEVICE, USR_SET_CONFIGURATION):
        D2(ebug("SetConfiguration: %d [%d]\n", value, index));
        if (index == 0) {
            sl->sl_RootConfiguration = value & 0xff;
            err = 0;
        }
        break;
    case CTLREQ(URTF_IN | URTF_STANDARD | URTF_DEVICE, USR_GET_STATUS): /* GetStatus */
        D2(ebug("GetDeviceStatus: %d [%d]\n", value, index));
        if (value == 0 && index == 0) {
            buff[0] = 1;        /* Self Powered */
            buff[1] = 0;
            err = sl811hs_AppendData(iou, &length, 2, buff);
        }
        break;
    case CTLREQ(URTF_IN | URTF_STANDARD | URTF_INTERFACE, USR_GET_STATUS): /* GetStatus */
        D2(ebug("GetInterfaceStatus: %d [%d]\n", value, index));
        if (value == 0 && index == 0) {
            buff[0] = 0;        
            buff[1] = 0;
            err = sl811hs_AppendData(iou, &length, 2, buff);
        }
        break;
    case CTLREQ(URTF_IN | URTF_STANDARD | URTF_ENDPOINT, USR_GET_STATUS): /* GetStatus */
        D2(ebug("GetEndpointStatus: %d [%d]\n", value, index));
        if (value == 0 && index == 0) {
            buff[0] = 0;        /* Not halted */ 
            buff[1] = 0;
            err = sl811hs_AppendData(iou, &length, 2, buff);
        }
        break;
     case CTLREQ(URTF_OUT | URTF_CLASS | URTF_DEVICE, USR_CLEAR_FEATURE): /* ClearHubFeature */
        D2(ebug("ClearHubFeature: %d [%d]\n", value, index));
        if (index == 0 && length == 0) {
            /* Nothing to do */
            err = 0;
        }
        break;
    case CTLREQ(URTF_OUT | URTF_CLASS | URTF_OTHER,  USR_CLEAR_FEATURE): /* ClearPortFeature */
        /* (Usually) nothing to do */
        D2(ebug("ClearPortFeature: %d [%d] (%04x %04x)\n", value, index, sl->sl_PortChange, sl->sl_PortStatus));
        if ((index & 0xff) == 1) {
            err = 0;

            if (value < 16) {
                switch (value) {
                case PORT_SUSPEND:
                    if (sl->sl_PortStatus & (1 << PORT_SUSPEND))
                        err = sl811hs_Resume(sl);
                    break;
                case PORT_POWER:
                    sl->sl_PortStatus &= ~(1 << value);
                    break;
                case PORT_ENABLE:
                    sl->sl_PortStatus &= ~(1 << value);
                    sl->sl_PortChange |= (1 << value);
                    break;
                }
            } else {
                /* Acknowledge change */
                sl->sl_PortChange &= ~(1 << (value - 16));
            }
        }
        break;
    case CTLREQ(URTF_IN  | URTF_CLASS | URTF_DEVICE, USR_GET_DESCRIPTOR): /* GetHubDescriptor */
        if (index == 0) {
            D2(ebug("GetHubDescriptor: %d [%d]\n", value, index));
            err = sl811hs_AppendData(iou, &length, sizeof(sl811hs_HubDesc), &sl811hs_HubDesc);
        }
        break;
    case CTLREQ(URTF_IN  | URTF_CLASS | URTF_DEVICE, USR_GET_STATUS): /* GetHubStatus */
        D2(ebug("GetHubStatus: %d [%d]\n", value, index));
        if (value == 0 && index == 0) {
            buff[0] = 0; /* HUB_LOCAL_POWER, HUB_OVER_CURRENT */
            buff[1] = 0;
            buff[2] = 0; /* C_HUB_LOCAL_POWER, C_HUB_OVER_CURRENT */
            buff[3] = 0;
            err = sl811hs_AppendData(iou, &length, 4, buff);
        }
        break;
    case CTLREQ(URTF_IN  | URTF_CLASS | URTF_OTHER,  USR_GET_STATUS): /* GetPortStatus/GetBusState */
        if (value == 0 && index == 1) {
            D2(ebug("GetPortStatus: %d [%d] (%04x %04x)\n", value, index, sl->sl_PortChange, sl->sl_PortStatus));
            buff[0] = (sl->sl_PortStatus >> 0) & 0xff;
            buff[1] = (sl->sl_PortStatus >> 8) & 0xff;
            buff[2] = (sl->sl_PortChange >> 0) & 0xff;
            buff[3] = (sl->sl_PortChange >> 8) & 0xff;
            err = sl811hs_AppendData(iou, &length, 4, buff);
        } else
            err = UHIOERR_STALL;
        break;
    case CTLREQ(URTF_OUT | URTF_CLASS | URTF_DEVICE, USR_SET_FEATURE): /* SetHubFeature */
        D2(ebug("SetHubFeature: %d [%d]\n", value, index));
        /* Nothing to do */
        err = 0;
        break;
    case CTLREQ(URTF_OUT | URTF_CLASS | URTF_OTHER, USR_SET_FEATURE): /* SetPortFeature */
        D2(ebug("SetPortFeature: %d [%d]\n", value, index));
        if (index == 1) {
            /* Nothing to do */
            err = 0;
            switch (value) {
            case PORT_SUSPEND:
                err = sl811hs_Suspend(sl);
                break;
            case PORT_POWER:
                sl->sl_PortStatus |= (1 << value);
                err = 0;
                break;
            case PORT_RESET:
                sl811hs_ResetUSB(sl, TRUE);
                err = sl811hs_ResetUSB(sl, FALSE);
                break;
            }
        }
        break;
    default:
        D(ebug("Unknown request - NAK\n"));
        err = UHIOERR_NAK;
        break;
    }

    D2(ebug("Return %d\n", err));
    return err;
}

static BYTE sl811hs_InterruptXferRoot(struct sl811hs *sl, struct IOUsbHWReq *iou)
{
    UWORD length = (UWORD)iou->iouh_Length;
    UBYTE port_one = (1 << 1);
    BYTE err = UHIOERR_STALL;

    D2(ebug("EndPoint %d\n", iou->iouh_Endpoint));

    if (iou->iouh_Endpoint == 1) {
        if (sl->sl_PortChange)
            err = sl811hs_AppendData(iou, &length, 1, &port_one);
        else
            err = UHIOERR_NAK;
    }

    return err;
}

static inline void sl811hs_Enqueue(struct sl811hs *sl, struct IOUsbHWReq *iou)
{
    /* Clear 'IOF_QUICK' flag */
    iou->iouh_Req.io_Flags &= ~IOF_QUICK;

    iou->iouh_DriverPrivate1 = NULL;
    iou->iouh_DriverPrivate2 = NULL;
    iou->iouh_Actual = 0;

    /* Add to the list of commands to do */
    PutMsg(sl->sl_CommandPort, (struct Message *)iou);
}

static inline LONG sl811hs_State(struct sl811hs *sl)
{
    return sl->sl_State;
}

struct sl811hs_Nak {
    struct timerequest tr;
    struct IOUsbHWReq *iou;
    ULONG time;         /* in uFrames */
    ULONG interval;     /* in uFrames */
};

#define UFRAME2MS(x)    ((x)/8)
#define UFRAME2US(x)    ((x)*125)
#define MS2UFRAME(x)    ((x)*8)

static void sl811hs_ReplyOrRetry(struct sl811hs *sl, struct IOUsbHWReq *iou)
{
    struct sl811hs_Nak *nak;

    do {
        if (iou->iouh_Req.io_Command < CMD_NONSTD)
            break;

        /* Handle runt transactions */
        if ((iou->iouh_Flags & UHFF_ALLOWRUNTPKTS) &&
            (iou->iouh_Req.io_Error == UHIOERR_RUNTPACKET)) {
            iou->iouh_Req.io_Error = 0;
            break;
        }

        /* Handle non-NAK/non-TIMEOUT error codes
         */
        if (iou->iouh_Req.io_Error != UHIOERR_NAK) {
            if (iou->iouh_DriverPrivate2) {
                D2(ebug("%p Clear iouh_DriverPrivate2\n", iou));
                FreeMem(iou->iouh_DriverPrivate2, sizeof(*nak));
                iou->iouh_DriverPrivate2 = NULL;
            }
            break;
        }

        /* From here on, we are in a NAK or Retry */
        if (iou->iouh_DriverPrivate2 == NULL) {
            nak = AllocMem(sizeof(*nak), MEMF_ANY);
            /* Clone the request from the main thread */
            CopyMem(sl->sl_TimeRequest, &nak->tr, sizeof(struct timerequest));
            iou->iouh_DriverPrivate2 = nak;
            nak->iou = iou;
            nak->interval = iou->iouh_Interval;
            nak->time = 0;
            if (sl->sl_PortStatus & (1 << PORT_LOW_SPEED))
                nak->interval = MS2UFRAME(nak->interval);
            if (nak->interval == 0)
                nak->interval = MS2UFRAME(iou->iouh_NakTimeout) / 16;
            if (nak->interval == 0)
                nak->interval = DEFAULT_INTERVAL;
        } else {
            nak = iou->iouh_DriverPrivate2;
            if ((iou->iouh_Flags & UHFF_NAKTIMEOUT) &&
                (UFRAME2MS(nak->time) > iou->iouh_NakTimeout)) {
                FreeMem(nak, sizeof(*nak));
                iou->iouh_DriverPrivate2 = NULL;
                iou->iouh_Req.io_Error = UHIOERR_NAKTIMEOUT;
                break;
            }
        }

        if (nak) {
            nak->tr.tr_time.tv_secs  = UFRAME2US(nak->interval) / 1000000;
            nak->tr.tr_time.tv_micro = UFRAME2US(nak->interval) % 1000000;
            nak->tr.tr_node.io_Command = TR_ADDREQUEST;
            D(ebug("%p NAK, retry in %d ms, %d ms left (%d frames waited)\n", iou, UFRAME2MS(nak->interval), (iou->iouh_Flags & UHFF_NAKTIMEOUT) ? (iou->iouh_NakTimeout - UFRAME2MS(nak->time)) : -1, nak->time));
            SendIO((struct IORequest *)nak);
            return;
        }

        iou->iouh_Req.io_Error = UHIOERR_OUTOFMEMORY;

    } while (0);

    D2(ebug("%p ReplyMsg(%d)\n", iou, iou->iouh_Req.io_Error));
    ReplyMsg((struct Message *)iou);
}

/* Get expired NAKs
 */
static struct IOUsbHWReq *sl811hs_GetNak(struct sl811hs *sl)
{
    struct sl811hs_Nak *nak;

    nak = (struct sl811hs_Nak *)GetMsg(sl->sl_TimeRequest->tr_node.io_Message.mn_ReplyPort);
    if (!nak)
        return NULL;

    nak->time += nak->interval;

    return nak->iou;
}

#if __EXEC_LIBAPI__ >= 50
static void sl811hs_CommandTask(struct sl811hs *sl)
{
#else
struct slTask {
    struct Task st_Task;
    struct MemList st_MemList;
    struct sl811hs *st_Device;
    ULONG st_Stack[1024];
};
static void sl811hs_CommandTask(void)
{
    struct slTask *st = (struct slTask *)FindTask(NULL);
    struct sl811hs *sl = st->st_Device;
    struct Message mn = {};
    struct MsgPort *mp;

    /* Create our own message port, replacing sl->sl_CommandPort */
    mp = sl->sl_CommandPort;
    sl->sl_CommandPort = CreateMsgPort();

    PutMsg(mp, &mn);
#endif
    struct IOUsbHWReq *iou;
    struct Message *dead = NULL;

    struct timerequest *tr;
    struct MsgPort *tr_mp;

    /* We can be called from more than one context, so
     * just open/doio/close here.
     */
    if ((tr_mp = CreateMsgPort())) {
        if ((tr = (struct timerequest *)CreateIORequest(tr_mp, sizeof(*tr)))) {
            if (0 == OpenDevice("timer.device", UNIT_MICROHZ, (struct IORequest *)tr, 0)) {
                ULONG sigmask;
                ULONG sigfport;
                ULONG sigfdone;
                ULONG sigftime;

                sl->sl_TimeRequest = tr;

                sl->sl_SigDone = AllocSignal(-1);
                sigfdone = (1 << sl->sl_SigDone);
                sigfport = (1 << sl->sl_CommandPort->mp_SigBit);
                sigftime = (1 << sl->sl_TimeRequest->tr_node.io_Message.mn_ReplyPort->mp_SigBit);
                sigmask  = sigfdone | sigfport | sigftime;

                SetSignal(sigmask, sigmask);

                sl->sl_Interrupt.is_Node.ln_Pri = 0;
                sl->sl_Interrupt.is_Node.ln_Type = NT_INTERRUPT;
                sl->sl_Interrupt.is_Node.ln_Name = "sl811hs";
                sl->sl_Interrupt.is_Data = sl;
                sl->sl_Interrupt.is_Code = (VOID (*)())sl811hs_IntServer;
                D2(ebug("Initializing IRQ handler (IRQ %d, handler %p)\n", sl->sl_Irq, &sl->sl_Interrupt));
#if DEBUG
                if (sl->sl_Addr == NULL)
                    sl811hs_sim_Init(&sl->sl_Sim, &sl->sl_Interrupt);
                else
#endif
                    AddIntServer(sl->sl_Irq, &sl->sl_Interrupt);

                sl811hs_ResetHW(sl);

                for (;;) {
                    ULONG sigset;
                    struct MinList todo;
                    NEWLIST(&todo);

                    sigset = Wait(sigmask);

                    /* Add NAKed-but-want-to-retry packets to
                     * the todo list.
                     */
                    if (sigset & sigftime) {
                        while ((iou = sl811hs_GetNak(sl))) {
                            AddTail((struct List *)&todo, (struct Node *)iou);
                        }
                    }

                    /* Signal from IRQ handler when there is something to do
                     */
                    if (sigset & sigfdone) {
                        BYTE err;

                        /* Scan for any port status changes */
                        sl811hs_PortScan(sl);

                        /* Completed xfers need to be processed and
                         * returned to the free list */
                        while (1) {
                            struct sl811hs_Xfer *xfer;

                            Disable();
                            xfer = (struct sl811hs_Xfer *)RemHead((struct List *)&sl->sl_XfersCompleted);
                            Enable();
                            if (!xfer)
                                break;
                            err = sl811hs_XferComplete(sl, xfer);
                            if (err != IOERR_UNITBUSY)
                                AddTail((struct List *)&sl->sl_XfersFree, (struct Node *)xfer);
                        }
                    }

                    if (sigset & sigfport) {
                        while ((iou = (struct IOUsbHWReq *)GetMsg(sl->sl_CommandPort))) {
                            AddTail((struct List *)&todo, (struct Node *)iou);
                        }
                    }

                    while ((iou = (struct IOUsbHWReq *)RemHead((struct List *)&todo))) {
                        ULONG state = sl811hs_State(sl);
                        BYTE err;

                        D2(ebug("%p Async processing, cmd %d\n", iou, (WORD)iou->iouh_Req.io_Command));

                        /* Command of Death */
                        if (iou->iouh_Req.io_Command == 0xffff) {
                            dead =  (struct Message *)iou;
                            continue;
                        }

                        /* If we are dead, or a message is marked
                         * as aborted, just reply as aborted
                         *
                         * NOTE: The initial 'Are you started?' message is
                         *       an empty io_Flags = IOF_ABORT message.
                         */
                        if (dead || (iou->iouh_Req.io_Flags & IOF_ABORT)) {
                            D(ebug("Aborting %p\n", iou));
                            err = IOERR_ABORTED;
                        } else switch (iou->iouh_Req.io_Command) {
                        case CMD_INVALID:
                            /* Startup message */
                            err = IOERR_NOCMD;
                            break;
                        case CMD_FLUSH:
                            /* Ditch any pending transfers, by marking
                             * their abort flag
                             */
                            Disable();
                            if (sl->sl_Xfer[0].iou)
                                sl->sl_Xfer[0].iou->iouh_Req.io_Flags |= IOF_ABORT;
                            if (sl->sl_Xfer[1].iou)
                                sl->sl_Xfer[1].iou->iouh_Req.io_Flags |= IOF_ABORT;
                            Enable();
                            err = 0;
                            break;
                        case CMD_RESET:
                            /* Reset hardware */
                            err = sl811hs_ResetHW(sl);
                            break;
                        case UHCMD_USBRESET:
                            /* Reset USB interface */
                            sl811hs_ResetUSB(sl, TRUE);
                            sl811hs_ResetUSB(sl, FALSE);
                            err = 0;
                            break;
                        case UHCMD_USBOPER:
                            do {
                                state = sl811hs_State(sl);

                                switch (state) {
                                case UHSF_OPERATIONAL:
                                    break;
                                case UHSF_RESUMING:
                                    sl811hs_msSleep(sl, 10);
                                    break;
                                case UHSF_SUSPENDED:
                                    sl811hs_Resume(sl);
                                    break;
                                case UHSF_RESET:
                                    sl811hs_msSleep(sl, 10);
                                    break;
                                }
                            } while (state != UHSF_OPERATIONAL);
                            iou->iouh_State = state;
                            err = UHIOERR_NO_ERROR;
                            break;
                        case UHCMD_CONTROLXFER:
                            /* Control transfer */
                            if (state != UHSF_OPERATIONAL) {
                                err = UHIOERR_USBOFFLINE;
                            } else {
                                if (iou->iouh_DevAddr == sl->sl_RootDevAddr) {
                                    /* Simulated host port */
                                    err = sl811hs_ControlXferRoot(sl, iou);
                                } else {
                                    /* Real transfer */
                                    err = sl811hs_ControlXfer(sl, iou);
                                }
                            }
                            break;
                        case UHCMD_BULKXFER:
                            if (state != UHSF_OPERATIONAL) {
                                err = UHIOERR_USBOFFLINE;
                            } else {
                                /* Bulk transfer */
                                if (iou->iouh_DevAddr == sl->sl_RootDevAddr) {
                                    /* Simulated host port has no bulk */
                                    err = UHIOERR_NAK;
                                } else {
                                    /* Real transfer */
                                    err = sl811hs_BulkXfer(sl, iou);
                                }
                            }
                            break;
                        case UHCMD_INTXFER:
                            if (state != UHSF_OPERATIONAL) {
                                err = UHIOERR_USBOFFLINE;
                            } else {
                                /* Interrupt transfer */
                                if (iou->iouh_DevAddr == sl->sl_RootDevAddr) {
                                    /* Simulated host port */
                                    err = sl811hs_InterruptXferRoot(sl, iou);
                                } else {
                                    /* Real transfer */
                                    err = sl811hs_InterruptXfer(sl, iou);
                                }
                            }
                            break;
                        case UHCMD_ISOXFER:
                            if (state != UHSF_OPERATIONAL) {
                                err = UHIOERR_USBOFFLINE;
                            } else {
                                /* Iso transfer */
                                if (iou->iouh_DevAddr == sl->sl_RootDevAddr) {
                                    /* Simulated host port has no iso */
                                    err = UHIOERR_NAK;
                                } else {
                                    /* Real transfer */
                                    err = sl811hs_IsoXfer(sl, iou);
                                }
                            }
                            break;
                        case UHCMD_USBSUSPEND:
                            if (state != UHSF_OPERATIONAL) {
                                err = UHIOERR_HOSTERROR;
                            } else {
                                /* Suspend */
                                err = sl811hs_Suspend(sl);
                            }
                            iou->iouh_State = sl811hs_State(sl);
                            break;
                        case UHCMD_USBRESUME:
                            if (state != UHSF_SUSPENDED) {
                                err = UHIOERR_HOSTERROR;
                            } else {
                                 /* Resume */
                                err = sl811hs_Resume(sl);
                            }
                            iou->iouh_State = sl811hs_State(sl);
                            break;
                        default:
                            /* AGH! WE CAN'T GET HERE! */
                            D(ebug("Terrifying - unexpected command %d\n", iou->iouh_Req.io_Command));
                            err = IOERR_NOCMD;
                            break;
                        }

                        /* Queue up transactions that require
                         * an interrupt.
                         */
                        if (err == IOERR_UNITBUSY) {
                            iou->iouh_Req.io_Error = 0;
                            AddTail((struct List *)&sl->sl_PacketsActive, (struct Node *)iou);
                            D2(ebug("%p => PacketsActive\n", iou));
                        } else {
                            /* Retry or reply */
                            iou->iouh_Req.io_Error = err;
                            sl811hs_ReplyOrRetry(sl, iou);
                        }
                    }

                    /* Handle the next queued transaction(s) */
                    D2(ebug("GetHead(sl_PacketsActive) = %p\n", GetHead((struct List *)&sl->sl_PacketsActive)));
                    while ((iou = (struct IOUsbHWReq *)GetHead((struct List *)&sl->sl_PacketsActive))) {
                        /* If we're dead, or aborted, just remove it */
                        if (dead || (iou->iouh_Req.io_Flags & IOF_ABORT)) {
                            D2(ebug("%p Aborted\n", iou));
                            iou->iouh_Req.io_Error = IOERR_ABORTED;
                        } else {
                            enum sl811hs_Perform_e state;

                            state = sl811hs_Perform(sl, iou);
                            if (state == PERFORM_ACTIVE)
                                break;

                            /* No ping-pongs free - try again later */
                            if (state == PERFORM_BUSY)
                                break;

                            /* Otherwise, we are in state PERFORM_DONE */
                        }

                        RemHead((struct List *)&sl->sl_PacketsActive);

                        /* Return the message */
                        D2(ebug("%p ReplyMsg(%d)\n", iou, iou->iouh_Req.io_Error));

                        /* Retry or Reply */
                        sl811hs_ReplyOrRetry(sl, iou);
                    }
                    D2(ebug("GetHead(sl_PacketsActive) = %p\n", GetHead((struct List *)&sl->sl_PacketsActive)));
                }

                /* Shut down interrupts */
                wb(sl, SL811HS_INTENABLE, 0);
#if DEBUG
                if (sl->sl_Addr != NULL)
#endif
                    RemIntServer(sl->sl_Irq, &sl->sl_Interrupt);

                FreeSignal(sl->sl_SigDone);

                CloseDevice((struct IORequest *)tr);
            }
            DeleteIORequest((struct IORequest *)tr);
        }
        DeleteMsgPort(tr_mp);
    }

#if __EXEC_LIBAPI__ < 50
    DeleteMsgPort(sl->sl_CommandPort);
#endif

    ReplyMsg(dead);
}

#define DC(field)       D2(ebug("%p->io_%s = 0x%x (%s)\n", ior, #field, ior->io_##field, CMDNAME(ior->io_##field)));
#define DF(field)       D2(ebug("%p->io_%s = 0x%x\n", ior, #field, ior->io_##field));
#define DU(field)       D2(ebug("%p->iouh_%s = 0x%x\n", iou, #field, iou->iouh_##field));

void sl811hs_BeginIO(struct sl811hs *sl, struct IORequest *ior)
{
    struct IOUsbHWReq *iou = (struct IOUsbHWReq *)ior;
    LONG err;

//    DF(Device);
//    DF(Unit);
    DC(Command);
    DF(Flags);

    err = IOERR_ABORTED;

#if DEBUG > 1
    if ((rb(sl, SL811HS_HWREVISION) & 0xfc) != 0x20) {
        ebug("Internal hardware failure detected!\n");
        ior->io_Error = IOERR_SELFTEST;
        if (!(ior->io_Flags & IOF_QUICK))
            ReplyMsg((struct Message *)ior);
        return;
    }
#endif

    switch (ior->io_Command) {
    case CMD_FLUSH:
        /* Abort all UHCMD_CONTROLXFER, UHCMD_ISOXFER, UHCMD_INTXFER
         *         and UHCMD_BULKXFER requests in progress or queued
         */
        sl811hs_Enqueue(sl, iou);
        break;
    case CMD_RESET:
        /* Enqueue the reset */
        sl811hs_Enqueue(sl, iou);
        break;
    case UHCMD_BULKXFER:
        /* Start a bulk transfer */
        DU(Flags);
        DU(Dir);
        DU(DevAddr);
        DU(Endpoint);
        DU(MaxPktSize);
        DU(Data);
        DU(Length);
        DU(SplitHubAddr);
        DU(SplitHubPort);
        DU(NakTimeout);
        sl811hs_Enqueue(sl, iou);
        break;
    case UHCMD_CONTROLXFER:
        /* Start a control transfer */
        DU(Flags);
        DU(DevAddr);
        DU(Endpoint);
        DU(MaxPktSize);
        DU(Data);
        DU(Length);
        DU(SetupData);
        DU(SplitHubAddr);
        DU(SplitHubPort);
        DU(NakTimeout);
        sl811hs_Enqueue(sl, iou);
        break;
    case UHCMD_INTXFER:
        /* Start an interrupt transfer */
        DU(Flags);
        DU(Dir);
        DU(DevAddr);
        DU(Endpoint);
        DU(MaxPktSize);
        DU(Data);
        DU(Length);
        DU(Interval);
        DU(SplitHubAddr);
        DU(SplitHubPort);
        DU(NakTimeout);
        sl811hs_Enqueue(sl, iou);
        break;
    case UHCMD_ISOXFER:
        /* Start an isochronous transfer */
        DU(Flags);
        DU(Dir);
        DU(DevAddr);
        DU(Endpoint);
        DU(MaxPktSize);
        DU(Data);
        DU(Length);
        DU(SplitHubAddr);
        DU(SplitHubPort);
        DU(NakTimeout);
        sl811hs_Enqueue(sl, iou);
        break;
    case UHCMD_QUERYDEVICE:
        DU(Data);
        if (iou->iouh_Data) {
            struct TagItem *tmp, *ti = (struct TagItem *)iou->iouh_Data;
            D2(int i=0;)
            while ((tmp = NextTagItem(&ti))) {
                D2(bug("  Tag[%d] = (%p, %p)\n", i++, (APTR)tmp->ti_Tag, (APTR)tmp->ti_Data));
                switch (tmp->ti_Tag) {
                case UHA_State:
                    iou->iouh_State = sl811hs_State(sl);
                    tmp->ti_Data = iou->iouh_State;
                    break;
                case UHA_Manufacturer:
                    tmp->ti_Data = (IPTR)"Cypress";
                    break;
                case UHA_ProductName:
                    tmp->ti_Data = (IPTR)"SL811HS";
                    break;
                case UHA_Version:
                    tmp->ti_Data = 1;
                    break;
                case UHA_Revision:
                    tmp->ti_Data = ((rb(sl, SL811HS_HWREVISION) & 0xf0) == 0x10) ? 2 : 5;
                    break;
                case UHA_Description:
                    tmp->ti_Data = (IPTR)"USB 1.1 Host";
                    break;
                case UHA_Copyright:
                    tmp->ti_Data = (IPTR)"Copyright 2013, Jason McMullan <jason.mcmullan@gmail.com>";
                    break;
                case UHA_DriverVersion:
                    tmp->ti_Data = 0x200;
                    break;
                default:
                    tmp->ti_Data = 0;
                    break;
                }
            }
            err = 0;
        } else { 
            err = IOERR_BADADDRESS;
        }
        break;
    case UHCMD_USBOPER:
        /* Try to get operational */
        sl811hs_Enqueue(sl, iou);
        break;
    case UHCMD_USBRESET:
        /* USB reset (with status return) */
        sl811hs_Enqueue(sl, iou);
        break;
    case UHCMD_USBRESUME:
        /* Resume from sleep mode */
        sl811hs_Enqueue(sl, iou);
        break;
    case UHCMD_USBSUSPEND:
        /* Enter sleep mode */
        sl811hs_Enqueue(sl, iou);
        break;
    default:
        err = IOERR_NOCMD;
        break;
    }

    /* If err == IOERR_ABORTED, then it has been enqueued
     * for further processing.
     */
    if (err != IOERR_ABORTED) {
        ior->io_Error = err;
        if ((ior->io_Flags & IOF_QUICK)) {
            D2(ebug("%p IOF_QUICK %d\n", ior, ior->io_Error));
        } else {
            D2(ebug("%p ReplyMsg(%d)\n", ior, ior->io_Error));
            ReplyMsg(&ior->io_Message);
        }
    }
}

LONG sl811hs_AbortIO(struct sl811hs *sl, struct IORequest *ior)
{
    Disable();
    ior->io_Flags |= IOF_ABORT;
    Enable();

    return 0;
}

struct sl811hs *sl811hs_Attach(IPTR addr, IPTR data, int irq)
{
    struct sl811hs *sl;

#if DEBUG < 1
    /* A tiny bit of sanity checking */
    if (addr == NULL || data == NULL || addr == data)
        return NULL;
#endif

    sl = AllocMem(sizeof(*sl), MEMF_ANY | MEMF_CLEAR);
    sl->sl_Addr = (volatile UBYTE *)addr;
    sl->sl_Data = (volatile UBYTE *)data;
    sl->sl_Irq = irq;

    /* Quick check to verify that the device is even there */
    resume(sl);
    if ((rb(sl, SL811HS_HWREVISION) & 0xfc) != SL811HS_HWREVISION_1_5) {
        D(ebug("Can't detect SL811HS v1.5 at $%0x/$%0x\n", addr, data));
        FreeMem(sl, sizeof(*sl));
        return NULL;
    }

    NEWLIST(&sl->sl_PacketsActive);
    NEWLIST(&sl->sl_XfersFree);
    NEWLIST(&sl->sl_XfersActive);
    NEWLIST(&sl->sl_XfersCompleted);

    sl->sl_Xfer[0].ab = 0;
    sl->sl_Xfer[0].base = 16;
#ifdef ENABLE_B
    sl->sl_Xfer[0].maxlen = 120;
    sl->sl_Xfer[1].ab = 8;
    sl->sl_Xfer[1].base = 136;
    sl->sl_Xfer[1].maxlen = 120;
#else
    sl->sl_Xfer[0].maxlen = 240;
    sl->sl_Xfer[1].ab = 8;
    sl->sl_Xfer[1].base = 0;
    sl->sl_Xfer[1].maxlen = 0;
#endif

    AddHead((struct List *)&sl->sl_XfersFree, (struct Node *)&sl->sl_Xfer[0]);
#ifdef ENABLE_B
    AddHead((struct List *)&sl->sl_XfersFree, (struct Node *)&sl->sl_Xfer[1]);
#endif

#if __EXEC_LIBAPI__ >= 50
    sl->sl_CommandTask = NewCreateTask(TASKTAG_PC, sl811hs_CommandTask,
                                       TASKTAG_NAME, "sl811hs",
                                       TASKTAG_ARG1, sl,
                                       TASKTAG_TASKMSGPORT, &sl->sl_CommandPort,
                                       TAG_END);
#else
    sl->sl_CommandTask = AllocMem(sizeof(struct slTask), MEMF_ANY | MEMF_CLEAR);
    if (sl->sl_CommandTask) {
        struct slTask *st = (struct slTask *)sl->sl_CommandTask;
        struct MsgPort *mp;

        st->st_Task.tc_Node.ln_Type = NT_TASK;
        st->st_Task.tc_Node.ln_Pri = 0;
        st->st_Task.tc_Node.ln_Name = "sl811hs";
        st->st_Task.tc_SPLower = &st->st_Stack[0];
        st->st_Task.tc_SPUpper = &st->st_Stack[1023];
        st->st_Task.tc_SPReg   = &st->st_Stack[1023];
        st->st_MemList.ml_NumEntries = 1;
        st->st_MemList.ml_ME[0].me_Addr = st;
        st->st_MemList.ml_ME[0].me_Length = sizeof(*st);
        st->st_Device = sl;
        NEWLIST(&st->st_Task.tc_MemEntry);
        AddTail(&st->st_Task.tc_MemEntry, (struct Node *)&st->st_MemList);

        mp = CreateMsgPort();
        sl->sl_CommandPort = mp;
        AddTask(&st->st_Task, sl811hs_CommandTask, NULL);
        /* Wait for start message */
        WaitPort(mp);
        GetMsg(mp);
        DeleteMsgPort(mp);
    }
#endif

    if (!sl->sl_CommandTask) {
        FreeMem(sl, sizeof(*sl));
    } else {
        /* Send, then wait for, startup message */
        struct IORequest io;
        io.io_Message.mn_ReplyPort = CreateMsgPort();
        io.io_Message.mn_Length = sizeof(io);
        io.io_Command = CMD_INVALID;
        PutMsg(sl->sl_CommandPort, (struct Message *)&io);
        WaitPort(io.io_Message.mn_ReplyPort);
        GetMsg(io.io_Message.mn_ReplyPort);
        DeleteMsgPort(io.io_Message.mn_ReplyPort);
    }

    return sl;
}

void sl811hs_Detach(struct sl811hs *sl)
{
    struct IORequest io;

    /* Send the 'death' IORequest */
    io.io_Message.mn_Node.ln_Type = NT_MESSAGE;
    io.io_Message.mn_ReplyPort = CreateMsgPort();
    io.io_Unit = (APTR)sl;
    io.io_Command = 0xffff;
    PutMsg(sl->sl_CommandPort, (struct Message *)&io);
    WaitPort(io.io_Message.mn_ReplyPort);
    GetMsg(io.io_Message.mn_ReplyPort);
    DeleteMsgPort(io.io_Message.mn_ReplyPort);

    /* Return to power-on state */
    wb(sl, SL811HS_HOSTCTRL, 0);
    wb(sl, SL811HS_HOSTCTRL+8, 0);
    wb(sl, SL811HS_CONTROL1, 0);

    FreeMem(sl, sizeof(*sl));
}
