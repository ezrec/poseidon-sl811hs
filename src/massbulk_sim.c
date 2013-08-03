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

#include <proto/exec.h>

#include <devices/usb.h>

#include "massbulk_sim.h"

#undef D2
#if DEBUG >= 2
#define D2(x)   x
#else
#define D2(x)
#endif

#if DEBUG >= 2
#define ebug(fmt, args...) do { bug("%s:%d ", __func__, __LINE__); bug(fmt ,##args); } while (0)
#else
#define ebug(fmt, args...) do { bug(fmt ,##args); } while (0)
#endif

struct USBSimMass {
    struct USBSim sm_USBSim;

    UBYTE sm_DevAddr;

    struct massbulk_Endpoint {
#define STATE_IDLE              0
#define STATE_SETUP             1
#define STATE_SETUP_IN          2
#define STATE_SETUP_OUT         3
#define STATE_IN                4
#define STATE_OUT               5
        UBYTE ep_State;
        UBYTE ep_Toggle;
        UBYTE ep_Reply;
        UBYTE ep_BuffReady;
        UBYTE ep_BuffPtr;
        UBYTE ep_BuffLen;
        UBYTE ep_Buff[256];
        struct UsbSetupData ep_SetupData;
    } sm_EP[3], *sm_Endpoint;
};

#define EP_CONTROL      0
#define EP_BULK_IN      1
#define EP_BULK_OUT     2

#ifdef AROS_BIG_ENDIAN
#define CONST_WORD2LE(x) ((((x) & 0x00ff) <<  8) | \
                         (((x) & 0xff00) >>  8))
#define CONST_LONG2LE(x) ((((x) & 0x000000ff) << 24) | \
                          (((x) & 0x0000ff00) <<  8) | \
                          (((x) & 0x00ff0000) >>  8) | \
                          (((x) & 0xff000000) >> 24))
#else
#define CONST_WORD2LE(x) (x)
#define CONST_LONG2LE(x) (x)
#endif

struct UsbStdDevDesc const massbulk_DevDesc = {
    .bLength = sizeof(struct UsbStdDevDesc),
    .bDescriptorType = UDT_DEVICE,
    .bcdUSB = CONST_WORD2LE(0x0200),
    .bDeviceClass = 0,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0, /* Full speed hub */
    .bMaxPacketSize0 = 64,
    .idVendor = CONST_WORD2LE(0x048d), /* Integrated Technology Express, Inc */
    .idProduct = CONST_WORD2LE(0x1336), /* SD/MMC Cardreader */
    .bcdDevice = CONST_WORD2LE(0x0100),        /* Version 1.0 */
    .iManufacturer = 1,         /* String index 1 */
    .iProduct = 2,              /* String index 2 */
    .iSerialNumber = 3,
    .bNumConfigurations = 1
};

struct UsbStdCfgDesc const massbulk_CfgDesc = {
    .bLength = sizeof(struct UsbStdCfgDesc),
    .bDescriptorType = UDT_CONFIGURATION,
    .wTotalLength = CONST_WORD2LE(sizeof(struct UsbStdCfgDesc) + sizeof(struct UsbStdIfDesc) + sizeof(struct UsbStdEPDesc)*2),
    .bNumInterfaces = 1,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = USCAF_ONE,
    .bMaxPower = 100/2,     /* Self-powered */
};

struct UsbStdIfDesc const massbulk_IntDesc = {
    .bLength = sizeof(struct UsbStdIfDesc),
    .bDescriptorType = UDT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = MASSSTORE_CLASSCODE,
    .bInterfaceSubClass = 6,    /* SCSI */
    .bInterfaceProtocol = 0x50,   /* Bulk-Only */
    .iInterface = 0,
};

struct UsbStdEPDesc const massbulk_EPDesc[2] = {
    {
        .bLength= sizeof(struct UsbStdEPDesc),
        .bDescriptorType = UDT_ENDPOINT,
        .bEndpointAddress = 0x01,
        .bmAttributes = 2,
        .wMaxPacketSize = CONST_WORD2LE(0x200),
        .bInterval = 0
    }, {
        .bLength= sizeof(struct UsbStdEPDesc),
        .bDescriptorType = UDT_ENDPOINT,
        .bEndpointAddress = 0x82,
        .bmAttributes = 2,
        .wMaxPacketSize = CONST_WORD2LE(0x200),
        .bInterval = 0
    }
};

struct smUsbStdStrDesc
{
    UBYTE bLength;             /* Size of this descriptor in bytes */
    UBYTE bDescriptorType;     /* UDT_STRING Descriptor Type */
    UWORD bString[12];         /* UNICODE encoded string */
};

#define CBW_SIGNATURE   0x43425355

struct smCBW {
    ULONG   dCBWSignature;
    ULONG   dCBWTag;
    ULONG   dCBWDataTransferLength;
    UBYTE   bmCBWFlags;
#define CBWFLAG_DIRECTION   0x80
#define   CBWFLAG_DIRECTION_IN   0x80
#define   CBWFLAG_DIRECTION_OUT  0x00
    UBYTE   bCBWLUN;
    UBYTE   bCBWCBLength;
    UBYTE   CBWCB[16];
};

#define CSW_SIGNATURE   0x53425355

struct smCSW {
    ULONG   dCSWSignature;
    ULONG   dCSWTag;
    ULONG   dCSWDataResidue;
    UBYTE   dCSWStatus;
#define CSWSTATUS_PASSED    0x00
#define CSWSTATUS_FAILED    0x01
#define CSWSTATUS_PHASE     0x02
};

struct smUsbStdStrDesc const massbulk_StrDesc[] = {
    {
        .bLength = sizeof(struct UsbStdStrDesc) + (1-1) * sizeof(UWORD),
        .bDescriptorType = UDT_STRING,
        .bString = { CONST_WORD2LE(0x0409) }, /* English, United States */
    }, {
        .bLength = sizeof(struct UsbStdStrDesc) + (7-1) * sizeof(UWORD),
        .bDescriptorType = UDT_STRING,
        .bString = { CONST_WORD2LE('S'), CONST_WORD2LE('i'),
                     CONST_WORD2LE('m'), CONST_WORD2LE('B'),
                     CONST_WORD2LE('u'), CONST_WORD2LE('l'),
                     CONST_WORD2LE('k') },
    }, {
        .bLength = sizeof(struct UsbStdStrDesc) + (7-1) * sizeof(UWORD),
        .bDescriptorType = UDT_STRING,
        .bString = { CONST_WORD2LE('M'), CONST_WORD2LE('a'),
                     CONST_WORD2LE('s'), CONST_WORD2LE('s'),
                     CONST_WORD2LE('D'), CONST_WORD2LE('r'),
                     CONST_WORD2LE('v') },
    }, {
        .bLength = sizeof(struct UsbStdStrDesc) + (12-1) * sizeof(UWORD),
        .bDescriptorType = UDT_STRING,
        .bString = { CONST_WORD2LE('1'), CONST_WORD2LE('2'),
                     CONST_WORD2LE('3'), CONST_WORD2LE('4'),
                     CONST_WORD2LE('5'), CONST_WORD2LE('6'),
                     CONST_WORD2LE('7'), CONST_WORD2LE('8'),
                     CONST_WORD2LE('9'), CONST_WORD2LE('a'),
                     CONST_WORD2LE('b'), CONST_WORD2LE('c'),
                   },
    }
};

static UBYTE sm_AppendData(struct massbulk_Endpoint *ep, UWORD *lengthp, int desc_len, CONST_APTR desc)
{
    int err;
    int length = *lengthp;
    int len = ep->ep_BuffLen - ep->ep_BuffPtr;
    UBYTE *data = &ep->ep_Buff[ep->ep_BuffPtr];

    D2(ebug("Append %d bytes to buffer (%d left of %d), want to send %d\n", desc_len, len, ep->ep_BuffLen, length));

    if (len != length) {
        D2(ebug("Setup length %d, expected %d\n", length, len));
    }
    if (length > len) {
        err = PID_NAK;
    } else if (length < len) {
        len = length;
        err = PID_NAK;
    } else {
        err = PID_ACK;
    }

    if (len > desc_len)
        len = desc_len;

    CopyMem(desc, data, len);
    length -= len;
    ep->ep_BuffPtr += len;

    *lengthp = length;

    return err;
}

#define CTLREQ(type,req)        (((type) << 8) | (req))

static UBYTE massbulk_SetupInOut(struct USBSimMass *sm, struct massbulk_Endpoint *ep)
{
    struct UsbSetupData *setup = &ep->ep_SetupData;
    UBYTE err = PID_NAK;
    UWORD value, index, length;
    UBYTE buff[4];

    value = AROS_LE2WORD(setup->wValue);
    index = AROS_LE2WORD(setup->wIndex);
    length = AROS_LE2WORD(setup->wLength);

    D2(ebug("%p value=$%04x, index=$%04x, length=$%04x\n", ep, value, index, length));
    D2(ebug("%p bmRequestType=$%02x, bRequest=$%02x\n", ep, setup->bmRequestType, setup->bRequest));
    switch (CTLREQ(setup->bmRequestType, setup->bRequest)) {
    case CTLREQ(URTF_OUT | URTF_STANDARD | URTF_DEVICE, USR_SET_ADDRESS):
        D2(ebug("SetAddress: %d\n", value));
        sm->sm_DevAddr = value;
        err = 0;
        break;
    case CTLREQ(URTF_IN | URTF_STANDARD | URTF_DEVICE, USR_GET_DESCRIPTOR):
        D2(ebug("GetDescriptor: %d [%d]\n", (value>>8) & 0xff, index));
        switch ((value>>8) & 0xff) {
        case UDT_DEVICE:
            err = sm_AppendData(ep, &length, sizeof(massbulk_DevDesc), &massbulk_DevDesc);
            break;
        case UDT_CONFIGURATION:
            err = sm_AppendData(ep, &length, sizeof(massbulk_CfgDesc), &massbulk_CfgDesc);
            if (err == 0 && length > 0)
                err = sm_AppendData(ep, &length, sizeof(massbulk_IntDesc), &massbulk_IntDesc);
            if (err == 0 && length > 0)
                err = sm_AppendData(ep, &length, sizeof(massbulk_EPDesc), &massbulk_EPDesc[0]);
            if (err == 0 && length > 0)
                err = sm_AppendData(ep, &length, sizeof(massbulk_EPDesc), &massbulk_EPDesc[1]);
            break;
        case UDT_INTERFACE:
            err = sm_AppendData(ep, &length, sizeof(massbulk_IntDesc), &massbulk_IntDesc);
            break;
        case UDT_ENDPOINT:
            err = sm_AppendData(ep, &length, sizeof(massbulk_EPDesc), &massbulk_EPDesc);
            break;
        case UDT_STRING:
            if ((value & 0xff) <= 3) {
                err = sm_AppendData(ep, &length, massbulk_StrDesc[value & 0xff].bLength, &massbulk_StrDesc[value & 0xff]);
            }
            break;
        default:
            break;
        }

        break;
    case CTLREQ(URTF_IN | URTF_STANDARD | URTF_DEVICE, USR_GET_CONFIGURATION):
        D2(ebug("GetConfiguration: %d [%d]\n", value, index));
        buff[0] = 0;
        err = sm_AppendData(ep, &length, 1, buff);
        break;
    case CTLREQ(URTF_IN | URTF_STANDARD | URTF_DEVICE, USR_GET_STATUS): /* GetStatus */
        D2(ebug("GetDeviceStatus: %d [%d]\n", value, index));
        if (value == 0 && index == 0) {
            buff[0] = 1;        /* Self Powered */
            buff[1] = 0;
            err = sm_AppendData(ep, &length, 2, buff);
        }
        break;
    case CTLREQ(URTF_IN | URTF_STANDARD | URTF_INTERFACE, USR_GET_STATUS): /* GetStatus */
        D2(ebug("GetInterfaceStatus: %d [%d]\n", value, index));
        if (value == 0 && index == 0) {
            buff[0] = 0;        
            buff[1] = 0;
            err = sm_AppendData(ep, &length, 2, buff);
        }
        break;
    case CTLREQ(URTF_IN | URTF_STANDARD | URTF_ENDPOINT, USR_GET_STATUS): /* GetStatus */
        D2(ebug("GetEndpointStatus: %d [%d]\n", value, index));
        if (value == 0 && index == 0) {
            buff[0] = 0;        /* Not halted */ 
            buff[1] = 0;
            err = sm_AppendData(ep, &length, 2, buff);
        }
        break;
    case CTLREQ(URTF_OUT | URTF_CLASS | URTF_INTERFACE, 0xff): /* Bulk-Only Mass Storage Reset */
        if (value == 0 && length == 0)
            err = PID_ACK;
        else
            err = PID_NAK;
        break;
    case CTLREQ(URTF_OUT | URTF_CLASS | URTF_INTERFACE, 0xfe): /* Get Max Lun */
        if (value == 0 && length == 1) {
            buff[0] = 0;
            err = sm_AppendData(ep, &length, 1, buff);
        }
        err = PID_ACK;
        break;
    default:
        D(ebug("Unknown request - NAK\n"));
        err = PID_NAK;
        break;
    }

    D2(ebug("Return %d\n", err));
    return err;
}

static void massbulk_Out(struct USBSim *sim, UBYTE pid, const UBYTE *buff, size_t len)
{
    struct USBSimMass *sm = (struct USBSimMass *)sim;
    int i;
    struct massbulk_Endpoint *ep;

    if (pid != PID_DATA0 && pid != PID_DATA1) {
        int epid;

        epid = ((buff[1] >> 4) & 0xe) | ((buff[0] >> 7) & 1);
        D(ebug("EP = %x\n", epid));

        if (epid >= 3)
            return;

        sm->sm_Endpoint = &sm->sm_EP[epid];
    }

    ep = sm->sm_Endpoint;

    D(ebug("OUT PID %x, State %d\n", pid, ep->ep_State));

    switch (pid) {
    case PID_SETUP:
        for (i = 0; i < 3; i++) {
            sm->sm_EP[i].ep_Toggle = FALSE;
            sm->sm_EP[i].ep_Reply = 0;
            sm->sm_EP[i].ep_BuffPtr = 0;
            sm->sm_EP[i].ep_BuffReady = 0;
            sm->sm_EP[i].ep_State = STATE_IDLE;
        }
        ep->ep_State = STATE_SETUP;
        ep->ep_Reply = PID_NAK;
        break;
    case PID_IN:
        ep->ep_Reply = (ep->ep_Toggle ? PID_DATA1 : PID_DATA0);
        if (ep->ep_State != STATE_SETUP)
            ep->ep_State = STATE_IN;
        break;
    case PID_OUT:
        if (ep->ep_State != STATE_SETUP)
            ep->ep_State = STATE_OUT;
        ep->ep_Reply = PID_NAK;
        break;
    case PID_DATA0:
    case PID_DATA1:
        if (((pid == PID_DATA1) && !ep->ep_Toggle) ||
            ((pid == PID_DATA0) && ep->ep_Toggle)) {
            ep->ep_Reply = PID_NAK;
        } else if (ep->ep_State == STATE_SETUP && len == 8) {
            CopyMem(buff, &ep->ep_SetupData, len);
            ep->ep_State = (ep->ep_SetupData.bmRequestType & 0x80) ? STATE_SETUP_IN : STATE_SETUP_OUT;
            if (ep->ep_State == STATE_SETUP_IN) {
                massbulk_SetupInOut(sm, ep);
            } else {
                ep->ep_BuffReady = TRUE;
                ep->ep_BuffPtr = 0;
            }
            ep->ep_Reply = PID_ACK;
        } else if (ep->ep_State == STATE_SETUP_OUT && ep->ep_BuffReady) {
            if ((ep->ep_BuffPtr + len) <= AROS_LE2WORD(ep->ep_SetupData.wLength)) {
                CopyMem(buff, &ep->ep_Buff[ep->ep_BuffPtr], len);
                ep->ep_BuffPtr += len;
                ep->ep_Reply = PID_ACK;
            } else {
                ep->ep_Reply = PID_NAK;
            }
        } else if (ep->ep_State == STATE_SETUP_IN && len == 0) {
            ep->ep_State = STATE_IDLE;
            ep->ep_Reply = PID_ACK;
        } else if (ep->ep_State == STATE_OUT) {
            if ((ep->ep_BuffPtr + len) <= sizeof(ep->ep_Buff)) {
                CopyMem(buff, &ep->ep_Buff[ep->ep_BuffPtr], len);
                ep->ep_BuffPtr += len;
                ep->ep_Reply = PID_ACK;
            } else {
                ep->ep_Reply = PID_NAK;
            }
        } else {
            ep->ep_Reply = PID_NAK;
        }
        break;
    case PID_ACK:
        ep->ep_Reply = PID_STALL;
        ep->ep_Toggle = !ep->ep_Toggle;
        break;
    default:
        ep->ep_Reply = PID_STALL;
        ep->ep_State = STATE_IDLE;
        break;
    }

    D(ebug("OUT PID %x, State %d, Reply %d\n", pid, ep->ep_State, ep->ep_Reply));
}

static void massbulk_In(struct USBSim *sim, UBYTE *pidp, UBYTE *buff, size_t len)
{
    struct USBSimMass *sm = (struct USBSimMass *)sim;
    struct massbulk_Endpoint *ep;

    ep = sm->sm_Endpoint;
    *pidp = ep->ep_Reply;
 
    D(ebug("IN PID %x, State %d\n", *pidp, ep->ep_State));

    switch (ep->ep_Reply) {
    case PID_DATA0:
    case PID_DATA1:
        if ((ep->ep_State == STATE_SETUP_IN || ep->ep_State == STATE_IN) &&
                ep->ep_BuffReady) {
            if ((ep->ep_BuffPtr + len) <= ep->ep_BuffLen) {
                CopyMem(buff, &ep->ep_Buff[ep->ep_BuffPtr], len);
                ep->ep_BuffPtr += len;
                ep->ep_Reply = PID_ACK;
            } else {
                ep->ep_Reply = PID_NAK;
            }
        } else if (ep->ep_State == STATE_SETUP_OUT && len == 0) {
            ep->ep_Reply = massbulk_SetupInOut(sm, ep);
        } else {
            ep->ep_Reply = PID_NAK;
        }
        break;
    case PID_ACK:
        ep->ep_Reply = PID_NAK;

        break;
    case PID_NAK:
        break;
    }

    D(ebug("IN PID %x, State %d, Reply %d\n", *pidp, ep->ep_State, ep->ep_Reply));
}

struct USBSim *massbulk_Attach(void)
{
    struct USBSimMass *sm;

    sm = AllocMem(sizeof(*sm), MEMF_ANY | MEMF_CLEAR);
    sm->sm_USBSim.out = massbulk_Out;
    sm->sm_USBSim.in = massbulk_In;

    sm->sm_Endpoint = &sm->sm_EP[0];

    return &sm->sm_USBSim;
}

void massbulk_Detach(struct USBSim *sim)
{
    struct USBSimMass *sm = (struct USBSimMass *)sim;

    FreeMem(sm, sizeof(*sm));
}
