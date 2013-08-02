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

#include <proto/exec.h>

#include <devices/usb.h>

#include "massbulk_sim.h"

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

static UBYTE massbulk_SetupOut(struct USBSimMass *sm, struct massbulk_Endpoint *ep)
{
    return PID_NAK;
}

static UBYTE massbulk_SetupIn(struct USBSimMass *sm, struct massbulk_Endpoint *ep)
{
    return PID_NAK;
}

static void massbulk_Out(struct USBSim *sim, UBYTE pid, const UBYTE *buff, size_t len)
{
    struct USBSimMass *sm = (struct USBSimMass *)sim;
    int i;
    struct massbulk_Endpoint *ep;

    if (pid != PID_DATA0 && pid != PID_DATA1) {
        int epid;

        epid = ((buff[1] >> 4) & 0xe) | ((buff[0] >> 7) & 1);

        if (epid >= 3)
            return;

        sm->sm_Endpoint = &sm->sm_EP[epid];
    }

    ep = sm->sm_Endpoint;

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
                massbulk_SetupIn(sm, ep);
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
}

static void massbulk_In(struct USBSim *sim, UBYTE *pidp, UBYTE *buff, size_t len)
{
    struct USBSimMass *sm = (struct USBSimMass *)sim;
    struct massbulk_Endpoint *ep;

    ep = sm->sm_Endpoint;
    *pidp = ep->ep_Reply;
 
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
            ep->ep_Reply = massbulk_SetupOut(sm, ep);
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
