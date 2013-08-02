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

#ifndef SL811HS_H
#define SL811HS_H

#include <exec/libraries.h>
#include <exec/io.h>

#define SL811HS_CP_ADDR       0xd80001
#define SL811HS_CP_DATA       0xd80005

/********* USB Host Registers **************
 * USB-A at N + 0x0
 * USB-B at N + 0x8
 */

#define SL811HS_HOSTCTRL        0x0
#define   SL811HS_HOSTCTRL_PREAMBLE     (1 << 7)
#define   SL811HS_HOSTCTRL_DATA         (1 << 6)
#define     SL811HS_HOSTCTRL_DATA0      (0 << 6)
#define     SL811HS_HOSTCTRL_DATA1      (1 << 6)
#define   SL811HS_HOSTCTRL_SYNCSOF      (1 << 5)
#define   SL811HS_HOSTCTRL_ISO          (1 << 4)
#define   SL811HS_HOSTCTRL_DIR          (1 << 2)
#define     SL811HS_HOSTCTRL_DIR_IN     (0 << 2)
#define     SL811HS_HOSTCTRL_DIR_OUT    (1 << 2)
#define   SL811HS_HOSTCTRL_ENABLE       (1 << 1)
#define   SL811HS_HOSTCTRL_ARM          (1 << 0)

#define SL811HS_HOSTBASE        0x1

#define SL811HS_HOSTLEN         0x2

#define SL811HS_HOSTSTATUS      0x3     /* On Read */
#define   SL811HS_HOSTSTATUS_STALL      (1 << 7)
#define   SL811HS_HOSTSTATUS_NAK        (1 << 6)
#define   SL811HS_HOSTSTATUS_OVERFLOW   (1 << 5)
#define   SL811HS_HOSTSTATUS_SETUP      (1 << 4)
#define   SL811HS_HOSTSTATUS_SEQ        (1 << 3)
#define   SL811HS_HOSTSTATUS_TIMEOUT    (1 << 2)
#define   SL811HS_HOSTSTATUS_ERROR      (1 << 1)
#define   SL811HS_HOSTSTATUS_ACK        (1 << 0)

#define SL811HS_HOSTID          0x3     /* On Write */
#define   SL811HS_HOSTID_PIDEP(pid,ep)  ((((pid) & 0xf) << 4) | ((ep) & 0xf))
#define   SL811HS_HOSTID_PID_of(x)      (((x) >> 4) & 0xf)
#define   SL811HS_HOSTID_EP_of(x)       ((x) & 0xf)

#define   SL811HS_PID_SETUP             0xd
#define   SL811HS_PID_IN                0x9
#define   SL811HS_PID_OUT               0x1
#define   SL811HS_PID_SOF               0x5
#define   SL811HS_PID_PREAMBLE          0xc
#define   SL811HS_PID_NAK               0xa
#define   SL811HS_PID_STALL             0xe
#define   SL811HS_PID_DATA0             0x3
#define   SL811HS_PID_DATA1             0xb

#define SL811HS_HOSTTXLEFT      0x4     /* On Read */

#define SL811HS_HOSTDEVICEADDR  0x4     /* On Write */


/********* Control Registers **************/

#define SL811HS_CONTROL1        0x5
#define   SL811HS_CONTROL1_SUSPEND      (1 << 6)
#define   SL811HS_CONTROL1_LOW_SPEED    (1 << 5)
#define   SL811HS_CONTROL1_JK_FORCE     (1 << 4)
#define   SL811HS_CONTROL1_USB_RESET    (1 << 3)
#define   SL811HS_CONTROL1_EOF2         (1 << 2)        /* OBSOLETE */
#define   SL811HS_CONTROL1_SOF_ENABLE   (1 << 0)

#define SL811HS_INTENABLE       0x6
#define SL811HS_INTSTATUS       0xd
#define  SL811HS_INTMASK_FULLSPEED    (1 << 7)        /* Status only, tied to USB D+ */
#define  SL811HS_INTMASK_DETECT       (1 << 6)
#define  SL811HS_INTMASK_CHANGED      (1 << 5)        /* Insert/Remove */
#define  SL811HS_INTMASK_SOF_TIMER    (1 << 4)
#define  SL811HS_INTMASK_USB_B        (1 << 1)
#define  SL811HS_INTMASK_USB_A        (1 << 0)

#define SL811HS_HWREVISION      0xe     /* On Read */
#define   SL811HS_HWREVISION_2_0        0x20

#define SL811HS_SOFLOW          0xe     /* On Write */

#define SL811HS_SOFHIGH         0xf     /* On Read */

#define SL811HS_CONTROL2        0xf     /* On Write */
#define  SL811HS_CONTROL2_MASTER        (1 << 7)
#define  SL811HS_CONTROL2_LOW_SPEED     (1 << 6)
#define  SL811HS_CONTROL2_SOF_HIGH(x)   ((x) & 0x3f)

/* This is a 'struct Node' internally,
 * so feel free to use it in a list.
 */
struct sl811hs;

struct sl811hs *sl811hs_Attach(IPTR addr, IPTR data, int irq);

void sl811hs_Detach(struct sl811hs *sl);

void sl811hs_BeginIO(struct sl811hs *sl, struct IORequest *ior);
LONG sl811hs_AbortIO(struct sl811hs *sl, struct IORequest *ior);

#endif /* SL811HS_H */
