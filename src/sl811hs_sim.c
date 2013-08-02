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
#include <exec/interrupts.h>

#include "sl811hs.h"
#include "sl811hs_sim.h"

#include "usb_sim.h"
#include "massbulk_sim.h"

void sl811hs_sim_Init(struct sl811hs_sim *ss, struct Interrupt *ihook)
{
    int i;
    ss->ss_Interrupt = ihook;

    for (i = 0; i < 256; i++)
        ss->ss_Reg[i] = 255-i;

    ss->ss_Reg[SL811HS_INTSTATUS] = 0x73;
    ss->ss_Reg[SL811HS_HOSTCTRL+0] &= ~1;
    ss->ss_Reg[SL811HS_HOSTCTRL+8] &= ~1;
    ss->ss_Reg[SL811HS_CONTROL1] = 0;
    ss->ss_Reg[7] = 0;  /* Unused in Master mode */
    ss->ss_Reg[SL811HS_SOFLOW] = 0;

    ss->ss_HostStatus[0] = 0;
    ss->ss_HostStatus[1] = 1;

    ss->ss_InIrq = FALSE;

    if (!ss->ss_Port)
        ss->ss_Port = massbulk_Attach();
}

UBYTE sl811hs_sim_Read(struct sl811hs_sim *ss, int a0)
{
    UBYTE val;

    if (a0 == 0) {
        val = ss->ss_Addr;
    } else {
        switch (ss->ss_Addr) {
        case SL811HS_HWREVISION:
            val = 0x20;
            break;
        case SL811HS_HOSTSTATUS+0:
            val = ss->ss_HostStatus[0];
            break;
        case SL811HS_HOSTSTATUS+8:
            val = ss->ss_HostStatus[1];
            break;
        default:
            val = ss->ss_Reg[ss->ss_Addr];
            break;
        }
        ss->ss_Addr++;
    }

    return val;
}

void  sl811hs_sim_Write(struct sl811hs_sim *ss, int a0, UBYTE val)
{
    UBYTE mask;
    BOOL update = FALSE;

    if (a0 == 0) {
        ss->ss_Addr = val;
    } else {
        D(bug("%s: %02x = %02x\n",  __func__, ss->ss_Addr, val));
        if (ss->ss_Addr == SL811HS_INTSTATUS) {
            ss->ss_Reg[ss->ss_Addr] &= ~val;
        } else {
            ss->ss_Reg[ss->ss_Addr] = val;
        }
        if (ss->ss_Addr == SL811HS_INTSTATUS ||
            ss->ss_Addr == SL811HS_HOSTCTRL+0 ||
            ss->ss_Addr == SL811HS_HOSTCTRL+8)
            update = TRUE;
        ss->ss_Addr++;
    }

    if (!update)
        return;

    /* Perform any updates at this time */

    /* If not in USB reset, update the external device simulations
     */
    if (!(ss->ss_Reg[SL811HS_CONTROL1] & SL811HS_CONTROL1_USB_RESET)) {
        int i;
        for (i = 0; i < 16; i+=8) {
            D(bug("%s: Update USB%c state\n", __func__, i ? 'B' : 'A'));
            UBYTE hc = ss->ss_Reg[SL811HS_HOSTCTRL+i];
            BOOL isEnabled = (hc & SL811HS_HOSTCTRL_ENABLE) ? TRUE : FALSE;
            BOOL isArmed = (hc & SL811HS_HOSTCTRL_ARM) ? TRUE : FALSE;

            if (isArmed & isEnabled) {
                UBYTE buff[256];
                UBYTE ctl = ss->ss_Reg[SL811HS_HOSTCTRL+i];
                int ep  = SL811HS_HOSTID_PID_of(ss->ss_Reg[SL811HS_HOSTID+i]);
                UBYTE pid = SL811HS_HOSTID_PID_of(ss->ss_Reg[SL811HS_HOSTID+i]);
                D(bug("%s: Send USB%c command\n", __func__, i ? 'B' : 'A'));
                buff[0] = ss->ss_Reg[SL811HS_HOSTDEVICEADDR] | ((ep & 1) << 7);
                buff[1] = ((ep & 0xe) << 4) | 0;    /* CRC5 is ignored */
                usbsim_Out(ss->ss_Port, pid, buff, 2);
                switch (pid) {
                case PID_SETUP:
                case PID_OUT:
                    usbsim_Out(ss->ss_Port, (ctl & SL811HS_HOSTCTRL_DATA) ? PID_DATA1 : PID_DATA0, &ss->ss_Reg[ss->ss_Reg[SL811HS_HOSTBASE]], ss->ss_Reg[SL811HS_HOSTLEN]);
                    usbsim_In(ss->ss_Port, &pid, NULL, 0);
                    switch (pid) {
                    case PID_ACK:
                        ss->ss_HostStatus[i/8] = SL811HS_HOSTSTATUS_ACK;
                        break;
                    case PID_NAK:
                        ss->ss_HostStatus[i/8] = SL811HS_HOSTSTATUS_NAK;
                        break;
                    case PID_STALL:
                        ss->ss_HostStatus[i/8] = SL811HS_HOSTSTATUS_STALL;
                        break;
                    default:
                        ss->ss_HostStatus[i/8] = SL811HS_HOSTSTATUS_ERROR;
                    }
                    break;
                case PID_IN:
                    ss->ss_HostStatus[i/8] = 0;
                    usbsim_In(ss->ss_Port, &pid, &ss->ss_Reg[ss->ss_Reg[SL811HS_HOSTBASE]], ss->ss_Reg[SL811HS_HOSTLEN]);
                    switch (pid) {
                    case PID_DATA0:
                    case PID_DATA1:
                        ss->ss_HostStatus[i/8] |= SL811HS_HOSTSTATUS_ACK;
                        if (pid == PID_DATA1)
                            ss->ss_HostStatus[i/8] |= SL811HS_HOSTSTATUS_SEQ;
                        usbsim_Out(ss->ss_Port, PID_ACK, NULL, 0);
                        break;
                    case PID_STALL:
                        ss->ss_HostStatus[i/8] |= SL811HS_HOSTSTATUS_STALL;
                        break;
                    case PID_NAK:
                        ss->ss_HostStatus[i/8] |= SL811HS_HOSTSTATUS_NAK;
                        break;
                    }
                    break;
                default:
                    D(bug("%s: What what? I didn't expect a PID=0x%x\n", __func__, pid));
                    ss->ss_HostStatus[i/8] = SL811HS_HOSTSTATUS_ERROR;
                }

                ss->ss_Reg[SL811HS_HOSTCTRL+i] &= ~SL811HS_HOSTCTRL_ARM;
                ss->ss_Reg[SL811HS_INTSTATUS] |= (i == 0) ? SL811HS_INTMASK_USB_A : SL811HS_INTMASK_USB_B;
            }
        }
    } else {
        int i;
        usbsim_Reset(ss->ss_Port);
        for (i = 0; i < 2; i++) {
            D(bug("%s: Reset USB%c state\n", __func__, i ? 'B' : 'A'));
            ss->ss_HostStatus[i] = 0;
        }
    }

    /* Signals any pending interrupts */
    mask = ((ss->ss_Reg[SL811HS_CONTROL1] & SL811HS_CONTROL1_SUSPEND) ? SL811HS_INTMASK_DETECT : 0) |
           SL811HS_INTMASK_CHANGED |
           
           ((ss->ss_Reg[SL811HS_CONTROL1] & SL811HS_CONTROL1_SOF_ENABLE) ? SL811HS_INTMASK_SOF_TIMER : 0) |
           SL811HS_INTMASK_USB_B |
           SL811HS_INTMASK_USB_A;

    if (!ss->ss_InIrq) {
        ss->ss_InIrq = TRUE;
        D(bug("%s: Call interrupt? IS=%02x, IE=%02x, IM=%02x = %02x\n", __func__, ss->ss_Reg[SL811HS_INTSTATUS], ss->ss_Reg[SL811HS_INTENABLE], mask, (ss->ss_Reg[SL811HS_INTSTATUS] & ss->ss_Reg[SL811HS_INTENABLE]) & mask));
        while ((ss->ss_Reg[SL811HS_INTSTATUS] & ss->ss_Reg[SL811HS_INTENABLE]) & mask) {
            D(bug("%s: Call interrupt! IS=%02x, IE=%02x, IM=%02x = %02x\n", __func__, ss->ss_Reg[SL811HS_INTSTATUS], ss->ss_Reg[SL811HS_INTENABLE], mask, (ss->ss_Reg[SL811HS_INTSTATUS] & ss->ss_Reg[SL811HS_INTENABLE]) & mask));
            AROS_INTC3(ss->ss_Interrupt->is_Code, ss->ss_Interrupt->is_Data, (1 << 6), (APTR)0xdff000);
        }
        ss->ss_InIrq = FALSE;
    } else {
        D(bug("%s: In IRQ\n", __func__));
    }
}
