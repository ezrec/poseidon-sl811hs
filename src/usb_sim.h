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

#ifndef USB_SIM_H
#define USB_SIM_H

#include <exec/types.h>
#include <exec/nodes.h>

#include <sys/types.h>

#define PID_OUT     0x1
#define PID_IN      0x9
#define PID_SOF     0x5
#define PID_SETUP   0xd
#define PID_DATA0   0x3
#define PID_DATA1   0xb
#define PID_ACK     0x2
#define PID_NAK     0xa
#define PID_STALL   0xe

struct USBSim {
    struct Node us_Node;
    void (*reset)(struct USBSim *sim);
    void (*out)(struct USBSim *sim, UBYTE pid, const UBYTE *packet, size_t len);
    void (*in)(struct USBSim *sim, UBYTE *pidp, UBYTE *packet, size_t maxlen);
};

static inline void usbsim_Reset(struct USBSim *sim)
{
    sim->reset(sim);
}

static inline void usbsim_Out(struct USBSim *sim, UBYTE pid, const UBYTE *packet, size_t len)
{
    sim->out(sim, pid, packet, len);
}

static inline void usbsim_In(struct USBSim *sim, UBYTE *pidp, UBYTE *packet, size_t maxlen)
{
    sim->in(sim, pidp, packet, maxlen);
}

#endif /* USB_SIM_H */
