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

#include <hardware/intbits.h>

#include "sl811hs.h"
#include "pathway_intern.h"


/* Unit numbers 0..15 are reserved for
 * clockports at fixed locations.
 *
 * Unit number 16 is the debug (simulation) unit,
 * which has a Mass Storage Bulk-only simulation.
 */
struct pathway_base { ULONG addr; ULONG data; LONG irq; } pb_Base[17] = {
    { 0xd80001, 0xd80005, INTB_EXTER },  /* Unit 0: A1200 clockport */
    { 0xd84001, 0xd84005, INTB_EXTER },  /* Unit 1: Zorro IV */
    { 0xd88001, 0xd88005, INTB_EXTER },  /* Unit 2: Zorro IV */
    { 0xd8c001, 0xd8c005, INTB_EXTER },  /* Unit 3: Zorro IV */
    { 0xd90001, 0xd90005, INTB_EXTER },  /* Unit 4: A604 2nd port */
};

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))
#endif

static int pathway_Init(struct PathwayBase *pb)
{
#if DEBUG
    /* Nasty hack to dump serial debug at 115200n81 */
#ifdef __mc68000
#define SERPER_BASE_NTSC 3579545
#define SERPER_BASE_PAL 3546895
#define SERPER_BAUD(base, x)  ((((base + (x)/2))/(x)-1) & 0x7fff)
    *(volatile UWORD *)(0xdff000 + 0x32) = SERPER_BAUD(SERPER_BASE_PAL, 115200);
#endif
#endif
    InitSemaphore(&pb->pb_UnitLock);
    pb->pb_Unit = AllocMem(sizeof(struct sl811hs *) * ARRAY_SIZE(pb_Base), MEMF_ANY | MEMF_CLEAR);

    return (pb->pb_Unit == NULL) ? 0 : 1;
}

static int pathway_Expunge(struct PathwayBase *pb)
{
    if (pb->pb_Unit) {
        int i;
        for (i = 0; i < ARRAY_SIZE(pb_Base); i++)
            sl811hs_Detach(pb->pb_Unit[i]);
        FreeMem(pb->pb_Unit, sizeof(struct sl811hs *) * ARRAY_SIZE(pb_Base));
    }

    return 1;
}

ADD2INITLIB(pathway_Init, 0);
ADD2EXPUNGELIB(pathway_Expunge, 0);

AROS_LH1(void, BeginIO,
    AROS_LHA(struct IORequest *, io, A1),
    struct PathwayBase *, pb, 5, pathway)
{
    AROS_LIBFUNC_INIT

    struct sl811hs *sl = (struct sl811hs *)(io->io_Unit);

    return sl811hs_BeginIO(sl, io);

    AROS_LIBFUNC_EXIT
}

AROS_LH1(LONG, AbortIO,
    AROS_LHA(struct IORequest *, io, A1),
    struct PathwayBase *, pb, 6, pathway)
{
    AROS_LIBFUNC_INIT

    struct sl811hs *sl;

    sl = (struct sl811hs *)io->io_Unit;
    return sl811hs_AbortIO(sl, io);

    AROS_LIBFUNC_EXIT
}

static int pathway_Open(struct PathwayBase *pb, struct IORequest *io, ULONG unitnum, ULONG flags)
{
    struct sl811hs *sl;

    if (unitnum >= ARRAY_SIZE(pb_Base))
        return FALSE;

    ObtainSemaphore(&pb->pb_UnitLock);
    if (pb->pb_Unit[unitnum] == NULL) {
        ULONG addr, data;
        ULONG irq;
        addr = pb_Base[unitnum].addr;
        data = pb_Base[unitnum].data;
        irq  = pb_Base[unitnum].irq;
        if (unitnum == 16) {
            addr = data = 0;
            irq = 0;
        } else if (addr == 0) {
            D(bug("%s: Unit %d not configured\n", __func__, unitnum));
            irq = (ULONG)-1;
        }
        if (irq != (ULONG)-1)
            pb->pb_Unit[unitnum] = sl811hs_Attach(addr, data, irq);
    }
    sl = pb->pb_Unit[unitnum];
    ReleaseSemaphore(&pb->pb_UnitLock);

    if (sl) {
        io->io_Unit = (struct Unit *)sl;
        return TRUE;
    }

    return FALSE;
}

static int pathway_Close(struct PathwayBase *pb, struct IORequest *ioreq)
{
    return TRUE;
}

ADD2OPENDEV(pathway_Open,0)
ADD2CLOSEDEV(pathway_Close,0)
