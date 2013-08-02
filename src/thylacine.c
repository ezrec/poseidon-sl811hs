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
#include <proto/expansion.h>

#include <hardware/intbits.h>

#include "sl811hs.h"
#include "thylacine_intern.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))
#endif

#define THYLACINE_VENDOR        5010
#define THYLACINE_PRODUCT       1

static int thylacine_Init(struct ThylacineBase *tb)
{
    struct Library *ExpansionBase;
    int unit = 0;

    InitSemaphore(&tb->tb_UnitLock);

    NEWLIST(&tb->tb_Units);

    if ((ExpansionBase = OpenLibrary("expansion.library", 36))) {
        struct ConfigDev *cd = NULL;

        while ((cd = FindConfigDev(cd, THYLACINE_VENDOR, THYLACINE_PRODUCT))) {
            struct sl811hs *sl;
            ULONG addr = (ULONG)(IPTR)cd->cd_BoardAddr;
            ULONG data = addr + 0x4000;
            // ULONG reset = addr + 0x100;
            sl = sl811hs_Attach(addr, data, INTB_EXTER);
            if (sl) {
                ((struct Node *)sl)->ln_Pri = unit++;
                ((struct Node *)sl)->ln_Name = "thylacine.device";
                AddTail(&tb->tb_Units, (struct Node *)sl);
            }
        }

        CloseLibrary(ExpansionBase);
    }

    return IsListEmpty(&tb->tb_Units) ? 0 : 1;
}

static int thylacine_Expunge(struct ThylacineBase *tb)
{
    struct sl811hs *sl;

    while ((sl = (struct sl811hs *)RemHead(&tb->tb_Units)))
        sl811hs_Detach(sl);

    return 1;
}

ADD2INITLIB(thylacine_Init, 0);
ADD2EXPUNGELIB(thylacine_Expunge, 0);

AROS_LH1(void, BeginIO,
    AROS_LHA(struct IORequest *, io, A1),
    struct ThylacineBase *, tb, 5, thylacine)
{
    AROS_LIBFUNC_INIT

    struct sl811hs *sl = (struct sl811hs *)(io->io_Unit);

    return sl811hs_BeginIO(sl, io);

    AROS_LIBFUNC_EXIT
}

AROS_LH1(LONG, AbortIO,
    AROS_LHA(struct IORequest *, io, A1),
    struct ThylacineBase *, tb, 6, thylacine)
{
    AROS_LIBFUNC_INIT

    struct sl811hs *sl;

    sl = (struct sl811hs *)io->io_Unit;
    return sl811hs_AbortIO(sl, io);

    AROS_LIBFUNC_EXIT
}

static int thylacine_Open(struct ThylacineBase *tb, struct IORequest *io, ULONG unitnum, ULONG flags)
{
    struct sl811hs *sl;

    ForeachNode(&tb->tb_Units, sl) {
        if (((struct Node *)sl)->ln_Pri == unitnum) {
            io->io_Unit = (struct Unit *)sl;
            return TRUE;
        }
    }

    return FALSE;
}

static int thylacine_Close(struct ThylacineBase *tb, struct IORequest *ioreq)
{
    return TRUE;
}

/* TODO: Add thylacine 5010/1 support */

ADD2OPENDEV(thylacine_Open,0)
ADD2CLOSEDEV(thylacine_Close,0)
