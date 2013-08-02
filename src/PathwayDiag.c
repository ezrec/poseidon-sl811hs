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

#include <aros/shcommands.h>

#include <proto/dos.h>

#include "sl811hs.h"

static UBYTE rb(IPTR base, UBYTE addr)
{
    *((volatile UBYTE *)base) = addr;
    return *((volatile UBYTE *)(base + 4));
}

static void wb(IPTR base, UBYTE addr, UBYTE val)
{
    *((volatile UBYTE *)base) = addr;
    *((volatile UBYTE *)(base + 4)) = val;
}

static ULONG HexToLong(CONST_STRPTR str, ULONG *valp)
{
    ULONG val = 0;
    CONST_STRPTR tmp = str;

    while (*(tmp)) {
        TEXT c = *(tmp++);
        if ((c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F')) {
            val <<= 4;
            val |= 10 + ((c | 0x20) - 'a');
        } else if (c >= '0' && c <= '9') {
            val <<= 4;
            val |= c - '0';
        } else {
            break;
        }
    }

    *valp = val;
    return (tmp - str);
}

static LONG AnyToLong(CONST_STRPTR str, ULONG *val, struct DosLibrary *DOSBase)
{
    if (str[0] == '$')
        return HexToLong(&str[1], val) + 1;
    else if (str[0] == '0' && str[1] == 'x')
        return HexToLong(&str[2], val) + 2;
    else
        return StrToLong(str, (LONG *)val);
}

AROS_SH3H(PathwayDiag, 1.0, "Pathway Diagnostic Utility",
        AROS_SHAH(STRPTR, B=, BASE, /K, "$d80001", "Base address of Pathway"),
        AROS_SHAH(STRPTR, A=, ADDR, , "$0e", "Address to read"),
        AROS_SHAH(STRPTR, V=, VAL, , NULL, "Value to write")
) {
    AROS_SHCOMMAND_INIT

    STRPTR tmp;
    IPTR base;
    ULONG addr, val;
    BOOL is_write = FALSE;

    AnyToLong(SHArg(BASE), &base, DOSBase);

    AnyToLong(SHArg(ADDR), &addr, DOSBase);

    tmp = SHArg(VAL);
    if (tmp && tmp[0] != 0) {
        is_write = TRUE;
        AnyToLong(tmp, &val, DOSBase);
    }

    if ((rb(base, 0x0e) & 0xfc) != 0x20) {
        Printf("Pathway not present at $%x\n", base);
        return RETURN_FAIL;
    }

    if (is_write) {
        wb(base, addr, val);
    } else {
        val = rb(base, addr);
        Printf("$%02lx: $%02lx\n", addr, val);
    }

    return RETURN_OK;

    AROS_SHCOMMAND_EXIT
}

