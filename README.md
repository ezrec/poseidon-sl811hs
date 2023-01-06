# AmigaOS/AROS Poseidon support for SL811HS based devices

```
******************************************************
**            !!!!! DEADLY WARNING !!!!!            **
******************************************************
* THIS DRIVER IS PRE-RELEASE, AND WILL CORRUPT DATA! *
******************************************************
```

## Introduction

This driver suppors the Pathway (Clockport) and Thylacine (Zorro) boards.

## Thylacine Boards

Use `thylacine.device` for Thylacine boards. Device unit numbers correspond
to the Zorro board order (ie Unit 0 for the first detected Thylacine,
Unit 1 for the next, etc).

## Pathway Clockport Boards

Use `pathway.device` for Pathway clockport USB adapters.
Unit numbers are as follows:

| Unit | Base     | Comment         |
| ---  | ---      | ---             |
|  0   | 0xd80001 | A1200 clockport |
|  1   | 0xd84001 | Zorro IV        |
|  2   | 0xd88001 | Zorro IV        |
|  3   | 0xd8c001 | Zorro IV        |
|  4   | 0xd90001 | A604 2nd port   |

## Building

Instructions for Linux cross-compilation:

- Clone the AROS development environment

  `$ git clone https://github.com/aros-development-team/AROS.git`

- Clone this repo into the development environment

  `$ cd AROS; git clone https://github.com/ezrec/poseidon-sl811hs`

- Configure the development environment to build amigaos-m68k targets

  `$ ./configure --target=amigaos-m68k`

- Build the target driver (as ELF executables)

  `$ make kernel-amiga-m68k-sl811hs`

- Convert AROS ELF format to AmigaOS HUNK format

  `$ bin/linux-x86_64/tools/elf2hunk bin/amiga-m68k/{AROS,AmigaOS}`

Final drivers in AmigaOS HUNK format, usable in both AmigaOS and AROS-M68K, are in `bin/amiga-m68k/AmigaOS`

