/*
 *  Copyright (C) 2002-2012  The DOSBox Team
 *  Copyright (C) 2013       bjt, elianda
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

/*
 * ------------------------------------------
 * SoftMPU by bjt - Software MPU-401 Emulator
 * ------------------------------------------
 *
 * C types/functions used from ASM
 *
 */

#ifndef EXPORT_H

#define RTCFREQ 4000 /* SOFTMPU: RTC interrupt frequency */

typedef unsigned char Bit8u;
typedef unsigned int Bitu;
typedef long Bit32;

typedef enum bool {false = 0,true = 1} bool;
typedef enum EventID {MPU_EVENT,RESET_DONE,EOI_HANDLER,NUM_EVENTS} EventID;

/* SOFTMPU: QEMM interface parameters */
typedef struct {
        bool installed;
        Bit32 QPIEntry;
} qemmQPI;

/* Interface functions */
void PIC_Update(bool blocking);

void MPU401_Init(Bitu sbport,Bitu irq,Bitu mpuport,bool delaysysex,bool fakeallnotesoff);
void MPU401_SetQEMMQPI(Bitu addr,Bitu segm);
void MPU401_SetEnableSBIRQ(bool enable);
Bitu MPU401_ReadStatus(void);
void MPU401_WriteCommand(Bitu val);
Bitu MPU401_ReadData(void);
void MPU401_WriteData(Bitu val);

#define EXPORT_H
#endif
