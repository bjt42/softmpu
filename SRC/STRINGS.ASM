;
; Copyright (C) 2013 bjt
;
; This program is free software; you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation; either version 2 of the License, or
; (at your option) any later version.
;
; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU General Public License for more details.
;
; You should have received a copy of the GNU General Public License
; along with this program; if not, write to the Free Software
; Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
;

; ------------------------------------------
; SoftMPU by bjt - Software MPU-401 Emulator
; ------------------------------------------
;
; String data
;

Header          DB              0DAh
                DB              0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h
                DB              0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h
                DB              0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h
                DB              0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h
                DB              0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0BFh,0Dh,0Ah
                DB              0B3h,'SoftMPU 1.4 by bjt ',0FEh,' Software MPU-401 Emulator',0B3h,0Dh,0Ah
                DB              0B3h,'                                              ',0B3h,0Dh,0Ah
                DB              0B3h,'Copyright (C) 2013        bjt                 ',0B3h,0Dh,0Ah
                DB              0B3h,'Copyright (C) 2002-2013   The DOSBox Team     ',0B3h,0Dh,0Ah
                DB              0C0h
                DB              0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h
                DB              0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h
                DB              0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h
                DB              0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0C4h
                DB              0C4h,0C4h,0C4h,0C4h,0C4h,0C4h,0D9h,0Dh,0Ah,'$'
NoEMM           DB              '! Error: EMM386 is not available',0Dh,0Ah,'$'
EMMError        DB              '! Error: Port trap failed. Please submit a bug report',0Dh,0Ah,'$'
LoadedError     DB              '! Error: SoftMPU is already loaded',0Dh,0Ah,'$'
RTCError        DB              '! Error: Real Time Clock support not detected',0Dh,0Ah,'$'
SBErrorStart    DB              '! Error: Sound Blaster not detected at port ','$'
SBErrorMid      DB              ' IRQ ','$'
SBErrorEnd      DB              0Dh,0Ah,'$'
SBSuccessStart  DB              0FBh,' Sound Blaster detected at port ','$'
SBSuccessMid    DB              ' IRQ ','$'
SBSuccessEnd    DB              0Dh,0Ah,'$'
SBPortNum       DB              '   ','$'
SBIRQNum        DB              '  ','$'
MPUErrorStart   DB              '! Error: MPU-401 not detected at port ','$'
MPUErrorEnd     DB              0Dh,0Ah,'$'
MPUSuccessStart DB              0FBh,' MPU-401 detected at port ','$'
MPUSuccessEnd   DB              0Dh,0Ah,'$'
MPUPortNum      DB              '   ','$'
DelaysEnabled   DB              'i SYSEX delays enabled',0Dh,0Ah,'$'
LoadedStart     DB              0Dh,0Ah,0Eh,' SoftMPU active at port ','$'
LoadedMid       DB              ' IRQ ','$'
LoadedEnd       DB              0Dh,0Ah,'$'
HelpText        DB              'Usage: SOFTMPU /SB:nnn /IRQ:nn /MPU:nnn [/DELAYSYSEX]',0Dh,0Ah
                DB              0Dh,0Ah
                DB              '  /SB:nnn       Sound Blaster base I/O port address (220-280)',0Dh,0Ah
                DB              '  /IRQ:nn       Sound Blaster interrupt (2-11)',0Dh,0Ah
                DB              '  /MPU:nnn      MPU-401 base I/O port address (220-350)',0Dh,0Ah
                DB              '  /DELAYSYSEX   Prevent Rev.0 MT-32 buffer overflow',0Dh,0Ah,'$'
