;
; Copyright (C) 2013-2014 bjt, elianda
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
; Resident data & routines
;

_TEXT           SEGMENT PUBLIC 'CODE'

		.STARTUP                                ; Init ds/ss/sp

		jmp             Start

PortTable       LABEL           WORD
MPUDataPortAddr DW              0                       ; Set from cmdline
DataHandlerAddr DW              OFFSET PortHandler
MPUCmdPortAddr  DW              ?                       ; Set from cmdline
CmdHandlerAddr  DW              OFFSET PortHandler

SBPortAddr      DW              0                       ; Set from cmdline
SBIRQ           DB              0                       ; Set from cmdline
SerialPortAddr  DW              0                       ; Set from cmdline
DelaySysex      DB              0                       ; Set from cmdline
FakeAllNotesOff DB              0                       ; Set from cmdline

OutMode         OutputMode      M_MPU401                ; MIDI output mode control

VirtualisePorts DB              0                       ; Port virtualisation control
SBEOI           DB              1                       ; SB ISR EOI control
SBIRQGen        DB              0                       ; SB IRQ generation control
MPUVerFix       DB              0                       ; MPU version fix for Gateway

OldSBISR        LABEL           DWORD
OldSBISRAddr    DW              ?
OldSBISRSeg     DW              ?                       ; Don't reorder these

OldTimerISR     LABEL           DWORD
OldTimerISRAddr DW              ?
OldTimerISRSeg  DW              ?                       ; Don't reorder these

OldDOSISR       LABEL           DWORD
OldDOSISRAddr   DW              ?
OldDOSISRSeg    DW              ?                       ; Don't reorder these

QEMMInstalled   DB              0
QPIEntry        DWORD           ?

OldQEMMIOCB	LABEL		DWORD
OldQEMMIOCBAddr	DW              ?
OldQEMMIOCBSeg	DW              ?                       ; Don't reorder these

SBDetected      DB              0

IDString        DB              '0S2M'                  ; Detection string

                INCLUDE         appstr.asm              ; App ID substrings
                INCLUDE         inthand.asm             ; Interrupt handlers
		INCLUDE         porthand.asm            ; Port handler code
                INCLUDE         resutils.asm            ; Resident utility routines

                ASSUME          ds:_DATA                ; Access local vars via cs

Start:          push            es
		mov             ax,cs
		mov             es,ax                   ; Copy cs->es for var access from init
		call            FAR PTR Init            ; Call transient segment
		pop             es                      ; Restore es

		; All good so far. Set up port handler
                push            ds
                mov             ax,cs
                mov             ds,ax                   ; Ensure cs=ds
                cmp             QEMMInstalled,1
                je              @@QEMMPortHandlerInstall

                ; Install EMM386 port handler
		mov             dx,MPUCmdPortAddr
		shl             edx,16
		mov             dx,MPUDataPortAddr      ; Cmd addr high, data addr low
		mov             cx,2                    ; Num ports
		mov             si,OFFSET PortTable
		mov             di,SEG _DATA            ; We set up our own data selector
		sub             di,SEG _TEXT
		shl             di,4                    ; Calculate offset to resident end
		mov             ax,04A15h
		mov             bx,0
                stc                                     ; Fail by default
		int             02Fh                    ; Install handler
		pop             ds                      ; Restore ds
		jnc             @@InitSoftMPU           ; Init MPU lib
                jmp             @@EMMErr

@@QEMMPortHandlerInstall:
		mov		di,OFFSET QEMMPortHandler
		push		es
                mov             ax,cs
                mov             es,ax                   ; es:di is callback address
		mov		ax,01A07h		; QPI_SetIOCallback
                call            QPIEntry
		pop		es
                jc              @@EMMErr                ; Fail
		mov             dx,MPUCmdPortAddr
                mov             ax,01A09h
		call		QPIEntry		; QPI_SetPortTrap
                jc              @@EMMErr                ; Fail
		mov             dx,MPUDataPortAddr
		mov		ax,01A09h
		call		QPIEntry		; QPI_SetPortTrap
                jc              @@EMMErr                ; Fail
                mov             DataSelector,SEG _DATA  ; Save real-mode seg addr
		pop		ds
                jmp             @@InitSoftMPU           ; Init MPU lib

@@EMMErr:       push            es
                mov             ax,cs
		mov             es,ax                   ; Copy cs->es for var access from init
		jmp             FAR PTR EMMErr          ; Fail

@@InitSoftMPU:  INVOKE          MPU401_Init,QPIEntry,SBPortAddr,SBIRQ,MPUDataPortAddr,SerialPortAddr,OutMode,DelaySysex,FakeAllNotesOff
		push            es
		mov             ax,cs
		mov             es,ax                   ; Copy cs->es for var access from init
		jmp             FAR PTR EMMOK           ; Go resident

_TEXT           ENDS
