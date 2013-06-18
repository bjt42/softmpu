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

SBPortAddr      DW              0
SBIRQ           DB              0
DelaySysEx      DB              0

NoVirtualise    DB              1                       ; Port virtualisation control

OldSBISR        LABEL           DWORD
OldSBISRAddr    DW              ?
OldSBISRSeg     DW              ?                       ; Don't reorder these

OldTimerISR     LABEL           DWORD
OldTimerISRAddr DW              ?
OldTimerISRSeg  DW              ?                       ; Don't reorder these

SBDetected      DB              0

		; Following data needs to be 4-byte aligned
		; Might need to add some padding here for detection to work
IDPad           DB              1 DUP (?)
IDString        DB              '0S2M'                  ; Detection string

                INCLUDE         inthand.asm             ; Interrupt handlers
		INCLUDE         porthand.asm            ; Port handler code

Start:          push            es
		mov             ax,cs
		mov             es,ax                   ; Copy cs->es for var access from init
		call            FAR PTR Init            ; Call transient segment
		pop             es                      ; Restore es

		; All good so far. Set up port handler
		push            ds
		mov             ax,cs
		mov             ds,ax                   ; Ensure cs=ds
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
		int             02Fh                    ; Install handler
		pop             ds                      ; Restore ds
		jnc             @@InitSoftMPU           ; Init MPU lib
		jmp             FAR PTR EMMErr          ; Fail

@@InitSoftMPU:  INVOKE          MPU401_Init,cs:SBPortAddr,cs:SBIRQ,cs:MPUDataPortAddr,cs:DelaySysEx
		push            es
		mov             ax,cs
		mov             es,ax                   ; Copy cs->es for var access from init
		jmp             FAR PTR EMMOK           ; Go resident

_TEXT           ENDS
