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
; RTC, SB DMA, DOS exec and IRQ watchdog interrupt handlers
;

LocalStackSize  EQU             0100h                   ; 256 byte stack for handlers
LocalStack      DB              LocalStackSize DUP (?)  ; Declare stack array
LocalSP         EQU             (OFFSET LocalStack)+LocalStackSize

SavedSS         DW              ?                       ; Used by QEMM port handler
SavedBP         DW              ?                       ; Used by QEMM port handler
SavedSP         DW              ?                       ; Used by QEMM port handler
SavedEAX        DWORD           ?
SavedEBX        DWORD           ?                       ; Used by QEMM port handler
SavedEDX        DWORD           ?

                ASSUME          ds:_DATA                ; Access local vars via cs

Int70Handler:   ; Ensure interrupts are disabled, in case something's chained us
                ; MPU functions aren't reentrant
                cli

                ; Save current stack + a scratch reg
                ; Share a stack with the QEMM port handler
                ; This should be safe as one should never interrupt the other
                mov             SavedSS,ss
                mov             SavedBP,bp
                mov             SavedSP,sp
                mov             WORD PTR SavedEAX,ax

                ; Set local stack
                mov             ax,cs
                mov             ss,ax
                mov             ax,OFFSET LocalStack
                mov             bp,ax
                mov             ax,LocalSP
                mov             sp,ax

                ; Disable virtualised port access
                mov             VirtualisePorts,0
                
                ; Handle timer tick
                ; MPU lib has been initialised
                ; Preserve state across call to C proc
                pusha
                push            ds
                mov             ax,SEG _DATA
                mov             ds,ax
                INVOKE          PIC_Update,false
                pop             ds
                popa

                ; Enable virtualised port access
                mov             VirtualisePorts,1
                
                ; Interrupts are still disabled
                ; Read status register C to clear pending interrupt
                mov             al,0Ch
                or              al,080h                 ; Disable NMI
                out             070h,al
                jmp             SHORT $+2               ; Short pause
                in              al,071h
                jmp             SHORT $+2
                mov             al,0Dh                  ; Leave index at status reg D
                out             070h,al                 ; Enable NMI
                jmp             SHORT $+2
                in              al,071h                 ; Always read/write 71h after writing 70h

                ; Send non-specific EOI to both PICs
                mov             al,020h                 ; EOI
                out             0A0h,al                 ; Send to secondary PIC
                out             020h,al                 ; Send to primary PIC

                ; Restore local stack + scratch reg
                mov             ax,SavedSS
                mov             ss,ax
                mov             ax,SavedBP
                mov             bp,ax
                mov             ax,SavedSP
                mov             sp,ax
                mov             ax,WORD PTR SavedEAX
                iret                                    ; Will restore int flag

SBIntHandler:   ; Ensure interrupts are disabled, in case something's chained us
                pushf
                cli

                ; Set detection flag
                mov             SBDetected,1            ; Signal detection success
                cmp             SBEOI,0                 ; Do we want to EOI?
                je              @@ExitISR

                ; Save scratch registers
                mov             WORD PTR SavedEAX,ax
                mov             WORD PTR SavedEDX,dx

                ; Acknowledge interrupt
                mov             dx,SBPortAddr
		add             dx,0Eh                  ; Select DSP read port
                in              al,dx                   ; Acknowledge IRQ
                mov             al,020h                 ; EOI
                cmp             SBIRQ,8
                jl              @@FirstPIC
                out             0A0h,al                 ; Send to secondary PIC
                jmp             SHORT $+2
@@FirstPIC:     out             020h,al                 ; Send to primary PIC

                ; Restore scratch registers
                mov             ax,WORD PTR SavedEAX
                mov             dx,WORD PTR SavedEDX
@@ExitISR:      popf
                jmp             OldSBISR

TimerIntHandler:; Call chained ISR first
                pushf
                call            OldTimerISR

                ; Ensure interrupts are disabled, in case something's chained us
                cli

                ; Save scratch registers
                mov             WORD PTR SavedEAX,ax

                ; Make sure IRQ 2 (cascade) is unmasked
                ; Some games (e.g. Gateway) disable it, causing problems with PS/2 mice
                in              al,021h                 ; Get current mask
                test            al,04h
                jz              @@IRQ2OK
                and             al,0FBh
		jmp             SHORT $+2               ; Enforce short pause between read & write
                out             021h,al                 ; Set new mask

@@IRQ2OK:       ; Restore scratch registers
                mov             ax,WORD PTR SavedEAX

                ; Chained ISR will have already sent an EOI
                iret                                    ; Will restore int flag

DOSIntHandler:  ; Check for exec call (ah=04Bh,al=00h)
                pushf
                cmp             ah,04Bh
                jne             @@ExitISR3
                cmp             al,0
                jne             @@ExitISR3

                ; Disable interrupts to keep setting changes atomic
                cli

                ; Save current stack + a scratch reg
                ; Share a stack with the QEMM port handler
                ; This should be safe as one should never interrupt the other
                mov             SavedSS,ss
                mov             SavedBP,bp
                mov             SavedSP,sp
                mov             SavedEAX,eax

                ; Set local stack
                mov             ax,cs
                mov             ss,ax
                mov             ax,OFFSET LocalStack
                mov             bp,ax
                mov             ax,LocalSP
                mov             sp,ax

                ; Default settings
                mov             SBEOI,1                 ; Enable SB EOI
                mov             SBIRQGen,0              ; Disable SB IRQ generation
                mov             MPUVerFix,0             ; Disable MPU version fix for Gateway

                ; Hash the incoming filename (case insensitive)
                push            si
                push            ebx
                mov             si,dx

                ; Find the end of the string first
                cld                                     ; Auto increment si
@@SearchEnd:    lodsb
                cmp             al,0
                jne             @@SearchEnd

                ; Now search backwards to find a delimiter
                dec             si
                std                                     ; Auto decrement si
@@SearchDelim:  lodsb
                cmp             al,05Ch                 ; Backslash?
                je              @@FoundDelim
                cmp             al,03Ah                 ; Colon?
                je              @@FoundDelim
                cmp             si,dx
                jge             @@SearchDelim           ; Not yet at start
                dec             si
@@FoundDelim:   add             si,2
                push            edx                     ; May be trashed by HashString
                call            HashString
                pop             edx
                cmp             bx,0
                je              @@ExitISR2

                ; Should we disable EOI?
                cmp             bx,0AC2Fh               ; It Came From The Desert (HUGE.EXE)
                jne             @@CheckMPUVerFix
@@DisableSBEOI: mov             SBEOI,0                 ; Don't send SB EOI

@@CheckMPUVerFix:
                ; Should we enable the Gateway MPU version fix?
                ; See www.vogons.org/viewtopic.php?f=32&t=38044
                cmp             bx,0DE4Fh               ; Gateway (GATE.EXE)
                je              @@EnableMPUVerFix
                cmp             bx,0703Dh               ; Gateway (MAIN.EXE)
                jne             @@CheckIRQGen
@@EnableMPUVerFix:
                mov             MPUVerFix,1             ; Enable MPU version fix for Gateway

@@CheckIRQGen:  ; Should we enable SB IRQ generation?
                push            ds
                mov             ax,cs
                mov             ds,ax                   ; Copy cs->ds for data access
                mov             si,OFFSET AppIDStrings
                cld                                     ; Auto increment si
@@ParseAppStrings:
                lodsw                                   ; Get an ID string
                cmp             al,0                    ; Check for null terminator
                je              @@EndParse
                cmp             ax,bx
                jne             @@ParseAppStrings
@@EnableIRQGen: mov             SBIRQGen,1              ; Enable SB interrupts
@@EndParse:     pop             ds
@@ExitISR2:     pop             ebx
                pop             si

                ; Never generate interrupts in SB-MIDI mode or if no Sound Blaster
                cmp             OutMode,M_SBMIDI
                je              @@DisableIRQ
                cmp             SBPortAddr,0
                jne             @@NoDisableIRQ
@@DisableIRQ:   mov             SBIRQGen,0

@@NoDisableIRQ: ; Enable/disable RTC interrupts as well
                in              al,0A1h
                mov             ah,SBIRQGen
                or              ah,DelaySysex           ; DelaySysex requires RTC interrupts
                or              al,01h                  ; Set bit 0 = IRQ 8 (RTC)
                xor             al,ah
                jmp             SHORT $+2               ; Short pause
                out             0A1h,al

                ; Now commit our changes
                pusha
                push            ds
                mov             ax,SEG _DATA
                mov             ds,ax
                INVOKE          MPU401_SetEnableSBIRQ,SBIRQGen
                INVOKE          MPU401_SetEnableMPUVerFix, MPUVerFix
                pop             ds
                popa

                ; Restore local stack + scratch reg
                mov             ax,SavedSS
                mov             ss,ax
                mov             ax,SavedBP
                mov             bp,ax
                mov             ax,SavedSP
                mov             sp,ax
                mov             eax,SavedEAX
@@ExitISR3:     popf
                jmp             OldDOSISR
