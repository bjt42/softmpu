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
; RTC, SB DMA and IRQ 2/8 Watchdog interrupt handlers
;

IntStackSize    EQU             0100h                   ; 256 byte stack for handler
IntStack        DB              IntStackSize DUP (?)    ; Declare stack array
IntSP           EQU             (OFFSET IntStack)+IntStackSize

SavedSS         DW              ?
SavedBP         DW              ?
SavedSP         DW              ?
SavedAX         DW              ?
SavedDX         DW              ?

Int70Handler:   ASSUME          ds:_DATA                ; Access local vars via cs

                ; Ensure interrupts are disabled, in case something's chained us
                ; MPU functions aren't reentrant
                cli

                ; Save current stack + a scratch reg
                mov             SavedSS,ss
                mov             SavedBP,bp
                mov             SavedSP,sp
                mov             SavedAX,ax

                ; Set local stack
                mov             ax,cs
                mov             ss,ax
                mov             ax,OFFSET IntStack
                mov             bp,ax
                mov             ax,IntSP
                mov             sp,ax

                ; Disable virtualised port access
                mov             NoVirtualise,1
                
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
                mov             NoVirtualise,0
                
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
                mov             ax,SavedAX
                iret                                    ; Will restore int flag

SBIntHandler:   ASSUME          ds:_DATA                ; Access local vars via cs

                ; Ensure interrupts are disabled, in case something's chained us
                pushf
                cli

                ; Save scratch registers
                mov             SavedAX,ax
                mov             SavedDX,dx

                ; Set detection flag
                mov             dx,SBPortAddr
                add             dx,4                    ; Mixer register port
                mov             al,082h                 ; Interrupt status register index
                out             dx,al
                inc             dx                      ; Mixer data port
                in              al,dx                   ; Get interrupt status
                test            al,01h                  ; 8-bit IRQ flag
                jz              @@ExitISR               ; Was the interrupt meant for us?
                mov             SBDetected,1            ; Signal detection success
                cmp             NoSBEOI,1               ; Do we want to EOI?
                je              @@ExitISR

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
@@ExitISR:      mov             ax,SavedAX
                mov             dx,SavedDX
                popf
                jmp             OldSBISR

TimerIntHandler:ASSUME          ds:_DATA                ; Access local vars via cs

                ; Call chained ISR first
                pushf
                call            OldTimerISR

                ; Ensure interrupts are disabled, in case something's chained us
                cli

                ; Save scratch registers
                mov             SavedAX,ax

                ; Make sure IRQ 2 (cascade) is unmasked
                ; Some games (e.g. Gateway) disable it, causing problems with PS/2 mice
                in              al,021h                 ; Get current mask
                test            al,04h
                jz              @@IRQ2OK
                and             al,0FBh
		jmp             SHORT $+2               ; Enforce short pause between read & write
                out             021h,al                 ; Set new mask

                ; Make sure IRQ 8 (RTC) is unmasked
                ; It's needed to drive the MPU timers
@@IRQ2OK:       in              al,0A1h                 ; Get current mask
                test            al,01h
                jz              @@IRQ8OK
                and             al,0FEh
		jmp             SHORT $+2               ; Enforce short pause between read & write
                out             0A1h,al                 ; Set new mask

                ; Restore scratch registers
@@IRQ8OK:       mov             ax,SavedAX

                ; Chained ISR will have already sent an EOI
                iret                                    ; Will restore int flag

DOSIntHandler:  ASSUME          ds:_DATA                ; Access local vars via cs

                pushf
                cmp             ah,04Bh
                jne             @@ExitISR2
                int             03h                     ; Break


@@ExitISR2:     popf
                jmp             OldDOSISR
