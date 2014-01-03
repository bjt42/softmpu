;
; Copyright (C) 2013 bjt, elianda
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
; String handing & detection routines
;

SecondReset     DB              0
LoopCounter     DW              ?

                ASSUME          ds:_DATA                ; Access local vars via cs

CharToNum:      ; Input char in al
		cmp             al,061h
		jl              @@NotLower
		sub             al,057h                 ; Lowercase hex
		retn

@@NotLower:     cmp             al,041h
		jl              @@NotUpper
		sub             al,037h                 ; Uppercase hex
		retn

@@NotUpper:     sub             al,030h                 ; Digit
		retn

NumToChar:      ; Input num 0-F in al
		cmp             al,0Ah
		jl              @@NotLetter
		add             al,037h                 ; Uppercase hex
		retn

@@NotLetter:    add             al,030h                 ; Digit
		retn

ReadRTC:        ; Assume al=index
		; Output also in al
		pushf
		cli                                     ; No interrupts while changing CMOS
		or              al,080h                 ; Disable NMI
		out             070h,al
		jmp             SHORT $+2               ; Short pause
		in              al,071h
		push            ax                      ; Save read value
		jmp             SHORT $+2
		mov             al,0Dh                  ; Leave index at status reg D
		out             070h,al                 ; Enable NMI
		jmp             SHORT $+2
		in              al,071h                 ; Always read/write 71h after writing 70h
		pop             ax
		popf                                    ; Enable interrupts
		retn

WriteRTC:       ; Assume al=index,ah=value
		pushf
		push            ax
		cli                                     ; No interrupts while changing CMOS
		or              al,080h                 ; Disable NMI
		out             070h,al                 ; Write index
		jmp             SHORT $+2
		shr             ax,8                    ; Shift value into al
		out             071h,al                 ; Write value
		jmp             SHORT $+2
		mov             al,0Dh                  ; Leave index at status reg
		out             070h,al                 ; Enable NMI
		jmp             SHORT $+2
		in              al,071h                 ; Always read/write 71h after writing 70h
		pop             ax
		popf                                    ; Enable interrupts
		retn

TrySetDSR:      mov             LoopCounter,0FFFFh
@@TryReadDSR:   mov             dx,es:MPUCmdPortAddr
		in              al,dx
		test            al,080h                 ; Any pending data?
		jnz             @@TryDSRSet
		mov             dx,es:MPUDataPortAddr
		in              al,dx                   ; Read pending data
		dec             LoopCounter
		cmp             LoopCounter,0
		jne             @@TryReadDSR            ; Try again
		stc
		retn                                    ; Failed to clear DSR!

@@TryDSRSet:    clc
		retn                                    ; All clear               

WaitForDSR:     mov             LoopCounter,0FFFFh
@@ReadDSR:      mov             dx,es:MPUCmdPortAddr
		in              al,dx
		test            al,080h                 ; Any pending data
		jz              @@DSRClear
		dec             LoopCounter
		cmp             LoopCounter,0
		jne             @@ReadDSR               ; Try again
		stc
		retn                                    ; Timeout

@@DSRClear:     clc
		retn                                    ; Found pending data

WaitForDRR:     mov             LoopCounter,0FFFFh
@@ReadDRR:      mov             dx,es:MPUCmdPortAddr
		in              al,dx
		test            al,040h                 ; Clear to send?
		jz              @@DRRClear
		dec             LoopCounter
		cmp             LoopCounter,0
		jne             @@ReadDRR               ; Try again
		stc
		retn                                    ; Timeout

@@DRRClear:     clc
		retn                                    ; OK to send

SwitchToUART:   call            ResetMPU                ; Ensure we're in a sane state
                jc              @@SwitchFailed
		call            TrySetDSR               ; Clear pending data
		jc              @@SwitchFailed
		call            WaitForDRR              ; OK to send?
		jc              @@SwitchFailed
		mov             dx,es:MPUCmdPortAddr
		mov             al,03Fh
		out             dx,al                   ; Set UART cmd
		call            WaitForDSR
		jc              @@SwitchFailed          ; No ACK
		mov             dx,es:MPUDataPortAddr
		in              al,dx
		cmp             al,0FEh
		jne             @@SwitchFailed          ; No ACK
		clc
		retn                                    ; Now in UART mode

@@SwitchFailed: stc
		retn                                    ; Can't switch to UART

ResetMPU:       mov             SecondReset,0
                call            TrySetDSR               ; Clear pending data
                jc              @@ResetFailed2
@@TryReset:     call            WaitForDRR              ; OK to send?
                jc              @@ResetFailed2
		mov             dx,es:MPUCmdPortAddr
		mov             al,0FFh
		out             dx,al                   ; Send reset cmd
		call            WaitForDSR
		jc              @@ResetFailed           ; No ACK
		mov             dx,es:MPUDataPortAddr
		in              al,dx
		cmp             al,0FEh
		jne             @@ResetFailed           ; No ACK
		clc
                retn                                    ; Now in normal mode

@@ResetFailed:  mov             al,SecondReset
		inc             SecondReset
                cmp             al,0                    ; Maybe we were in UART mode
		je              @@TryReset
@@ResetFailed2: stc
                retn                                    ; Failed to reset

DetectSB:       ; Try to detect a Sound Blaster at the specifed port & irq
                mov             al,es:SBIRQ             ; Get hardware irq number
		add             al,8
		cmp             al,010h
		jl              @@FirstPIC2
		add             al,060h                 ; Interrupt is on the secondary PIC

@@FirstPIC2:    ; Save the existing handler
		push            es
		mov             ah,035h
		int             021h                    ; Get interrupt vector
		mov             dx,es
		pop             es
                mov             es:OldSBISRSeg,dx       ; Save segment addr
                mov             es:OldSBISRAddr,bx      ; Save handler addr

		; Register the new handler
                push            ds
                mov             bx,es
                mov             ds,bx                   ; Copy es->ds for API call
		mov             ah,025h                 ; Interrupt num still in al
                mov             dx,OFFSET SBIntHandler
                int             021h                    ; Register new handler
                pop             ds

		; Unmask the interrupt
                mov             cl,es:SBIRQ
		cmp             cl,8
		jl              @@FirstPIC3
		sub             cl,8
@@FirstPIC3:    mov             bl,1
		shl             bl,cl                   ; Construct IRQ mask
		mov             dx,021h                 ; Primary PIC by default
                cmp             es:SBIRQ,8
		jl              @@FirstPIC4
		mov             dx,0A1h                 ; Secondary PIC
@@FirstPIC4:    push            dx                      ; Save the PIC address
		pushf
		cli                                     ; No interrupts while progamming PIC
		in              al,dx                   ; Get current mask
		mov             cl,al
		and             cl,bl
		xor             bl,0FFh
		and             al,bl
		jmp             SHORT $+2               ; Enforce short pause between read & write
		out             dx,al                   ; Set new mask
		popf                                    ; Enable interrupts
		push            cx                      ; Save the restore mask

		; Ask the Sound Blaster to raise the interrupt
                mov             dx,es:SBPortAddr
		add             dx,0Ch                  ; Select DSP command port
		mov             al,0F2h                 ; DSP cmd 0xF2 = Force 8-bit IRQ
		out             dx,al                   ; Raise IRQ

		; Wait to see if the detect flag gets set
		mov             LoopCounter,0FFFFh
@@WaitLoop:     cmp             es:SBDetected,1
                je              @@ExitLoop
		dec             LoopCounter
		cmp             LoopCounter,0
		jne             @@WaitLoop

@@ExitLoop:     ; Restore the interrupt mask
                pop             bx                      ; Get restore mask
                pop             dx                      ; Get PIC address
                cmp             es:SBIRQ,9
                je              @@RestoreISR            ; Allow IRQ 9<->2 mapping
                pushf
                cli                                     ; Disable interrupts
                in              al,dx
                or              al,bl
                jmp             SHORT $+2
                out             dx,al                   ; Restore initial enable state
                popf

@@RestoreISR:   ; Don't restore the handler on success
                cmp             es:SBDetected,1         ; Did the detection flag get set?
                je              @@SBDetected
                call            RestoreSBISR            ; Restore the old handler
		stc
		retn                                    ; Failed to detect SB

@@SBDetected:   clc
		retn

RestoreSBISR:   ; Restore the old SB handler
                mov             al,es:SBIRQ             ; Get hardware irq number
		add             al,8
		cmp             al,010h
                jl              @@FirstPIC5
		add             al,060h                 ; Interrupt is on the secondary PIC
@@FirstPIC5:    mov             ah,025h
                push            ds
                mov             ds,es:OldSBISRSeg
                mov             dx,es:OldSBISRAddr
                int             021h                    ; Register old handler
                pop             ds
                retn

DetectRTC:      push            es
                mov             ah,0C0h
                int             015h                    ; Get configuration
                jc              @@RTCNotDetected
                mov             al,es:[bx+5]            ; Get feature info 1
                test            al,020h                 ; Check for RTC
                jz              @@RTCNotDetected
                pop             es
                clc                                     ; Got an RTC
                retn

@@RTCNotDetected:
                pop             es
                stc
                retn                                    ; Failed to detect RTC

InstTimerISR:   ; Install a handler on int 08h (IRQ 0/Timer) that makes sure IRQ 2 (cascade) remains unmasked
                ; Doesn't matter if the timer frequency is increased by another app later
                ; Save the existing handler
		push            es
                mov             al,08h
		mov             ah,035h
		int             021h                    ; Get interrupt vector
		mov             dx,es
		pop             es
                mov             es:OldTimerISRSeg,dx    ; Save segment addr
                mov             es:OldTimerISRAddr,bx   ; Save handler addr

		; Register the new handler
                push            ds
                mov             bx,es
                mov             ds,bx                   ; Copy es->ds for API call
		mov             ah,025h                 ; Interrupt num still in al
                mov             dx,OFFSET TimerIntHandler
                int             021h                    ; Register new handler
                pop             ds
                retn

InstDOSISR:     ; Install a handler on int 21h for exec (4Bh) calls
                ; This allows us to implement workarounds for specific apps
                ; Save the existing handler
		push            es
                mov             al,021h
		mov             ah,035h
		int             021h                    ; Get interrupt vector
		mov             dx,es
		pop             es
                mov             es:OldDOSISRSeg,dx      ; Save segment addr
                mov             es:OldDOSISRAddr,bx     ; Save handler addr

		; Register the new handler
                push            ds
                mov             bx,es
                mov             ds,bx                   ; Copy es->ds for API call
		mov             ah,025h                 ; Interrupt num still in al
                mov             dx,OFFSET DOSIntHandler
                int             021h                    ; Register new handler
                pop             ds
                retn
