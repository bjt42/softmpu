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
; Protected mode port trap handler
;

HookedGDT       DB              0                       ; Have we munged the GDT yet?
DataSelector    DW              ?                       ; Data segment selector
ReturnByte      DB              ?                       ; Temporary storage for return vals

QEMMPortHandler:ASSUME          ds:_DATA                ; RM handler, access local vars via cs

                cmp             dx,MPUDataPortAddr
                je              @@QEMMPortHandler2
                cmp             dx,MPUCmdPortAddr
                je              @@QEMMPortHandler2
                jmp             OldQEMMIOCB

@@QEMMPortHandler2:
                ; QEMM passes the caller's stack
                ; Might be too small so we'll switch to our own
                ; Might also pass the caller's flags so disable interrupts
                pushf
                cli

                ; Save current stack + a scratch reg
                ; Share a stack with the RTC handler
                ; This should be safe as one should never interrupt the other
                mov             SavedSS,ss
                mov             SavedBP,bp
                mov             SavedSP,sp
                mov             WORD PTR SavedEBX,bx

                ; Set local stack
                mov             bx,cs
                mov             ss,bx
                mov             bx,OFFSET LocalStack
                mov             bp,bx
                mov             bx,LocalSP
                mov             sp,bx

                ; Ensure ds=cs
                push            ds
                mov             bx,cs
                mov             ds,bx
                jmp             @@SetDataSel

PortHandler:    ASSUME          ds:_TEXT                ; PM handler, can't read data from cs
		
		; EMM386 appears to set up a nice 4K stack for the handler
		; So we won't worry about a context switch...
                ; Interrupts are disabled already in here as well
                ; If we're inside the interrupt handler, pass through port access
                cmp             VirtualisePorts,1
		je              @@PortHandler2
		stc                                     ; Don't virtualise
		retf

@@PortHandler2: cmp             HookedGDT,1             ; Have we hooked the GDT yet?
		je              @@SetDataSel

		; We need access to _DATA for our C procedures
		; So, find a free entry in the GDT and set it up
		; We assume selector 08h points to the GDT itself (!)
		; Seems to be the case at least for DOS 6.22 EMM386 and later
		pusha                                   ; At least we only do this once
		push            ds
		mov             ax,08h
		mov             ds,ax                   ; Select GDT seg 08h
		mov             si,ds:[08h]             ; Get 16-bit limit
		inc             si                      ; Limit is inclusive rather than end ptr
@@NextEntry:    sub             si,8                    ; Get next entry
		mov             al,[si+5]               ; Get access byte
		test            al,080h                 ; Is this selector present?
		jz              @@FreeEntry
		cmp             si,0
		je              @@NotFound
		jmp             @@NextEntry

@@NotFound:     pop             ds
		popa
		jmp             @@ExitHandlerVirt       ; OH DEAR. Couldn't get a free entry.
							; We're probably screwed but let's try again
							; next time.

@@FreeEntry:    ; Found a free entry. Set up our new selector
		mov             ax,SEG STACK
		sub             ax,SEG _DATA
		shl             ax,4
		dec             ax                      ; Limit is inclusive rather than end ptr
		mov             [si],ax                 ; Set data seg size (assume <64K)
		mov             ax,SEG _DATA
		shl             ax,4
		mov             [si+2],ax               ; Set data seg addr (low word)
		mov             ax,SEG _DATA
		shr             ax,12
		mov             [si+4],al               ; Set data seg addr (high byte, assuming <1MB)
		mov             al,092h                 ; Present, ring 0, data sel, grows up, writeable
		mov             [si+5],al               ; Set access byte
		mov             ax,0                    
		mov             [si+6],ax               ; Clear high limit byte, flags & high base addr byte
		pop             ds                      ; Flags = byte granularity, 16 bit PM
		mov             DataSelector,si
		popa
		mov             HookedGDT,1
@@SetDataSel:   pusha
                mov             bx,MPUDataPortAddr
		push            ds
		mov             ds,DataSelector         ; Set ds so we can read from _DATA

		; Test for input/output
@@HandlePort:   test            cl,cl
                jz              @@HandleInput

                ; Handle output
		cmp             dx,bx
		jne             @@WriteCommand
		INVOKE          MPU401_WriteData,al
		pop             ds
                popa
                cmp             QEMMInstalled,1
                je              @@ExitHandlerVirtQEMM
		jmp             @@ExitHandlerVirt

@@WriteCommand: INVOKE          MPU401_WriteCommand,al
		pop             ds
                popa
                cmp             QEMMInstalled,1
                je              @@ExitHandlerVirtQEMM
		jmp             @@ExitHandlerVirt

@@HandleInput:  cmp             dx,bx
		jne             @@ReadStatus
		INVOKE          MPU401_ReadData         ; Result in ax
		pop             ds
                mov             ReturnByte,al
                popa
                mov             al,ReturnByte
                cmp             QEMMInstalled,1
                je              @@ExitHandlerVirtQEMM
		jmp             @@ExitHandlerVirt

@@ReadStatus:   INVOKE          MPU401_ReadStatus       ; Result in ax
		pop             ds
                mov             ReturnByte,al
                popa
                mov             al,ReturnByte
@@ExitHandlerVirt:
                cmp             QEMMInstalled,1
                je              @@ExitHandlerVirtQEMM
		clc                                     ; Always virtualised so don't call original handler
		retf

@@ExitHandlerVirtQEMM:
                ASSUME          ds:_DATA                ; RM handler, access local vars via cs

                ; Clear up after QEMM handler
                pop             ds

                ; Restore local stack + scratch reg
                mov             bx,SavedSS
                mov             ss,bx
                mov             bx,SavedBP
                mov             bp,bx
                mov             bx,SavedSP
                mov             sp,bx
                mov             bx,WORD PTR SavedEBX
                popf                                    ; Enable interrupts
                clc
                retf
