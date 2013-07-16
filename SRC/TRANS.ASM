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
; Initialisation routines (non-resident)
;

TransientSeg    SEGMENT 'transient'

EMMVersion      LABEL           WORD                    ; EMM386 version buffer
EMMMajor        DB              02h
EMMMinor        DB              00h

Init:           ; Ensure ds=cs
		push            ds
		mov             ax,cs
		mov             ds,ax

		; Print banner
		mov             ah,9
		mov             dx,OFFSET Header
		int             021h

		; Parse command line
		mov             ah,051h
		int             021h                    ; Get PSP seg addr in bx
		push            ds
		mov             ds,bx                   ; Set PSP seg addr
		xor             cx,cx
		mov             bx,080h
		mov             cl,[bx]
		test            cx,cx                   ; Any command line?
		jnz             @@GotCmdline
		pop             ds                      ; Finished parsing
		jmp             PrintHelp

@@GotCmdline:   lea             si,[bx+1]
		cld                                     ; Auto increment si
@@ParseCmdline: lodsd                                   ; Get a dword from the cmd line
		cmp             eax,':UPM'              ; Check for MPU base param
		je              @@MPUBaseParam
		cmp             eax,':upm'
		je              @@MPUBaseParam
		cmp             eax,':BS/'              ; Check for SB base param
		je              @@SBBaseParam
		cmp             eax,':bs/'
		je              @@SBBaseParam
		cmp             eax,':QRI'
		je              @@SBIRQParam
		cmp             eax,':qri'
		je              @@SBIRQParam
		cmp             eax,'LED/'              ; Check for delay switch
		je              @@EnableDelay
		cmp             eax,'led/'
		jne             @@NextParam
@@EnableDelay:  mov             es:DelaySysex,1         ; Enable sysex delays
		jmp             @@NextParam

@@MPUBaseParam: cmp             cx,7                    ; Command plus 3-digit port
		jl              @@NextParam             ; Not enough chars left
		xor             bx,bx
		mov             al,[si]                 ; 1st hex digit
		call            CharToNum
		or              bh,al
		mov             al,[si+1]               ; 2nd hex digit
		call            CharToNum
		or              bl,al
		shl             bl,4
		mov             al,[si+2]               ; Last hex digit
		call            CharToNum
		or              bl,al
		mov             es:MPUDataPortAddr,bx   ; Set data port
		inc             bx                               
		mov             es:MPUCmdPortAddr,bx    ; Set command port
		jmp             @@NextParam

@@SBBaseParam:  cmp             cx,7                    ; Command plus 3-digit port
		jl              @@NextParam             ; Not enough chars left
		xor             bx,bx
		mov             al,[si]                 ; 1st hex digit
		call            CharToNum
		or              bh,al
		mov             al,[si+1]               ; 2nd hex digit
		call            CharToNum
		or              bl,al
		shl             bl,4
		mov             al,[si+2]               ; Last hex digit
		call            CharToNum
		or              bl,al
		mov             es:SBPortAddr,bx        ; Set port
		jmp             @@NextParam

@@SBIRQParam:   cmp             cx,5                    ; Command plus 1-digit IRQ
		jl              @@NextParam             ; Not enough chars left
		xor             bl,bl
		mov             al,[si]                 ; 1st decimal digit
		call            CharToNum
		add             bl,al
		cmp             cx,6                    ; Do we have a second char?
		jl              @@SetSBIRQ
		mov             al,[si+1]               ; Last decimal digit
                cmp             al,030h                 ; Is it a digit 0-9?
		jl              @@SetSBIRQ
		cmp             al,039h
		jg              @@SetSBIRQ
		cmp             bl,0
		je              @@ZeroDigit             ; Check if the first digit was zero
		mov             bl,10                   ; Got a second digit
@@ZeroDigit:    call            CharToNum
		add             bl,al
@@SetSBIRQ:     mov             es:SBIRQ,bl             ; Set IRQ
@@NextParam:    sub             si,3                    ; Only step a byte at a time
		dec             cx                      ; We've consumed a char
		cmp             cx,0                    ; Consumed all chars?
		jg              @@ParseCmdline
@@CheckPorts:   pop             ds                      ; Finished parsing so restore ds

		; Check we have valid port ranges
		cmp             es:MPUDataPortAddr,0220h
		jl              PrintHelp
		cmp             es:MPUDataPortAddr,0350h
		jg              PrintHelp
		cmp             es:SBPortAddr,0220h
		jl              PrintHelp
		cmp             es:SBPortAddr,0280h
		jg              PrintHelp

		; Write the port addresses into our strings
		mov             bx,es:MPUDataPortAddr
		mov             ax,bx
		shr             ax,8
		call            NumToChar
		mov             MPUPortNum,al
		mov             ax,bx
		shr             ax,4
		and             al,0Fh
		call            NumToChar
		mov             MPUPortNum+1,al
		mov             ax,bx
		and             al,0Fh
		call            NumToChar
		mov             MPUPortNum+2,al
		mov             bx,es:SBPortAddr
		mov             ax,bx
		shr             ax,8
		call            NumToChar
		mov             SBPortNum,al
		mov             ax,bx
		shr             ax,4
		and             al,0Fh
		call            NumToChar
		mov             SBPortNum+1,al
		mov             ax,bx
		and             al,0Fh
		call            NumToChar
		mov             SBPortNum+2,al

		; Check we have a valid IRQ
		cmp             es:SBIRQ,2
		jl              PrintHelp
		cmp             es:SBIRQ,11
		jg              PrintHelp

		; Write the IRQ into our string
		mov             al,es:SBIRQ
		cmp             al,10
		jl              @@SingleDigit           ; More than one digit needed?
		sub             al,10
		call            NumToChar
		mov             SBIRQNum+1,al
		mov             al,1
@@SingleDigit:  call            NumToChar
		mov             SBIRQNum,al

		; Check for EMM386
                push            ds                      ; Installation checks
                xor             ax,ax
                mov             ds,ax
                mov             ax,ds:[019Ch]
                or              ax,ds:[019Eh]
                pop             ds
                jz              @@NoEMM
                mov             ax,0FFA5h
                int             067h
                cmp             ax,0845Ah
                jne             @@NoEMM
                mov             ax,03D00h               ; Open file R/O
                mov             dx,OFFSET EMMDevice
                int             021h                    ; Get handle
                jnc             @@CheckVersion
                mov             ax,03D00h               ; Open file R/O
                mov             dx,OFFSET EMMDevice2
                int             021h                    ; Get handle
                jc              @@NoEMM
@@CheckVersion: mov             bx,ax
                mov             ax,04402h               ; Get manager version
                mov             cx,02h
                mov             dx,OFFSET EMMVersion
                int             021h
                mov             ah,03Eh                 ; Close file
                int             021h
                cmp             EMMMajor,4              ; Require EMM386 >= 4.46
                jl              @@NoEMM
                cmp             EMMMinor,45
                jg              @@FoundEMM

@@NoEMM:        ; EMM386 not present
		mov             ah,9
		mov             dx,OFFSET NoEMM
		int             021h
		jmp             Terminate

@@FoundEMM:     ; Check if already loaded
                ; Disable interrupts so registers aren't pushed to memory
                pushf
                cli
		mov             si,OFFSET es:IDString
		mov             eax,DWORD PTR es:[si]
		inc             eax                     ; Construct valid ID string
		mov             ebx,eax

		; Iterate through base + high memory and look for string
		push            ds
		mov             esi,00000000h
@@CompareNext:  mov             ecx,esi
		and             ecx,0FFFF0000h
		shr             ecx,4
		mov             ds,cx                   ; Set segment base
		mov             eax,DWORD PTR [si]      ; Use 16-bit offset only
		cmp             eax,ebx
		je              FoundID
		add             esi,4                   ; Can't guarantee alignment :(
		cmp             esi,00100000h           ; Stop at 1MB
		jne             @@CompareNext
		pop             ds
                popf                                    ; Enable interrupts

		; Check we've got a BIOS with RTC support
		call            DetectRTC
		jnc             @@RTCOK

		; Failed to detect the RTC
		mov             ah,9
		mov             dx,OFFSET RTCError
		int             021h
		jmp             Terminate

@@RTCOK:        ; Try to init the MPU interface and switch to UART mode
		call            SwitchToUART
		jnc             @@UARTOK

		; Failed to init the hardware MPU
		mov             ah,9
		mov             dx,OFFSET MPUErrorStart
		int             021h
		mov             dx,OFFSET MPUPortNum
		int             021h
		mov             dx,OFFSET MPUErrorEnd
		int             021h
		jmp             Terminate

@@UARTOK:       ; MPU detected
		mov             ah,9
		mov             dx,OFFSET MPUSuccessStart
		int             021h
		mov             dx,OFFSET MPUPortNum
		int             021h
		mov             dx,OFFSET MPUSuccessEnd
		int             021h

                ; Try to detect a Sound Blaster at the specified port & irq
		call            DetectSB
		jnc             @@SBOK

		; Failed to detect a Sound Blaster
		mov             ah,9
		mov             dx,OFFSET SBErrorStart
		int             021h
		mov             dx,OFFSET SBPortNum
		int             021h
		mov             dx,OFFSET SBErrorMid
		int             021h
		mov             dx,OFFSET SBIRQNum
		int             021h
		mov             dx,OFFSET SBErrorEnd
		int             021h
		jmp             Terminate

@@SBOK:         ; Sound Blaster detected
		mov             ah,9
		mov             dx,OFFSET SBSuccessStart
		int             021h
		mov             dx,OFFSET SBPortNum
		int             021h
		mov             dx,OFFSET SBSuccessMid
		int             021h
		mov             dx,OFFSET SBIRQNum
		int             021h
		mov             dx,OFFSET SBSuccessEnd
		int             021h
                cmp             es:DelaySysex,0
                je              @@NoDelay
                mov             dx,OFFSET DelaysEnabled
                int             021h                    ; Print sysex delay message
@@NoDelay:      pop             ds
		retf                                    ; Return to caller

EMMOK:          ; Ensure ds=cs
		push            ds
		mov             ax,cs
		mov             ds,ax
		mov             si,OFFSET es:IDString
		mov             eax,DWORD PTR es:[si]
		inc             eax
		mov             DWORD PTR es:[si],eax   ; Store complete ID string

                ; Install support handlers
                call            InstTimerISR            ; Install the IRQ 2/8 watchdog
                call            InstDOSISR              ; Install the exec handler

                ; Set RTC busy flag
		; Disable interrupts until we've replaced the handler
		pushf
		cli
                push            ds
                mov             ax,040h
                mov             ds,ax
                mov             si,0A0h
                mov             BYTE PTR ds:[si],1      ; Set busy flag
                pop             ds

		; Register our new timer handler
		mov             dx,OFFSET es:Int70Handler
		push            ds
		mov             ax,es
		mov             ds,ax                   ; New handler addr in ds:dx
		mov             ax,02570h               ; Int 70 = RTC
		int             021h                    ; Register new handler
		pop             ds
		popf                                    ; Enable interrupts

		; Initialise the RTC
		mov             al,0Bh                  ; Read status reg B
		call            ReadRTC
		or              al,040h                 ; Select periodic interrupts
		shl             ax,8
		mov             al,0Bh
		call            WriteRTC
		mov             al,0Ch
		call            ReadRTC                 ; Clear pending interrupt with a read
		mov             al,0Ah                  ; Read status reg A
		call            ReadRTC
		and             al,0F0h
                or              al,04h                  ; Select 4kHz interrupt rate
		shl             ax,8
		mov             al,0Ah
		call            WriteRTC

		; Enable RTC interrupts
		pushf
		cli                                     ; No interrupts while programming PIC
		in              al,0A1h
                and             al,0FEh                 ; Clear bit 0 = IRQ 8 (RTC)
		jmp             SHORT $+2               ; Short pause
		out             0A1h,al
		popf                                    ; Start RTC interrupts

		; Print success message
		mov             ah,9
		mov             dx,OFFSET LoadedStart
		int             021h
		mov             dx,OFFSET MPUPortNum
		int             021h
		mov             dx,OFFSET LoadedMid
		int             021h
		mov             dx,OFFSET SBIRQNum
		int             021h
		mov             dx,OFFSET LoadedEnd
		int             021h
		pop             ds

		; Free environment block
		push            es
		mov             ah,051h
		int             021h                    ; Get PSP seg addr in bx
		mov             es,bx
		mov             es,es:[02Ch]            ; Get environment block address
		mov             ah,049h
		int             021h                    ; Deallocate environment
		pop             es
                mov             es:VirtualisePorts,1    ; Enable port virtualisation
		mov             ah,031h
                mov             dx,SEG STACK            ; Int handlers have their own stack
		sub             dx,bx                   ; Calc resident size from PSP in para
		int             021h                    ; TSR

PrintHelp:      mov             ah,9
		mov             dx,OFFSET HelpText
		int             021h
		jmp             Terminate

FoundID:        pop             ds                      ; Restore segment base
                popf                                    ; Enable interrupts
		mov             ah,9
		mov             dx,OFFSET LoadedError
		int             021h
		jmp             Terminate

EMMErr:         ; Ensure ds=cs
		push            ds
		mov             ax,cs
		mov             ds,ax

                ; Clean up after ourselves
                call            ResetMPU                ; Switch back to normal mode
                call            RestoreSBISR            ; Remove SB handler

                ; Print an error and exit
		mov             ah,9
		mov             dx,OFFSET EMMError
		int             021h
Terminate:      pop             ds
		mov             ax,04C01h               ; Terminate program
		int             021h

		INCLUDE         utils.asm
		INCLUDE         strings.asm

TransientSeg    ENDS
