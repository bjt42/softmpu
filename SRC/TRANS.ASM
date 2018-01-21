;
; Copyright (C) 2013-2018 bjt, elianda
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

                ASSUME          ds:_DATA                ; Access local vars via cs

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
                cmp             eax,'MOC:'
                je              @@SerialParam
                cmp             eax,'moc:'
                je              @@SerialParam
                cmp             eax,'MBS:'
                je              @@EnableSBMIDI
                cmp             eax,'mbs:'
                je              @@EnableSBMIDI
		cmp             eax,'LED/'              ; Check for delay switch
		je              @@EnableDelay
		cmp             eax,'led/'
                je              @@EnableDelay
                cmp             eax,'5AR/'
                je              @@EnableFakeOff
                cmp             eax,'5ar/'
		jne             @@NextParam
@@EnableFakeOff:mov             es:FakeAllNotesOff,1    ; Enable simulated "All Notes Off"
                jmp             @@NextParam
@@EnableDelay:  mov             es:DelaySysex,1         ; Enable sysex delays
		jmp             @@NextParam
@@EnableSBMIDI: mov             es:OutMode,M_SBMIDI     ; Enable SB-MIDI mode
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
                jmp             @@NextParam
@@SerialParam:  cmp             cx,5                    ; Command plus 1-digit port
		jl              @@NextParam             ; Not enough chars left
		mov             al,[si]                 ; 1st hex digit
		call            CharToNum

                ; Check we've got a valid serial port number
                cmp             al,1
                jl              @@NextParam
                cmp             al,4
                jg              @@NextParam

                ; Get the serial port base address from the BIOS
                call            DetectSerial
                mov             es:SerialPortAddr,ax    ; Set port address
                mov             es:OutMode,M_SERIAL     ; Select serial output mode
@@NextParam:    sub             si,3                    ; Only step a byte at a time
                dec             cx
                cmp             cx,0
                jne             @@ParseCmdline
@@CheckPorts:   pop             ds                      ; Finished parsing so restore ds

		; Check we have valid port ranges
		cmp             es:MPUDataPortAddr,0220h
		jl              PrintHelp
		cmp             es:MPUDataPortAddr,0350h
		jg              PrintHelp
		cmp             es:SBPortAddr,0220h
                jl              @@SBPortNotOK
		cmp             es:SBPortAddr,0280h
                jle             @@SBPortOK

@@SBPortNotOK:  ; The SB port was out of range
                mov             es:SBPortAddr,0

@@SBPortOK:     ; Write the port addresses into our strings
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
                mov             bx,es:SerialPortAddr
		mov             ax,bx
		shr             ax,8
		call            NumToChar
                mov             COMPortNum,al
		mov             ax,bx
		shr             ax,4
		and             al,0Fh
		call            NumToChar
                mov             COMPortNum+1,al
		mov             ax,bx
		and             al,0Fh
		call            NumToChar
                mov             COMPortNum+2,al

		; Check we have a valid IRQ
		cmp             es:SBIRQ,2
                jl              @@SBIRQNotOK
		cmp             es:SBIRQ,11
                jle             @@SBIRQOK

@@SBIRQNotOK:   ; The SB IRQ was out of range
                mov             es:SBIRQ,0
                mov             es:SBPortAddr,0         ; Zero the SB port too

@@SBIRQOK:      ; Write the IRQ into our string
		mov             al,es:SBIRQ
		cmp             al,10
		jl              @@SingleDigit           ; More than one digit needed?
		sub             al,10
		call            NumToChar
		mov             SBIRQNum+1,al
		mov             al,1
@@SingleDigit:  call            NumToChar
		mov             SBIRQNum,al

                ; Check if QEMM is loaded
                mov             ax,03D00h               ; Open file R/O
                mov             dx,OFFSET QEMMDevice
                int             021h                    ; Get handle
                jc              @@CheckEMM              ; Check for EMM386 instead
                mov             bx,ax
                push            ds
                mov             ax,es
                mov             ds,ax
                mov             dx,OFFSET es:QPIEntry   ; Set read buffer
                mov             cx,4
                mov             ax,04402h
                int             021h                    ; Read QPI entry
                pop             ds
                pushf
                mov             ah,03Eh
                int             021h                    ; Close file
                popf
                jc              @@NoEMM
                mov             ah,03h                  ; QPI_GetVersion
                call            es:QPIEntry
                cmp             bx,0703h                ; Require QEMM >= 7.03
                jl              @@NoEMM
                push            es
                mov             ax,01A06h               ; QPI_GetIOCallback
		call		es:QPIEntry
		mov		ax,es
		mov		bx,di
		pop		es
                mov             es:OldQEMMIOCBSeg,ax
                mov             es:OldQEMMIOCBAddr,bx
                mov             es:QEMMInstalled,1      ; Set QEMM flag
                jmp             @@FoundEMM

@@CheckEMM:     ; Check if EMM386 is loaded
                mov             ax,03D00h               ; Open file R/O
                mov             dx,OFFSET EMMDevice
                int             021h                    ; Get handle
                jnc             @@CheckEMMVersion
                mov             ax,03D00h               ; Open file R/O
                mov             dx,OFFSET EMMDevice2
                int             021h                    ; Get handle
                jc              @@NoEMM                 ; Fail
@@CheckEMMVersion:
                mov             bx,ax
                mov             dx,OFFSET EMMVersion
                mov             cx,02h
                mov             ax,04402h               ; Get manager version
                int             021h
                pushf                                   ; Save error flag
                mov             ah,03Eh                 ; Close file
                int             021h
                popf
                jc              @@NoEMM
                cmp             EMMMajor,4              ; Require EMM386 >= 4.46
                jl              @@NoEMM
                cmp             EMMMinor,45
                jg              @@FoundEMM

@@NoEMM:        ; EMM not present
		mov             ah,9
		mov             dx,OFFSET NoEMM
		int             021h
                jmp             Terminate

@@FoundEMM:     ; Check if already loaded
		; Disable interrupts so registers aren't pushed to memory
		pushf
		cli
		mov             si,OFFSET es:IDString
		mov		ebx,es:[si]
                inc             ebx                     ; Installed TSRs ID is IDString+1

                ; Iterate through base + high memory and look for string
		push            ds
		xor		cx,cx
		mov             si,cx
		mov		ds,cx
                cld                                     ; Auto increment si
@@CompareNext:  lodsb                                   ; Low word/low byte
                mov             dl,al                   ; Save in dl
                lodsb                                   ; Low/high
                mov             dh,al                   ; Order
                lodsb                                   ; High/low
                mov             ah,al                   ; Move to ah
                lodsb                                   ; Load high/high to al
                xchg            al,ah                   ; Order high/high,high/low
                shl             eax,16                  ; To upper 16 bits
                mov             ax,dx                   ; Move lower 16 bits from dx to lower eax
		cmp             eax,ebx
		je              FoundID
                sub             si,3                    ; After 4 lodsb effectively +1
                cmp             si,16                   ; Inc seg instead, this is seg boundary safe
		jne             @@CompareNext
		mov		si,0
		inc		cx
		mov		ds,cx
                cmp             cx,0
                jne             @@CompareNext           ; cx will overflow at 1MB
		pop             ds
		popf                                    ; Enable interrupts

                ; Only init HW MPU when required
                cmp             es:OutMode,M_MPU401
                jne             @@SkipUART

                ; Try to init the MPU interface and switch to UART mode
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

@@SkipUART:     ; Only init serial interface when required
                cmp             es:OutMode,M_SERIAL
                jne             @@NoSerial
                mov             ah,9
                cmp             es:SerialPortAddr,0
                jne             @@SerialOK
                mov             dx,OFFSET COMError
                int             021h
                jmp             Terminate
@@SerialOK:     mov             dx,OFFSET COMSuccessStart
                int             021h
                mov             dx,OFFSET COMPortNum
                int             021h
                mov             dx,OFFSET COMSuccessEnd
                int             021h
                cmp             es:SBPortAddr,0
                jne             @@NoSerial
                mov             dx,OFFSET MPUIRQDisabled
                int             021h

@@NoSerial:     ; Only init SB when required
                cmp             es:SBPortAddr,0
                je              @@NoSBMIDI

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
                mov             dx,OFFSET SBDSPVer
		int             021h
                mov             dx,OFFSET SBSuccessMid1
		int             021h
		mov             dx,OFFSET SBPortNum
		int             021h
                mov             dx,OFFSET SBSuccessMid2
		int             021h
		mov             dx,OFFSET SBIRQNum
		int             021h
		mov             dx,OFFSET SBSuccessEnd
		int             021h
                cmp             es:OutMode,M_SBMIDI
                jne             @@NoSBMIDI
                mov             dx,OFFSET SBMIDIEnabled
                int             021h
                mov             dx,OFFSET MPUIRQDisabled
                int             021h
@@NoSBMIDI:     cmp             es:DelaySysex,0
                je              @@NoDelay
                mov             dx,OFFSET DelaysEnabled
                int             021h                    ; Print sysex delay message
@@NoDelay:      cmp             es:FakeAllNotesOff,0
                je              @@NoFakeNotes
                mov             dx,OFFSET FakeEnabled
                int             021h
@@NoFakeNotes:  pop             ds
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
                call            InstTimerISR            ; Install the IRQ 2 watchdog
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

                ; Disable RTC interrupts
                in              al,0A1h
                or              al,01h                  ; Set bit 0 = IRQ 8 (RTC)
                jmp             SHORT $+2               ; Short pause
                out             0A1h,al

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

		; Print success message
		mov             ah,9
		mov             dx,OFFSET LoadedStart
		int             021h
		mov             dx,OFFSET MPUPortNum
		int             021h

                ; Don't print IRQ in SB-MIDI mode or if no Sound Blaster
                cmp             es:OutMode,M_SBMIDI
                je              @@NoIRQMsg
                cmp             es:SBPortAddr,0
                je              @@NoIRQMsg
		mov             dx,OFFSET LoadedMid
		int             021h
		mov             dx,OFFSET SBIRQNum
		int             021h
@@NoIRQMsg:     mov             dx,OFFSET LoadedEnd
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
