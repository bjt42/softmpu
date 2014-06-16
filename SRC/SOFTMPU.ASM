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
; CHANGELOG
;
; 0.5
;
; - Initial release
;
; 0.51
;
; - Fixed ID string search segment offsets
; - Fixed ACK behaviour after 'fake' UART switch
;
; 1.0
;
; - MPU IRQ emulation using SB DSP
; - Intelligent mode emulation (thanks to DOSBox Team!)
;
; 1.1
;
; - Optional SYSEX delays for Rev.0 MT-32
;
; 1.2
;
; - Fixed loaded detection
;
; 1.3
;
; - Merged latest DOSBox reset delay patch
; - Fixed "It Came From The Desert"
;
; 1.4
;
; - Fixed games with digital FX
; - Improved EMM386 detection
;
; 1.5
;
; - Optional simulated "All Notes Off" commands for RA-50
;
;
; 1.6
;
; - QEMM support (requires version 7.03+)
;
; 1.7
;
; - SB-MIDI support (disables interrupt generation)
;
; 1.8
;
; - Serial output support
; - Improved performance on slower systems
; - Tweaked playback speed
; - Fixed ESS ISA chipset detection
; - Fixed SYSEX delay timing
;
; 1.9
;
; - Large SYSEX message support
; ------------------------------------------

StackSize       EQU             0400h                   ; 1k stack for init

		.MODEL          SMALL,C                 ; Match model of MPU objfile
		.386                                    ; 16-bit segments

		.STACK          StackSize

		INCLUDE         export.inc              ; C functions
		INCLUDE         resident.asm            ; Resident code
		INCLUDE         trans.asm               ; Transient code

		END
