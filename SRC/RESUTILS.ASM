;
; Copyright (C) 2014 bjt
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
; Resident utility routines
;

                ASSUME          ds:_DATA                ; Access local vars via cs

HashString:     ; Generate a non case-sensitive hash from an ASCII string (K&R p.144)
                ; Input string address in ds:si
                ; Output hash in ebx
                ; Trashes eax, edx, flags
                mov             ebx,0
                cld                                     ; Auto increment si
                jmp             @@StartHash
@@NextByte:     cmp             al,061h
                jl              @@UpperCase
                cmp             al,07Ah
                jg              @@UpperCase
                sub             al,020h                 ; Convert to uppercase
@@UpperCase:    add             ax,31
                mul             ebx                     ; Accumulate hash value
                mov             ebx,eax
@@StartHash:    mov             eax,0
                lodsb
                cmp             al,0                    ; Null terminator?
                jne             @@NextByte
                retn
