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
; App ID substrings
;

                ; Disable SB interrupts for these intelligent mode apps
AppIDStrings    DB              'FB.E'                  ; Flashback
                DB              'MM3.'                  ; Might and Magic III
                DB              'SOC.'                  ; Sensible Soccer
                DB              'UNVG'                  ; Waxworks
                DB              'WC2.'                  ; Wing Commander II
                DB              00h                     ; End
