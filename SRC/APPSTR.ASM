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

                ; Enable SB interrupts for these intelligent mode apps
AppIDStrings    DB              'SER.'                  ; Laser Squad
                DB              'UAD.'                  ; Laser Squad
                DB              'BCC.'                  ; Laser Squad
                DB              'RO1.'                  ; Laser Squad
                DB              'RO2.'                  ; Laser Squad
                DB              'RO3.'                  ; Laser Squad
                DB              'ATE.'                  ; Gateway
                DB              'AIN.'                  ; Gateway
                DB              00h                     ; End
