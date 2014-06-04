/*
 *  Copyright (C) 2002-2013  The DOSBox Team
 *  Copyright (C) 2013-2014  bjt, elianda
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

/*
 * ------------------------------------------
 * SoftMPU by bjt - Software MPU-401 Emulator
 * ------------------------------------------
 *
 * Based on original midi.c from DOSBox
 *
 */

/* SOFTMPU: Moved exported functions & types to header */
#include "export.h"

/* SOFTMPU: Don't need these includes */
/*#include <assert.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <string>
#include <algorithm>

#include "SDL.h"

#include "dosbox.h"
#include "cross.h"
#include "support.h"
#include "setup.h"
#include "mapper.h"
#include "pic.h"
#include "hardware.h"
#include "timer.h"*/

/* SOFTMPU: Additional defines, typedefs etc. for C */
typedef unsigned long Bit32u;
typedef int Bits;

/* SOFTMPU: Sysex buffer is now allocated externally */
/*#define SYSEX_SIZE 1024*/
#define RAWBUF  1024

/* SOFTMPU: Note tracking for RA-50 */
#define MAX_TRACKED_CHANNELS 16
#define MAX_TRACKED_NOTES 8

static char* MIDI_welcome_msg = "\xf0\x41\x10\x16\x12\x20\x00\x00    SoftMPU v1.9    \x24\xf7"; /* SOFTMPU */

static Bit8u MIDI_note_off[3] = { 0x80,0x00,0x00 }; /* SOFTMPU */

static Bit8u MIDI_evt_len[256] = {
  0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,  // 0x00
  0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,  // 0x10
  0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,  // 0x20
  0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,  // 0x30
  0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,  // 0x40
  0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,  // 0x50
  0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,  // 0x60
  0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,  // 0x70

  3,3,3,3, 3,3,3,3, 3,3,3,3, 3,3,3,3,  // 0x80
  3,3,3,3, 3,3,3,3, 3,3,3,3, 3,3,3,3,  // 0x90
  3,3,3,3, 3,3,3,3, 3,3,3,3, 3,3,3,3,  // 0xa0
  3,3,3,3, 3,3,3,3, 3,3,3,3, 3,3,3,3,  // 0xb0

  2,2,2,2, 2,2,2,2, 2,2,2,2, 2,2,2,2,  // 0xc0
  2,2,2,2, 2,2,2,2, 2,2,2,2, 2,2,2,2,  // 0xd0

  3,3,3,3, 3,3,3,3, 3,3,3,3, 3,3,3,3,  // 0xe0

  0,2,3,2, 0,0,1,0, 1,0,1,1, 1,0,1,0   // 0xf0
};

/* SOFTMPU: Note tracking for RA-50 */
typedef struct {
        Bit8u used;
        Bit8u next;
        Bit8u notes[MAX_TRACKED_NOTES];
} channel;

channel tracked_channels[MAX_TRACKED_CHANNELS];

static struct {
	Bitu mpuport;
        Bitu sbport;
        Bitu serialport;
	Bitu status;
	Bitu cmd_len;
	Bitu cmd_pos;
	Bit8u cmd_buf[8];
	Bit8u rt_buf[8];
	struct {
                Bit8u* buf;
                Bitu maxsize;
		Bitu used;
		Bitu delay;
		Bit32u start;
	} sysex;
        bool fakeallnotesoff;
	bool available;
	/*MidiHandler * handler;*/ /* SOFTMPU */
} midi;

/* SOFTMPU: Sysex delay is decremented from PIC_Update */
Bitu MIDI_sysex_delay;

/* SOFTMPU: Also used by MPU401_ReadStatus */
OutputMode MIDI_output_mode;

/* SOFTMPU: Initialised in mpu401.c */
extern QEMMInfo qemm;

static void PlayMsg_SBMIDI(Bit8u* msg,Bitu len)
{
        /* Output a MIDI message to the hardware using SB-MIDI */
        /* Wait for WBS clear, then output a byte */
	_asm
	{
			mov     bx,msg
			mov     cx,len                  ; Assume len < 2^16
			add     cx,bx                   ; Get end ptr
                        mov     dx,midi.sbport
                        add     dx,0Ch                  ; Select DSP write port
	NextByte:       cmp     bx,cx
			je      End
        WaitWBS:        cmp     qemm.installed,1
                        jne     WaitWBSUntrappedIN
			push	bx
                        mov     ax,01A00h               ; QPI_UntrappedIORead
                        call    qemm.qpi_entry
			mov	al,bl
			pop	bx
                        _emit   0A8h                    ; Emit test al,(next opcode byte)
                                                        ; Effectively skips next instruction
        WaitWBSUntrappedIN:
                        in      al,dx
                        or      al,al
                        js      WaitWBS
                        mov     al,038h                 ; Normal mode MIDI output
                        cmp     qemm.installed,1
                        jne     WaitWBSUntrappedOUT
			push 	bx
                        mov     bl,al                   ; bl = value
                        mov     ax,01A01h               ; QPI_UntrappedIOWrite
                        call    qemm.qpi_entry
			pop	bx
                        _emit   0A8h                    ; Emit test al,(next opcode byte)
                                                        ; Effectively skips next instruction
        WaitWBSUntrappedOUT:
                        out     dx,al
        WaitWBS2:       cmp     qemm.installed,1
                        jne     WaitWBS2UntrappedIN
			push	bx
                        mov     ax,01A00h               ; QPI_UntrappedIORead
                        call    qemm.qpi_entry
			mov	al,bl
			pop	bx
                        _emit   0A8h                    ; Emit test al,(next opcode byte)
                                                        ; Effectively skips next instruction
        WaitWBS2UntrappedIN:
                        in      al,dx
                        or      al,al
                        js      WaitWBS2
			mov     al,[bx]
                        cmp     qemm.installed,1
                        jne     WaitWBS2UntrappedOUT
			push 	bx
                        mov     bl,al                   ; bl = value
                        mov     ax,01A01h               ; QPI_UntrappedIOWrite
                        call    qemm.qpi_entry
			pop	bx
                        _emit   0A8h                    ; Emit test al,(next opcode byte)
                                                        ; Effectively skips next instruction
        WaitWBS2UntrappedOUT:
			out     dx,al
                        inc     bx
			jmp     NextByte

                        ; Nothing more to send
	End:            nop
	}
};

static void PlayMsg_Serial(Bit8u* msg,Bitu len)
{
        /* Output a MIDI message to a serial port */
        /* Wait for transmit register clear, then output a byte */
	_asm
	{
			mov     bx,msg
			mov     cx,len                  ; Assume len < 2^16
			add     cx,bx                   ; Get end ptr
                        mov     dx,midi.serialport
        NextByte:       add     dx,05h                  ; Select line status register
                        cmp     bx,cx
			je      End
        WaitTransmit:   cmp     qemm.installed,1
                        jne     WaitTransmitUntrappedIN
			push	bx
                        mov     ax,01A00h               ; QPI_UntrappedIORead
                        call    qemm.qpi_entry
			mov	al,bl
			pop	bx
                        _emit   0A8h                    ; Emit test al,(next opcode byte)
                                                        ; Effectively skips next instruction
        WaitTransmitUntrappedIN:
                        in      al,dx
                        and     al,040h                 ; Shift register empty?
                        jz      WaitTransmit
                        sub     dx,05h                  ; Select transmit data register
			mov     al,[bx]
                        cmp     qemm.installed,1
                        jne     WaitTransmitUntrappedOUT
			push 	bx
                        mov     bl,al                   ; bl = value
                        mov     ax,01A01h               ; QPI_UntrappedIOWrite
                        call    qemm.qpi_entry
			pop	bx
                        _emit   0A8h                    ; Emit test al,(next opcode byte)
                                                        ; Effectively skips next instruction
        WaitTransmitUntrappedOUT:
			out     dx,al
                        inc     bx
			jmp     NextByte

                        ; Nothing more to send
	End:            nop
	}
};

static void PlayMsg(Bit8u* msg,Bitu len)
{
        switch (MIDI_output_mode)
        {
        case M_MPU401:
                /* Output a MIDI message to the hardware */
                /* Wait for DRR clear, then output a byte */
                _asm
                {
                                mov     bx,msg
                                mov     cx,len                  ; Assume len < 2^16
                                add     cx,bx                   ; Get end ptr
                                mov     dx,midi.mpuport
                NextByte:       cmp     bx,cx
                                je      End
                                inc     dx                      ; Set cmd port
                WaitDRR:        cmp     qemm.installed,1
                                jne     WaitDRRUntrappedIN
                                push    bx
                                mov     ax,01A00h               ; QPI_UntrappedIORead
                                call    qemm.qpi_entry
                                mov     al,bl
                                pop     bx
                                _emit   0A8h                    ; Emit test al,(next opcode byte)
                                                                ; Effectively skips next instruction
                WaitDRRUntrappedIN:
                                in      al,dx
                                test    al,040h
                                jnz     WaitDRR
                                dec     dx                      ; Set data port
                                mov     al,[bx]
                                cmp     qemm.installed,1
                                jne     WaitDRRUntrappedOUT
                                push    bx
                                mov     bl,al                   ; bl = value
                                mov     ax,01A01h               ; QPI_UntrappedIOWrite
                                call    qemm.qpi_entry
                                pop     bx
                                _emit   0A8h                    ; Emit test al,(next opcode byte)
                                                                ; Effectively skips next instruction
                WaitDRRUntrappedOUT:
                                out     dx,al
                                inc     bx
                                jmp     NextByte

                                ; Nothing more to send
                End:            nop
                }
                break;
        case M_SBMIDI:
                return PlayMsg_SBMIDI(msg,len);
        case M_SERIAL:
                return PlayMsg_Serial(msg,len);
        default:
                break;
        }
};

/* SOFTMPU: Fake "All Notes Off" for Roland RA-50 */
static void FakeAllNotesOff(Bitu chan)
{
        Bitu note;
        channel* pChan;

        MIDI_note_off[0] &= 0xf0;
        MIDI_note_off[0] |= (Bit8u)chan;

        pChan=&tracked_channels[chan];

        for (note=0;note<pChan->used;note++)
        {
                MIDI_note_off[1]=pChan->notes[note];
                PlayMsg(MIDI_note_off,3);
        }

        pChan->used=0;
        pChan->next=0;
}

void MIDI_RawOutByte(Bit8u data) {
        channel* pChan; /* SOFTMPU */

        if (midi.sysex.start && MIDI_sysex_delay) {
                _asm
                {
                                ; Bit 4 of port 061h toggles every 15.085us
                                ; Use this to time the remaining sysex delay
                                mov     ax,MIDI_sysex_delay
                                mov     bx,17                   ; Assume 4kHz RTC
                                mul     bx                      ; Convert to ticks, result in ax
                                mov     cx,ax
                                in      al,061h
                                and     al,010h                 ; Get initial value
                                mov     bl,al
                TestPort:       in      al,061h
                                and     al,010h
                                cmp     al,bl
                                je      TestPort                ; Loop until toggled
                                xor     bl,010h                 ; Invert
                                loop    TestPort
                                mov     MIDI_sysex_delay,0      ; Set original delay to zero
                }
                /*Bit32u passed_ticks = GetTicks() - midi.sysex.start;
                if (passed_ticks < midi.sysex.delay) SDL_Delay(midi.sysex.delay - passed_ticks);*/ /* SOFTMPU */
        }

	/* Test for a realtime MIDI message */
	if (data>=0xf8) {
		midi.rt_buf[0]=data;
		PlayMsg(midi.rt_buf,1);
		return;
	}        
	/* Test for a active sysex tranfer */
	if (midi.status==0xf0) {
		if (!(data&0x80)) { 
                        if (midi.sysex.used<(midi.sysex.maxsize-1)) midi.sysex.buf[midi.sysex.used++] = data; /* SOFTMPU */
			return;
		} else {
			midi.sysex.buf[midi.sysex.used++] = 0xf7;

			if ((midi.sysex.start) && (midi.sysex.used >= 4) && (midi.sysex.used <= 9) && (midi.sysex.buf[1] == 0x41) && (midi.sysex.buf[3] == 0x16)) {
				/*LOG(LOG_ALL,LOG_ERROR)("MIDI:Skipping invalid MT-32 SysEx midi message (too short to contain a checksum)");*/ /* SOFTMPU */
			} else {
				/*LOG(LOG_ALL,LOG_NORMAL)("Play sysex; address:%02X %02X %02X, length:%4d, delay:%3d", midi.sysex.buf[5], midi.sysex.buf[6], midi.sysex.buf[7], midi.sysex.used, midi.sysex.delay);*/
				PlayMsg(midi.sysex.buf, midi.sysex.used); /* SOFTMPU */
				if (midi.sysex.start) {
					if (midi.sysex.buf[5] == 0x7F) {
                                            /*midi.sysex.delay = 290;*/ /* SOFTMPU */ // All Parameters reset
                                            MIDI_sysex_delay = 290*(RTCFREQ/1000);
					} else if (midi.sysex.buf[5] == 0x10 && midi.sysex.buf[6] == 0x00 && midi.sysex.buf[7] == 0x04) {
                                            /*midi.sysex.delay = 145;*/ /* SOFTMPU */ // Viking Child
                                            MIDI_sysex_delay = 145*(RTCFREQ/1000);
					} else if (midi.sysex.buf[5] == 0x10 && midi.sysex.buf[6] == 0x00 && midi.sysex.buf[7] == 0x01) {
                                            /*midi.sysex.delay = 30;*/ /* SOFTMPU */ // Dark Sun 1
                                            MIDI_sysex_delay = 30*(RTCFREQ/1000);
                                        } else MIDI_sysex_delay = ((midi.sysex.used/2)+2)*(RTCFREQ/1000); /*(Bitu)(((float)(midi.sysex.used) * 1.25f) * 1000.0f / 3125.0f) + 2;
                                        midi.sysex.start = GetTicks();*/ /* SOFTMPU */
				}
			}

			/*LOG(LOG_ALL,LOG_NORMAL)("Sysex message size %d",midi.sysex.used);*/ /* SOFTMPU */
			/*if (CaptureState & CAPTURE_MIDI) {
				CAPTURE_AddMidi( true, midi.sysex.used-1, &midi.sysex.buf[1]);
			}*/ /* SOFTMPU */
		}
	}
	if (data&0x80) {
		midi.status=data;
		midi.cmd_pos=0;
		midi.cmd_len=MIDI_evt_len[data];
		if (midi.status==0xf0) {
			midi.sysex.buf[0]=0xf0;
			midi.sysex.used=1;
		}
	}
	if (midi.cmd_len) {
		midi.cmd_buf[midi.cmd_pos++]=data;
		if (midi.cmd_pos >= midi.cmd_len) {
			/*if (CaptureState & CAPTURE_MIDI) {
				CAPTURE_AddMidi(false, midi.cmd_len, midi.cmd_buf);
                        }*/ /* SOFTMPU */

                        if (midi.fakeallnotesoff)
                        {
                                /* SOFTMPU: Test for "Note On" */
                                if ((midi.status&0xf0)==0x90)
                                {
                                        if (midi.cmd_buf[2]>0)
                                        {
                                                pChan=&tracked_channels[midi.status&0x0f];
                                                pChan->notes[pChan->next++]=midi.cmd_buf[1];
                                                if (pChan->next==MAX_TRACKED_NOTES) pChan->next=0;
                                                if (pChan->used<MAX_TRACKED_NOTES) pChan->used++;
                                        }

                                        PlayMsg(midi.cmd_buf,midi.cmd_len);
                                }
                                /* SOFTMPU: Test for "All Notes Off" */
                                else if (((midi.status&0xf0)==0xb0) &&
                                         (midi.cmd_buf[1]>=0x7b) &&
                                         (midi.cmd_buf[1]<=0x7f))
                                {
                                        FakeAllNotesOff(midi.status&0x0f);
                                }
                                else
                                {
                                        PlayMsg(midi.cmd_buf,midi.cmd_len);
                                }
                        }
                        else
                        {
                                PlayMsg(midi.cmd_buf,midi.cmd_len);
                        }
                        midi.cmd_pos=1;         //Use Running status
		}
	}
}

bool MIDI_Available(void)  {
	return midi.available;
}

/* SOFTMPU: Initialisation */
void MIDI_Init(Bit8u* sysexbuf,Bitu maxsysexsize,Bitu mpuport,Bitu sbport,Bitu serialport,OutputMode outputmode,bool delaysysex,bool fakeallnotesoff){
        Bitu i; /* SOFTMPU */
        midi.sysex.buf = sysexbuf;
        midi.sysex.maxsize = maxsysexsize;
	midi.sysex.delay = 0;
	midi.sysex.start = 0;
	MIDI_sysex_delay = 0; /* SOFTMPU */

        if (delaysysex==true)
	{
		midi.sysex.start = 1; /*GetTicks();*/ /* SOFTMPU */
		/*LOG_MSG("MIDI:Using delayed SysEx processing");*/ /* SOFTMPU */
	}
	midi.mpuport=mpuport;
        midi.sbport=sbport;
        midi.serialport=serialport;
	midi.status=0x00;
	midi.cmd_pos=0;
	midi.cmd_len=0;
        midi.fakeallnotesoff=fakeallnotesoff;
        midi.available=true;
        MIDI_output_mode=outputmode;

        /* SOFTMPU: Display welcome message on MT-32 */
        for (i=0;i<30;i++)
        {
                MIDI_RawOutByte(MIDI_welcome_msg[i]);
        }

        /* SOFTMPU: Init note tracking */
        for (i=0;i<MAX_TRACKED_CHANNELS;i++)
        {
                tracked_channels[i].used=0;
                tracked_channels[i].next=0;
        }
}

/* DOSBox initialisation code */
#if 0
class MIDI:public Module_base{
public:
	MIDI(Section* configuration):Module_base(configuration){
		Section_prop * section=static_cast<Section_prop *>(configuration);
		const char * dev=section->Get_string("mididevice");
		std::string fullconf=section->Get_string("midiconfig");
		/* If device = "default" go for first handler that works */
		MidiHandler * handler;
//              MAPPER_AddHandler(MIDI_SaveRawEvent,MK_f8,MMOD1|MMOD2,"caprawmidi","Cap MIDI");
		midi.sysex.delay = 0;
		midi.sysex.start = 0;
		if (fullconf.find("delaysysex") != std::string::npos) {
			midi.sysex.start = GetTicks();
			fullconf.erase(fullconf.find("delaysysex"));
			LOG_MSG("MIDI:Using delayed SysEx processing");
		}
		std::remove(fullconf.begin(), fullconf.end(), ' ');
		const char * conf = fullconf.c_str();
		midi.status=0x00;
		midi.cmd_pos=0;
		midi.cmd_len=0;
		if (!strcasecmp(dev,"default")) goto getdefault;
		handler=handler_list;
		while (handler) {
			if (!strcasecmp(dev,handler->GetName())) {
				if (!handler->Open(conf)) {
					LOG_MSG("MIDI:Can't open device:%s with config:%s.",dev,conf);
					goto getdefault;
				}
				midi.handler=handler;
				midi.available=true;    
				LOG_MSG("MIDI:Opened device:%s",handler->GetName());
				return;
			}
			handler=handler->next;
		}
		LOG_MSG("MIDI:Can't find device:%s, finding default handler.",dev);     
getdefault:     
		handler=handler_list;
		while (handler) {
			if (handler->Open(conf)) {
				midi.available=true;    
				midi.handler=handler;
				LOG_MSG("MIDI:Opened device:%s",handler->GetName());
				return;
			}
			handler=handler->next;
		}
		/* This shouldn't be possible */
	}
	~MIDI(){
		if(midi.available) midi.handler->Close();
		midi.available = false;
		midi.handler = 0;
	}
};


static MIDI* test;
void MIDI_Destroy(Section* /*sec*/){
	delete test;
}
void MIDI_Init(Section * sec) {
	test = new MIDI(sec);
	sec->AddDestroyFunction(&MIDI_Destroy,true);
}
#endif

/* DOSBox MIDI handler code */
#if 0
class MidiHandler;

MidiHandler * handler_list=0;

class MidiHandler {
public:
	MidiHandler() {
		next=handler_list;
		handler_list=this;
	};
	virtual bool Open(const char * /*conf*/) { return true; };
	virtual void Close(void) {};
	virtual void PlayMsg(Bit8u * /*msg*/) {};
	virtual void PlaySysex(Bit8u * /*sysex*/,Bitu /*len*/) {};
	virtual const char * GetName(void) { return "none"; };
	virtual ~MidiHandler() { };
	MidiHandler * next;
};

MidiHandler Midi_none;

/* Include different midi drivers, lowest ones get checked first for default */

#if defined(MACOSX)

#include "midi_coremidi.h"
#include "midi_coreaudio.h"

#elif defined (WIN32)

#include "midi_win32.h"

#else

#include "midi_oss.h"

#endif

#if defined (HAVE_ALSA)

#include "midi_alsa.h"

#endif
#endif /* if 0 */
