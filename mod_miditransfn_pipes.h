/*
__ __| |           |  /_) |     ___|             |           |
  |   __ \   _ \  ' /  | |  / |      _ \ __ \   |      _` | __ \   __|
  |   | | |  __/  . \  |   <  |   |  __/ |   |  |     (   | |   |\__ \
 _|  _| |_|\___| _|\_\_|_|\_\\____|\___|_|  _| _____|\__,_|_.__/ ____/
  -----------------------------------------------------------------------------
  USBMIDIKLIK 4X4 - USB Midi advanced firmware for STM32F1 platform.
  Copyright (C) 2019 by The KikGen labs.
  LICENCE CREATIVE COMMONS - Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)

  This file is part of the USBMIDIKLIK-4x4 distribution
  https://github.com/TheKikGen/USBMidiKliK4x4
  Copyright (c) 2019 TheKikGen Labs team.
  -----------------------------------------------------------------------------
  Disclaimer.

  This work is licensed under the Creative Commons Attribution-NonCommercial 4.0 International License.
  To view a copy of this license, visit http://creativecommons.org/licenses/by-nc/4.0/
  or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

  NON COMMERCIAL - PERSONAL USE ONLY : You may not use the material for pure
  commercial closed code solution without the licensor permission.

  You are free to copy and redistribute the material in any medium or format,
  adapt, transform, and build upon the material.

  You must give appropriate credit, a link to the github site
  https://github.com/TheKikGen/USBMidiKliK4x4 , provide a link to the license,
  and indicate if changes were made. You may do so in any reasonable manner,
  but not in any way that suggests the licensor endorses you or your use.

  You may not apply legal terms or technological measures that legally restrict
  others from doing anything the license permits.

  You do not have to comply with the license for elements of the material
  in the public domain or where your use is permitted by an applicable exception
  or limitation.

  No warranties are given. The license may not give you all of the permissions
  necessary for your intended use.  This program is distributed in the hope that
  it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

*/

///////////////////////////////////////////////////////////////////////////////
//  MIDI TRANSFORMATION PIPES FUNCTIONS
//-----------------------------------------------------------------------------
// Due to the unusual make process of Arduino platform, this file is directly
// included in the main ino one.
//
// The midi transformation engine relies on a pipeline process. A "pipe" is
// an elementary midi tranformation function accepting a midi packet and some
// parameters. So a pipeline, is a chain of transformation functions, delivering
// a packet (or many) from a source port to a destination port.
//
// For each pipe  :
//   . A MidiTransFn_(pipe name)_CheckParms function to validate input prms
//   . A MidiTransFn_(pipe name) function being the transformation pipe itself
//   . Declaration of an pId in the MidiTransPipepId enum before FN_TRANSPIPE_VECTOR_SIZE
//   . An entry in the MidiTransFnVector functio pointer array with the pipe fn address
//   . An entry in the MidiTransFnVector_CheckParms array with the check parms fn address

// parameters are not checked at all at run time for speed optimization reason.
///////////////////////////////////////////////////////////////////////////////

#pragma once

//boolean MidiTransFn_MYPIPE_CheckParms(transPipe_t *);
//boolean MidiTransFn_MYPIPE(uint8_t, midiPacket_t *, transPipe_t *);

boolean MidiTransFn_MessageFilter_CheckParms(transPipe_t *);
boolean MidiTransFn_MessageFilter(uint8_t, midiPacket_t *, transPipe_t *);

boolean MidiTransFn_NoteChanger_CheckParms(transPipe_t *);
boolean MidiTransFn_NoteChanger(uint8_t, midiPacket_t *, transPipe_t *);

boolean MidiTransFn_ChannelMapper_CheckParms(transPipe_t *);
boolean MidiTransFn_ChannelMapper(uint8_t, midiPacket_t *, transPipe_t *);

boolean MidiTransFn_VeloChanger_CheckParms(transPipe_t *);
boolean MidiTransFn_VeloChanger(uint8_t, midiPacket_t *, transPipe_t *);

boolean MidiTransFn_CCChanger_CheckParms(transPipe_t *);
boolean MidiTransFn_CCChanger(uint8_t, midiPacket_t *, transPipe_t *);

boolean MidiTransFn_ClockDivider_CheckParms(transPipe_t *);
boolean MidiTransFn_ClockDivider(uint8_t, midiPacket_t *, transPipe_t *);

boolean MidiTransFn_LoopBack_CheckParms(transPipe_t *);
boolean MidiTransFn_LoopBack(uint8_t, midiPacket_t *, transPipe_t *);

boolean MidiTransFn_SlotChain_CheckParms(transPipe_t *);
boolean MidiTransFn_SlotChain(uint8_t, midiPacket_t *, transPipe_t *);

boolean MidiTransFn_KbSplit_CheckParms(transPipe_t *);
boolean MidiTransFn_KbSplit(uint8_t, midiPacket_t *, transPipe_t *);

boolean MidiTransFn_VeloSplit_CheckParms(transPipe_t *);
boolean MidiTransFn_VeloSplit(uint8_t, midiPacket_t *, transPipe_t *);

///////////////////////////////////////////////////////////////////////////////
//  Midi transformation functions vector
///////////////////////////////////////////////////////////////////////////////

// 1 Id by pipe. FN_TRANSPIPE_VECTOR_SIZE must the last one
// PID is the array index in the vector fn table, so the order must be strictly the same.
enum MidiTransPipeId {
  FN_TRANSPIPE_MSG_FILTER,
  FN_TRANSPIPE_NOTE_CHANGER,
  FN_TRANSPIPE_CHANNEL_MAPPER,
  FN_TRANSPIPE_VELO_CHANGER,
  FN_TRANSPIPE_CC_CHANGER,
  FN_TRANSPIPE_CLOCK_DIVIDER,
  FN_TRANSPIPE_LOOPBACK,
  FN_TRANSPIPE_SLOT_CHAIN,
  FN_TRANSPIPE_KB_SPLIT,
  FN_TRANSPIPE_VELO_SPLIT,
  FN_TRANSPIPE_VECTOR_SIZE,
} ;

// Transformation pipe function vector

typedef boolean (*MidiTransFnP_t) (uint8_t portType, midiPacket_t *pk, transPipe_t *pipe) ;
typedef boolean (*MidiTransFn_CheckParmsP_t) (transPipe_t *pipe) ;
typedef struct {
    char *                    shortName;
    MidiTransFnP_t            pipeFn;
    MidiTransFn_CheckParmsP_t checkFn;
} __packed MidiTransFnVector_t;

const MidiTransFnVector_t MidiTransFnVector[FN_TRANSPIPE_VECTOR_SIZE] = {
   {"MSGFLTR", &MidiTransFn_MessageFilter, &MidiTransFn_MessageFilter_CheckParms},
   {"NOTECHG", &MidiTransFn_NoteChanger,   &MidiTransFn_NoteChanger_CheckParms},
   {"CHANMAP", &MidiTransFn_ChannelMapper, &MidiTransFn_ChannelMapper_CheckParms},
   {"VELOCHG", &MidiTransFn_VeloChanger,   &MidiTransFn_VeloChanger_CheckParms},
   {"CCCHANG", &MidiTransFn_CCChanger,     &MidiTransFn_CCChanger_CheckParms},
   {"CLKDIVD", &MidiTransFn_ClockDivider,  &MidiTransFn_ClockDivider_CheckParms},
   {"LOOPBCK", &MidiTransFn_LoopBack,      &MidiTransFn_LoopBack_CheckParms},
   {"SLOTCHN", &MidiTransFn_SlotChain,     &MidiTransFn_SlotChain_CheckParms},
   {"KBSPLIT", &MidiTransFn_KbSplit,       &MidiTransFn_KbSplit_CheckParms},
   {"VLSPLIT", &MidiTransFn_VeloSplit,     &MidiTransFn_VeloSplit_CheckParms},
};

///////////////////////////////////////////////////////////////////////////////
// PIPES.
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// PipeID     par1                 par2                     par3            par4
//----------------------------------------------------------------------------
// 00 MSGFLTR 0:high level filter  Filter bits mask 	       00              00
//                                 channel voice:0001 (1)	   00              00
// 			                          syst. common :0010 (2)     00              00
// 			                          realtime     :0100 (4)     00              00
// 			                          sysex        :1000 (8)     00              00
// The mask must match with xParser definition.
///////////////////////////////////////////////////////////////////////////////
boolean MidiTransFn_MessageFilter_CheckParms(transPipe_t *pipe) {
  if ( pipe->par1 > 0 ) return false;
  if ( pipe->par2 == 0 || pipe->par2 > 0B1111 ) return false;

  return true;
}

boolean MidiTransFn_MessageFilter(uint8_t portType, midiPacket_t *pk, transPipe_t *pipe) {

	// Apply high level midi filters before pipeline
  if ( pipe->par1 == 0 && (midiXparser::getMidiStatusMsgTypeMsk(pk->packet[1]) & pipe->par2)  )
          return true;

  return false;
}

///////////////////////////////////////////////////////////////////////////////
// PipeID     par1                 par2                     par3            par4
//----------------------------------------------------------------------------
//                par1                  par2                par3            par4
// 01 NOTECHG transpose+:0         semitone:00-7F            00              00
//            transpose-:1         semitone:00-7F            00              00
///////////////////////////////////////////////////////////////////////////////
boolean MidiTransFn_NoteChanger_CheckParms(transPipe_t *pipe) {
  if ( pipe->par1 > 1 ) return false;
  if ( pipe->par2 == 0 || pipe->par2 == 0x7F ) return false;
  return true;
}

boolean MidiTransFn_NoteChanger(uint8_t portType, midiPacket_t *pk, transPipe_t *pipe) {

  uint8_t midiStatus = pk->packet[1] & 0xF0;

  // Only notes ON/OFF
  if ( midiStatus != midiXparser::noteOffStatus && midiStatus != midiXparser::noteOnStatus)
        return true;

  // Add semitones to the note value
  if (pipe->par1 == 0 ) {
    if ( (pk->packet[2] + pipe->par2 > 127) ) return false ; // Drop packet
    pk->packet[2] += pipe->par2;
  }
  else

  // Sub semitones to the note value
  if (pipe->par1 == 1 ) {
    if ( pk->packet[2] - pipe->par2 < 0 ) return false;
    pk->packet[2] -= pipe->par2;
  }
  else
  return false; // Error

  return true;
}

///////////////////////////////////////////////////////////////////////////////
// PipeID     par1                 par2                     par3            par4
//----------------------------------------------------------------------------
// 02 CHANMAP ch map:0             source ch:0-F|any ch:7F   dest ch:0-F     00
//            ch map to port:1     source ch:0-F           midi port:0-F     00
//            ch offset:2          offset:0-F                00              00
///////////////////////////////////////////////////////////////////////////////
boolean MidiTransFn_ChannelMapper_CheckParms(transPipe_t *pipe) {
  if ( pipe->par1 > 2 ) return false;
  if ( pipe->par1 == 0 && pipe->par2 > 0x0F && pipe->par2 != 0x7F ) return false;
  if ( (pipe->par1 == 1 || pipe->par1 == 2) && pipe->par2 > 0x0F ) return false;
  if ( pipe->par3 > 0x0F ) return false;
  return true;
}

boolean MidiTransFn_ChannelMapper(uint8_t portType, midiPacket_t *pk, transPipe_t *pipe) {

  //Channel voice only
  if (midiXparser::getMidiStatusMsgTypeMsk(pk->packet[1]) !=  midiXparser::channelVoiceMsgTypeMsk)
      return true;

  // Map channel to channel
  if (pipe->par1 == 0 ) {
    // Any or matching channel
    if ( (pipe->par2 == 0x7f) || ( pipe->par2 == (pk->packet[1] & 0x0F) ) )
      pk->packet[1] = (pk->packet[1] & 0xF0) + pipe->par3;
  }
  else
  // Map channel to port (of the same port type)
  if (pipe->par1 == 1 ) {
    if ( pipe->par2 == (pk->packet[1] & 0x0F) )
        pk->packet[0] = (pk->packet[0] & 0x0F) + ( pipe->par3 << 4);
  }
  // channel offset
  if (pipe->par1 == 2 ) {
      uint8_t c = (pk->packet[1] & 0x0F) + pipe->par2;
      if ( c > 0x0F  ) c = 16 - c;
      pk->packet[1] = (pk->packet[1] & 0xF0) + c;
  }
  else return false; // Error

  return true;
}

///////////////////////////////////////////////////////////////////////////////
// PipeID     par1                 par2                    par3            par4
//----------------------------------------------------------------------------
// 03 VELOCHG  half velo:0        00                        00               00
//             full velo:1        00                        00               00
//                 range:2     [ value1 ,              value2 ]:00-7F        00
//               exclude:3     ] value1 ,              value2 [:00-7F        00
//                custom:4       add:00                   value:00-7F        00
//                               sub:01                   value:00-7F        00
//                             fixed:02                   value:00-7F        00
///////////////////////////////////////////////////////////////////////////////
boolean MidiTransFn_VeloChanger_CheckParms(transPipe_t *pipe) {
  if ( pipe->par1 > 4 ) return false;
  if ( (pipe->par1 == 2 || pipe->par1 == 3) && pipe->par2 > pipe->par3 ) return false;
  if ( pipe->par1 == 4 && pipe->par2 > 2) return false;
  if ( pipe->par1 == 4 && ( pipe->par2 == 0 || pipe->par2 == 0x7F) ) return false;
  return true;
}

boolean MidiTransFn_VeloChanger(uint8_t portType, midiPacket_t *pk, transPipe_t *pipe) {

  uint8_t midiStatus = pk->packet[1] & 0xF0;

  // Only notes ON
  if ( midiStatus != midiXparser::noteOnStatus) return true;

  // Half velocity
  if ( pipe->par1 == 0x00 ) {
    pk->packet[3] = 0x40;
  }
  else
  // Full velocity
  if ( pipe->par1 == 0x01 ) {
    pk->packet[3] = 127;
  }
  else
  // Include range
  if ( pipe->par1 == 0x02 ) {
    if ( pk->packet[3] < pipe->par2 || pk->packet[3] > pipe->par3 )
        return false; // drop packet
  }
  else
  // Exclude range
  if ( pipe->par1 == 0x03 ) {
    if ( pk->packet[3] >= pipe->par2 || pk->packet[3] <= pipe->par3 )
        return false; // drop packet
  }
  else
  // Custom
  if (pipe->par1 == 0x04 ) {
      if ( pipe->par2 == 00 ) { CONSTRAINT_ADD(pk->packet[3],pipe->par4,127); } // Add
      else if ( pipe->par2 == 01 ) { CONSTRAINT_SUB(pk->packet[3],pipe->par4,0); } // Sub
      else if ( pipe->par3 == 02 ) pk->packet[3] = pipe->par4; // Fixed
      else return false;
  }
  else return false; // Error

  return true;
}

///////////////////////////////////////////////////////////////////////////////
// PipeID     par1                 par2                    par3            par4
//----------------------------------------------------------------------------
// 04 CCCHANG        map:0     source cc:00-7F           dest cc:00-7F       00
//                 range:1     [ value1 ,              value2 ]:00-7F        00
//               exclude:2     ] value1 ,              value2 [:00-7F        00
//                invert:3     source cc:00-7F              00               00
///////////////////////////////////////////////////////////////////////////////
boolean MidiTransFn_CCChanger_CheckParms(transPipe_t *pipe) {
  if ( pipe->par1 > 3 ) return false;
  if ( (pipe->par1 == 1 || pipe->par1 == 2 ) && ( pipe->par2 > pipe->par3 ) ) return false;
  return true;
}

boolean MidiTransFn_CCChanger(uint8_t portType, midiPacket_t *pk, transPipe_t *pipe) {

  uint8_t midiStatus = pk->packet[1] & 0xF0;

  // Only CC
  if ( midiStatus != midiXparser::controlChangeStatus)
        return true;

  // Map
  if ( pipe->par1 == 0x00 ) {
    if ( pk->packet[2] == pipe->par2 ) pk->packet[2] = pipe->par3;
  }
  else
  // Include range
  if ( pipe->par1 == 0x01 ) {
    if ( pk->packet[2] < pipe->par2 || pk->packet[3] > pipe->par3 )
        return false; // drop packet
  }
  else
  // Exclude range
  if ( pipe->par1 == 0x02 ) {
    if ( pk->packet[2] >= pipe->par2 || pk->packet[3] <= pipe->par3 )
        return false; // drop packet
  }
  else
  // Invert
  if ( pipe->par1 == 0x03 ) {
    if ( pk->packet[2] == pipe->par2 )
        pk->packet[3] = 127 - pk->packet[3] ;
  }
  else return false; // Error

  return true;
}

///////////////////////////////////////////////////////////////////////////////
// PipeID     par1                 par2                    par3            par4
//----------------------------------------------------------------------------
// 05 CLKDIVD   ratio:2-16        00                        00               00
// Example for a div 2 : 1---(2)---3---(4)---5---(6)---7---(8)
// Example for a div 3 : 1---2---(3)---4---5---(6)---7---8---(9)
// (x) means clock sent
///////////////////////////////////////////////////////////////////////////////
boolean MidiTransFn_ClockDivider_CheckParms(transPipe_t *pipe) {
  if ( pipe->par1 < 2 || pipe->par1 >16 ) return false;
  return true;
}

boolean MidiTransFn_ClockDivider(uint8_t portType, midiPacket_t *pk, transPipe_t *pipe) {

  if ( pk->packet[1] != midiXparser::timingClockStatus)
      return true;

  static uint8_t clockCounter = 0;
  if ( ++clockCounter == pipe->par1 ) {
      clockCounter = 0;
      return true;
  }

  return false;
}

///////////////////////////////////////////////////////////////////////////////
// PipeID     par1                 par2                    par3            par4
//----------------------------------------------------------------------------
// 06 LOOPBCK dest portype dest    filter bits mask       port:0-F        00
//              cbl out:0
//                jk in:1
//              virt.in:2
//               no chg:3
///////////////////////////////////////////////////////////////////////////////
boolean MidiTransFn_LoopBack_CheckParms(transPipe_t *pipe) {
  if ( pipe->par1 > 3 ) return false;
  if ( pipe->par1 == 0 && pipe->par3 > USBCABLE_INTERFACE_MAX ) return false;
  if ( pipe->par1 == 1 && pipe->par3 > B_SERIAL_INTERFACE_MAX ) return false;
  if ( pipe->par1 == 2 && pipe->par3 > VIRTUAL_INTERFACE_MAX ) return false;
  if ( pipe->par2 == 0 || pipe->par2 > 0B1111 ) return false;

  return true;
}

boolean MidiTransFn_LoopBack(uint8_t portType, midiPacket_t *pk, transPipe_t *pipe) {
  uint8_t cin         = pk->packet[0] & 0x0F;
  uint8_t sourcePort  = pk->packet[0] >> 4;
  // Protect sysex integrity. Don't route a sysex pk 2 times on the same port
  // Cb0 F0 01 02 => Loopback cb0 F0 01 02 = a corrupted pk F0 01 02 F0 01 02
  // but let the packet in the pipeline.
  if ( cin <= 7 && cin >= 4 ) {
      if ( pipe->par1 == 3 || (pipe->par1 == portType && pipe->par3 == sourcePort) )
          return true;
  }

  // Apply filter
  if ( midiXparser::getMidiStatusMsgTypeMsk(pk->packet[1]) & pipe->par2  ) {
    midiPacket_t pk2 = { .i = pk->i }; // make a copy of the packet

    // No change
    if  ( pipe->par1 == 3 )
      RoutePacketToTarget(portType, &pk2);
    else {
      // Adjust the port/cable nible but keep the CIN
      pk2.packet[0] = cin + ( pipe->par3 << 4 );
      // Route with par1 value as  USB or JACK or VIRTUAL
      RoutePacketToTarget(pipe->par1 , &pk2);
    }
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////////
// PipeID     par1                 par2                    par3            par4
//----------------------------------------------------------------------------
// 07 SLOTCHN slot:1-8
///////////////////////////////////////////////////////////////////////////////
boolean MidiTransFn_SlotChain_CheckParms(transPipe_t *pipe) {
  if (pipe->par1 < 1 || pipe->par1 > TRANS_PIPELINE_SLOT_SIZE ) return false;
  return true;
}

boolean MidiTransFn_SlotChain(uint8_t portType, midiPacket_t *pk, transPipe_t *pipe) {
  return TransPacketPipelineExec(portType, pipe->par1,  pk) ;
}

///////////////////////////////////////////////////////////////////////////////
// PipeID     par1                 par2                    par3            par4
//----------------------------------------------------------------------------
// 07 KBSPLIT split note: 00-7F   midi ch:0-F       transpose+:0     semitone:00-7F
//                                                  transpose-:1     semitone:00-7F
//                                                  no chg    :2
///////////////////////////////////////////////////////////////////////////////
boolean MidiTransFn_KbSplit_CheckParms(transPipe_t *pipe) {
  if (pipe->par2 > 0x0F || pipe->par3 > 2) return false;
  return true;
}

boolean MidiTransFn_KbSplit(uint8_t portType, midiPacket_t *pk, transPipe_t *pipe) {

  uint8_t midiStatus = pk->packet[1] & 0xF0;
  // Only notes ON Notes OFF
  if ( midiStatus != midiXparser::noteOnStatus && midiStatus != midiXparser::noteOffStatus)
      return true;

  if ( pk->packet[2] >= pipe->par1  ) {
    pk->packet[1] = midiStatus + pipe->par2; // Channel
    if ( pipe->par3 < 2 ) {
        // Transpose
        transPipe_t p =  { FN_TRANSPIPE_NOTE_CHANGER,0,pipe->par3,pipe->par4,0,0 };
        return MidiTransFn_NoteChanger(portType, pk, &p);
    }
  }
  return true;
}


///////////////////////////////////////////////////////////////////////////////
// VeloSplit : Velocity split to midi channel .
//----------------------------------------------------------------------------
//                   par1               par2         par3            par4
// 09 VLSPLIT split velo: 00-7F   midi ch:0-F       add:0          value:00:7F
//                                                  sub:1          value:00:7F
//                                                  fixed:2        value:00:7F
//                                                  nochg:3            00
///////////////////////////////////////////////////////////////////////////////
boolean MidiTransFn_VeloSplit_CheckParms(transPipe_t *pipe) {
  if (pipe->par2 > 0x0F || pipe->par3 > 3) return false;
  return true;
}

boolean MidiTransFn_VeloSplit(uint8_t portType, midiPacket_t *pk, transPipe_t *pipe) {

  uint8_t midiStatus = pk->packet[1] & 0xF0;
  // Only notes ON Notes OFF
  if ( midiStatus != midiXparser::noteOnStatus&& midiStatus != midiXparser::noteOffStatus)
        return true;

  // Send a note off to the dest channel to ensure note stop  in all case
  // as no history of note on exists.
  if ( midiStatus == midiXparser::noteOffStatus ) {
      // Current packet copy
      midiPacket_t pk2 = { .i = pk->i };
      pk2.packet[1] = midiStatus + pipe->par2; // Change Channel
      RoutePacketToTarget(portType, &pk2);
      return true;
  }
  // Note On
  if ( pk->packet[3] >= pipe->par1  ) {
    pk->packet[1] = midiStatus + pipe->par2; // Channel
    if ( pipe->par3 == 0 ) { CONSTRAINT_ADD(pk->packet[3],pipe->par4,127); }
    else if ( pipe->par3 == 1 ) { CONSTRAINT_SUB(pk->packet[3],pipe->par4,0); }
    else if ( pipe->par3 == 2 ) pk->packet[3] = pipe->par4; // Fixed
  }
  return true;
}
