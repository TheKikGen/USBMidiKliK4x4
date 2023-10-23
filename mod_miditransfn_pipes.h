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
boolean MidiTransFn_MessageFilter(uint8_t, midiPacket_t *, midiPacket_t *, transPipe_t *);

boolean MidiTransFn_NoteChanger_CheckParms(transPipe_t *);
boolean MidiTransFn_NoteChanger(uint8_t, midiPacket_t *, midiPacket_t *, transPipe_t *);

boolean MidiTransFn_ChannelMapper_CheckParms(transPipe_t *);
boolean MidiTransFn_ChannelMapper(uint8_t, midiPacket_t *, midiPacket_t *, transPipe_t *);

boolean MidiTransFn_VeloChanger_CheckParms(transPipe_t *);
boolean MidiTransFn_VeloChanger(uint8_t, midiPacket_t *, midiPacket_t *, transPipe_t *);

boolean MidiTransFn_CCChanger_CheckParms(transPipe_t *);
boolean MidiTransFn_CCChanger(uint8_t, midiPacket_t *, midiPacket_t *, transPipe_t *);

boolean MidiTransFn_ClockDivider_CheckParms(transPipe_t *);
boolean MidiTransFn_ClockDivider(uint8_t, midiPacket_t *, midiPacket_t *, transPipe_t *);

boolean MidiTransFn_LoopBack_CheckParms(transPipe_t *);
boolean MidiTransFn_LoopBack(uint8_t, midiPacket_t *, midiPacket_t *, transPipe_t *);

boolean MidiTransFn_SlotChain_CheckParms(transPipe_t *);
boolean MidiTransFn_SlotChain(uint8_t, midiPacket_t *, midiPacket_t *, transPipe_t *);

boolean MidiTransFn_KbSplit_CheckParms(transPipe_t *);
boolean MidiTransFn_KbSplit(uint8_t, midiPacket_t *, midiPacket_t *, transPipe_t *);

boolean MidiTransFn_VeloSplit_CheckParms(transPipe_t *);
boolean MidiTransFn_VeloSplit(uint8_t, midiPacket_t *, midiPacket_t *, transPipe_t *);

int8_t MidiTransFn_VeloCurveLine(uint8_t veloIn, float a, float b, float c, float d); // Utility function only
boolean MidiTransFn_VeloCurv1_CheckParms(transPipe_t *);
boolean MidiTransFn_VeloCurv1(uint8_t, midiPacket_t *, midiPacket_t *, transPipe_t *);

boolean MidiTransFn_VeloCurv2_CheckParms(transPipe_t *);
boolean MidiTransFn_VeloCurv2(uint8_t, midiPacket_t *, midiPacket_t *, transPipe_t *);

boolean MidiTransFn_VeloCurv3_CheckParms(transPipe_t *);
boolean MidiTransFn_VeloCurv3(uint8_t, midiPacket_t *, midiPacket_t *, transPipe_t *);

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
  FN_TRANSPIPE_VELO_CURV1,
  FN_TRANSPIPE_VELO_CURV2,
  FN_TRANSPIPE_VELO_CURV3,
  FN_TRANSPIPE_VECTOR_SIZE,
} ;

// Transformation pipe function vector

typedef boolean (*MidiTransFnP_t) (uint8_t portType, midiPacket_t *pkSource, midiPacket_t *pk, transPipe_t *pipe) ;
typedef boolean (*MidiTransFn_CheckParmsP_t) (transPipe_t *pipe) ;
typedef struct {
    const char *                    shortName;
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
   {"VLCURV1", &MidiTransFn_VeloCurv1,     &MidiTransFn_VeloCurv1_CheckParms},
   {"VLCURV2", &MidiTransFn_VeloCurv2,     &MidiTransFn_VeloCurv2_CheckParms},
   {"VLCURV3", &MidiTransFn_VeloCurv3,     &MidiTransFn_VeloCurv3_CheckParms},
};

///////////////////////////////////////////////////////////////////////////////
// PIPES.
///////////////////////////////////////////////////////////////////////////////
// -----------------------------------------------------------------------------------------------
// | PipeID  |        par1        |        par2        |        par3        |         par4       |
// |---------+--------------------+--------------------+--------------------+--------------------|
// | MSGFLTR |   Bits mask        |   include:0        | bits mask.         |     00 (unused)    |
// |   00    |   filter:0         |   exclude:1        |  ch voice:0001 (1) |                    |
// |         |                    |   select:2         |  Sys.cmn :0010 (2) |                    |
// |         |                    |                    |  realtime:0100 (4) |                    |
// |         |                    |                    |  sysex   :1000 (8) |                    |
// |         |                    |                    |                    |                    |
// |         |   MidiStatus       |   include:0        |  midi status id1** OR  midi status id2  |
// |         |  dble filter:1     |   exclude:1        |                    |  if par4 unused:0  |
// |         |                    |   select:2         |  (see Midi status ids table for values) |
// |         |                    |                    |                    |                    |
// |         |  midi channel      |   include:0        |  from channel 0-F  |  to channel 0-F    |
// |         |  filter:2          |   exclude:1        |                    |                    |
// |         |                    |   select:2         |                    |                    |  
// -----------------------------------------------------------------------------------------------
// --------------------------------
// |   ** Midi status ids table   |
// |------------------------------|
// | noteOffStatus         | 0X08 |
// | noteOnStatus          | 0X09 |
// | polyKeyPressureStatus | 0X0A |
// | controlChangeStatus   | 0X0B |
// | programChangeStatus   | 0X0C |
// | channelPressureStatus | 0X0D |
// | pitchBendStatus       | 0X0E |
// | midiTimeCodeStatus    | 0X11 |
// | songPosPointerStatus  | 0X12 |
// | songSelectStatus      | 0X13 |
// | tuneRequestStatus     | 0X16 |
// | timingClockStatus     | 0X18 |
// | startStatus           | 0X1A |
// | continueStatus        | 0X1B |
// | stopStatus            | 0X1C |
// | activeSensingStatus   | 0X1E |
// | systemResetStatus     | 0X1F |
// --------------------------------
boolean MidiTransFn_MessageFilter_CheckParms(transPipe_t *pipe)
{
  if ( pipe->par1 > 2 ) return false;
  if ( pipe->par1 == 0 && (pipe->par2 > 2 || pipe->par3 == 0 || pipe->par3 > 0B1111) ) return false;
  if ( pipe->par1 == 1 ) {
     if ( pipe->par2 > 2 ) return false;
     if ( pipe->par3 < 0x08 || pipe->par3 > 0x1F
          || pipe->par3 == 0x10 || pipe->par3 == 0x14 || pipe->par3 == 0x15
          || pipe->par3 == 0x17 || pipe->par3 == 0x19 || pipe->par3 == 0x1D ) {
          return false;
      }
      if ( pipe->par4 != 0 ) {
        if ( pipe->par4 < 0x08 || pipe->par4 > 0x1F
            || pipe->par4 == 0x10 || pipe->par4 == 0x14 || pipe->par4 == 0x15
            || pipe->par4 == 0x17 || pipe->par4 == 0x19 || pipe->par4 == 0x1D ) {
              return false;
            }
      }
  } else
  // Midi channel filtering
  if ( pipe->par1 == 2 ) {
      if ( pipe->par2 > 2 ) return false;
      if ( pipe->par3 > 0x0F || pipe->par4 > 0x0F || pipe->par3 > pipe->par4 ) return false;
  }
  return true;
}

boolean MidiTransFn_MessageFilter(uint8_t portType, midiPacket_t *pkSource, midiPacket_t *pk, transPipe_t *pipe)
{

  boolean match = false;

	// Apply global bits mask include/exclude filter. Drop if not matching or route/keep if select.
  if ( pipe->par1 == 0  ) {
    uint8_t msgType = 0;
    uint8_t cin   = pk->packet[0] & 0x0F ;
    // Check if SysEx filter first and modify the msgType mask because it is a multipacket msg
    // Note the CIN 5 exception for tune request
    if ( ( cin <= 7 && cin >= 4 && pk->packet[1] != midiXparser::tuneRequestStatus ) )
          msgType = midiXparser::sysExMsgTypeMsk;
    else
          msgType = midiXparser::getMidiStatusMsgTypeMsk(pk->packet[1]); // All other msg
    match = ( pipe->par3 & msgType );
  
  }

  // Midi Status double filter
  else  if ( pipe->par1 == 1  ) {

    uint8_t midiStatus = ( pk->packet[1] >= 0xF0 ? pk->packet[1] - 0xE0 : pk->packet[1]>>4 ) ;
    match = ( midiStatus == pipe->par3 || midiStatus == pipe->par4  ) ;
    
  }

  // Midi channel from/to filtering
  else   if ( pipe->par1 == 2 ) {
    
    // Channel voice only
    if (midiXparser::getMidiStatusMsgTypeMsk(pk->packet[1]) !=  midiXparser::channelVoiceMsgTypeMsk)
        return true;
    
    uint8_t channel = pk->packet[1] & 0x0F;
    match = (channel >= pipe->par3 && channel <= pipe->par4) ;

  } 
  else return false; // Error

  // Packet match any filter ?

  if ( match ) {

    // Exclude matching packet (drop)
    return (pipe->par2 == 1 ? false : true );

  } 
  else {
    // select = let the matching packet in the transformation chain without droping not matching ones
    if ( pipe->par2 == 2 ) {
      RoutePacketToTarget(portType, pk); // No transformation will be applied as the slot is locked
      return false;
    }
    else {
      // the packet doesn't match : exclude = keep not matching /drop matching
      return (pipe->par2 == 1 ? true : false );
    }

  }

  return false; // Error
}
// -----------------------------------------------------------------------------------------------
// | PipeID  |        par1        |        par2        |        par3        |         par4       |
// |---------+--------------------+--------------------+--------------------+--------------------|
// | NOTECHG |    transpose+:0    |   semitone:00-7F   |     00 (unused)    |     00 (unused)    |
// |   01    |    transpose-:1    |   semitone:00-7F   |     00 (unused)    |     00 (unused)    |
// -----------------------------------------------------------------------------------------------
boolean MidiTransFn_NoteChanger_CheckParms(transPipe_t *pipe)
{
  if ( pipe->par1 > 1 ) return false;
  if ( pipe->par2 == 0 || pipe->par2 == 0x7F ) return false;
  return true;
}

boolean MidiTransFn_NoteChanger(uint8_t portType, midiPacket_t *pkSource, midiPacket_t *pk, transPipe_t *pipe)
{

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

// -----------------------------------------------------------------------------------------------
// | PipeID  |        par1        |        par2        |        par3        |         par4       |
// |---------+--------------------+--------------------+--------------------+--------------------|
// | CHANMAP |    channel map:0   | srce ch:0-F|any:7F |      dst ch:0-F    |     00 (unused)    |
// |   02    | ch map to port:1   |   source ch:0-F    |   midi port:0-F    |     00 (unused)    |
// |         | ch rotat.offset:2  |   ch offset:0-F    |     00 (unused)    |     00 (unused)    |
// -----------------------------------------------------------------------------------------------
boolean MidiTransFn_ChannelMapper_CheckParms(transPipe_t *pipe)
{
  if ( pipe->par1 > 2 ) return false;
  if ( pipe->par1 == 0 && pipe->par2 > 0x0F && pipe->par2 != 0x7F ) return false;
  if ( (pipe->par1 == 1 || pipe->par1 == 2) && pipe->par2 > 0x0F ) return false;
  if ( pipe->par3 > 0x0F ) return false;
  return true;
}

boolean MidiTransFn_ChannelMapper(uint8_t portType, midiPacket_t *pkSource, midiPacket_t *pk, transPipe_t *pipe)
{

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
  else
  // channel offset
  if (pipe->par1 == 2 ) {
      uint8_t c = (pk->packet[1] & 0x0F) + pipe->par2;
      if ( c > 0x0F  ) c = 16 - c;
      pk->packet[1] = (pk->packet[1] & 0xF0) + c;
  }
  else return false; // Error

  return true;
}
// -----------------------------------------------------------------------------------------------
// | PipeID  |        par1        |        par2        |        par3        |         par4       |
// |---------+--------------------+--------------------+--------------------+--------------------|
// | VELOCHG |     include:0      | from vl value:0-7F | to vl value:0-7F   |     00 (unused)    |
// |   03    |     exclude:1      | from vl value:0-7F | to vl value:0-7F   |     00 (unused)    |
// |         |       fixed:2      |     value:0-7F     |     00 (unused)    |     00 (unused)    |
// |         |         add:3      |     value:0-7F     |     00 (unused)    |     00 (unused)    |
// |         |         sub:4      |     value:0-7F     |     00 (unused)    |     00 (unused)    |
// |         |        half:5      |     00 (unused)    |     00 (unused)    |     00 (unused)    |
// -----------------------------------------------------------------------------------------------
boolean MidiTransFn_VeloChanger_CheckParms(transPipe_t *pipe)
{
  if ( pipe->par1 > 5 ) return false;
  if ( (pipe->par1 == 0 || pipe->par1 == 1) && pipe->par2 > pipe->par3 ) return false;
  if ( ( pipe->par1 == 3 || pipe->par1 == 4) && pipe->par2 == 0) return false;
  return true;
}

boolean MidiTransFn_VeloChanger(uint8_t portType, midiPacket_t *pkSource, midiPacket_t *pk, transPipe_t *pipe)
{

  uint8_t midiStatus = pk->packet[1] & 0xF0;

  // Only notes ON
  if ( midiStatus != midiXparser::noteOnStatus) return true;

  // Include range
  if ( pipe->par1 == 0x00 ) {
    if ( pk->packet[3] < pipe->par2 || pk->packet[3] > pipe->par3 )
        return false; // drop packet
  }
  else
  // Exclude range
  if ( pipe->par1 == 0x01 ) {
    if ( pk->packet[3] >= pipe->par2 || pk->packet[3] <= pipe->par3 )
        return false; // drop packet
  }
  else
  // Fixed value
  if ( pipe->par1 == 0x02 ) pk->packet[3] = pipe->par2;
  else
  // Add value
  if ( pipe->par1 == 0x03 ) { CONSTRAINT_ADD(pk->packet[3],pipe->par2,127); }
  else
  // Sub value
  if ( pipe->par1 == 0x04 ) { CONSTRAINT_SUB(pk->packet[3],pipe->par2,127); }
  else
  // Half velocity
  if ( pipe->par1 == 0x05 ) {
    pk->packet[3] /= 2;
  }
  else return false; // Error

  return true;
}

// -----------------------------------------------------------------------------------------------
// | PipeID  |        par1        |        par2        |        par3        |         par4       |
// |---------+--------------------+--------------------+--------------------+--------------------|
// | CCCHANG |         map:0      |    src cc :0-7F    |    dest cc:00-7F   |     00 (unused)    |
// |   04    |     include:1      | from cc value:0-7F |  to cc value:0-7F  |     00 (unused)    |
// |         |     exclude:2      | from cc value:0-7F |  to cc value:0-7F  |     00 (unused)    |
// |         |      invert:3      |    src cc :0-7F    |     00 (unused)    |     00 (unused)    |
// -----------------------------------------------------------------------------------------------
boolean MidiTransFn_CCChanger_CheckParms(transPipe_t *pipe)
{
  if ( pipe->par1 > 3 ) return false;
  if ( (pipe->par1 == 1 || pipe->par1 == 2 ) && ( pipe->par2 > pipe->par3 ) ) return false;
  return true;
}

boolean MidiTransFn_CCChanger(uint8_t portType, midiPacket_t *pkSource, midiPacket_t *pk, transPipe_t *pipe)
{

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

// -----------------------------------------------------------------------------------------------
// | PipeID  |        par1        |        par2        |        par3        |         par4       |
// |---------+--------------------+--------------------+--------------------+--------------------|
// | CLKDIVD |     ratio:2-10     |     00 (unused)    |     00 (unused)    |     00 (unused)    |
// |   05    |                    |                    |                    |                    |
// -----------------------------------------------------------------------------------------------
// Example for a div 2 : 1---(2)---3---(4)---5---(6)---7---(8)
// Example for a div 3 : 1---2---(3)---4---5---(6)---7---8---(9)
// (x) means clock sent

boolean MidiTransFn_ClockDivider_CheckParms(transPipe_t *pipe)
{
  if ( pipe->par1 < 2 || pipe->par1 >16 ) return false;
  return true;
}

boolean MidiTransFn_ClockDivider(uint8_t portType, midiPacket_t *pkSource, midiPacket_t *pk, transPipe_t *pipe)
{

  if ( pk->packet[1] != midiXparser::timingClockStatus)
      return true;

  static uint8_t clockCounter = 0;
  if ( ++clockCounter == pipe->par1 ) {
      clockCounter = 0;
      return true;
  }

  return false;
}

// -----------------------------------------------------------------------------------------------
// | PipeID  |        par1        |        par2        |        par3        |         par4       |
// |---------+--------------------+--------------------+--------------------+--------------------|
// | LOOPBCK |   dest port type.  |  inclusive filter  |      port:0-F      |     00 (unused)    |
// |   06    |      cbl out:0     |     bits mask      |                    |                    |
// |         |        jk in:1     |   (see MSGFLTR)    |                    |                    |
// |         |      virt.in:2     |                    |                    |                    |
// |         |       no chg:3     |                    |                    |                    |
// -----------------------------------------------------------------------------------------------
boolean MidiTransFn_LoopBack_CheckParms(transPipe_t *pipe)
{
  if ( pipe->par1 > 3 ) return false;
  if ( pipe->par1 == 0 && pipe->par3 > USBCABLE_INTERFACE_MAX ) return false;
  if ( pipe->par1 == 1 && pipe->par3 > B_SERIAL_INTERFACE_MAX ) return false;
  if ( pipe->par1 == 2 && pipe->par3 > VIRTUAL_INTERFACE_MAX ) return false;
  if ( pipe->par2 == 0 || pipe->par2 > 0B1111 ) return false;

  return true;
}

boolean MidiTransFn_LoopBack(uint8_t portType, midiPacket_t *pkSource, midiPacket_t *pk, transPipe_t *pipe)
{
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
  if ( midiXparser::getMidiStatusMsgTypeMsk(pk->packet[1]) & pipe->par2  )
  {
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

// -----------------------------------------------------------------------------------------------
// | PipeID  |        par1        |        par2        |        par3        |         par4       |
// |---------+--------------------+--------------------+--------------------+--------------------|
// | SLOTCHN |     slot:1-8       |     00 (unused)    |     00 (unused)    |     00 (unused)    |
// |   07    |                    |                    |                    |                    |
// -----------------------------------------------------------------------------------------------

boolean MidiTransFn_SlotChain_CheckParms(transPipe_t *pipe)
{
  if (pipe->par1 < 1 || pipe->par1 > TRANS_PIPELINE_SLOT_SIZE ) return false;
  return true;
}

boolean MidiTransFn_SlotChain(uint8_t portType, midiPacket_t *pkSource, midiPacket_t *pk, transPipe_t *pipe)
{
  return TransPacketPipelineExec(portType, pipe->par1,  pk) ;
}

// -----------------------------------------------------------------------------------------------
// | PipeID  |        par1        |        par2        |        par3        |         par4       |
// |---------+--------------------+--------------------+--------------------+--------------------|
// | KBSPLIT |  split note: 0-7F  |     midi ch:0-F    |     no change:0    |     00 (unused)    |
// |   08    |                    |                    |    transpose+:1    |   semitone:00-7F   |
// |         |                    |                    |    transpose-:2    |   semitone:00-7F   |
// -----------------------------------------------------------------------------------------------
boolean MidiTransFn_KbSplit_CheckParms(transPipe_t *pipe)
{
  if (pipe->par2 > 0x0F || pipe->par3 > 2) return false;
  return true;
}

boolean MidiTransFn_KbSplit(uint8_t portType, midiPacket_t *pkSource, midiPacket_t *pk, transPipe_t *pipe)
{
  uint8_t midiStatus = pk->packet[1] & 0xF0;
  // Only notes ON Notes OFF
  if ( midiStatus != midiXparser::noteOnStatus && midiStatus != midiXparser::noteOffStatus)
      return true;

  if ( pk->packet[2] >= pipe->par1  ) {
    pk->packet[1] = midiStatus + pipe->par2; // Channel
    if ( pipe->par3 == 1 || pipe->par3 == 2  ) {
      // Transpose+
      transPipe_t p =  { FN_TRANSPIPE_NOTE_CHANGER,0, (uint8_t)(pipe->par3 - 1),pipe->par4,0,0 };
      return MidiTransFn_NoteChanger(portType, pkSource, pk, &p);
    }
  }
  return true;
}
// -----------------------------------------------------------------------------------------------
// | PipeID  |        par1        |        par2        |        par3        |         par4       |
// |---------+--------------------+--------------------+--------------------+--------------------|
// | VLSPLIT | split veloc.: 0-7F |     midi ch:0-F    |       nochg:0      |     00 (unused)    |
// |   09    |                    |                    |       fixed:1      |     value:0-7F     |
// |         |                    |                    |         add:2      |     value:0-7F     |
// |         |                    |                    |         sub:3      |     value:0-7F     |
// -----------------------------------------------------------------------------------------------
boolean MidiTransFn_VeloSplit_CheckParms(transPipe_t *pipe)
{
  if (pipe->par2 > 0x0F || pipe->par3 > 3) return false;
  return true;
}

boolean MidiTransFn_VeloSplit(uint8_t portType, midiPacket_t *pkSource, midiPacket_t *pk, transPipe_t *pipe)
{
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
  }
  else
  // Note On
  if ( pk->packet[3] >= pipe->par1  ) {
    pk->packet[1] = midiStatus + pipe->par2; // Channel
    if ( pipe->par3 == 1 ) pk->packet[3] = pipe->par4; // Fixed
    else if ( pipe->par3 == 2 ) { CONSTRAINT_ADD(pk->packet[3],pipe->par4,127); }
    else if ( pipe->par3 == 3 ) { CONSTRAINT_SUB(pk->packet[3],pipe->par4,0); }
  }
  return true;
}

// -----------------------------------------------------------------------------------------------
// | PipeID  |        par1        |        par2        |        par3        |         par4       |
// |---------+--------------------+--------------------+--------------------+--------------------|
// | VLCURV1 | value 1 : 0-7F     | value 2 : 0-7F     | value 3 :  0-7F    |  value 4 : 0-7F    |
// |   0A    |                    |                    |                    |                    |
// -----------------------------------------------------------------------------------------------

// Utility function to compute a line (a,b)-(c,d) segment formula. Not a sysex function.
int8_t MidiTransFn_VeloCurveLine(uint8_t veloIn, float a, float b, float c, float d)
{
   // Does velocity value must be adjusted ?
  if ( veloIn >= a && veloIn <= c  ) return roundf( ( ( d - b ) / ( c - a ) ) * ( veloIn - a ) + b ) ; 

  return -1;
}

boolean MidiTransFn_VeloCurv1_CheckParms(transPipe_t *pipe)
{ 
  return true;
}

boolean MidiTransFn_VeloCurv1(uint8_t portType, midiPacket_t *pkSource, midiPacket_t *pk, transPipe_t *pipe)
{
 
  uint8_t midiStatus = pk->packet[1] & 0xF0;

  // Only notes ON
  if ( midiStatus != midiXparser::noteOnStatus) return true;

  // Do not process null velocity (equivalent to note off)
  if ( pk->packet[3] == 0 ) return true;
 
  //  5 curve segments from 0 to 127
  #define VLCURV1_X1 25.
  #define VLCURV1_X2 50.
  #define VLCURV1_X3 76.
  #define VLCURV1_X4 101. 
  
  // Determine velocity segment and compute the new velociy with a linear equation

  int8_t veloOut = -1;

  if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] , 0.0,0.0,VLCURV1_X1,(float)pipe->par1) )                       >= 0 ) pk->packet[3] = veloOut;
  if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] , VLCURV1_X1,(float)pipe->par1,VLCURV1_X2,(float)pipe->par2) )  >= 0 ) pk->packet[3] = veloOut;
  if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] , VLCURV1_X2,(float)pipe->par2,VLCURV1_X3,(float)pipe->par3) )  >= 0 ) pk->packet[3] = veloOut;
  if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] , VLCURV1_X3,(float)pipe->par3,VLCURV1_X4,(float)pipe->par4) )  >= 0 ) pk->packet[3] = veloOut;
  if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] , VLCURV1_X4,(float)pipe->par4,127.,127.) )                     >= 0 ) pk->packet[3] = veloOut;
  
  return false;
}

// -----------------------------------------------------------------------------------------------
// | PipeID  |        par1        |        par2        |        par3        |         par4       |
// |---------+--------------------+--------------------+--------------------+--------------------|
// | VLCURV2 |  in 1 : 0-7F       | out 1 : 0-7F       | in 2 :  in 1-7F    | out 2 : 0-7F       |
// |   0B    |                    |                    |                    |                    |
// -----------------------------------------------------------------------------------------------
// NB : Apply on the velocity of the original midi packet (before any transformation)

boolean MidiTransFn_VeloCurv2_CheckParms(transPipe_t *pipe)
{
  if (pipe->par3 < pipe->par1  ) return false;
    
  return true;
}

boolean MidiTransFn_VeloCurv2(uint8_t portType, midiPacket_t *pkSource, midiPacket_t *pk, transPipe_t *pipe)
{
 
  int8_t veloOut = -1;

  // PkSource is the unmodified midi packet.  VLCURV2 uses only the original packet velocity to apply changes rather than the current value,
  // because of tranformation chaining that could be incoherent with what want to do with velocity.
  // It is often better that VLCURV2 be the first pipe in the chain for that reason.

  if ( ( veloOut = MidiTransFn_VeloCurveLine(pkSource->packet[3],(float) pipe->par1,(float) pipe->par2,(float) pipe->par3,(float) pipe->par4) ) >= 0 ) pk->packet[3] = veloOut;

  return true;
}

// -----------------------------------------------------------------------------------------------
// | PipeID  |        par1        |        par2        |        par3        |         par4       |
// |---------+--------------------+--------------------+--------------------+--------------------|
// | VLCURV3 | Hard player   : 1  |     00 (unused)    |     00 (unused)    |     00 (unused)    |
// |   0C    | Medium velo.  : 2  |                    |                    |                    |
// |         | Comp.Expander : 3  |                    |                    |                    |
// |         | Low velo. 1   : 4  |                    |                    |                    |
// |         | Low velo. 2   : 5  |                    |                    |                    |
// |         | Ends cut      : 6  |                    |                    |                    |
// -----------------------------------------------------------------------------------------------

boolean MidiTransFn_VeloCurv3_CheckParms(transPipe_t *pipe)
{ 

  if (pipe->par1 < 1 || pipe->par1 > 6) return false;
  return true;

}

boolean MidiTransFn_VeloCurv3(uint8_t portType, midiPacket_t *pkSource, midiPacket_t *pk, transPipe_t *pipe)
{
 
  uint8_t midiStatus = pk->packet[1] & 0xF0;

  // Only notes ON
  if ( midiStatus != midiXparser::noteOnStatus) return true;

  // Do not process null velocity (equivalent to note off)
  if ( pk->packet[3] == 0 ) return true;
 
  // Determine velocity segment and compute the new velociy with a linear equation
  
  int8_t veloOut = -1;
     
  // 1. Hard player
  if ( pipe->par1 == 1 ) {
      
      if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3],0.,0.,32.,16.) )          >= 0 ) pk->packet[3] =  veloOut;
      else if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3],32.,16.,64.,32.) )   >= 0 ) pk->packet[3] =  veloOut;
      else if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3],64.,32.,95.,64.) )   >= 0 ) pk->packet[3] =  veloOut;
      else if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3],95.,64.,127.,127.) ) >= 0 ) pk->packet[3] =  veloOut;

  } 
  else

  // 2. Medium velocity
  if ( pipe->par1 == 2 ) {
      
      if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] ,0.,0.,16.,48.) )          >= 0 ) pk->packet[3] =  veloOut;
      else if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] ,16.,48.,95.,64.) )   >= 0 ) pk->packet[3] =  veloOut;
      else if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] ,95.,64.,127.,127.) ) >= 0 ) pk->packet[3] =  veloOut;

  } 
  else

  // 3. Compression / Expander
  if ( pipe->par1 == 3 ) {

      if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] ,0.,13.,13.,29.) )          >= 0 ) pk->packet[3] =  veloOut;
      else if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] ,13.,29.,105.,37.) )   >= 0 ) pk->packet[3] =  veloOut;
      else if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] ,105.,37.,117.,70.) )  >= 0 ) pk->packet[3] =  veloOut;
      else if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] ,117.,70.,127.,127.) ) >= 0 ) pk->packet[3] =  veloOut;

  } 
  else

  // Low velocity 1
  if ( pipe->par1 == 4 ) {

      if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] ,0.,0.,8.,8.) )              >= 0 ) pk->packet[3] =  veloOut;
      else if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] ,8.,8.,32.,8.) )        >= 0 ) pk->packet[3] =  veloOut;
      else if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] ,32.,8.,64.,24.) )      >= 0 ) pk->packet[3] =  veloOut;
      else if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] ,64.,24.,84.,43.) )     >= 0 ) pk->packet[3] =  veloOut;
      else if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] ,84.,43.,111.,79.) )    >= 0 ) pk->packet[3] =  veloOut;
      else if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] ,111.,79.,117.,121.) )  >= 0 ) pk->packet[3] =  veloOut;
      else if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] ,117.,121.,127.,127.) ) >= 0 ) pk->packet[3] =  veloOut;

  } 
  else

  // Low velocity 2
  if ( pipe->par1 == 5 ) {

      if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] ,0.,0.,8.,8.) )              >= 0 )  pk->packet[3] =  veloOut ;
      else if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] ,8.,8.,48.,8.) )        >= 0 )  pk->packet[3] =  veloOut ;
      else if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] ,48.,8.,68.,19.) )      >= 0 )  pk->packet[3] =  veloOut ;
      else if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] ,68.,19.,92.,40.) )     >= 0 )  pk->packet[3] =  veloOut ;
      else if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] ,92.,40.,111.,79.) )    >= 0 )  pk->packet[3] =  veloOut ;
      else if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] ,111.,79.,117.,121.) )  >= 0 )  pk->packet[3] =  veloOut ;
      else if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] ,117.,121.,127.,127.) ) >= 0 )  pk->packet[3] =  veloOut ;

  }
  else

  // Ends cut
  if ( pipe->par1 == 6 ) {

      if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] ,0.,0.,6.,24.) )             >= 0 )  pk->packet[3] =  veloOut ;
      else if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] ,6.,24.,56.,35.) )      >= 0 )  pk->packet[3] =  veloOut ;
      else if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] ,56.,35.,95.,57.) )     >= 0 )  pk->packet[3] =  veloOut ;
      else if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] ,95.,57.,119.,86.) )    >= 0 )  pk->packet[3] =  veloOut ;
      else if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] ,119.,86.,122.,121.) )  >= 0 )  pk->packet[3] =  veloOut ;
      else if ( ( veloOut = MidiTransFn_VeloCurveLine(pk->packet[3] ,122.,121.,127.,127.) ) >= 0 )  pk->packet[3] =  veloOut ;

  }

  return false ; // Error
 
}
