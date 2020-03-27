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
//  MIDI TRANSFORMATION FUNCTIONS
//-----------------------------------------------------------------------------
// Due to the unusual make process of Arduino platform, this file is directly
// included in the main ino one.
//
// The midi transformation engine relies on a pipeline process. A "pipe" is
// an elementary midi tranformation function accepting a midi packet and some
// parameters. So a pipeline, is a chain of transformation functions, delivering
// a packet (or many) from a source port to a destination port.
//
// SRC PORT (midi packet) -> | fn1 | fn3 | fn4 -> (modified packet) DEST PORT
//
// To reduce memory consumption, and facilitate resuse of pipelines, thery are
// stored in a "slot".  When a port requires a transformation pipeline,
// you simply attach the slot to it.
//
// Number of pipes in a pipelie is defined by MIDI_TRANS_PIPELINE_SIZE
// and number of pipelines slots is defined by MIDI_TRANS_PIPELINE_SLOT_SIZE
///////////////////////////////////////////////////////////////////////////////
#pragma once

///////////////////////////////////////////////////////////////////////////////
//  FUNCTIONS PROTOTYPES
///////////////////////////////////////////////////////////////////////////////

void TransPacketPipeline_Clear(midiTransPipeline_t *);
boolean TransPacketPipeline_ClearSlot(uint8_t);
boolean TransPacketPipeline_CopySlot(uint8_t ,uint8_t ) ;
uint8_t TransPacketPipeline_FindAttachedSlot(uint8_t , uint8_t );
boolean TransPacketPipeline_AttachPort(boolean,uint8_t,uint8_t,uint8_t );

void TransPacketPipe_Clear(midiTransPipe_t *);

boolean TransPacketPipe_AddToSlot(uint8_t , midiTransPipe_t *);
boolean TransPacketPipe_InsertToSlot(uint8_t , uint8_t , midiTransPipe_t *);
boolean TransPacketPipe_ClearSlotIndexPid(uint8_t , boolean isIndex,uint8_t);
boolean TransPacketPipe_ByPass(uint8_t , uint8_t ,uint8_t);

boolean TransPacketPipelineExec(uint8_t, uint8_t,  midiPacket_t *);

void ShowPipelineSlot(uint8_t );

///////////////////////////////////////////////////////////////////////////////
// Include PIPES.
// Do not change location of the include file.
///////////////////////////////////////////////////////////////////////////////

#include "mod_miditransfn_pipes.h"

///////////////////////////////////////////////////////////////////////////////
// Reset a tranformation pipeline
///////////////////////////////////////////////////////////////////////////////
void TransPacketPipeline_Clear(midiTransPipeline_t *pl) {

  midiTransPipe_t *pipe = pl->pipeline;

  for (uint8_t p=0 ; p != MIDI_TRANS_PIPELINE_SIZE ; p++ )
      TransPacketPipe_Clear( pipe++ );

  pl->attachedCablesMsk = 0;
  pl->attachedJacksMsk = 0 ;
  pl->attachedIthruJacksMsk = 0 ;
}

///////////////////////////////////////////////////////////////////////////////
// Clear a pipeline slot. 0x7F = ALL
///////////////////////////////////////////////////////////////////////////////
boolean TransPacketPipeline_ClearSlot(uint8_t pipelineSlot) {

  // clear all
  if ( pipelineSlot == 0x7F ) {

     for (uint8_t s=0 ; s != MIDI_TRANS_PIPELINE_SLOT_SIZE ; s++ )
      TransPacketPipeline_Clear(&EEPROM_Params.midiTransPipelineSlots[s]);
     return true;
  } else

  if ( pipelineSlot < 1 || pipelineSlot > MIDI_TRANS_PIPELINE_SLOT_SIZE )
      return false;

  // Clear one
  TransPacketPipeline_Clear(&EEPROM_Params.midiTransPipelineSlots[--pipelineSlot]);
  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Copy source slot to destination slot
///////////////////////////////////////////////////////////////////////////////
boolean TransPacketPipeline_CopySlot(uint8_t sourceSlot,uint8_t destSlot) {

  if ( sourceSlot < 1 || sourceSlot > MIDI_TRANS_PIPELINE_SLOT_SIZE )
      return false;

  if ( destSlot < 1 || destSlot > MIDI_TRANS_PIPELINE_SLOT_SIZE )
      return false;

  if ( sourceSlot == destSlot ) return false;

  sourceSlot--; destSlot--;
  memcpy(&EEPROM_Params.midiTransPipelineSlots[destSlot],
          &EEPROM_Params.midiTransPipelineSlots[sourceSlot],sizeof(midiTransPipeline_t));

  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Find the slot of an attahced port
///////////////////////////////////////////////////////////////////////////////
uint8_t TransPacketPipeline_FindAttachedSlot(uint8_t portType, uint8_t port) {

  uint16_t attachedMsk;
  port = (1 << port ); // Make port a bit mask

  for (uint8_t i=0 ; i != MIDI_TRANS_PIPELINE_SLOT_SIZE ; i++ ) {

    if ( portType == USBCABLE_RULE) attachedMsk = EEPROM_Params.midiTransPipelineSlots[i].attachedCablesMsk;
    else if ( portType == SERIAL_RULE) attachedMsk = EEPROM_Params.midiTransPipelineSlots[i].attachedJacksMsk;
    else if ( portType == INTELLITHRU_RULE) attachedMsk = EEPROM_Params.midiTransPipelineSlots[i].attachedIthruJacksMsk;
    else return 0;

    if ( attachedMsk & port ) return i+1;
  }
  return 0;
}

///////////////////////////////////////////////////////////////////////////////
// Detach/Attach a port from/to a pipeline slot
// Only one slot attached to a given port
// SourcetYpe = 0 : USB / 1: JACK / 2 : ITHRU
///////////////////////////////////////////////////////////////////////////////
boolean TransPacketPipeline_AttachPort(boolean attach,uint8_t pipelineSlot,uint8_t portType,uint8_t port ) {

  if ( pipelineSlot < 1 || pipelineSlot > MIDI_TRANS_PIPELINE_SLOT_SIZE )
      return false;

  if ( port > 0x0F ) return false;

  pipelineSlot--;

  uint16_t *attachedMsk;
  port = (1 << port ); // Make port a bit mask

  for (uint8_t s = 0 ; s < MIDI_TRANS_PIPELINE_SLOT_SIZE ; s++ ) {

    if ( portType == USBCABLE_RULE ) attachedMsk = &EEPROM_Params.midiTransPipelineSlots[s].attachedCablesMsk;
    else if ( portType ==  SERIAL_RULE) attachedMsk = &EEPROM_Params.midiTransPipelineSlots[s].attachedJacksMsk;
    else if ( portType == INTELLITHRU_RULE ) attachedMsk = &EEPROM_Params.midiTransPipelineSlots[s].attachedIthruJacksMsk;
    else return false;

    *(attachedMsk) &= ~port; // Clear bit

    if ( s == pipelineSlot ) *(attachedMsk) |= port; // Set bit

  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Reset a tranformation pipe
///////////////////////////////////////////////////////////////////////////////
void TransPacketPipe_Clear(midiTransPipe_t *p) {

  p->pId   = FN_TRANSPIPE_NOPIPE;
  p->byPass = p->par1 = p->par2 = p->par3 = p->par4 = 0;

}

///////////////////////////////////////////////////////////////////////////////
// Add a pipe to a transformation pipeline slot
// Return true if pipe added succefully
///////////////////////////////////////////////////////////////////////////////
boolean TransPacketPipe_AddToSlot(uint8_t pipelineSlot, midiTransPipe_t *pipe) {

  // The slot must exists here
  if (pipelineSlot < 1 || pipelineSlot > MIDI_TRANS_PIPELINE_SLOT_SIZE )
    return false;

  pipelineSlot--; // Adjust for C array

  // Check the pipe parameters
  if ( pipe->pId >= FN_TRANSPIPE_VECTOR_SIZE ) return false;

  if ( !MidiTransFnVector[pipe->pId].checkFn(pipe) ) return false;

  // Get a pointer of the 1st pipe in the attached pipeline
  midiTransPipe_t *pipeLine = EEPROM_Params.midiTransPipelineSlots[pipelineSlot].pipeline ;

  // Find a location for the pipe in the pipeline.
  for (uint8_t i=0; i != MIDI_TRANS_PIPELINE_SIZE ; i++) {
      if ( pipeLine->pId == FN_TRANSPIPE_NOPIPE ) {
        // Copy the pipe at this free location
        *pipeLine = *pipe;
        return true;
      }
      pipeLine++;
  }
  return false;
}

///////////////////////////////////////////////////////////////////////////////
// Insert a pipe to a transformation pipeline slot at a specific index
// Return true if pipe added succefully
///////////////////////////////////////////////////////////////////////////////
boolean TransPacketPipe_InsertToSlot(uint8_t pipelineSlot, uint8_t index, midiTransPipe_t *pipe) {

  // The slot must exists here
  if (pipelineSlot < 1 || pipelineSlot > MIDI_TRANS_PIPELINE_SLOT_SIZE ) return false;

  if ( index >= MIDI_TRANS_PIPELINE_SIZE) return false;

  pipelineSlot--; // Adjust for C array

  // Get a pointer of the 1st/last pipes in the attached pipeline
  midiTransPipe_t *pipeLine = EEPROM_Params.midiTransPipelineSlots[pipelineSlot].pipeline ;
  midiTransPipe_t *pipeLine2 = &EEPROM_Params.midiTransPipelineSlots[pipelineSlot].pipeline[MIDI_TRANS_PIPELINE_SIZE-1] ;

  // Slot full ?
  if ( pipeLine2->pId != FN_TRANSPIPE_NOPIPE ) return false;

  // Check the pipe parameters
  if ( pipe->pId >= FN_TRANSPIPE_VECTOR_SIZE ) return false;
  if ( !MidiTransFnVector[pipe->pId].checkFn(pipe) ) return false;

  // Find the location where to insert the pipe in the pipeline.
  for (uint8_t i=0; i != MIDI_TRANS_PIPELINE_SIZE ; i++) {
      if ( pipeLine->pId == FN_TRANSPIPE_NOPIPE && i < index ) return false;
      if ( i == index) {
        // Move pipes down from the next index.
        while ( pipeLine2-- > pipeLine ) *(pipeLine2+1) = *pipeLine2;
        // Copy the pipe at this now free location
        *pipeLine = *pipe;
        return true;
      }
      pipeLine++;
  }
  return false;
}

///////////////////////////////////////////////////////////////////////////////
// Clear a pipe in a transformation pipeline slot at a specific index
// and compress pipes to remove "holes".
// Return true if pipe cleared succefully.
///////////////////////////////////////////////////////////////////////////////

boolean TransPacketPipe_ClearSlotIndexPid(uint8_t pipelineSlot, boolean isIndex,uint8_t value) {
  // The slot must exists here
  if (pipelineSlot < 1 || pipelineSlot > MIDI_TRANS_PIPELINE_SLOT_SIZE ) return false;

  if ( isIndex && value >= MIDI_TRANS_PIPELINE_SIZE) return false;
  else if ( !isIndex && value  >= FN_TRANSPIPE_VECTOR_SIZE) return false;

  pipelineSlot--; // Adjust for C array

  // Get a pointer of the 1st/last pipes in the attached pipeline
  midiTransPipe_t *pipeLine = EEPROM_Params.midiTransPipelineSlots[pipelineSlot].pipeline ;
  midiTransPipe_t *pipeLine2 = &EEPROM_Params.midiTransPipelineSlots[pipelineSlot].pipeline[MIDI_TRANS_PIPELINE_SIZE-1] ;

  // Find the index/pId for the pipe in the pipeline.
  for (uint8_t i=0; i != MIDI_TRANS_PIPELINE_SIZE ; i++) {

      // Nothing to do if pipe is empty
      if ( pipeLine->pId == FN_TRANSPIPE_NOPIPE ) return false;
      if ( (isIndex && i == value) || (!isIndex && pipeLine->pId == value) ) {
        // // move other pipes up at the current index
        while ( pipeLine < pipeLine2) {
          *pipeLine = *(pipeLine+1);
          pipeLine++;
        }
        // Clear the last pipe
        TransPacketPipe_Clear(pipeLine2);

        return true;
      }
      pipeLine++;
  }
  return false;
}

///////////////////////////////////////////////////////////////////////////////
// ByPass/enable a pipe in a transformation pipeline slot with at a specific index
// Return true if pipe bypassed succefully.
///////////////////////////////////////////////////////////////////////////////
boolean TransPacketPipe_ByPass(uint8_t pipelineSlot, uint8_t index,uint8_t byPass) {
  // The slot must exists here
  if (pipelineSlot < 1 || pipelineSlot > MIDI_TRANS_PIPELINE_SLOT_SIZE ) return false;

  if ( index >= MIDI_TRANS_PIPELINE_SIZE) return false;
  if (byPass >1) return false;

  pipelineSlot--; // Adjust for C array

  // Get a pointer of the 1st pipe in the attached pipeline
  midiTransPipe_t *pipeLine = EEPROM_Params.midiTransPipelineSlots[pipelineSlot].pipeline ;

  // Find the index for the pipe in the pipeline.
  for (uint8_t i=0; i != MIDI_TRANS_PIPELINE_SIZE ; i++) {
    // Nothing to do if pipe is empty
    if ( pipeLine->pId == FN_TRANSPIPE_NOPIPE ) return false;
    if ( i == index ) {
      pipeLine->byPass = byPass;
      return true;
    }
    pipeLine++;
  }
  return false;
}


///////////////////////////////////////////////////////////////////////////////
// Packet transformer pipeline - Exec
//-----------------------------------------------------------------------------
// Modify the packet according pipeline slot affected to the source port
// return true if the result must be routed, false if the packet is finally dropped
///////////////////////////////////////////////////////////////////////////////
boolean TransPacketPipelineExec(uint8_t source, uint8_t pipelineSlot,  midiPacket_t *pk)
{
  // Nb : First slot is 1 but 0 in the C array
  // Ignore if slot is out of bounds

  if (pipelineSlot < 1 || pipelineSlot > MIDI_TRANS_PIPELINE_SLOT_SIZE ) return true;
  pipelineSlot--;

  // Get a pointer of the 1st pipe in the attached pipeline
  midiTransPipe_t *pipeLine = EEPROM_Params.midiTransPipelineSlots[pipelineSlot].pipeline ;

  // Apply transformation function pipes
  for (uint8_t i=0; i != MIDI_TRANS_PIPELINE_SIZE ; i++) {
      // Apply active pipes only
      if ( pipeLine->pId == FN_TRANSPIPE_NOPIPE ) break;
      if ( pipeLine->byPass ) continue; // ByPass
      if (! MidiTransFnVector[pipeLine->pId].pipeFn(source, pk , pipeLine++) )
          return false;
  }

  return true;
}
//////////////////////////////////////////////////////////////////////////////
// Show the content of a pipeline
///////////////////////////////////////////////////////////////////////////////
void ShowPipelineSlot(uint8_t s) {

  if (s < 1 || s > MIDI_TRANS_PIPELINE_SLOT_SIZE ) return ;

/*
  PIPELINE SLOT 1

  |Idx| Pipe id    ( p1, p2, p3, p4 ) |Bypass|
  | 0 | 01 NOTECHG ( 01, 00, 00, 00 ) |   X  |
  | 0 | 01 NOTECHG ( 01, 00, 00, 00 ) |   X  |
  | 0 | 01 NOTECHG ( 01, 00, 00, 00 ) |   X  |
  | 0 | 01 NOTECHG ( 01, 00, 00, 00 ) |   X  |
  | 0 | 01 NOTECHG ( 01, 00, 00, 00 ) |   X  |
  | Attached ports        |          1111111 |
  |                       | 1234567890123456 |
  | Cables                | X............... |
  | Jacks                 | .X.............. |
  | Jacks Ithru           | ..X............. |

*/

  // Get a pointer of the 1st pipe in the attached pipeline
  midiTransPipe_t *pipeLine = EEPROM_Params.midiTransPipelineSlots[--s].pipeline ;


  Serial.println();
  Serial.print("PIPELINE SLOT ");Serial.print(s+1);Serial.print(" :");

  if ( pipeLine->pId == FN_TRANSPIPE_NOPIPE ) {
      Serial.println(" EMPTY");
      Serial.println();
      return;
  }

  Serial.println();Serial.println();
  Serial.println("|Idx| Pipe id    ( p1, p2, p3, p4 ) |Bypass|");

  for (uint8_t i=0; i != MIDI_TRANS_PIPELINE_SIZE ; i++) {
      Serial.print("| ");Serial.print(i);
      Serial.print(" | ");
      if ( pipeLine->pId <= 0x0F ) Serial.print( "0");
      Serial.print(pipeLine->pId,HEX);
      Serial.print(" ");
      Serial.print(MidiTransFnVector[i].shortName);
      Serial.print(" ( ");
      if ( pipeLine->par1 <= 0x0F ) Serial.print( "0");
      Serial.print(pipeLine->par1,HEX);Serial.print(", ");
      if ( pipeLine->par2 <= 0x0F ) Serial.print( "0");
      Serial.print(pipeLine->par2,HEX);Serial.print(", ");
      if ( pipeLine->par3 <= 0x0F ) Serial.print( "0");
      Serial.print(pipeLine->par3,HEX);Serial.print(", ");
      if ( pipeLine->par4 <= 0x0F ) Serial.print( "0");
      Serial.print(pipeLine->par4,HEX);Serial.print(" ) |   ");
      Serial.print( pipeLine->byPass ? "X":" ");
      Serial.println("  |");
      pipeLine++;
      if ( pipeLine->pId == FN_TRANSPIPE_NOPIPE ) break;
  }
  Serial.println();
  Serial.println("| Attached ports        |          1111111 |");
  Serial.println("|                       | 1234567890123456 |");
    Serial.print("| Cables                | ");
  for (uint8_t j=0; j < 16 ; j++) {
    Serial.print ( EEPROM_Params.midiTransPipelineSlots[s].attachedCablesMsk & (1<<j) ?"X":".");
  }
  Serial.println(" |");
  Serial.print("| Jacks                 | ");
  for (uint8_t j=0; j < 16 ; j++) {
    Serial.print ( EEPROM_Params.midiTransPipelineSlots[s].attachedJacksMsk & (1<<j) ?"X":".");
  }
  Serial.println(" |");
  Serial.print("| Jacks Ithru           | ");
  for (uint8_t j=0; j < 16 ; j++) {
    Serial.print ( EEPROM_Params.midiTransPipelineSlots[s].attachedIthruJacksMsk & (1<<j) ?"X":".");
  }
  Serial.println(" |");

}
