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
// Number of pipes in a pipelie is defined by TRANS_PIPELINE_SIZE
// and number of pipelines slots is defined by TRANS_PIPELINE_SLOT_SIZE
///////////////////////////////////////////////////////////////////////////////
#pragma once

///////////////////////////////////////////////////////////////////////////////
//  FUNCTIONS PROTOTYPES
///////////////////////////////////////////////////////////////////////////////

void    TransPacketPipeline_Clear(transPipeline_t *);
boolean TransPacketPipeline_ClearSlot(uint8_t);
//Shared. See usbmidiKlik4x4.h
//boolean TransPacketPipeline_CopySlot(uint8_t ,uint8_t ) ;
//boolean TransPacketPipeline_AttachPort(uint8_t ,uint8_t ,uint8_t );
void    TransPacketPipe_Clear(transPipe_t *);
//Shared. See usbmidiKlik4x4.h
//boolean TransPacketPipe_AddToSlot(uint8_t , transPipe_t *);
//boolean TransPacketPipe_InsertToSlot(uint8_t , uint8_t , transPipe_t *,boolean);
//boolean TransPacketPipe_ClearSlotIndexPid(uint8_t , boolean ,uint8_t);
//boolean TransPacketPipe_ByPass(uint8_t , uint8_t ,uint8_t);
boolean TransPacketPipelineExec(uint8_t, uint8_t,  midiPacket_t *);
//Shared. See usbmidiKlik4x4.h
//void    ShowPipelineSlot(uint8_t );

// SLOT LOCK to avaid infinite loop and filter internal sysex
// Eight Slot are possible currently. Change to uint16_t if more.
#if TRANS_PIPELINE_SLOT_SIZE > 8
  uint16_t slotLockMsk = 0;
#else
  uint8_t slotLockMsk = 0;
#endif
///////////////////////////////////////////////////////////////////////////////
// PIPES.
///////////////////////////////////////////////////////////////////////////////

#include "mod_miditransfn_pipes.h"

///////////////////////////////////////////////////////////////////////////////
// Reset a tranformation pipeline
///////////////////////////////////////////////////////////////////////////////
void TransPacketPipeline_Clear(transPipeline_t *pl) {

  transPipe_t *pipe = pl->pipeline;

  for (uint8_t p=0 ; p != TRANS_PIPELINE_SIZE ; p++ )
      TransPacketPipe_Clear( pipe++ );
}

///////////////////////////////////////////////////////////////////////////////
// Clear a pipeline slot. 0x7F = ALL
///////////////////////////////////////////////////////////////////////////////
boolean TransPacketPipeline_ClearSlot(uint8_t pipelineSlot) {

  // clear all
  if ( pipelineSlot == 0x7F ) {

     for (uint8_t s=0 ; s != TRANS_PIPELINE_SLOT_SIZE ; s++ )
      TransPacketPipeline_Clear(&EE_Prm.pipelineSlot[s]);
     return true;
  } else

  if ( pipelineSlot < 1 || pipelineSlot > TRANS_PIPELINE_SLOT_SIZE )
      return false;

  // Clear one
  TransPacketPipeline_Clear(&EE_Prm.pipelineSlot[--pipelineSlot]);
  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Copy source slot to destination slot
///////////////////////////////////////////////////////////////////////////////
boolean TransPacketPipeline_CopySlot(uint8_t sourceSlot,uint8_t destSlot) {

  if ( sourceSlot < 1 || sourceSlot > TRANS_PIPELINE_SLOT_SIZE )
      return false;

  if ( destSlot < 1 || destSlot > TRANS_PIPELINE_SLOT_SIZE )
      return false;

  if ( sourceSlot == destSlot ) return false;

  sourceSlot--; destSlot--;
  memcpy(&EE_Prm.pipelineSlot[destSlot],
          &EE_Prm.pipelineSlot[sourceSlot],sizeof(transPipeline_t));

  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Detach/Attach a port from/to a pipeline slot
// Only one slot attached to a given port.
// To Detach pass a pipeline == 0.
// If portType = 0x7F , ALL ports are detached.
///////////////////////////////////////////////////////////////////////////////
boolean TransPacketPipeline_AttachPort(uint8_t portType,uint8_t port,uint8_t pipelineSlot )
{
    // Detach ALL
    if (portType == 0x7F) {
        for (uint8_t i = 0; i != USBCABLE_INTERFACE_MAX ; i++)
              EE_Prm.rtRulesCable[i].slot=0;
        for (uint8_t i = 0; i != SERIAL_INTERFACE_COUNT ; i++) {
              EE_Prm.rtRulesJack[i].slot = 0;
              EE_Prm.rtRulesIthru[i].slot=0;
        }
        for (uint8_t i = 0; i != VIRTUAL_INTERFACE_MAX ; i++) {
              EE_Prm.rtRulesVirtual[i].slot = 0;
        }
        return true;
    }

    if ( pipelineSlot > TRANS_PIPELINE_SLOT_SIZE )    return false;

  	if (portType == PORT_TYPE_CABLE ) {
      if ( port < USBCABLE_INTERFACE_MAX )
        EE_Prm.rtRulesCable[port].slot = pipelineSlot ;
      else return false;
    }
    else
    if (portType == PORT_TYPE_JACK ) {
      if ( port < SERIAL_INTERFACE_COUNT )
        EE_Prm.rtRulesJack[port].slot = pipelineSlot ;
      else return false;
    }
    else
    if (portType == PORT_TYPE_VIRTUAL ) {
      if ( port < VIRTUAL_INTERFACE_MAX )
        EE_Prm.rtRulesVirtual[port].slot = pipelineSlot ;
      else return false;
    }
    else
    if (portType == PORT_TYPE_ITHRU ) {
      if ( port < SERIAL_INTERFACE_COUNT )
        EE_Prm.rtRulesIthru[port].slot = pipelineSlot ;
      else return false;
    }
    else return false;

    return true;
}

///////////////////////////////////////////////////////////////////////////////
// Reset a tranformation pipe
///////////////////////////////////////////////////////////////////////////////
void TransPacketPipe_Clear(transPipe_t *p) {

  p->pId   = FN_TRANSPIPE_NOPIPE;
  p->byPass = p->par1 = p->par2 = p->par3 = p->par4 = 0;
}

///////////////////////////////////////////////////////////////////////////////
// Add a pipe to a transformation pipeline slot
// Return true if pipe added succefully
///////////////////////////////////////////////////////////////////////////////
boolean TransPacketPipe_AddToSlot(uint8_t pipelineSlot, transPipe_t *pipe) {

  // The slot must exists here
  if (pipelineSlot < 1 || pipelineSlot > TRANS_PIPELINE_SLOT_SIZE )
    return false;

  pipelineSlot--; // Adjust for C array

  // Check the pipe parameters
  if ( pipe->pId >= FN_TRANSPIPE_VECTOR_SIZE ) return false;

  if ( !MidiTransFnVector[pipe->pId].checkFn(pipe) ) return false;

  // Get a pointer of the 1st pipe in the attached pipeline
  transPipe_t *pipeline = EE_Prm.pipelineSlot[pipelineSlot].pipeline ;

  // Find a location for the pipe in the pipeline.
  for (uint8_t i=0; i != TRANS_PIPELINE_SIZE ; i++) {
      if ( pipeline->pId == FN_TRANSPIPE_NOPIPE ) {
        // Copy the pipe at this free location
        *pipeline = *pipe;
        return true;
      }
      pipeline++;
  }
  return false;
}

///////////////////////////////////////////////////////////////////////////////
// Insert a pipe to a transformation pipeline slot at a specific index
// Return true if pipe added succefully
///////////////////////////////////////////////////////////////////////////////
boolean TransPacketPipe_InsertToSlot(uint8_t pipelineSlot, uint8_t index, transPipe_t *pipe, boolean replace) {

  // The slot must exists here
  if (pipelineSlot < 1 || pipelineSlot > TRANS_PIPELINE_SLOT_SIZE ) return false;

  if ( index >= TRANS_PIPELINE_SIZE) return false;

  pipelineSlot--; // Adjust for C array

  // Get a pointer of the 1st/last pipes in the attached pipeline
  transPipe_t *pipeline  = &EE_Prm.pipelineSlot[pipelineSlot].pipeline[0] ;
  transPipe_t *pipeline2 = &EE_Prm.pipelineSlot[pipelineSlot].pipeline[TRANS_PIPELINE_SIZE-1] ;

  // Slot full ? can't insert .
  if ( !replace && pipeline2->pId != FN_TRANSPIPE_NOPIPE ) return false;

  // Check the pipe parameters
  if ( pipe->pId >= FN_TRANSPIPE_VECTOR_SIZE ) return false;
  if ( !MidiTransFnVector[pipe->pId].checkFn(pipe) ) return false;

  // Find the location where to insert the pipe in the pipeline.
  for (uint8_t i=0; i != TRANS_PIPELINE_SIZE ; i++) {
      if ( pipeline->pId == FN_TRANSPIPE_NOPIPE && i < index ) return false;
      if ( i == index) {
        if (! replace) {
          // Move pipes down from the next index.
          while ( pipeline2-- > pipeline ) *(pipeline2+1) = *pipeline2;
          // Copy the pipe at this now free location
        }
        *pipeline = *pipe;
        return true;
      }
      pipeline++;
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
  if (pipelineSlot < 1 || pipelineSlot > TRANS_PIPELINE_SLOT_SIZE ) return false;

  if ( isIndex && value >= TRANS_PIPELINE_SIZE) return false;
  else if ( !isIndex && value  >= FN_TRANSPIPE_VECTOR_SIZE) return false;

  pipelineSlot--; // Adjust for C array

  // Get a pointer of the 1st/last pipes in the attached pipeline
  transPipe_t *pipeline = EE_Prm.pipelineSlot[pipelineSlot].pipeline ;
  transPipe_t *pipeline2 = &EE_Prm.pipelineSlot[pipelineSlot].pipeline[TRANS_PIPELINE_SIZE-1] ;

  // Find the index/pId for the pipe in the pipeline.
  for (uint8_t i=0; i != TRANS_PIPELINE_SIZE ; i++) {

      // Nothing to do if pipe is empty
      if ( pipeline->pId == FN_TRANSPIPE_NOPIPE ) return false;
      if ( (isIndex && i == value) || (!isIndex && pipeline->pId == value) ) {
        // // move other pipes up at the current index
        while ( pipeline < pipeline2) {
          *pipeline = *(pipeline+1);
          pipeline++;
        }
        // Clear the last pipe
        TransPacketPipe_Clear(pipeline2);

        return true;
      }
      pipeline++;
  }
  return false;
}

///////////////////////////////////////////////////////////////////////////////
// ByPass/enable a pipe in a transformation pipeline slot with at a specific index
// Return true if pipe bypassed succefully.
///////////////////////////////////////////////////////////////////////////////
boolean TransPacketPipe_ByPass(uint8_t pipelineSlot, uint8_t index,uint8_t byPass) {
  // The slot must exists here
  if (pipelineSlot < 1 || pipelineSlot > TRANS_PIPELINE_SLOT_SIZE ) return false;

  if ( index >= TRANS_PIPELINE_SIZE) return false;
  if (byPass >1) return false;

  pipelineSlot--; // Adjust for C array

  // Get a pointer of the 1st pipe in the attached pipeline
  transPipe_t *pipeline = EE_Prm.pipelineSlot[pipelineSlot].pipeline ;

  // Find the index for the pipe in the pipeline.
  for (uint8_t i=0; i != TRANS_PIPELINE_SIZE ; i++) {
    // Nothing to do if pipe is empty
    if ( pipeline->pId == FN_TRANSPIPE_NOPIPE ) return false;
    if ( i == index ) {
      pipeline->byPass = byPass;
      return true;
    }
    pipeline++;
  }
  return false;
}


///////////////////////////////////////////////////////////////////////////////
// Packet transformer pipeline - Exec
//-----------------------------------------------------------------------------
// Modify the packet according pipeline slot affected to the source port
// return true if the result must be routed, false if the packet is finally dropped
///////////////////////////////////////////////////////////////////////////////
boolean TransPacketPipelineExec(uint8_t source, uint8_t slot ,  midiPacket_t *pk) {



  if ( slot < 1 || slot > TRANS_PIPELINE_SLOT_SIZE) return false;

  // Check if the pipeline slot is already running, and block it to avoid infinite loop
  if ( slotLockMsk & ( 1 << --slot) ) return true; // True will allow port routing

  slotLockMsk |= (1 << slot); // Lock slot

  transPipe_t *pipeline = EE_Prm.pipelineSlot[slot].pipeline;

  boolean r = true ;
  // Apply transformation function pipes
  for (uint8_t i=0; i != TRANS_PIPELINE_SIZE ; i++) {
      // Apply active pipes only
      if ( pipeline->pId == FN_TRANSPIPE_NOPIPE ) break;
      if ( ! pipeline->byPass ) {
         r = MidiTransFnVector[pipeline->pId].pipeFn(source, pk , pipeline);
         if (! r )  break;
      }
      pipeline++;
  }

  slotLockMsk &= ~(1 << slot);  // Unlock slot
  return r;
}
//////////////////////////////////////////////////////////////////////////////
// Show the content of a pipeline
///////////////////////////////////////////////////////////////////////////////
void ShowPipelineSlot(uint8_t s) {

  if (s < 1 || s > TRANS_PIPELINE_SLOT_SIZE ) return ;

  // Get a pointer of the 1st pipe in the attached pipeline
  transPipe_t *pipeline = EE_Prm.pipelineSlot[s-1].pipeline ;

  Serial.println();
  SerialPrintf("%M %M %d :",str_PIPELINE,str_SLOT,s);

  if ( pipeline->pId == FN_TRANSPIPE_NOPIPE ) {
      Serial.println(" EMPTY");
  } else {
    Serial.println();Serial.println();
    SerialPrintf("| Idx | Pipe id    ( p1, p2, p3, p4 ) | %y |",str_BYPASS);

    for (uint8_t i=0; i != TRANS_PIPELINE_SIZE ; i++) {
        SerialPrintf("|  %2d | %02x %s (",i,pipeline->pId,MidiTransFnVector[pipeline->pId].shortName);
        SerialPrintf(" %02x, %02x, %02x, %02x ) |",pipeline->par1,pipeline->par2,pipeline->par3,pipeline->par4);
        SerialPrintf("   %c    |%n",pipeline->byPass ? 'X':' ');
        pipeline++;
        if ( pipeline->pId == FN_TRANSPIPE_NOPIPE ) break;
    }
  }
  Serial.println();
  SerialPrintf("| Attached %s            |          %s |%n",str_PORT,str_71DIGITS);
  SerialPrintf("|                          | %s |%n",str_16DIGITS);
  SerialPrintf("| %y %s                | ",str_CABLE,str_OUT);

  for (uint8_t j=0; j < 16 ; j++) {
    if (j > USBCABLE_INTERFACE_MAX) Serial.print(" ");
    else if ( EE_Prm.rtRulesCable[j].slot == s)
        Serial.print ("X");
    else Serial.print(".");
  }

  Serial.println(" |");
  SerialPrintf("| %y %s                  | ",str_JACK,str_IN);
  for (uint8_t j=0; j < 16 ; j++) {
    if (j > SERIAL_INTERFACE_COUNT) Serial.print(" ");
    else if ( EE_Prm.rtRulesJack[j].slot == s)
        Serial.print ("X");
    else Serial.print(".");
  }

  Serial.println(" |");
  SerialPrintf("| %y %s %y            | ",str_JACK,str_IN,str_ITHRU);
  for (uint8_t j=0; j < 16 ; j++) {
    if (j > SERIAL_INTERFACE_COUNT) Serial.print(" ");
    else if ( EE_Prm.rtRulesIthru[j].slot == s)
        Serial.print ("X");
    else Serial.print(".");
  }

  Serial.println(" |");
  SerialPrintf("| %y %s               | ",str_VIRTUAL,str_IN);
  for (uint8_t j=0; j < 16 ; j++) {
    if (j > VIRTUAL_INTERFACE_MAX) Serial.print(" ");
    else if ( EE_Prm.rtRulesVirtual[j].slot == s)
        Serial.print ("X");
    else Serial.print(".");
  }

  Serial.println(" |");

}
