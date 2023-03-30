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
//  CONFIGURATION USER INTERFACE FUNCTIONS
//-----------------------------------------------------------------------------
// Due to the unusual make process of Arduino platform, this file is directly
// included in the main ino one.
///////////////////////////////////////////////////////////////////////////////
#pragma once

///////////////////////////////////////////////////////////////////////////////
//  FUNCTIONS PROTOTYPES
///////////////////////////////////////////////////////////////////////////////

void I2C_BusSerialSendMidiPacket(midiPacket_t *, uint8_t );
int8_t I2C_ParseDataSync(uint8_t ,uint8_t,uint8_t );
void I2C_ParseImmediateCmd();
void I2C_SlaveReceiveEvent(int);
void I2C_SlaveRequestEvent ();
void __O_SMALL I2C_BusChecks();
void __O_SMALL I2C_BusStartWire();
int16_t I2C_SendCommand(uint8_t,BusCommand);
//Shared. See usbmidiKlik4x4.h
//void __O_SMALL I2C_ShowActiveDevice() ;
int16_t I2C_getPacket(uint8_t , masterMidiPacket_t *);
boolean I2C_isDeviceActive(uint8_t );
int8_t I2C_SendData(uint8_t, uint8_t, uint8_t, uint8_t * , uint16_t );
//Shared. See usbmidiKlik4x4.h
//void I2C_SlavesRoutingSyncFromMaster();
void I2C_ProcessMaster ();
void I2C_ProcessSlave ();

///////////////////////////////////////////////////////////////////////////////
// Send a midi packet to I2C remote MIDI device on BUS
///////////////////////////////////////////////////////////////////////////////
void I2C_BusSerialSendMidiPacket(midiPacket_t *pk, uint8_t targetPort)
{
	if ( EE_Prm.I2C_BusModeState == B_DISABLED) return;

	// Check if it is a local port to avoid bus
	// bus traffic for Nothing
	if ( EE_Prm.I2C_DeviceId == GET_DEVICEID_FROM_SERIALNO(targetPort) ) {
		SerialMidi_SendPacket(pk, targetPort % SERIAL_INTERFACE_MAX );
		return;
	}

	// packet copy to change the local target serial of the source
	midiPacket_t pk2 = { .i = pk->i };
	pk2.packet[0] = ( (targetPort % SERIAL_INTERFACE_MAX) << 4 ) + (pk2.packet[0] & 0x0F);

	// If we are a slave, we can't talk directly to the bus.
	// So, store the "to transmit" packet, waiting for a RequestFrom.
  // Master packet have  a "dest" byte before the midi packet.
  if ( IS_SLAVE ) {
    masterMidiPacket_t mpk;
    mpk.mpk.dest = PORT_TYPE_JACK;
    mpk.mpk.pk.i = pk2.i;// Copy the midi packet &  queue it
		I2C_QPacketsToMaster.write(mpk.packet,sizeof(masterMidiPacket_t));
		return;
	}

	// We are a MASTER !
	// Compute the device ID from the serial port Id
	uint8_t deviceId = GET_DEVICEID_FROM_SERIALNO(targetPort);

	// Send a midi packet to device
	Wire.beginTransmission(deviceId);
	Wire.write(pk2.packet,sizeof(midiPacket_t));
	Wire.endTransmission();
}

///////////////////////////////////////////////////////////////////////////////
// THIS IS INSIDE AN ISR ! - PARSE DATA FROM MASTER TO SYNC ROUTING RULES
//////////////////////////////////////////////////////////////////////////////
int8_t I2C_ParseDataSync(uint8_t dataType,uint8_t arg1,uint8_t arg2)
{
  // midiRoutingRule.
	// arg1 = index
	// arg2 = 0: cable, 1:Jk, 2:virtual,
  if (dataType == B_DTYPE_ROUTING_RULE )
  {
    if (Wire.available() != sizeof(routingRule_t)) return -1;
		routingRule_t *mr ;
		if ( arg2 == PORT_TYPE_CABLE && arg1 < USBCABLE_INTERFACE_MAX) mr = &EE_Prm.rtRulesCable[arg1];
		else if ( arg2 == PORT_TYPE_JACK && arg1 < B_SERIAL_INTERFACE_MAX) mr = &EE_Prm.rtRulesJack[arg1];
		else return -1;

    routingRule_t r;
    Wire.readBytes((uint8_t *)&r,sizeof(routingRule_t));
    if (memcmpcpy((void*)mr,(void*)&r,sizeof(routingRule_t))) I2C_SlaveSyncDoUpdate = true;
  }
  else
	// Alternate midi routing rule
	// arg1 : pipeIndex
	// arg2 : 2 : virtual, 3:Ithru
	if ( dataType == B_DTYPE_ROUTING_RULE_ALT )
  {
    if (Wire.available() != sizeof(routingRuleAlt_t)) return -1;
		routingRuleAlt_t *mr;
		if (arg2 == PORT_TYPE_VIRTUAL && arg1 < VIRTUAL_INTERFACE_MAX ) mr = &EE_Prm.rtRulesVirtual[arg1];
		else if ( arg2 == PORT_TYPE_ITHRU && arg1 < B_SERIAL_INTERFACE_MAX) mr = &EE_Prm.rtRulesIthru[arg1];
    else return -1;

    routingRuleAlt_t r;
    Wire.readBytes((uint8_t *)&r,sizeof(routingRuleAlt_t));
    if (memcmpcpy((void*)mr,(void*)&r,sizeof(routingRuleAlt_t))) I2C_SlaveSyncDoUpdate = true;
  }
  else
  // ithruJackInMsk
  if (dataType == B_DTYPE_ROUTING_ITHRU_JACKIN_MSK) {
    if (Wire.available() != sizeof(uint16_t)) return -1;
    uint16_t jmsk;
    Wire.readBytes((uint8_t *)&jmsk,sizeof(uint16_t));
    if ( EE_Prm.ithruJackInMsk != jmsk) {
        EE_Prm.ithruJackInMsk = jmsk;
        I2C_SlaveSyncDoUpdate = true;
    }
  }
  else
  // ithruUSBIdleTimePeriod
  if (dataType == B_DTYPE_ROUTING_ITHRU_USB_IDLE_TIME_PERIOD) {
    if (Wire.available() != sizeof(uint8_t) ) return -1;
    uint8_t dp = Wire.read();
    if ( EE_Prm.ithruUSBIdleTimePeriod != dp) {
      EE_Prm.ithruUSBIdleTimePeriod = dp;
      I2C_SlaveSyncDoUpdate = true;
    }
  }
	else
	// Bpm clocks (not used when slave but useful to monitor master)
  if (dataType == B_DTYPE_BPM_CLOCK) {
		if (Wire.available() != sizeof(bpmClock_t)) return -1;
		bpmClock_t *mr;
		if (arg1 < MIDI_CLOCKGEN_MAX ) mr = &EE_Prm.bpmClocks[arg1];
    else return -1;

		bpmClock_t r;
    Wire.readBytes((uint8_t *)&r,sizeof(bpmClock_t));
    if (memcmpcpy((void*)mr,(void*)&r,sizeof(bpmClock_t))) I2C_SlaveSyncDoUpdate = true;
	}
	else
	// Pipeline slots . Pipe by pipe
	if (dataType == B_DTYPE_MIDI_TRANSPIPE) {
		if (Wire.available() != sizeof(transPipe_t)) return -1;
		if (arg1 >= TRANS_PIPELINE_SLOT_SIZE)  return -1;
		if (arg2 >= TRANS_PIPELINE_SIZE)  return -1;
		transPipe_t pSrc;
		transPipe_t *pDest = &EE_Prm.pipelineSlot[arg1].pipeline[arg2];
		Wire.readBytes((uint8_t *)&pSrc, sizeof(transPipe_t));
		if (memcmpcpy((void*)pDest,(void*)&pSrc,sizeof(transPipe_t))) I2C_SlaveSyncDoUpdate = true;
	}

  return 0;
}

///////////////////////////////////////////////////////////////////////////////
// THIS IS INSIDE AN ISR ! - PARSE IMMEDIATE Command
//////////////////////////////////////////////////////////////////////////////
//    Manage immediate commands not followed by a requestFrom
void I2C_ParseImmediateCmd()
{

  I2C_Command = Wire.read();

  switch (I2C_Command)
  {
    case B_CMD_USBCX_UNAVAILABLE:
      midiUSBCx = false;
      break;

    case B_CMD_USBCX_AVAILABLE:
      midiUSBCx = true;
      break;

    case B_CMD_USBCX_SLEEP:
      midiUSBIdle = true;
      break;

    case B_CMD_USBCX_AWAKE:
      midiUSBIdle = false;
      break;

    case B_CMD_INTELLITHRU_ENABLED:
      midiIthruActive = true;
      break;

    case B_CMD_INTELLITHRU_DISABLED:
      midiIthruActive = false;
      break;

    case B_CMD_HARDWARE_RESET:
      nvic_sys_reset();
      break;

    case B_CMD_START_SYNC:
      I2C_SlaveSyncStarted = true;
      break;

    case B_CMD_END_SYNC:
      I2C_SlaveSyncStarted = false;
      break;
  }
}

///////////////////////////////////////////////////////////////////////////////
// THIS IS AN ISR ! - I2C Receive event trigger for a SLAVE
//////////////////////////////////////////////////////////////////////////////
void I2C_SlaveReceiveEvent(int howMany)
{
	I2C_MasterIsActive = true;

  // Is it a DATA SYNC ?
  if ( I2C_SlaveSyncStarted ) {

    if (howMany <= 0 ) {
      I2C_SlaveSyncStarted = false; I2C_SlaveSyncDoUpdate = false; // Abort
      return;
    }

    // Command. Save to EEPROM if the END sync is received
    if (howMany == 1 ) {
      I2C_ParseImmediateCmd();
      return;
    }

    // Error. No data.
    if (howMany <=3 ) {
      I2C_SlaveSyncStarted = false; I2C_SlaveSyncDoUpdate = false; // Abort
      return;
    }

    // Correct message format. Parse the data.
    uint8_t dataType = Wire.read();
    uint8_t arg1 = Wire.read();
    uint8_t arg2 = Wire.read();

    if  ( I2C_ParseDataSync(dataType,arg1,arg2) != 0) {
      I2C_SlaveSyncStarted = false; I2C_SlaveSyncDoUpdate = false; // Abort
      return;
    }
  }
  else
  {
    if (howMany <= 0) return;
    // It's a command ?
    // Immediate commands are managed here.
    // Those necessiting a requestFrom in the requestEvent ISR
    if (howMany == 1) {
      I2C_ParseImmediateCmd();
    }
    else // We only store packet here.
  	if (howMany == sizeof(midiPacket_t) ) {
  		midiPacket_t pk;
      // Read the bus and Write a packet in the ring buffer
      Wire.readBytes( (uint8_t*)&pk,sizeof(midiPacket_t) );
  		I2C_QPacketsFromMaster.write((uint8_t *)&pk,sizeof(midiPacket_t) );
  	}
  }
}

///////////////////////////////////////////////////////////////////////////////
// THIS IS AN ISR ! - I2C Request From Master event trigger. SLAVE ONLY..
//////////////////////////////////////////////////////////////////////////////
void I2C_SlaveRequestEvent ()
{
	I2C_MasterIsActive = true;

  switch (I2C_Command) {

    case B_CMD_ISPACKET_AVAIL: {
        uint8_t nb = I2C_QPacketsToMaster.available() / sizeof(masterMidiPacket_t);
        Wire.write(nb);
        break;
    }

    case B_CMD_GET_MPACKET: {
        masterMidiPacket_t mpk;
        if (I2C_QPacketsToMaster.available()) {
          I2C_QPacketsToMaster.readBytes(mpk.packet,sizeof(masterMidiPacket_t));
        }
        else memset(mpk.packet,0,sizeof(masterMidiPacket_t));  // a null packet
        Wire.write(mpk.packet,sizeof(masterMidiPacket_t));
        break;
    }

    case B_CMD_IS_SLAVE_READY:
        Wire.write(B_STATE_READY);
        break;
  }

  I2C_Command = B_CMD_NONE;
}

///////////////////////////////////////////////////////////////////////////////
//  I2C Bus Checking.
//////////////////////////////////////////////////////////////////////////////
void I2C_BusChecks()
{
	if ( EE_Prm.I2C_DeviceId < B_SLAVE_DEVICE_BASE_ADDR &&
			 EE_Prm.I2C_DeviceId > B_SLAVE_DEVICE_LAST_ADDR )

			 EE_Prm.I2C_BusModeState = B_DISABLED; // Overwrite setting
}

///////////////////////////////////////////////////////////////////////////////
//  I2C Bus Start WIRE
//////////////////////////////////////////////////////////////////////////////
void I2C_BusStartWire()
{
	if ( EE_Prm.I2C_BusModeState == B_DISABLED ) return;

	if ( EE_Prm.I2C_DeviceId == B_MASTERID ) {

      // NB : default timemout is 1 sec. Possible to change that with Wire.setTimeout(x);
      // before the begin.
    	Wire.begin();
      Wire.setClock(B_FREQ) ;

			delay(500);
      // Scan BUS for active slave DEVICES. Table used only by the master.
			for ( uint8_t d=0; d != sizeof(I2C_DeviceIdActive) ; d++) {
          if ( I2C_isDeviceActive(d + B_SLAVE_DEVICE_BASE_ADDR) )
							I2C_DeviceIdActive[I2C_DeviceActiveCount++] = d + B_SLAVE_DEVICE_BASE_ADDR;
  		}

			// If no slave active, reboot master in config mode
			if ( ! I2C_DeviceActiveCount ) {
				 SetBootMagicWord(BOOT_CONFIG_MAGIC);
				 Wire.end();
				 delay(10) ; nvic_sys_reset();
			}

			// Check slaves availibility, and reboot if one not ready
			uint8_t d=0;
			while ( d < I2C_DeviceActiveCount ) {
    		if ( I2C_SendCommand(I2C_DeviceIdActive[d],B_CMD_IS_SLAVE_READY) == B_STATE_READY ) d++;
				else { Wire.end(); delay(10) ; nvic_sys_reset(); }
			}

//			// Reset all slaves if they are now listening
//		  I2C_SendCommand(0,B_CMD_HARDWARE_RESET);
//		  delay(3000);

      // Synchronize slaves routing rules
      I2C_SlavesRoutingSyncFromMaster();

	}
		// Slave initialization
	else 	{

      // NB : default timemout is 1 sec. Possible to change that with Wire.setTimeout(x);
      // before the begin.
      Wire.begin(EE_Prm.I2C_DeviceId);
	  	Wire.onRequest(I2C_SlaveRequestEvent);
			Wire.onReceive(I2C_SlaveReceiveEvent);

      // start USB serial
  		Serial.begin(115200);
      Serial.flush();
			delay(500);
	}
}

///////////////////////////////////////////////////////////////////////////////
// SEND an USBMIDIKLIK command on the I2C bus and wait for answer. MASTER ONLY
// If return is < 0 : error, else return the number of bytes received
//////////////////////////////////////////////////////////////////////////////
int16_t I2C_SendCommand(uint8_t deviceId,BusCommand cmd)
{
    Wire.beginTransmission(deviceId);
    Wire.write(cmd);

    if ( Wire.endTransmission() ) return -1;

    if ( BusCommandRequestSize[cmd] > 0) {
        uint8_t nb = Wire.requestFrom(deviceId,BusCommandRequestSize[cmd]);
        if ( nb != BusCommandRequestSize[cmd] ) return -1 ;
        if ( BusCommandRequestSize[cmd] == 1) return Wire.read();
        return nb;
    }

    return 0;
}
///////////////////////////////////////////////////////////////////////////////
//  I2C Show active slave devices on screen
//////////////////////////////////////////////////////////////////////////////
void I2C_ShowActiveDevice()
{
	uint8_t deviceId;

	// Scan BUS for active slave DEVICES. Table used only by the master.
	for ( uint8_t d=0; d != sizeof(I2C_DeviceIdActive) ; d++) {
			deviceId = d + B_SLAVE_DEVICE_BASE_ADDR;
			Wire.beginTransmission(deviceId);
      delay(5);
			if ( Wire.endTransmission() == 0 ) {
					Serial.print("Slave device ")	;Serial.print(deviceId);
					Serial.println(" is active.");
			}
	}

}

///////////////////////////////////////////////////////////////////////////////
// I2C Get a master packet from slave
///////////////////////////////////////////////////////////////////////////////
int16_t I2C_getPacket(uint8_t deviceId, masterMidiPacket_t *mpk)
{
    uint8_t nb = I2C_SendCommand(deviceId,B_CMD_GET_MPACKET);
    if ( nb > 0 ) Wire.readBytes( mpk->packet,sizeof(masterMidiPacket_t)) ;

    return (nb) ;
}

///////////////////////////////////////////////////////////////////////////////
// I2C check if a device is active on the bus
//////////////////////////////////////////////////////////////////////////////
boolean I2C_isDeviceActive(uint8_t deviceId)
{
    Wire.beginTransmission(deviceId);
    return Wire.endTransmission() == 0;
}

///////////////////////////////////////////////////////////////////////////////
// I2C Send bulk data on the bus.
//////////////////////////////////////////////////////////////////////////////
int8_t I2C_SendData(uint8_t dataType, uint8_t arg1, uint8_t arg2, uint8_t * data, uint16_t sz)
{

  if (sz > 29 ) return -1; // Wire buffer is limited to 32 char. Enough for us.

  Wire.beginTransmission(0); // Broadcast
  Wire.write(dataType);
  Wire.write(arg1);
  Wire.write(arg2);
  Wire.write(data,sz);
  if ( Wire.endTransmission()) return -1;
  return 0;
}

///////////////////////////////////////////////////////////////////////////////
// I2C Slaves SYNC routing rules at boot
///////////////////////////////////////////////////////////////////////////////
void I2C_SlavesRoutingSyncFromMaster()
{
  I2C_SendCommand(0,   B_CMD_START_SYNC);

	uint8_t i=0;
  // Send rtRulesCable
  for ( i=0 ; i != USBCABLE_INTERFACE_MAX ; i ++ ) {
    I2C_SendData(B_DTYPE_ROUTING_RULE, i, PORT_TYPE_CABLE, (uint8_t *)&EE_Prm.rtRulesCable[i], sizeof(routingRule_t));
  }

  // Send rtRulesJack -  rtRulesIthru
  for ( i=0 ; i != B_SERIAL_INTERFACE_MAX ; i ++ ) {
    I2C_SendData(B_DTYPE_ROUTING_RULE, i, PORT_TYPE_JACK, (uint8_t *)&EE_Prm.rtRulesJack[i], sizeof(routingRule_t));
    I2C_SendData(B_DTYPE_ROUTING_RULE_ALT, i, PORT_TYPE_ITHRU, (uint8_t *)&EE_Prm.rtRulesIthru[i], sizeof(routingRuleAlt_t));
  }

	// Send rtRulesVirtual
  for ( i=0 ; i != VIRTUAL_INTERFACE_MAX ; i ++ ) {
    I2C_SendData(B_DTYPE_ROUTING_RULE_ALT, i, PORT_TYPE_VIRTUAL, (uint8_t *)&EE_Prm.rtRulesVirtual[i], sizeof(routingRuleAlt_t));
  }

	// Send Bpm clocks (not used when slave but useful to monitor master)
  for ( i=0 ; i != MIDI_CLOCKGEN_MAX ; i ++ ) {
  	I2C_SendData(B_DTYPE_BPM_CLOCK, i, 0, (uint8_t *)&EE_Prm.bpmClocks[i], sizeof(bpmClock_t));
	}

  I2C_SendData(B_DTYPE_ROUTING_ITHRU_JACKIN_MSK, 0, 0,(uint8_t *)&EE_Prm.ithruJackInMsk, sizeof(EE_Prm.ithruJackInMsk));
  I2C_SendData(B_DTYPE_ROUTING_ITHRU_USB_IDLE_TIME_PERIOD, 0, 0, (uint8_t *)&EE_Prm.ithruUSBIdleTimePeriod, sizeof(EE_Prm.ithruUSBIdleTimePeriod));

	// Pipelines slots
	for ( i=0 ; i != TRANS_PIPELINE_SLOT_SIZE ; i ++ ) {

			for ( uint j=0 ; j !=  TRANS_PIPELINE_SIZE ; j++) {
					I2C_SendData(B_DTYPE_MIDI_TRANSPIPE, i, j,(uint8_t *)&EE_Prm.pipelineSlot[i].pipeline[j], sizeof(transPipe_t));
			}
	}

  I2C_SendCommand(0,   B_CMD_END_SYNC);

}

///////////////////////////////////////////////////////////////////////////////
// I2C Loop Process for a MASTER
///////////////////////////////////////////////////////////////////////////////
void I2C_ProcessMaster ()
{
  masterMidiPacket_t mpk;

  // Broadcast slaves of USB midi state & intellithru mode
  I2C_SendCommand(0, midiUSBCx ?  B_CMD_USBCX_AVAILABLE:B_CMD_USBCX_UNAVAILABLE);
  I2C_SendCommand(0, midiUSBIdle ?  B_CMD_USBCX_SLEEP:B_CMD_USBCX_AWAKE );
  I2C_SendCommand(0, midiIthruActive ?  B_CMD_INTELLITHRU_ENABLED:B_CMD_INTELLITHRU_DISABLED ) ;

  for ( uint8_t d = 0 ; d != I2C_DeviceActiveCount ; d++) {

      // Get a slave midi packet eventually
      if ( I2C_SendCommand(I2C_DeviceIdActive[d],B_CMD_ISPACKET_AVAIL) <= 0) continue;  // No packets or error
      if ( I2C_getPacket(I2C_DeviceIdActive[d],&mpk) <= 0 ) continue; // Error or nothing got
      if ( mpk.mpk.pk.i == 0 ) continue;  // Process only non empty packets

      if ( mpk.mpk.dest == PORT_TYPE_JACK ) {
          uint8_t targetPort = mpk.mpk.pk.packet[0] >> 4;
          I2C_BusSerialSendMidiPacket(&(mpk.mpk.pk), targetPort );
         // Send to a cable IN only if USB is available on the master
       } else if ( mpk.mpk.dest == PORT_TYPE_CABLE )
        //&& midiUSBCx && !midiUSBdle && !midiIthruActive)
       {
          MidiUSB.writePacket(&(mpk.mpk.pk.i));
       }
      // else ignore that packet....

	} // for
}

///////////////////////////////////////////////////////////////////////////////
// I2C Loop Process for a SLAVE
///////////////////////////////////////////////////////////////////////////////
void I2C_ProcessSlave ()
{
	static unsigned long I2C_LedTimerMillis = 0;

	// Update timer to avoid a permanent flash due to polling
	// millis() can't be used in ISR
	if ( I2C_MasterIsActive ) {
			I2C_MasterIsActive = false;
			if ( millis() > I2C_LedTimerMillis ) {
					I2C_LedTimerMillis = millis() + I2C_LED_TIMER_MILLIS;
					LED_Flash(&LED_ConnectTick);
			}
	}

	// Routing midi sync from master
	if ( I2C_SlaveSyncDoUpdate ) {
			// For now, the slave doesn't save master update as it is ALWAYS
			// synchronized at boot time.
			I2C_SlaveSyncDoUpdate = false;
	}

	// Packet from I2C master available ? (nb : already routed)
	if ( I2C_QPacketsFromMaster.available() ) {
			midiPacket_t pk;
			// These packets are mandatory local as a result of the routing
			I2C_QPacketsFromMaster.readBytes(pk.packet,sizeof(midiPacket_t));
			SerialMidi_SendPacket(&pk, (pk.packet[0] >> 4) % SERIAL_INTERFACE_MAX );
	}

	// Activate the configuration menu if a terminal is opened in Slave mode
  // and key pressed
  #if OPTION_CONFIGUI
	if (Serial.available()) {
      char key = Serial.read();
			Serial.println();
			ShowMidiKliKHeader();Serial.println();
			SerialPrintf("Slave %02d ready and listening.", EE_Prm.I2C_DeviceId);
			if (I2C_MasterIsActive)  Serial.print(" (MASTER READY)");
			else                     Serial.print(" (NO MASTER)");
			if (midiUSBIdle)  Serial.print(" (USB IDLE)");
			if (midiIthruActive) Serial.print(" (ITHRU MODE)");
			Serial.println();Serial.println();
			Serial.println("(r)outing rules - (1-8) pipeline slots - e(X)it to configuration menu :");
			Serial.println();
      if ( key == 'X') {
        Wire.flush();
        Wire.end();
        ShowConfigMenu();
      }
      else if ( key == 'r') {
				ShowMidiRouting(PORT_TYPE_VIRTUAL);
        ShowMidiRouting(PORT_TYPE_CABLE);
        ShowMidiRouting(PORT_TYPE_JACK);
      }
			else if ( key >= '1' || key <= '8' )
				ShowPipelineSlot(key - '0');
	}
#endif // OPTION_CONFIGUI
}
