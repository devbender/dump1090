// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// serial_io.c: serial output handling.
//
//Copyright (c) 2021 Juan Benitez <juan.a.benitez(a)gmail.com>
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.
//
// This file is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

// This file incorporates work covered by the following copyright and
// permission notice:
//
//   Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
//
//   All rights reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are
//   met:
//
//    *  Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//    *  Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//   HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "dump1090.h"

/* for Mavlink definitions */
#include "modules/c_library_v2/common/mavlink.h"

int fd = -1;
int serialOpen = 0;

//
//=========================================================================
//
// Open serial port
//
int openSerial(const char* port) {
    // Open serial port
	// O_RDWR - Read and write
	// O_NOCTTY - Ignore special chars like CTRL-C

	printf("Opening serial port: %s >> ", port);

	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);

	// Check for Errors
	if (fd == -1)  {
		printf("[FAILED]\n");  // could not open port	
		return(-1);               
	} else {
		printf("[OK]\n");
		fcntl(fd, F_SETFL, 0); // success	
	}

    serialOpen = 1;

	// Done!
	return fd;
}

//
//=========================================================================
//
// Close serial port
//
int closeSerial(const char* port) {

    if(!serialOpen) return 0;
    
    printf("Closing serial port: %s >> ", port);

	int result = close(fd);

	if (result) fprintf(stderr,"ERROR (%i)\n", result );
	else printf("[OK]");

	serialOpen = 0;

	printf("\n");

	return 1;
}


//
//=========================================================================
//
// Initialize serial port
//
int initSerial(int baud) {

    // Check file descriptor
	if(!isatty(fd))	{
		fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", fd);
		return 0;
	}

	// Read file descritor configuration
	struct termios  config;
	if(tcgetattr(fd, &config) < 0) 	{
		fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
		return 0;
	}

	// Input flags - Turn off input processing
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
						INLCR | PARMRK | INPCK | ISTRIP | IXON);

	// Output flags - Turn off output processing
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
						 ONOCR | OFILL | OPOST);

	#ifdef OLCUC
		config.c_oflag &= ~OLCUC;
	#endif

	#ifdef ONOEOT
		config.c_oflag &= ~ONOEOT;
	#endif

	// No line processing:
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	// Turn off character processing
	// clear current char size mask, no parity checking,
	// no output processing, force 8 bit input
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;

	// One input byte is enough to return from read()
	// Inter-character timer off
	config.c_cc[VMIN]  = 1;
	config.c_cc[VTIME] = 10; // was 0

	// Get the current options for the port
	////struct termios options;
	////tcgetattr(fd, &options);

	// Apply baudrate
	switch (baud) {
		
        case 9600:
			cfsetispeed(&config, B9600);
			cfsetospeed(&config, B9600);
			break;
		
        case 19200:
			cfsetispeed(&config, B19200);
			cfsetospeed(&config, B19200);
			break;
		
        case 38400:
			if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0) 			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return 0;
			}
			break;

		case 57600:
			if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0) {
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return 0;
			}
			break;
		
        case 115200:
			if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0) {
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return 0;
			}
			break;

		case 921600:
			if (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0) {
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return 0;
			}
			break;

		default:
			fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
			return 0;

			break;
	}

	// Finally, apply the configuration
	if(tcsetattr(fd, TCSAFLUSH, &config) < 0) {
		fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
		return 0;
	}

	// Done!
	return 1;
}


//
//=========================================================================
//
// Write to serial port
//
int serialWrite(uint8_t *buf, unsigned len) {
    
	//printf("WRITING >> %i bytes\n", len);

    // Write packet via serial link
	const int bytesWritten = (int)write(fd, buf, len);

    // Wait until all data has been written
	tcdrain(fd);

    return bytesWritten;
}


//
//=========================================================================
//
// Init
//
void modesInitSerial(void) {
    
    // Open Port
    if( openSerial(Modes.serial.port) == -1) {
        printf("serial: error opening port.\n");
        exit(1);
    }

    // Init
    if(serialOpen) {
        initSerial(Modes.serial.baud);
    }
}


//
//=========================================================================
//
// Close
//
void modesCloseSerial(void) {
	closeSerial(Modes.serial.port);
}

//
//=========================================================================
//
// Send Mavlink Heartbeat
//
void modesSerialHeartbeat() {

	mavlink_message_t mav_heartbeat_message;    
    uint8_t mav_heartbeat_buffer[MAVLINK_MAX_PACKET_LEN];

    // Pack mavlink message
    mavlink_msg_heartbeat_pack(254,
                               MAV_COMP_ID_ADSB,
                               &mav_heartbeat_message,
                               MAV_TYPE_ADSB,
                               MAV_AUTOPILOT_INVALID,
                               0, 0,
                               MAV_STATE_ACTIVE);

    // Send packed message to buffer
    uint16_t len = mavlink_msg_to_send_buffer(mav_heartbeat_buffer, &mav_heartbeat_message);

	// Write to Serial Port
	serialWrite(mav_heartbeat_buffer, len);
}



//
//=========================================================================
//
// Close
//
void modesSerialSendAircrafts(void) {
	
	struct aircraft *a;
	
	for (a = Modes.aircrafts; a; a = a->next) {
        
		// We require a tracked/reliable aircraft for serial output
		if (!a || !a->reliable) {
            continue;
        }

		// Output serial in specified format
		else {
			if(Modes.serial.format == MAVLINK_SERIAL) 
				modesSerialMavlinkOutput(a);
			else if(Modes.serial.format == RAW_SERIAL) 
				modesSerialRawOutput(a);
			else if(Modes.serial.format == SBS_SERIAL) 
				modesSerialSBSOutput(a);
			else 
				modesSerialMavlinkOutput(a);
		}
	}	
}


void modesSerialRawOutput(struct aircraft *a) {

}



void modesSerialSBSOutput(struct aircraft *a) {

}



//
//=========================================================================
//
// Serial Out
//
void modesSerialMavlinkOutput(struct aircraft *a) {

    // Suppress aircrafts without valid position
    if(!trackDataValid(&a->position_valid))
        return;
	
    mavlink_message_t adsb_message;
    uint8_t adsb_buffer[MAVLINK_MAX_PACKET_LEN];

    uint8_t system_id = 254; // 255 is GCS, high enough to avoid ID collisions
    uint8_t component_id = MAV_COMP_ID_ADSB; 

    uint32_t icao_address = a->addr;
    int32_t lat = 0, lon = 0;
    uint8_t altitude_type = 0;
    int32_t altitude = 0;
    uint16_t heading = 0;
    uint16_t hor_velocity = 0;
    int16_t ver_velocity = 0;
    uint8_t emitter_type = 0;
    uint32_t tslc = (mstime() - a->seen) / 1000;
    uint16_t flags = 0;
    uint16_t squawk = 0;
    char callsign[9];

    // Callsign
    memset(callsign, '\0', sizeof(callsign));

    if (trackDataValid(&a->callsign_valid)){
        flags |= ADSB_FLAGS_VALID_CALLSIGN;
        strncpy(callsign, a->callsign, sizeof(callsign));
    }

    // Altitude (barometric preferred) [milimiters]
    if (trackDataValid(&a->altitude_baro_valid)) {
        altitude_type = ADSB_ALTITUDE_TYPE_GEOMETRIC;
        flags |= ADSB_FLAGS_VALID_ALTITUDE;
        altitude = a->altitude_baro * 304.8; // ft -> mm

    } else if (trackDataValid(&a->altitude_geom_valid)) {
        altitude_type = ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
        flags |= ADSB_FLAGS_VALID_ALTITUDE;
        altitude = a->altitude_geom * 304.8; // ft -> mm
    }

    // Speed (ground speed used) and vrate (barometric preferred) [centimeter per second]
    if (trackDataValid(&a->gs_valid) && trackDataValid(&a->baro_rate_valid)) {
        flags |= ADSB_FLAGS_VALID_VELOCITY;
        hor_velocity = a->gs * 51.44; // kts -> cm/s
        ver_velocity = a->baro_rate * 0.508; // ft/min -> cm/s

    } else if(trackDataValid(&a->gs_valid) && trackDataValid(&a->geom_rate_valid)) {
        flags |= ADSB_FLAGS_VALID_VELOCITY;
        hor_velocity = a->gs * 51.44; // kts -> cm/s
        ver_velocity = a->geom_rate * 0.508; // ft/min -> cm/s
    }
    
    // Heading (ground track preferred) [degrees x 100]
    if (trackDataValid(&a->track_valid)) {
        flags |= ADSB_FLAGS_VALID_HEADING;
        heading = a->track * 100; // deg -> deg * 1e2

    } else if (trackDataValid(&a->mag_heading_valid)) {
        flags |= ADSB_FLAGS_VALID_HEADING;
        heading = a->mag_heading * 100; // deg -> deg * 1e2
    }
    
    // Position: Lat & Lon [deg x 10^7]
    if (trackDataValid(&a->position_valid)) {
        flags |= ADSB_FLAGS_VALID_COORDS;
        lat = (int32_t)(a->lat * 1e7); // deg * 1e7
        lon = (int32_t)(a->lon * 1e7); // deg * 1e7
    }

    // Squawk
    if (trackDataValid(&a->squawk_valid)) {
        squawk = a->squawk;
    }

    // Category    
	switch(a->category) {
		case 0x00: emitter_type = ADSB_EMITTER_TYPE_NO_INFO;            break;
		case 0xA0: emitter_type = ADSB_EMITTER_TYPE_NO_INFO;            break;
		case 0xA1: emitter_type = ADSB_EMITTER_TYPE_LIGHT;              break;
		case 0xA2: emitter_type = ADSB_EMITTER_TYPE_SMALL;              break;
		case 0xA3: emitter_type = ADSB_EMITTER_TYPE_LARGE;              break;
		case 0xA4: emitter_type = ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE;  break;
		case 0xA5: emitter_type = ADSB_EMITTER_TYPE_HEAVY;              break;
		case 0xA6: emitter_type = ADSB_EMITTER_TYPE_HIGHLY_MANUV;       break;
		case 0xA7: emitter_type = ADSB_EMITTER_TYPE_ROTOCRAFT;          break;

		case 0xB0: emitter_type = ADSB_EMITTER_TYPE_UNASSIGNED;         break;
		case 0xB1: emitter_type = ADSB_EMITTER_TYPE_GLIDER;             break;
		case 0xB2: emitter_type = ADSB_EMITTER_TYPE_LIGHTER_AIR;        break;
		case 0xB3: emitter_type = ADSB_EMITTER_TYPE_PARACHUTE;          break;
		case 0xB4: emitter_type = ADSB_EMITTER_TYPE_ULTRA_LIGHT;        break;
		case 0xB5: emitter_type = ADSB_EMITTER_TYPE_UNASSIGNED2;        break;
		case 0xB6: emitter_type = ADSB_EMITTER_TYPE_UAV;                break;
		case 0xB7: emitter_type = ADSB_EMITTER_TYPE_SPACE;              break;

		case 0xC0: emitter_type = ADSB_EMITTER_TYPE_UNASSGINED3;        break;
		case 0xC1: emitter_type = ADSB_EMITTER_TYPE_EMERGENCY_SURFACE;  break;
		case 0xC2: emitter_type = ADSB_EMITTER_TYPE_SERVICE_SURFACE;    break;
		case 0xC3: emitter_type = ADSB_EMITTER_TYPE_POINT_OBSTACLE;     break;
		
		default: emitter_type = ADSB_EMITTER_TYPE_NO_INFO;
	}    

    // Pack mavlink message
    mavlink_msg_adsb_vehicle_pack(system_id,
                                  component_id,
                                  &adsb_message,
                                  icao_address,
                                  lat,
                                  lon,
                                  altitude_type,
                                  altitude,
                                  heading,
                                  hor_velocity,
                                  ver_velocity,
                                  callsign,
                                  emitter_type,
                                  tslc < 255 ? (uint8_t)tslc : 255,
                                  flags,
                                  squawk);

    // Send packed message to buffer
    uint16_t len = mavlink_msg_to_send_buffer(adsb_buffer, &adsb_message);

	// Write to Serial Port
	serialWrite(adsb_buffer, len);
}