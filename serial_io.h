// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// serial_io.h: serial output handling.
//
// Copyright (c) 2021 Juan Benitez <juan.a.benitez(a)gmail.com>
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

#ifndef DUMP1090_SERIALIO_H
#define DUMP1090_SERIALIO_H

typedef enum {
    MAVLINK_SERIAL,
    RAW_SERIAL,
    SBS_SERIAL
} serial_format_t;

struct serial_writer {
    int     enabled;
    char    *port;
    int     baud;

    uint64_t lastSend;
    uint64_t interval;

    serial_format_t format;    
};

int openSerial(const char* port);
int initSerial(int baud);
int closeSerial();
int serialWrite(uint8_t *buf, unsigned len);

void modesInitSerial(const char* port, int baud, int format);
void modesCloseSerial(const char *port);

void modesSerialMavlinkOutput(struct aircraft*);
void modesSerialRawOutput(struct modesMessage*, struct aircraft*);
void modesSerialSBSOutput(struct modesMessage*, struct aircraft*);

void modesSerialMavlinkHeartbeat(void);
void modesSerialMavlinkSendAircrafts(void);

#endif