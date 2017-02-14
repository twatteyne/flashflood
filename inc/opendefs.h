#ifndef __OPENDEFS_H
#define __OPENDEFS_H

//==== frame content
#define FRAME_CONTROL_BYTE0   0x61 // 0b0110 0001  |bit6: panId compressed|bit5: AR set|bit4: no frame pending|bit3: sec disable|bit0-2: frame type,data|
#define FRAME_CONTROL_BYTE1   0x18 // 0b0001 1000  |bit14-15: src addr is elided|bit12-13:frame version, may not useful|bit10-11:16-bit dest addr|
// 2B fcf + 1B dsn + 2B dest panId + 2B dest address + 2B crc
#define FRAME_LENGTH 2 + 1 + 2 + 2 + 2
#define ACK_LENGTH   5

#define CHANNEL               26

//==== mote role
#define SOURCE_ID                          0x00  // no source

#define FIRST_HOP_1                        0xba
#define FIRST_HOP_2                        0xc8

#define SECOND_HOP_1                       0x0f
#define SECOND_HOP_2                       0x05

#define THIRD_HOP_1                        0x2b
#define THIRD_HOP_2                        0x5e

#define FOURTH_HOP_1                       0x16
#define FOURTH_HOP_2                       0xdd

#define DESTINATION_ID                     0x57

//=========================== variables =======================================

//=========================== prototypes ======================================

#endif
