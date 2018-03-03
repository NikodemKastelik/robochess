/*
 * messages.h
 *
 *  Created on: 18 gru 2017
 *      Author: DELL
 */

#ifndef MESSAGES_H_
#define MESSAGES_H_

#define MSG_MOVE_GLOBAL			0
#define MSG_MOVE_GLOBAL_OFFSET	12

#define MSG_MOVE_JOINT			1
#define MSG_MOVE_JOINT_OFFSET	11

#define MSG_MOVE_TOOL			2
#define MSG_MOVE_TOOL_OFFSET	10

#define MSG_MOVE_PIECE			3
#define MSG_MOVE_PIECE_OFFSET	11

#define MSG_CAPTURE_PIECE		4
#define MSG_CAPTURE_PIECE_OFFSET	14

#define MSG_GRIPPER				5

#define MSG_BT_ALIVE			6

#define MSG(x)	((char*)(msg[x]))

extern const char* const msg[];

#endif /* MESSAGES_H_ */
