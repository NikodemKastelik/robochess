#include "messages.h"

const char msg_move_global[] = "move_global=";
const char msg_move_joint[] = "move_joint=";
const char msg_move_tool[] = "move_tool=";
const char msg_move_piece_fromto[] = "move_piece=";
const char msg_capture_piece[] = "capture_piece=";
const char msg_gripper_toggle[] = "gripper";
const char msg_bt_alive[] = "bt_alive";

const char* const msg[] = {
		msg_move_global,
		msg_move_joint,
		msg_move_tool,
		msg_move_piece_fromto,
		msg_capture_piece,
		msg_gripper_toggle,
		msg_bt_alive
};
