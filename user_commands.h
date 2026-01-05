#pragma once
#include <vector>

enum CommandType {
    MOVE_FWD,
    MOVE_BWD,
    TURN_LEFT,
    TURN_RIGHT,
    DELAY
};

struct UserCommand {
    CommandType type;
    float value; // mm, degrees, or ms
};

const uint32_t TARGET_TIME_MS = 15000; 

const std::vector<UserCommand> command_sequence = {
    {MOVE_FWD, 508.0f},
    {TURN_RIGHT, 90},
    {MOVE_FWD, 500.0f},
    {TURN_RIGHT, 90},
    {MOVE_FWD, 500.0f},
    {TURN_RIGHT, 90},
    {MOVE_FWD, 508.0f},
    {TURN_RIGHT, 90},
};
