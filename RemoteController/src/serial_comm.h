#include <comm_structs.h>
#include "config.h"

void format_state(StateMessage s_msg, char *buffr, size_t bsize);

void format_control(ControlMessage ctrl_msg, char *buffr, size_t bsize);

void format_attitude(AttitudeMessage att_msg, char *buffr, size_t bsize);

void format_ekf(EKFMessage ekf_msg, char *buffr, size_t bsize);

void format_floats_to_buffer(char *buffr, float *all_floats, uint8_t num_floats);

void print_message_to_serial(Message& msg);

bool string_valid_pid_axis(String str);

PID_AXIS string_to_pid_axis(String str);

bool parse_serial(Message &msg);

ConfigMessage config_from_line(String serial_line);

void print_commands(ControlMessage msg);
