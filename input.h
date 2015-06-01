#ifndef INPUT_H_INCLUDED
#define INPUT_H_INCLUDED

typedef struct {
  double cmd_h;
  double cmd_yaw;
  double cmd_vel_x;
  double cmd_vel_y;
} input_commands_t;

void read_test_inputs(input_commands_t* out_cmds){
  out_cmds->cmd_h = 1.0;
  out_cmds->cmd_yaw = 0.0;
  out_cmds->cmd_vel_x = 0.0;
  out_cmds->cmd_vel_y = 0.0;
}

#endif
