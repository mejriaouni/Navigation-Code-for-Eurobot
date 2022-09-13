#include "Odometry.h"
#include "Motors.h"

void orientate (float orientation, int R, int L);
void Robot_Locate(float goal_x, float goal_y, int R, int L);

void move_distance (float distance, int R, int L);
void rotate(float angle, int R, int L);
