#ifndef CAMERA_H
#define CAMERA_H

#include <raylib.h>

typedef struct {
  float yaw;
  float pitch;
  float radius;
} Orbit;

void camera_update_position(Camera3D *camera, const Orbit *orbit);
void camera_handle_input(Orbit *orbit);

#endif
