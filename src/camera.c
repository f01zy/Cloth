#include "camera.h"
#include "raymath.h"
#include <math.h>
#include <raylib.h>

#define RADIANS(A) ((PI) / 180.0f * (A))
#define MAX_RADIUS (100.0f)

bool is_first_mouse = true;

void camera_update_position(Camera3D *camera, const Orbit *orbit) {
  camera->position.x = orbit->radius * cos(RADIANS(orbit->yaw)) * cos(RADIANS(orbit->pitch));
  camera->position.y = orbit->radius * sin(RADIANS(orbit->pitch));
  camera->position.z = orbit->radius * sin(RADIANS(orbit->yaw)) * cos(RADIANS(orbit->pitch));
}

void camera_handle_input(Orbit *orbit) {
  // Position
  Vector2 pos_dt = GetMouseDelta();
  if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
    orbit->pitch = Clamp(orbit->pitch + pos_dt.y, -89.0f, 89.0f);
    orbit->yaw += pos_dt.x;
  }

  // Radius
  float wheel = GetMouseWheelMove();
  float new_radius = orbit->radius - wheel;
  if (new_radius >= 0.0f && new_radius <= MAX_RADIUS) orbit->radius = new_radius;
}
