#include "camera.h"
#include <float.h>
#include <math.h>
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(PLATFORM_WEB)
#include <emscripten/emscripten.h>
#endif

/********************************************
 * DEFINES                                  *
 ********************************************/

#define G                  (9.8f)
#define K                  (20000.0f)
#define EPS                (0.00001)
#define FPS                (60.0f)
#define STIFFNESS          (0.8f)
#define DRAG_FORCE         (50000.0f)
#define DAMPING            (0.998f)
#define FRICTION           (0.9f)
#define SKIN_WIDTH         (0.05f)
#define PHYSICS_ITERATIONS (20)
#define SCREEN_WIDTH       (900.0f)
#define SCREEN_HEIGHT      (600.0f)
#define SCREEN_TITLE       ("Cloth simulation")

/********************************************
 * TYPES                                    *
 ********************************************/

typedef struct {
  Vector3 pos, prev_pos;
  Vector3 acceleration;
  float mass;
  bool is_pinned;
} Point;

typedef struct {
  size_t p1, p2;
  float rest_length;
  float k;
} Spring;

typedef struct {
  size_t p1, p2, p3;
} Polygon;

typedef struct {
  Point *points;
  size_t points_len;
  Spring *springs;
  size_t springs_len;
  Polygon *polygons;
  size_t polygons_len;
  Color color;
  bool is_wireframe;
} Cloth;

typedef enum {
  OBJECT_SPHERE,
  OBJECT_RECTANGLE,
} ObjectType;

typedef union {
  struct {
    float radius;
  } sphere;
  struct {
    float width;
    float height;
    float length;
  } rec;
} ObjectData;

typedef struct {
  ObjectType type;
  Vector3 pos;
  Color color;
  bool is_invisible;
  ObjectData as;
} Object;

typedef struct {
  Vector3 normal;
  float depth;
} Contact;

typedef struct {
  bool is_dragging;
  size_t point_idx;
  Vector3 normal;
  Vector3 pos;
  Ray ray;
} Drag;

/********************************************
 * GLOBAL VARIABLES                         *
 ********************************************/

const Object objects[] = {
  {.type = OBJECT_RECTANGLE, .pos = {0.0f, -0.5f, 0.0f}, .color = GRAY, .is_invisible = true, .as.rec = {.width = 30.0f, .height = 1.0f, .length = 30.0f,}},
  {.type = OBJECT_SPHERE, .pos = {0.0f, 2.0f, 0.0f}, .color = GRAY, .is_invisible = false, .as.sphere.radius = 2.0f},
};
size_t objects_len = sizeof(objects) / sizeof(*objects);

Vector4 pin = {false, false, false, false};
Vector3 pos = {-2.5f, 5.0f, -2.5f};
Vector2 size = {5.0f, 5.0f};
Vector2 res = {40.0f, 40.0f};
Cloth cloth = {0};
Drag drag = {0};

Orbit orbit = {
  .yaw = 0.0f,
  .pitch = 30.0f,
  .radius = 20.0f,
};

Camera3D camera = {
  .target = {0.0f, 0.0f, 0.0f},
  .up = {0.0f, 1.0f, 0.0f},
  .fovy = 45.0f,
  .projection = CAMERA_PERSPECTIVE,
};

/********************************************
 * UTILITY                                  *
 ********************************************/

void set_cloth_pin(Cloth *cloth, Vector2 res, Vector4 pin) {
  for (int i = 0; i < res.y; i++) {
    for (int j = 0; j < res.x; j++) {
      size_t idx = res.x * i + j;
      Point *point = &cloth->points[idx];
      point->is_pinned = false;
      // clang-format off
      if ((pin.x && i == 0 && j == 0)            ||
	  (pin.y && i == 0 && j == res.x - 1)   ||
	  (pin.z && i == res.y - 1 && j == 0) ||
	  (pin.w && i == res.y - 1 && j == res.x - 1)) {
	  point->is_pinned = true;
      }
      // clang-format on
    }
  }
}

void set_cloth_position(Cloth *cloth, Vector3 pos, Vector2 size, Vector2 res) {
  float step_x = size.x / res.x;
  float step_z = size.y / res.y;
  for (int i = 0; i < res.y; i++) {
    for (int j = 0; j < res.x; j++) {
      size_t idx = res.x * i + j;
      Point *point = &cloth->points[idx];
      point->mass = 1.0f;
      point->pos = point->prev_pos = (Vector3){pos.x + step_x * j, pos.y, pos.z + step_z * i};
      point->acceleration = (Vector3){0.0f, 0.0f, 0.0f};
    }
  }
}

Cloth create_cloth(Vector3 pos, Vector2 size, Vector2 res, Vector4 pin, Color color) {
  size_t points_len = res.x * res.y;
  Point *points = malloc(sizeof(Point) * points_len);
  float step_x = size.x / res.x;
  float step_z = size.y / res.y;

  size_t springs_len = res.x * res.y * 4 - res.x * 3 - res.y * 3 + 2;
  Spring *springs = malloc(sizeof(Spring) * springs_len);
  float diagonal = sqrt(step_x * step_x + step_z * step_z);
  size_t curr_spring = 0;
  for (int i = 0; i < res.y; i++) {
    for (int j = 0; j < res.x; j++) {
      size_t idx = res.x * i + j;
      if (i < res.y - 1) springs[curr_spring++] = (Spring){idx, idx + res.x, step_z, K};
      if (j < res.x - 1) springs[curr_spring++] = (Spring){idx, idx + 1, step_x, K};
      if (i < res.y - 1 && j < res.x - 1) springs[curr_spring++] = (Spring){idx, idx + res.x + 1, diagonal, K * 0.5f};
      if (i < res.y - 1 && j > 0) springs[curr_spring++] = (Spring){idx, idx + res.x - 1, diagonal, K * 0.5f};
    }
  }

  size_t polygons_len = (res.x - 1) * (res.y - 1) * 2;
  Polygon *polygons = malloc(sizeof(Polygon) * polygons_len);
  size_t curr_poligon = 0;
  for (int i = 0; i < res.y - 1; i++) {
    for (int j = 0; j < res.x - 1; j++) {
      size_t idx = res.x * i + j;
      polygons[curr_poligon++] = (Polygon){idx, idx + res.x, idx + 1};
      polygons[curr_poligon++] = (Polygon){idx + 1, idx + res.x, idx + res.x + 1};
    }
  }

  Cloth cloth = {
    .points = points,
    .points_len = points_len,
    .springs = springs,
    .springs_len = springs_len,
    .polygons = polygons,
    .polygons_len = polygons_len,
    .color = color,
  };
  set_cloth_position(&cloth, pos, size, res);
  set_cloth_pin(&cloth, res, pin);
  return cloth;
}

void free_cloth(const Cloth *cloth) {
  free(cloth->points);
  free(cloth->springs);
  free(cloth->polygons);
}

Vector3 get_normal(Vector3 a, Vector3 b, Vector3 c) {
  Vector3 ab = Vector3Subtract(b, a);
  Vector3 ac = Vector3Subtract(c, a);
  return Vector3CrossProduct(ab, ac);
}

/********************************************
 * RENDERERS                                *
 ********************************************/

void draw_cloth(const Cloth *cloth) {
  for (int i = 0; i < cloth->polygons_len; i++) {
    const Polygon *polygon = &cloth->polygons[i];
    const Point *a = &cloth->points[polygon->p1], *b = &cloth->points[polygon->p2], *c = &cloth->points[polygon->p3];
    if (cloth->is_wireframe) {
      DrawLine3D(a->pos, b->pos, cloth->color);
      DrawLine3D(b->pos, c->pos, cloth->color);
      DrawLine3D(c->pos, a->pos, cloth->color);
      continue;
    }
    DrawTriangle3D(a->pos, b->pos, c->pos, cloth->color);
  }
}

void draw_objects(const Object *objects, size_t len) {
  for (int i = 0; i < len; i++) {
    const Object *object = &objects[i];
    if (object->is_invisible) continue;
    switch (object->type) {
    case OBJECT_SPHERE:
      DrawSphere(object->pos, object->as.sphere.radius, object->color);
      break;
    case OBJECT_RECTANGLE:
      DrawCube(object->pos, object->as.rec.width, object->as.rec.height, object->as.rec.length, object->color);
      break;
    }
  }
}

/********************************************
 * COLLISIONS                               *
 ********************************************/

bool get_plane_intersection(Vector3 plane_point, Vector3 plane_normal, Ray ray, Vector3 *dest) {
  float denom = Vector3DotProduct(ray.direction, plane_normal);
  if (denom < 0.0f) {
    plane_normal = Vector3Scale(plane_normal, -1.0f);
    denom = Vector3DotProduct(ray.direction, plane_normal);
  }
  Vector3 tmp = Vector3Subtract(plane_point, ray.position);
  float num = Vector3DotProduct(tmp, plane_normal);
  float t = num / denom;
  if (t < 0.0f) return false;
  *dest = Vector3Add(ray.position, Vector3Scale(ray.direction, t));
  return true;
}

bool get_triangle_intersetion(Vector3 a, Vector3 b, Vector3 c, Ray ray) {
  Vector3 x;
  Vector3 n = get_normal(a, b, c);
  if (!get_plane_intersection(a, n, ray, &x)) return false;
  Vector3 n1 = Vector3CrossProduct(Vector3Subtract(b, a), Vector3Subtract(x, a));
  Vector3 n2 = Vector3CrossProduct(Vector3Subtract(c, b), Vector3Subtract(x, b));
  Vector3 n3 = Vector3CrossProduct(Vector3Subtract(a, c), Vector3Subtract(x, c));
  float d1 = Vector3DotProduct(n1, n);
  float d2 = Vector3DotProduct(n2, n);
  float d3 = Vector3DotProduct(n3, n);
  if ((d1 >= 0 && d2 >= 0 && d3 >= 0) || (d1 <= 0 && d2 <= 0 && d3 <= 0)) return true;
  return false;
}

int get_ray_intersection_point(const Cloth *cloth, Ray ray) {
  int ans = -1;
  float dis = FLT_MAX;
  for (int i = 0; i < cloth->polygons_len; i++) {
    const Polygon *polygon = &cloth->polygons[i];
    const Point *a = &cloth->points[polygon->p1], *b = &cloth->points[polygon->p2], *c = &cloth->points[polygon->p3];
    float curr_dis = Vector3Distance(ray.position, a->pos);
    if (get_triangle_intersetion(a->pos, b->pos, c->pos, ray) && curr_dis < dis) {
      ans = polygon->p1;
      dis = curr_dis;
    }
  }
  return ans;
}

void check_sphere_collision(Point *point, const Object *sphere) {
  Vector3 dir = Vector3Subtract(point->pos, sphere->pos);
  Vector3 v = Vector3Subtract(point->pos, point->prev_pos);
  Vector3 n = Vector3Normalize(dir);
  float r = sphere->as.sphere.radius;
  bool is_collision = Vector3Length(dir) <= r + SKIN_WIDTH;
  if (is_collision) {
    point->pos = Vector3Add(sphere->pos, Vector3Scale(n, r + SKIN_WIDTH));
    if (Vector3DotProduct(v, n) < 0.0f) {
      Vector3 vn = Vector3Scale(n, Vector3DotProduct(v, n));
      Vector3 vt = Vector3Subtract(v, vn);
      point->prev_pos = Vector3Subtract(point->pos, Vector3Scale(vt, FRICTION));
    }
  }
}

Contact get_rectangle_contact(Vector3 pos, Vector3 min, Vector3 max) {
  float depths[6] = {pos.x - min.x, max.x - pos.x, pos.y - min.y, max.y - pos.y, pos.z - min.z, max.z - pos.z};
  Vector3 normals[6] = {{-1.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, -1.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, -1.0f}, {0.0f, 0.0f, 1.0f}};
  size_t min_idx = 0;
  for (int i = 1; i < 6; i++) {
    if (depths[i] < depths[min_idx]) { min_idx = i; }
  }
  return (Contact){normals[min_idx], depths[min_idx]};
}

void check_rectangle_collision(Point *point, const Object *rec) {
  float width = rec->as.rec.width, height = rec->as.rec.height, length = rec->as.rec.length;
  Vector3 half = {width / 2.0f, height / 2.0f, length / 2.0f};
  Vector3 min = Vector3Subtract(rec->pos, half);
  Vector3 max = Vector3Add(rec->pos, half);
  // clang-format off
  if (point->pos.x >= min.x - SKIN_WIDTH && point->pos.x <= max.x + SKIN_WIDTH &&
      point->pos.y >= min.y - SKIN_WIDTH && point->pos.y <= max.y + SKIN_WIDTH &&
      point->pos.z >= min.z - SKIN_WIDTH && point->pos.z <= max.z + SKIN_WIDTH) {
    Contact contact = get_rectangle_contact(point->pos, min, max);  
    point->pos = Vector3Add(point->pos, Vector3Scale(contact.normal, contact.depth + SKIN_WIDTH));
    Vector3 v = Vector3Subtract(point->pos, point->prev_pos);
    if (Vector3DotProduct(v, contact.normal) < 0.0f) {
      Vector3 vn = Vector3Scale(contact.normal, Vector3DotProduct(v, contact.normal));
      Vector3 vt = Vector3Subtract(v, vn);
      point->prev_pos = Vector3Subtract(point->pos, Vector3Scale(vt, FRICTION));
    }
  }
  // clang-format on
}

void check_collisions(const Cloth *cloth, const Object *objects, size_t objects_len) {
  for (int i = 0; i < cloth->points_len; i++) {
    Point *point = &cloth->points[i];
    for (int j = 0; j < objects_len; j++) {
      const Object *object = &objects[j];
      switch (object->type) {
      case OBJECT_SPHERE:
        check_sphere_collision(point, object);
        break;
      case OBJECT_RECTANGLE:
        check_rectangle_collision(point, object);
        break;
      }
    }
  }
}

/********************************************
 * PHYSICS                                  *
 ********************************************/

void apply_external_forces(Cloth *cloth, const Drag *drag) {
  Vector3 spring_forces[cloth->points_len];
  for (int i = 0; i < cloth->points_len; i++) {
    spring_forces[i] = (Vector3){0.0f, 0.0f, 0.0f};
  }
  for (int i = 0; i < cloth->springs_len; i++) {
    Spring *spring = &cloth->springs[i];
    Point *p1 = &cloth->points[spring->p1], *p2 = &cloth->points[spring->p2];
    Vector3 dir = Vector3Normalize(Vector3Subtract(p2->pos, p1->pos));
    float dt = Vector3Distance(p1->pos, p2->pos) - spring->rest_length;
    Vector3 spring_force = Vector3Scale(dir, spring->k * dt);
    spring_forces[spring->p1] = Vector3Add(spring_forces[spring->p1], spring_force);
    spring_forces[spring->p2] = Vector3Subtract(spring_forces[spring->p2], spring_force);
  }
  for (int i = 0; i < cloth->points_len; i++) {
    Point *point = &cloth->points[i];
    Vector3 spring_force = spring_forces[i];
    Vector3 gravity_force = {0.0f, -point->mass * G, 0.0f};
    Vector3 force = Vector3Add(spring_force, gravity_force);
    if (drag->is_dragging && drag->point_idx == i) {
      Vector3 x;
      if (get_plane_intersection(drag->pos, drag->normal, drag->ray, &x)) {
        Vector3 dir = Vector3Normalize(Vector3Subtract(x, point->pos));
        force = Vector3Add(force, Vector3Scale(dir, DRAG_FORCE));
      }
    }
    point->acceleration = Vector3Scale(force, 1.0f / point->mass);
  }
}

void integrate_positions(Cloth *cloth, float dt) {
  for (int i = 0; i < cloth->points_len; i++) {
    Point *point = &cloth->points[i];
    if (point->is_pinned) continue;
    Vector3 v = Vector3Subtract(point->pos, point->prev_pos);
    Vector3 damp = Vector3Scale(v, DAMPING);
    point->prev_pos = point->pos;
    point->pos = Vector3Add(point->pos, Vector3Add(damp, Vector3Scale(point->acceleration, dt * dt)));
  }
}

void update_cloth_physics(Cloth *cloth, const Drag *drag, const Object *objects, size_t objects_len, float dt) {
  float sub_dt = dt / PHYSICS_ITERATIONS;
  for (int i = 0; i < PHYSICS_ITERATIONS; i++) {
    apply_external_forces(cloth, drag);
    integrate_positions(cloth, sub_dt);
    check_collisions(cloth, objects, objects_len);
  }
}

/********************************************
 * MAIN LOOP                                  *
 ********************************************/

void update_draw_frame(void) {
  float dt = GetFrameTime();
  Vector2 mouse_pos = GetMousePosition();
  Ray ray = GetScreenToWorldRay(mouse_pos, camera);

  // Dragging
  if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
    int idx = get_ray_intersection_point(&cloth, ray);
    if (idx != -1) {
      const Point *point = &cloth.points[idx];
      drag.is_dragging = true;
      drag.point_idx = idx;
      drag.pos = point->pos;
      drag.normal = Vector3Subtract(camera.position, point->pos);
    }
  }
  if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT)) drag.is_dragging = false;
  if (drag.is_dragging) drag.ray = ray;

  // Pin
  Vector4 new_pin = pin;
  if (IsKeyPressed(KEY_Q)) new_pin.x = !new_pin.x;
  if (IsKeyPressed(KEY_W)) new_pin.y = !new_pin.y;
  if (IsKeyPressed(KEY_A)) new_pin.z = !new_pin.z;
  if (IsKeyPressed(KEY_S)) new_pin.w = !new_pin.w;
  if (memcmp(&pin, &new_pin, sizeof(pin))) {
    pin = new_pin;
    set_cloth_pin(&cloth, res, new_pin);
  }
  if (IsKeyPressed(KEY_R)) set_cloth_position(&cloth, pos, size, res);

  // Other input
  if (IsKeyPressed(KEY_F1)) cloth.is_wireframe = !cloth.is_wireframe;

  camera_handle_input(&orbit);
  camera_update_position(&camera, &orbit);
  update_cloth_physics(&cloth, &drag, objects, objects_len, dt);

  BeginDrawing();
  ClearBackground(BLACK);
  DrawFPS(10, 10);
  BeginMode3D(camera);
  DrawGrid(30, 1);
  draw_cloth(&cloth);
  draw_objects(objects, objects_len);
  EndMode3D();
  EndDrawing();
}

int main() {
  InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_TITLE);
  rlDisableBackfaceCulling();
  cloth = create_cloth(pos, size, res, pin, MAROON);

#if defined(PLATFORM_WEB)
  emscripten_set_main_loop(update_draw_frame, 0, 1);
#else
  SetTargetFPS(FPS);
  while (!WindowShouldClose()) {
    update_draw_frame();
  }
#endif

  CloseWindow();
  free_cloth(&cloth);
}
