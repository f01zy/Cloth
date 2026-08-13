#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#define G             (9.8f)
#define K             (2000.0f)
#define EPS           (0.00001)
#define FPS           (120.0f)
#define DAMPING       (0.98f)
#define FRICTION      (0.9f)
#define SKIN_WIDTH    (0.1f)
#define SCREEN_WIDTH  (900.0f)
#define SCREEN_HEIGHT (600.0f)
#define SCREEN_TITLE  ("Cloth simulation")

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
  } rectangle;
} ObjectData;

typedef struct {
  ObjectType type;
  Vector3 pos;
  Color color;
  ObjectData as;
} Object;

Cloth create_cloth(Vector3 pos, Vector2 size, Vector2 res, Color color) {
  size_t points_len = res.x * res.y;
  Point *points = malloc(sizeof(Point) * points_len);
  float step_x = size.x / res.x;
  float step_z = size.y / res.y;
  for (int i = 0; i < res.y; i++) {
    for (int j = 0; j < res.x; j++) {
      size_t idx = res.x * i + j;
      Point *point = &points[idx];
      point->pos = point->prev_pos = (Vector3){pos.x + step_x * j, pos.y, pos.z + step_z * i};
      point->mass = 1.0f;
      point->is_pinned = false;
    }
  }

  size_t springs_len = res.x * res.y * 2 - res.x - res.y;
  Spring *springs = malloc(sizeof(Spring) * springs_len);
  size_t curr_spring = 0;
  for (int i = 0; i < res.y; i++) {
    for (int j = 0; j < res.x; j++) {
      size_t idx = res.x * i + j;
      if (j < res.x - 1) springs[curr_spring++] = (Spring){idx, idx + 1, step_x, K};
      if (i < res.y - 1) springs[curr_spring++] = (Spring){idx, idx + res.x, step_z, K};
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

  return (Cloth){
    .points = points,
    .points_len = points_len,
    .springs = springs,
    .springs_len = springs_len,
    .polygons = polygons,
    .polygons_len = polygons_len,
    .color = color,
  };
}

void apply_physics(Cloth *cloth) {
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
    point->acceleration = Vector3Scale(force, 1.0f / point->mass);
  }
}

void move_cloth(Cloth *cloth, float dt) {
  for (int i = 0; i < cloth->points_len; i++) {
    Point *point = &cloth->points[i];
    if (point->is_pinned) continue;
    Vector3 v = Vector3Subtract(point->pos, point->prev_pos);
    Vector3 damp = Vector3Scale(v, DAMPING);
    point->prev_pos = point->pos;
    point->pos = Vector3Add(point->pos, Vector3Add(damp, Vector3Scale(point->acceleration, dt * dt)));
  }
}

void draw_cloth(const Cloth *cloth) {
  for (int i = 0; i < cloth->polygons_len; i++) {
    const Polygon *polygon = &cloth->polygons[i];
    DrawTriangle3D(cloth->points[polygon->p1].pos, cloth->points[polygon->p2].pos, cloth->points[polygon->p3].pos, cloth->color);
  }
}

void draw_objects(const Object *objects, size_t len) {
  for (int i = 0; i < len; i++) {
    const Object *object = &objects[i];
    switch (object->type) {
    case OBJECT_SPHERE:
      DrawSphere(object->pos, object->as.sphere.radius, object->color);
      break;
    case OBJECT_RECTANGLE:
      DrawCube(object->pos, object->as.rectangle.width, object->as.rectangle.height, object->as.rectangle.length, object->color);
      break;
    }
  }
}

void check_sphere_collision(Point *point, const Object *sphere) {
  Vector3 dir = Vector3Subtract(point->pos, sphere->pos);
  Vector3 v = Vector3Subtract(point->pos, point->prev_pos);
  Vector3 n = Vector3Normalize(dir);
  float r = sphere->as.sphere.radius;
  bool is_collision = Vector3Length(dir) <= r + SKIN_WIDTH;
  if (is_collision && Vector3DotProduct(v, n) < 0.0f) {
    Vector3 vn = Vector3Scale(n, Vector3DotProduct(v, n));
    Vector3 vt = Vector3Subtract(v, vn);
    point->pos = Vector3Add(sphere->pos, Vector3Scale(n, r + SKIN_WIDTH));
    point->prev_pos = Vector3Subtract(point->pos, Vector3Scale(vt, FRICTION));
  }
}

void check_rectangle_collision(Point *point, const Object *rec) {
  // TODO
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

void free_cloth(const Cloth *cloth) {
  free(cloth->points);
  free(cloth->springs);
  free(cloth->polygons);
}

int main() {
  InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_TITLE);
  SetTargetFPS(FPS);
  DisableCursor();
  rlDisableBackfaceCulling();

  const Object objects[] = {
    {
      .type = OBJECT_RECTANGLE,
      .pos = {0.0f, -0.1f, 0.0f},
      .color = BLACK,
      .as.rectangle = {
	.width = 30.0f,
	.height = 0.1f,
	.length = 30.0f,
      }
    },
    {
      .type = OBJECT_SPHERE,
      .pos = {0.0f, 2.0f, 0.0f},
      .color = RED,
      .as.sphere.radius = 2.0f,
    },
  };
  size_t objects_len = sizeof(objects) / sizeof(*objects);

  Camera3D camera = {0};
  camera.position = (Vector3){0.0f, 10.0f, 10.0f};
  camera.target = (Vector3){0.0f, 0.0f, 0.0f};
  camera.up = (Vector3){0.0f, 1.0f, 0.0f};
  camera.fovy = 45.0f;
  camera.projection = CAMERA_PERSPECTIVE;

  Cloth cloth = create_cloth((Vector3){-1.5f, 5.0f, -1.5f}, (Vector2){3.0f, 3.0f}, (Vector2){10.0f, 10.0f}, BLUE);
  while (!WindowShouldClose()) {
    float dt = GetFrameTime();

    UpdateCamera(&camera, CAMERA_FREE);
    BeginDrawing();
    ClearBackground(BLACK);
    DrawFPS(10, 10);
    BeginMode3D(camera);
    DrawGrid(30, 1);

    apply_physics(&cloth);
    move_cloth(&cloth, dt);
    check_collisions(&cloth, objects, objects_len);
    draw_cloth(&cloth);
    draw_objects(objects, objects_len);

    EndMode3D();
    EndDrawing();
  }

  free_cloth(&cloth);
  CloseWindow();
}
