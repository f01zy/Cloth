#include <math.h>
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define K             (100.0f)
#define G             (9.8f)
#define FPS           (60.0f)
#define SCREEN_WIDTH  (900.0f)
#define SCREEN_HEIGHT (600.0f)
#define SCREEN_TITLE  ("Cloth simulation")

typedef struct {
  Vector3 pos, prev_pos;
  Vector3 acceleration;
  float mass;
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
  Vector3 p1, p2, p3;
} Triangle;

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
  Vector3 position;
  Color color;
  Triangle *triangles;
  size_t triangles_len;
  ObjectData as;
} Object;

Cloth create_cloth(Vector3 pos, Vector2 size, Vector2 res, Color color) {
  size_t points_len = res.x * res.y;
  Point *points = malloc(sizeof(Point) * points_len);
  float step_x = size.x / res.x;
  float step_z = size.y / res.y;
  for (int i = 0; i < res.y; i++) {
    for (int j = 0; j < res.x; j++) {
      size_t idx = res.y * i + j;
      Point *point = &points[idx];
      point->pos = point->prev_pos = (Vector3){pos.x + step_x * j, pos.y, pos.z + step_z * i};
      point->mass = 1.0f;
    }
  }

  size_t springs_len = res.x * res.y * 2 - res.x - res.y;
  Spring *springs = malloc(sizeof(Spring) * springs_len);
  size_t curr_spring = 0;
  for (int i = 0; i < res.y; i++) {
    for (int j = 0; j < res.x; j++) {
      size_t idx = res.y * i + j;
      if (j < res.x - 1) springs[curr_spring++] = (Spring){idx, idx + 1, step_x, K};
      if (i < res.y - 1) springs[curr_spring++] = (Spring){idx, idx + res.x, step_z, K};
    }
  }

  size_t polygons_len = (res.x - 1) * (res.y - 1) * 2;
  Polygon *polygons = malloc(sizeof(Polygon) * polygons_len);
  size_t curr_poligon = 0;
  for (int i = 0; i < res.y - 1; i++) {
    for (int j = 0; j < res.x - 1; j++) {
      size_t idx = res.y * i + j;
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

Object create_object(ObjectType type, Vector3 position, Color color, ObjectData data) {
  Mesh mesh;
  switch (type) {
  case OBJECT_SPHERE:
    mesh = GenMeshSphere(data.sphere.radius, 72, 24);
    break;
  case OBJECT_RECTANGLE:
    mesh = GenMeshCube(data.rectangle.width, data.rectangle.height, data.rectangle.length);
    break;
  }
  Triangle *triangles = malloc(sizeof(Triangle) * mesh.triangleCount);
  Vector3 *vertices = (Vector3 *)mesh.vertices;
  uint16_t *indices = mesh.indices;
  for (int i = 0; i < mesh.triangleCount; i++) {
    Vector3 p1, p2, p3;
    if (indices) {
      p1 = vertices[indices[i * 3 + 0]];
      p2 = vertices[indices[i * 3 + 1]];
      p3 = vertices[indices[i * 3 + 2]];
    } else {
      p1 = vertices[i * 3 + 0];
      p2 = vertices[i * 3 + 1];
      p3 = vertices[i * 3 + 2];
    }
    triangles[i] = (Triangle){p1, p2, p3};
  }
  return (Object){
    .type = type,
    .position = position,
    .color = color,
    .triangles = triangles,
    .triangles_len = mesh.triangleCount,
    .as = data,
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
    float delta = Vector3Distance(p1->pos, p2->pos) - spring->rest_length;
    Vector3 force = Vector3Scale(dir, spring->k * delta);
    spring_forces[spring->p1] = Vector3Add(spring_forces[spring->p1], force);
    spring_forces[spring->p2] = Vector3Subtract(spring_forces[spring->p2], force);
  }
  for (int i = 0; i < cloth->points_len; i++) {
    Point *point = &cloth->points[i];
    Vector3 force = Vector3Add(spring_forces[i], (Vector3){0.0f, -point->mass * G, 0.0f});
    point->acceleration = Vector3Scale(force, 1.0f / point->mass);
  }
}

void move_cloth(Cloth *cloth, float dt) {
  for (int i = 0; i < cloth->points_len; i++) {
    Point *point = &cloth->points[i];
    Vector3 tmp1 = Vector3Scale(point->pos, 2.0f);
    Vector3 tmp2 = Vector3Scale(point->acceleration, dt * dt);
    Vector3 tmp3 = point->prev_pos;
    point->prev_pos = point->pos;
    point->pos = Vector3Add(Vector3Subtract(tmp1, tmp3), tmp2);
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
      DrawSphere(object->position, object->as.sphere.radius, object->color);
      break;
    case OBJECT_RECTANGLE:
      DrawCube(object->position, object->as.rectangle.width, object->as.rectangle.height, object->as.rectangle.length, object->color);
      break;
    }
  }
}

bool get_intersection_point(Vector3 p1, Vector3 p2, Vector3 a, Vector3 b, Vector3 c, Vector3 *dest) {
  Vector3 ab = Vector3Subtract(b, a);
  Vector3 ac = Vector3Subtract(c, a);
  Vector3 n = Vector3Normalize(Vector3CrossProduct(ab, ac));
  Vector3 v = Vector3Subtract(p2, p1);
  float denom = Vector3DotProduct(v, n);
  if (fabsf(denom) < EPSILON) return false;
  Vector3 tmp = Vector3Subtract(a, p1);
  float num = Vector3DotProduct(tmp, n);
  float t = num / denom;
  if (t < 0 || t > 1) return false;
  Vector3 x = Vector3Add(p1, Vector3Scale(v, t));
  Vector3 n1 = Vector3CrossProduct(Vector3Subtract(b, a), Vector3Subtract(x, a));
  Vector3 n2 = Vector3CrossProduct(Vector3Subtract(c, b), Vector3Subtract(x, b));
  Vector3 n3 = Vector3CrossProduct(Vector3Subtract(a, c), Vector3Subtract(x, c));
  float d1 = Vector3DotProduct(n1, n);
  float d2 = Vector3DotProduct(n2, n);
  float d3 = Vector3DotProduct(n3, n);
  if ((d1 >= 0 && d2 >= 0 && d3 >= 0) || (d1 <= 0 && d2 <= 0 && d3 <= 0)) {
    *dest = x;
    return true;
  }
  return false;
}

void check_collision(Point *point, Triangle *triangle) {
  Vector3 x;
  if (!get_intersection_point(point->prev_pos, point->pos, triangle->p1, triangle->p2, triangle->p3, &x)) return;
  Vector3 dir = Vector3Subtract(point->pos, point->prev_pos);
  float dis = Vector3Distance(point->pos, x);
  Vector3 offset = Vector3Scale(dir, dis);
  point->pos = Vector3Subtract(point->pos, offset);
}

void check_collisions(const Cloth *cloth, const Object *objects, size_t objects_len) {
  for (int i = 0; i < cloth->points_len; i++) {
    Point *point = &cloth->points[i];
    Vector3 dir = Vector3Normalize(Vector3Subtract(point->pos, point->prev_pos));
    for (int j = 0; j < cloth->polygons_len; j++) {
      Polygon *polygon = &cloth->polygons[j];
      if (polygon->p1 == i || polygon->p2 == i || polygon->p3 == i) continue;
      Triangle triangle = {cloth->points[polygon->p1].pos, cloth->points[polygon->p2].pos, cloth->points[polygon->p3].pos};
      check_collision(point, &triangle);
    }
    for (int j = 0; j < objects_len; j++) {
      const Object *object = &objects[j];
      for (int k = 0; k < object->triangles_len; k++) {
        check_collision(point, &objects->triangles[k]);
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
    create_object(OBJECT_SPHERE, (Vector3){0.0f, 1.0f, 0.0f}, RED, (ObjectData){.sphere.radius = 1.0f}),
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
