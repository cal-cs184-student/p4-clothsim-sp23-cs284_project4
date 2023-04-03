#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with spheres.
  Vector3D direction = pm.position - origin;
  double distance_from_origin = direction.norm();

  if (distance_from_origin <= radius) {
    Vector3D tangent_point = origin + radius * direction.unit();
    Vector3D correction_vector = tangent_point - pm.last_position;
    pm.position = pm.last_position + (1 - friction) * correction_vector;
  }

}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_sphere_mesh.draw_sphere(shader, origin, radius * 0.92);
}
