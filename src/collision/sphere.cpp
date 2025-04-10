#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with spheres.
	Vector3D dir = pm.position - this->origin;
	if (dir.norm2() < this->radius2) {
		dir.normalize();
		Vector3D tang = dir * this->radius + this->origin;
		Vector3D corr = tang - pm.last_position;
		pm.position = (1 - this->friction) * corr + pm.last_position;
	}
}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_sphere_mesh.draw_sphere(shader, origin, radius * 0.92);
}
