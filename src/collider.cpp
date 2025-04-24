#include "collider.h"
#include <cmath>

// PlaneCollider collision: simple bounce + friction against an infinite plane
void PlaneCollider::handleCollision(float dt, Jello& jello) {
    const float epsilon = 0.01f;
    const float coefResist = 0.8f;
    const float coefFriction = 0.7f;
    for (auto& pm : jello.point_masses) {
        // distance from plane along normal: positive means above plane
        float dist = dot(normal, (pm.position - position));
        // only collide when penetrating (dist < epsilon) and moving into the plane
        if (dist < epsilon && dot(normal, pm.velocity) < 0) {
            Vector3D v_norm = normal * dot(pm.velocity, normal);
            Vector3D v_tan = pm.velocity - v_norm;
            pm.position += normal * (epsilon - dist);
            // bounce + keep tangential velocity
            pm.velocity = -coefResist * v_norm + v_tan;
            // compute contact forces from penetration
            float forceAlongN = dot(-pm.forces, normal);
            if (forceAlongN > 0) {
                Vector3D resistF = normal * forceAlongN;
                Vector3D frictionF = -coefFriction * forceAlongN * v_tan;
                pm.forces += (resistF + frictionF);
            }
        }
    }
}

// SphereCollider collision: bounce + friction against a rigid sphere
void SphereCollider::handleCollision(float dt, Jello& jello) {
    const float epsilon = 0.01f;
    const float coefResist = 0.8f;
    const float coefFriction = 0.3f;

    for (auto& pm : jello.point_masses) {
        // vector from sphere center to particle
        Vector3D dir = (pm.position - center).unit();
        float   dist = (pm.position - center).norm();
        // check penetration: dist < radius + epsilon
        if (dist < radius + epsilon && dot(dir, pm.velocity) < 0) {
            // velocity components
            Vector3D v_norm = dir * dot(pm.velocity, dir);
            Vector3D v_tan = pm.velocity - v_norm;
            pm.position = center + dir * (radius + epsilon);
            // bounce + keep tangential component
            pm.velocity = -coefResist * v_norm + v_tan;
            // contact forces
            float forceAlongN = dot(-pm.forces, dir);
            if (forceAlongN > 0) {
                Vector3D resistF = dir * forceAlongN;
                Vector3D frictionF = -coefFriction * forceAlongN * v_tan;
                pm.forces += (resistF + frictionF);
            }
        }
    }
}
