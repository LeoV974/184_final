#pragma once
#include "jello.h"

enum class ColliderType { Plane, Sphere };

// Base interface for all colliders
class Collider {
public:
    virtual ~Collider() = default;
    virtual ColliderType getType() const = 0;
    // Apply collision response to the Jello object over time dt
    virtual void handleCollision(float dt, Jello& jello) = 0;
};

// Simple infinite plane collider
class PlaneCollider : public Collider {
public:
    // Construct with plane position and normal (defaults to horizontal plane at y = -1)
    PlaneCollider(
        const Vector3D& pos = Vector3D(0.0f, -1.0f, 0.0f),
        const Vector3D& normal = Vector3D(0.0f, 1.0f, 0.0f)
    ) : position(pos), normal(normal.unit()) {
    }

    ColliderType getType() const override { return ColliderType::Plane; }
    void handleCollision(float dt, Jello& jello) override;

private:
    Vector3D position;  // a point on the plane
    Vector3D normal;    // plane normal, must be unit length
};

// Simple sphere collider
class SphereCollider : public Collider {
public:
    // Construct with sphere center and radius (defaults to center at y=-1, radius 3)
    SphereCollider(
        const Vector3D& center = Vector3D(0.0f, -1.0f, 0.0f),
        float radius = 3.0f
    ) : center(center), radius(radius) {
    }

    ColliderType getType() const override { return ColliderType::Sphere; }
    void handleCollision(float dt, Jello& jello) override;

private:
    Vector3D center;  // sphere center in world space
    float    radius;  // sphere radius
};