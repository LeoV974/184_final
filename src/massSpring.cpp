#include <iostream>
#include <math.h>
#include <random>
#include <vector>
#include "massSpring.h"
#include "collider.h"
constexpr float delt = 0.001f;
constexpr float k = 2000.0f;
constexpr float d = 60.0f;
MassSpringSystem::MassSpringSystem(){
    this->isDrawingCube = true;
    this->isDrawingStruct = false;
    this->isDrawingShear = false;
    this->isDrawingBending = false;
    this->isSimulating = false;
    this->cubeCount = 1;
    this->numatedge = 10;
    this->cubeID = 1;
    this->delt = delt;
    this->kstruct = k;
    this->kshear = k;
    this->kbend = k;
    this->dstruct = d;
    this->dshear = d;
    this->dbend = d;
    this->rotation = 0;
    this->cubeLength = 4.0;
    this->collider = std::make_shared<PlaneCollider>();

    this->pos = Vector3D(0, 5, 0);
    this->gravity = Vector3D(0, -9.8, 0);
    initializeJello();
    reset();
}


void MassSpringSystem::setSpringCoef(const float springCoef, e_spring_type springType)
{
    for (int i = 0; i < cubeCount; i++) {
        if (springType == STRUCTURAL) {
            jellos[i].setSpringCoef(springCoef, STRUCTURAL);
        }
        else if (springType == SHEARING) {
            jellos[i].setSpringCoef(springCoef, SHEARING);
        }
        else if (springType == BENDING) {
            jellos[i].setSpringCoef(springCoef, BENDING);
        }
    }
}

void MassSpringSystem::setDamperCoef(const float damperCoef, e_spring_type springType)
{
    for (int i = 0; i < cubeCount; i++) {
        if (springType == STRUCTURAL) {
            jellos[i].setDamperCoef(damperCoef, STRUCTURAL);
        }
        else if (springType == SHEARING) {
            jellos[i].setDamperCoef(damperCoef, SHEARING);
        }
        else if (springType == BENDING) {
            jellos[i].setDamperCoef(damperCoef, BENDING);
        }
    }
}

void MassSpringSystem::reset() {
    for (int i = 0; i < cubeCount; i++) {
        jellos[i].resetCube(pos, rotation);
    }
}

void MassSpringSystem::simulateone() {
    if (isSimulating) {
        eulerstep();
    }
}


void MassSpringSystem::initializeJello() {
    for (int i = 0; i < cubeCount; i++) {
        Jello NewJello(Vector3D(0.0f, cubeLength, 0.0f - (2.0f * cubeLength * i)), cubeLength,
            numatedge, kstruct, dstruct);
        jellos.push_back(NewJello);
    }
}

void MassSpringSystem::computeAllForce() {
    for (int i = 0; i < cubeCount; i++) {
        Jello& j = (jellos[i]);
        j.addExternalForces(gravity);
        j.computeInternalForce();
        collider->handleCollision(delt, j);
    }
}

void MassSpringSystem::eulerstep() {
    // zero out
    for (auto& jello : jellos) {
        for (auto& pm : jello.point_masses) {
            pm.forces = Vector3D(0.0f, 0.0f, 0.0f);
        }
    }
    computeAllForce();
    for (auto& jello : jellos) {
        for (auto& pm : jello.point_masses) {
            Vector3D a = pm.forces / pm.mass;
            pm.velocity += a * delt;
            pm.position += pm.velocity * delt;
        }
    }
}
