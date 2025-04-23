#include <iostream>
#include <math.h>
#include <random>
#include <vector>
#include "jello.h"
#include <iostream>



// Since particles are stored in a flat vector, this helper translates a 2D (i,j) index on one of the six faces into the correct 1D index. It uses: r = faceOffset + i * edgeCount + j;
// with different faceOffset for bottom, top, front, back, left, right.

Jello::Jello(Vector3D& init_pos, float cubeLength, int numAtEdge, float springconst, float dampingconst){
    this->init_pos = init_pos;
    this->cubeLength = cubeLength;
    this->numatedge = numAtEdge;
    this->kstruct = springconst;
    this->kshear = springconst;
    this->kbend = springconst;
    this->dstruct = dampingconst;
    this->dshear = dampingconst;
    this->dbend = dampingconst;
    this->pointperface= numAtEdge * numAtEdge;
    initializePoint();
    initializeSpring();
}

int Jello::getPointNum() {
    return point_masses.size();
}
int Jello::getSpringNum() {
    return springs.size();
}
int Jello::getNumAtEdge() {
    return numatedge;
}

unsigned int Jello::indexMap(int face, int i, int j) {
    int res = -1;
    int N = numatedge;
    int faceSize = N * N;

    if (face == 0) {               // bottom (z = 0)
        res = faceSize * i + N * j;
    }
    else if (face == 1) {          // top (z = N-1)
        res = faceSize * i + N * j + (N - 1);
    }
    else if (face == 2) {          // front (y = 0)
        res = faceSize * i + j;
    }
    else if (face == 3) {          // back (y = N-1)
        res = faceSize * i + N * (N - 1) + j;
    }
    else if (face == 4) {          // left (x = 0)
        res = N * i + j;
    }
    else if (face == 5) {          // right (x = N-1)
        res = faceSize * (N - 1) + N * i + j;
    }

    return res;
}

PointMass& Jello::getPointMass(int i) { 
    return point_masses[i]; 
}

Spring& Jello::getSpring(int i) {
    return springs[i];
}

void Jello::setSpringCoef(float springconst, e_spring_type spring_type)
{
    if (spring_type == STRUCTURAL) {
        kstruct = springconst;
        updateSpringCoef(kstruct, STRUCTURAL);
    }
    else if (spring_type == SHEARING) {
        kshear = springconst;
        updateSpringCoef(kshear, SHEARING);
    }
    else if (spring_type == BENDING) {
        kbend = springconst;
        updateSpringCoef(kbend, BENDING);
    }
}

void Jello::setDamperCoef(float dampconst, e_spring_type spring_type)
{
    if (spring_type == STRUCTURAL) {
        dstruct = dampconst;
        updateDampCoef(kstruct, STRUCTURAL);
    }
    else if (spring_type == SHEARING) {
        dshear = dampconst;
        updateDampCoef(kshear, SHEARING);
    }
    else if (spring_type == BENDING) {
        dbend = dampconst;
        updateDampCoef(kbend, BENDING);
    }
}

void Jello::resetCube(Vector3D& offset, float& rotate)
{
    float theta = radians(rotate);
    int N = numatedge;
    int faceCount = N * N;
    float spacing = cubeLength / float(N - 1);
    Vector3D axis = Vector3D(1, 0, 1).unit();
    for (int idx = 0; idx < point_masses.size(); ++idx) {
        int i = idx / faceCount;
        int j = (idx / N) % N;
        int k = idx % N;

        Vector3D base{
          (i - (N - 1) * 0.5f) * spacing,
          (j - (N - 1) * 0.5f) * spacing,
          (k - (N - 1) * 0.5f) * spacing
        };

        // Rodrigues' formula: rotate 'base' about 'axis' by theta
        Vector3D rotated =
            base * cos(theta)
            + cross(axis, base) * sin(theta)
            + axis * (dot(axis, base)) * (1 - cos(theta));

        auto &pm = point_masses[idx];
        pm.position = pm.position + offset + rotated;
        pm.velocity = Vector3D(0, 0, 0);
        pm.forces = Vector3D(0, 0, 0);
    }
}

void Jello::addExternalForces(Vector3D& accel) {
    for (int i = 0; i < point_masses.size(), i++) {
        // F = m * a
        PointMass pm = point_masses[i];
        pm.forces += pm.mass * accel;
    }
}

void Jello::updateSpringCoef(const float springCoef, e_spring_type spring_type)
{
    for (int i = 0; i < springs.size(); i++) {
        if (springs[i].spring_type == spring_type) {
            springs[i].springCoef = springCoef;
        }
    }
}

void Jello::updateDampCoef(const float damperCoef, e_spring_type spring_type)
{
    for (int i = 0; i < springs.size(); i++) {
        if (springs[i].spring_type == spring_type) {
            springs[i].damperCoef = damperCoef;
        }
    }
}

Vector3D Jello::computeSpringForce(Vector3D& pos_a, Vector3D& pos_b, float k, float restLength)
{
    return Vector3D();
}

Vector3D Jello::computeDampForce(Vector3D& pos_a, Vector3D& pos_b, Vector3D& vel_a, Vector3D& vel_b, float dampk)
{
    return Vector3D();
}
