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
        updateDampCoef(dstruct, STRUCTURAL);
    }
    else if (spring_type == SHEARING) {
        dshear = dampconst;
        updateDampCoef(dshear, SHEARING);
    }
    else if (spring_type == BENDING) {
        dbend = dampconst;
        updateDampCoef(dbend, BENDING);
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
    // F = ma
    for (auto& pm : point_masses) {
        pm.forces += pm.mass * accel;
    }
}

void Jello::computeInternalForce()
// Loop over every spring, computes both the elastic (Hooke's law) force and the damping force, then apply them to the two particles that the spring connects
{
    for (int i = 0; i < springs.size(); i++) {
        PointMass* A = springs[i].pm_a;
        PointMass* B = springs[i].pm_b;
        Vector3D springf = computeSpringForce(
            A->position,
            B->position,
            springs[i].springCoef,
            springs[i].rest_length
        );
        Vector3D dampf = computeDampForce(
            A->position,
            B->position,
            A->velocity,
            B->velocity,
            springs[i].damperCoef
        );
        // apply to A (+) and B (−), Newton’s 3rd law:
        A->forces += springf + dampf;
        B->forces -= springf + dampf;
    }
}

void Jello::initializePoint()
// (x,y,z) is the position of the particle in local cube-space, and then you translate by init_pos to get its world-space pos
{
    for (int i = 0; i < numatedge; i++) {
        for (int j = 0; j < numatedge; j++) {
            for (int k = 0; k < numatedge; k++) {
                Vector3D pos;
                float x = (float)((i - numatedge / 2) * cubeLength / (numatedge - 1));
                float y = (float)((j - numatedge / 2) * cubeLength / (numatedge - 1));
                float z = (float)((k - numatedge / 2) * cubeLength / (numatedge - 1));
                pos = init_pos + Vector3D(x, y, z);
                point_masses.emplace_back(pos, false);
            }
        }
    }
}

void Jello::initializeSpring()
{
    initalizeSpringStruct();
    initalizeSpringBending();
    initializeSpringShear();
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
    //Hooke's law: F = -k * (dist - restLength) * dir
    Vector3D res(0);
    double dist = (pos_a - pos_b).norm();
    Vector3D dir = (pos_a - pos_b) / dist;
    res = -k * (dist - restLength) * dir;
    return res;
}

Vector3D Jello::computeDampForce(Vector3D& pos_a, Vector3D& pos_b, Vector3D& vel_a, Vector3D& vel_b, float dampk)
{
    Vector3D res(0);
    double dist = (pos_a - pos_b).norm();
    Vector3D dir = (pos_a - pos_b) / dist;
    Vector3D relV = vel_a - vel_b;
    // (vA−vB)·(xA−xB) / |xA−xB|
    double  proj = dot(relV, pos_a - pos_b) / dist;  

    // damping force: F = −dampk * proj * dir
    return -dampk * proj * dir;
}

void Jello::initalizeSpringBending()
{
    int idx = 0;
    // For each lattice point, connect to the neighbor two steps away along each axis
    for (int i = 0; i < numatedge; i++) {
        for (int j = 0; j < numatedge; j++) {
            for (int k = 0; k < numatedge; k++) {
                idx = i * pointperface + j * numatedge + k;
                // along X: (i, j, k) , (i-2, j, k)
                if (i > 1) {
                    int ni = (i - 2) * pointperface + j * numatedge + k;
                    springs.emplace_back(
                        &point_masses[idx],
                        &point_masses[ni],
                        BENDING
                    );
                }
                // along Y: (i, j, k) , (i, j-2, k)
                if (j > 1) {
                    int nj = i * pointperface + (j - 2) * numatedge + k;
                    springs.emplace_back(
                        &point_masses[idx],
                        &point_masses[nj],
                        BENDING
                    );
                }
                // along Z: (i, j, k) , (i, j, k-2)
                if (k > 1) {
                    int nk = i * pointperface + j * numatedge + (k - 2);
                    springs.emplace_back(
                        &point_masses[idx],
                        &point_masses[nk],
                        BENDING
                    );
                }
            }
        }
    }
}

void Jello::initalizeSpringStruct()
{
    int idx = 0;
    for (int i = 0; i < numatedge; i++) {
        for (int j = 0; j < numatedge; j++) {
            for (int k = 0; k < numatedge; k++) {
                idx = i* pointperface + j * numatedge + k;
                if (i > 0) {
                    int ni = (i - 1) * pointperface + j * numatedge + k;
                    springs.emplace_back(
                        &point_masses[idx],
                        &point_masses[ni],
                        STRUCTURAL
                    );
                }
                if (j > 0) {
                    int nj = i * pointperface + (j-1) * numatedge + k;
                    springs.emplace_back(
                        &point_masses[idx],
                        &point_masses[nj],
                        STRUCTURAL
                    );
                }
                if (k > 0) {
                    int nk = i * pointperface + j * numatedge + (k-1);
                    springs.emplace_back(
                        &point_masses[idx],
                        &point_masses[nk],
                        STRUCTURAL
                    );
                }
            }
        }
    }
    
}

void Jello::initializeSpringShear()
{
    int idx = 0;
    for (int i = 0; i < numatedge; ++i) {
        for (int j = 0; j < numatedge; ++j) {
            for (int k = 0; k < numatedge; ++k) {
                idx = i * pointperface + j * numatedge + k;
                initializeSpringShearLine(idx, i, j, k);
            }
        }
    }
}

void Jello::initializeSpringShearLine(int idx, int i, int j, int k)
/*

      4--------5
     /|       /|            j
    6--------7-|            |
    | 0------|-1            . - i
    |/       |/           /
    2--------3          k

 */
/*
0–6: diagonal in the Y-Z plane “back and up”

0–7: body-diagonal (across X, Y, Z)

0–5: diagonal in the X-Z plane “right and up”

0–3: diagonal in the X-Y plane “right and forward”

2–4, 2–1, 2–5: the three diagonals emanating from vertex 2 (the “bottom front left” corner)

1–6, 4–3, 4–1: the remaining three to cover all opposite corners in that cell
*/
{
    //0-6
    if (k != numatedge - 1 && j != numatedge - 1) {
        int n06 = idx + numatedge + 1;
        springs.emplace_back(
            &point_masses[idx],
            &point_masses[n06],
            SHEARING
        );
    }
    //0-7
    if (k != numatedge - 1 && j != numatedge - 1 && i != numatedge - 1) {
        int n07 = idx + pointperface + numatedge + 1;
        springs.emplace_back(
            &point_masses[idx],
            &point_masses[n07],
            SHEARING
        );
    }
    //0-5
    if (j != numatedge - 1 && i != numatedge - 1) {
        int n05 = idx + pointperface + numatedge;
        springs.emplace_back(
            &point_masses[idx],
            &point_masses[n05],
            SHEARING
        );
    }
    //0-3
    if (k != numatedge - 1 && i != numatedge - 1) {
        int n03 = idx + pointperface + 1;
        springs.emplace_back(
            &point_masses[idx],
            &point_masses[n03],
            SHEARING
        );
    }
    // 2-4
    if (k != 0 && j != numatedge - 1)
    {
        int n24 = idx + numatedge - 1;
        springs.emplace_back(
            &point_masses[idx],
            &point_masses[n24],
            SHEARING
        );
    }

    // 2-1
    if (k != 0 && i != numatedge - 1)
    {
        int n21 = idx + pointperface - 1;
        springs.emplace_back(
            &point_masses[idx],
            &point_masses[n21],
            SHEARING
        );
    }

    // 2-5
    if (k != 0 && j != numatedge - 1 && i != numatedge - 1)
    {
        int n25 = idx + numatedge + pointperface - 1;
        springs.emplace_back(
            &point_masses[idx],
            &point_masses[n25],
            SHEARING
        );
    }

    // 1-6
    if (k != numatedge - 1 && j != numatedge - 1 && i != 0)
    {
        int n16 = idx + numatedge - pointperface + 1;
        springs.emplace_back(
            &point_masses[idx],
            &point_masses[n16],
            SHEARING
        );
    }

    // 4-3
    if (k != numatedge - 1 && j != 0 && i != numatedge - 1)
    {
        int n43 = idx - numatedge + pointperface + 1;
        springs.emplace_back(
            &point_masses[idx],
            &point_masses[n43],
            SHEARING
        );
    }

    // 4-1
    if (j != 0 && i != numatedge - 1)
    {
        int n41 = idx - numatedge + pointperface;
        springs.emplace_back(
            &point_masses[idx],
            &point_masses[n41],
            SHEARING
        );
    }
}
