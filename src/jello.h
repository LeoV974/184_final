#ifndef JELLO_H
#define JELLO_H

#include <vector>
#include <unordered_map>


#include "CGL/misc.h"
#include "pointMass.h"
#include "spring.h"

using namespace CGL;
using namespace std;


class Jello {
public:
    Jello(Vector3D& init, float cubeLength, int numAtEdge, float springconst, float dampingconst);
    int getPointNum();  // return number of masses in the cube
    int getSpringNum();    // return number of springs in the cube
    int getNumAtEdge();    // return number of masses at edge
    // return index used to access particle at face
    unsigned int indexMap(int face, int i, int j);
    // get a particle in container according to index
    PointMass& getPointMass(int i);
    // get a spring in container according to index
    Spring& getSpring(int springi);

    void setSpringCoef(float springconst, e_spring_type spring_type);
    void setDamperCoef(float dampconst, e_spring_type spring_type);

    // set rotation and offset of the Jello
    void resetCube(Vector3D& offset, float& rotate);
    // add gravity
    void addExternalForces(Vector3D& accel);
    void computeInternalForce();
    std::vector<PointMass> point_masses;
private:
    Vector3D init_pos;
    float kstruct;
    float kshear;
    float kbend;
    float dstruct;
    float dshear;
    float dbend;

    int numatedge;  // number of particles at cube's edge
    int pointperface;  // number of particles at cube's face
    float cubeLength;

    
    std::vector<Spring> springs;

    void initializePoint();
    void initializeSpring();

    void updateSpringCoef(const float springCoef, e_spring_type spring_type);
    void updateDampCoef(const float damperCoef, e_spring_type spring_type);

    Vector3D computeSpringForce(Vector3D &pos_a, Vector3D &pos_b, float k, float restLength);
    Vector3D computeDampForce(Vector3D& pos_a, Vector3D& pos_b, Vector3D& vel_a, Vector3D& vel_b, float dampk);

private:
    void initalizeSpringBending();
    void initalizeSpringStruct();
    void initializeSpringShear();
    void initializeSpringShearLine(int idx, int i, int j, int k);
};
#endif /* JELLO_H */