#include "jello.h"
#include "collider.h"
class MassSpringSystem {
public:
    bool isDrawingCube;
    bool isDrawingStruct;
    bool isDrawingShear;
    bool isDrawingBending;
    bool isSimulating;
    int cubeCount;             
    int numatedge;
    int cubeID;
    float delt;
    float kstruct;
    float kshear;
    float kbend;
    float dstruct;
    float dshear;
    float dbend;
    float rotation;
    float cubeLength;

    Vector3D pos;
    Vector3D gravity;
    std::vector<Jello> jellos;

public:
    MassSpringSystem();

    void setSpringCoef(const float springCoef, e_spring_type springType);
    void setDamperCoef(const float damperCoef, e_spring_type springType);

    void simulateone();
    void computeAllForce();
    void reset();

private:
    shared_ptr<Collider> collider;
    void initializeJello();
    void eulerstep();
};