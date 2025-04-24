# Final Project 184, Jello Simulation 
A quick guide to the core components, their responsibilities, and the main simulation flow.

---

## Components

### PointMass
- Stores: position, velocity, force accumulator, pinned state.

### Spring
- Connects two **PointMass** objects.
- Stores: rest length, type (STRUCTURAL, SHEARING, BENDING), spring (`k`) & damper (`d`) coefficients.

### Jello
- Builds a cubic lattice of point masses.
- Initializes three spring topologies: structural, shear, bending.
- Computes internal forces (Hooke’s law + damping).
- Applies external forces (e.g., gravity).
- Provides methods to reset/rotate the cube.
- Exposes `setSpringCoef(...)` and `setDamperCoef(...)` APIs for runtime coefficient changes.

### Collider (abstract)
- Declares `handleCollision(float dt, Jello &jello)`.
- Defines interface for penetration correction, bounce, and friction.

### PlaneCollider
- Infinite plane collider implementation.
- Projects particles onto plane, corrects penetration, reflects velocity, adds friction.

### SphereCollider
- Rigid sphere collider implementation.
- Computes normal from sphere center, resolves penetration, bounce, and friction.

### MassSpringSystem
- Manages one or more **Jello** cubes.
- Each frame:
  1. Zeros all point-mass forces.
  2. Applies gravity, internal (spring + damping) forces, and collision response.
  3. Integrates motion via explicit Euler.
- Holds rendering flags (`isDrawingStruct`, etc.) for visual debugging.

---

## Simulation Flow

### 1. Initialization
- **Constructor** of `MassSpringSystem`:
  - Sets defaults (`delt`, `kstruct`, `gravity`, etc.).
  - Calls `initializeJello()` to build cube(s).
  - Instantiates a `Collider` (e.g. `PlaneCollider`).

### 2. Per-Frame Step
- **simulateone()**:
  - If `isSimulating` is true, invoke `eulerstep()`.

#### Euler Step (`eulerstep`)
1. **Zero Forces**
   ```cpp
   for (auto &j : jellos)
     for (auto &pm : j.point_masses)
       pm.forces = {0,0,0};
