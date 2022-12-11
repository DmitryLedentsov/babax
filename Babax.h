

#if !defined(BABAX_H)
#define BABAX_H

// #define BABAX_STATIC

// #define  BABAX_STANDALONE
 #define  BABAX_DEBUG

#if defined(BABAX_STATIC)
    #define BABAXDEF inline static            // Functions just visible to module including this file
#else
    #if defined(__cplusplus)
        #define BABAXDEF inline extern "C"    // Functions visible from other files (no name mangling of functions in C++)
    #else
        #define BABAXDEF extern inline        // Functions visible from other files
    #endif
#endif

//----------------------------------------------------------------------------------
// Defines and Macros
//----------------------------------------------------------------------------------
#define     BABAX_MAX_BODIES               64
#define     BABAX_MAX_MANIFOLDS            4096
#define     BABAX_MAX_VERTICES             24
#define     BABAX_CIRCLE_VERTICES          24

#define     BABAX_COLLISION_ITERATIONS     1000 //100
#define     BABAX_PENETRATION_ALLOWANCE    0.05f
#define     BABAX_PENETRATION_CORRECTION   0.4f

#define     BABAX_PI                       3.14159265358979323846
#define     BABAX_DEG2RAD                  (BABAX_PI/180.0f)

#define     BABAX_MALLOC(size)             malloc(size)
#define     BABAX_FREE(ptr)                free(ptr)

#define BABAX_STANDALONE
#define BABAX_IMPLEMENTATION

//----------------------------------------------------------------------------------
// Types and Structures Definition
//----------------------------------------------------------------------------------
#if defined(BABAX_STANDALONE)
    // Vector2 type
    typedef struct Vector2 {
        float x;
        float y;
    } Vector2;

    // Boolean type
    #if !defined(_STDBOOL_H)
        typedef enum { false, true } bool;
        #define _STDBOOL_H
    #endif
#endif



typedef enum PhysicsBodyType { PHYSICS_STATIC, PHYSICS_DYNAMIC } PhysicsBodyType;
typedef enum PhysicsShapeType { PHYSICS_CIRCLE, PHYSICS_POLYGON } PhysicsShapeType;

// Previously defined to be used in PhysicsShape struct as circular dependencies
typedef struct PhysicsBodyData *PhysicsBody;

// Mat2 type (used for polygon shape rotation matrix)
typedef struct Mat2 {
    float m00;
    float m01;
    float m10;
    float m11;
} Mat2;

typedef struct PolygonData {
    unsigned int vertexCount;                   // Current used vertex and normals count
    Vector2 positions[BABAX_MAX_VERTICES];     // Polygon vertex positions vectors
    Vector2 normals[BABAX_MAX_VERTICES];       // Polygon vertex normals vectors
} PolygonData;

typedef struct PhysicsShape {
    PhysicsShapeType type;                      // Physics shape type (circle or polygon)
    PhysicsBody body;                           // Shape physics body reference
    float radius;                               // Circle shape radius (used for circle shapes)
    Mat2 transform;                             // Vertices transform matrix 2x2
    PolygonData vertexData;                     // Polygon shape vertices position and normals data (just used for polygon shapes)
} PhysicsShape;

typedef struct PhysicsBodyData {
    unsigned int id;                            // Reference unique identifier
    bool enabled;                               // Enabled dynamics state (collisions are calculated anyway)
    Vector2 position;                           // Physics body shape pivot
    Vector2 velocity;                           // Current linear velocity applied to position
    Vector2 force;                              // Current linear force (reset to 0 every step)
    float angularVelocity;                      // Current angular velocity applied to orient
    float torque;                               // Current angular force (reset to 0 every step)
    float orient;                               // Rotation in radians
    float inertia;                              // Moment of inertia
    float inverseInertia;                       // Inverse value of inertia
    float mass;                                 // Physics body mass
    float inverseMass;                          // Inverse value of mass
    float staticFriction;                       // Friction when the body has not movement (0 to 1)
    float dynamicFriction;                      // Friction when the body has movement (0 to 1)
    float restitution;                          // Restitution coefficient of the body (0 to 1)
    bool useGravity;                            // Apply gravity force to dynamics
    bool isGrounded;                            // Physics grounded on other body state
    bool freezeOrient;                          // Physics rotation constraint
    PhysicsShape shape;                         // Physics body shape information (type, radius, vertices, normals)
} PhysicsBodyData;

typedef struct PhysicsManifoldData {
    unsigned int id;                            // Reference unique identifier
    PhysicsBody bodyA;                          // Manifold first physics body reference
    PhysicsBody bodyB;                          // Manifold second physics body reference
    float penetration;                          // Depth of penetration from collision
    Vector2 normal;                             // Normal direction vector from 'a' to 'b'
    Vector2 contacts[2];                        // Points of contact during collision
    unsigned int contactsCount;                 // Current collision number of contacts
    float restitution;                          // Mixed restitution during collision
    float dynamicFriction;                      // Mixed dynamic friction during collision
    float staticFriction;                       // Mixed static friction during collision
} PhysicsManifoldData, *PhysicsManifold;

typedef struct {
    unsigned int usedMemory;                         // Total allocated dynamic memory
    bool physicsThreadEnabled;                   // Physics thread enabled state
    double baseTime;                               // Offset time for MONOTONIC clock
    double startTime;                              // Start time in milliseconds
    double deltaTime;             // Delta time used for physics steps, in milliseconds
    double currentTime;                            // Current time in milliseconds
    uint64_t frequency;                              // Hi-res clock freq   
    double accumulator;                            // Physics time step delta time accumulator
    unsigned int stepsCount;                         // Total physics steps processed
    Vector2 gravityForce;              // Physics world gravity force
    PhysicsBody bodies[BABAX_MAX_BODIES];               // Physics bodies pointers array
    unsigned int physicsBodiesCount;                 // Physics world current bodies counter

}PhysicsStateData, *PhysicsState;


#if defined(__cplusplus)
extern "C" {
#endif

//----------------------------------------------------------------------------------
// Module Functions Declaration
//----------------------------------------------------------------------------------
BABAXDEF void InitPhysics(PhysicsState state);                                                                           // Initializes physics values, pointers and creates physics loop thread
BABAXDEF void RunPhysicsStep(PhysicsState state);                                                                        // Run physics step, to be used if PHYSICS_NO_THREADS is set in your main loop
BABAXDEF void SetPhysicsTimeStep(PhysicsState state, double delta);                                                            // Sets physics fixed time step in milliseconds. 1.666666 by default
BABAXDEF bool IsPhysicsEnabled(PhysicsState state);                                                                      // Returns true if physics thread is currently enabled
BABAXDEF void SetPhysicsGravity(PhysicsState state, float x, float y);                                                         // Sets physics global gravity force
BABAXDEF PhysicsBody CreatePhysicsBodyCircle(PhysicsState state, Vector2 pos, float radius, float density);                    // Creates a new circle physics body with generic parameters
BABAXDEF PhysicsBody CreatePhysicsBodyRectangle(PhysicsState state,Vector2 pos, float width, float height, float density);    // Creates a new rectangle physics body with generic parameters
BABAXDEF PhysicsBody CreatePhysicsBodyPolygon(PhysicsState state, Vector2 pos, float radius, int sides, float density);        // Creates a new polygon physics body with generic parameters
BABAXDEF void PhysicsAddForce(PhysicsBody body, Vector2 force);                                            // Adds a force to a physics body
BABAXDEF void PhysicsAddTorque(PhysicsBody body, float amount);                                            // Adds an angular force to a physics body
BABAXDEF void PhysicsShatter(PhysicsState state, PhysicsBody body, Vector2 position, float force);                             // Shatters a polygon shape physics body to little physics bodies with explosion force
BABAXDEF int GetPhysicsBodiesCount(PhysicsState state);                                                                  // Returns the current amount of created physics bodies
BABAXDEF PhysicsBody GetPhysicsBody(PhysicsState state, int index);                                                            // Returns a physics body of the bodies pool at a specific index
BABAXDEF int GetPhysicsShapeType(PhysicsState state, int index);                                                               // Returns the physics body shape type (PHYSICS_CIRCLE or PHYSICS_POLYGON)
BABAXDEF int GetPhysicsShapeVerticesCount(PhysicsState state, int index);                                                      // Returns the amount of vertices of a physics body shape
BABAXDEF Vector2 GetPhysicsShapeVertex(PhysicsBody body, int vertex);                                      // Returns transformed position of a body shape (body position + vertex transformed position)
BABAXDEF void SetPhysicsBodyRotation(PhysicsBody body, float radians);                                     // Sets physics body shape transform based on radians parameter
BABAXDEF void DestroyPhysicsBody(PhysicsState state, PhysicsBody body);                                                        // Unitializes and destroy a physics body
BABAXDEF void ClosePhysics(PhysicsState state);                                                                          // Unitializes physics pointers and closes physics loop thread

#if defined(__cplusplus)
}
#endif

#endif // BABAX_H

/***********************************************************************************
*
*   BABAX IMPLEMENTATION
*
************************************************************************************/

#if defined(BABAX_IMPLEMENTATION)


#if defined(BABAX_DEBUG)
    #include <stdio.h>              // Required for: printf()
#endif

#include <stdlib.h>                 // Required for: malloc(), free(), srand(), rand()
#include <math.h>                   // Required for: cosf(), sinf(), fabs(), sqrtf()
#include <stdint.h>                 // Required for: uint64_t




//----------------------------------------------------------------------------------
// Defines and Macros
//----------------------------------------------------------------------------------
#define     min(a,b)                    (((a)<(b))?(a):(b))
#define     max(a,b)                    (((a)>(b))?(a):(b))
#define     BABAX_FLT_MAX              3.402823466e+38f
#define     BABAX_EPSILON              0.000001f
#define     BABAX_K                    1.0f/3.0f
#define     BABAX_VECTOR_ZERO          (Vector2){ 0.0f, 0.0f }


//----------------------------------------------------------------------------------
// Global Variables Definition
//----------------------------------------------------------------------------------


//----------------------------------------------------------------------------------
// Module Internal Functions Declaration
//----------------------------------------------------------------------------------
static int FindAvailableBodyIndex();                                                                        // Finds a valid index for a new physics body initialization
static PolygonData CreateRandomPolygon(float radius, int sides);                                            // Creates a random polygon shape with max vertex distance from polygon pivot
static PolygonData CreateRectanglePolygon(Vector2 pos, Vector2 size);                                       // Creates a rectangle polygon shape based on a min and max positions
static void *PhysicsLoop(PhysicsState state,void *arg);                                                                        // Physics loop thread function
static void PhysicsStep(PhysicsState state);                                                                              // Physics steps calculations (dynamics, collisions and position corrections)
static int FindAvailableManifoldIndex(PhysicsState state);                                                                    // Finds a valid index for a new manifold initialization
static PhysicsManifoldData CreatePhysicsManifold( PhysicsBody a, PhysicsBody b);                                 // Creates a new physics manifold to solve collision
static void SolvePhysicsManifold(PhysicsManifold manifold);                                                 // Solves a created physics manifold between two physics bodies
static void SolveCircleToCircle(PhysicsManifold manifold);                                                  // Solves collision between two circle shape physics bodies
static void SolveCircleToPolygon(PhysicsManifold manifold);                                                 // Solves collision between a circle to a polygon shape physics bodies
static void SolvePolygonToCircle(PhysicsManifold manifold);                                                 // Solves collision between a polygon to a circle shape physics bodies
static void SolveDifferentShapes(PhysicsManifold manifold, PhysicsBody bodyA, PhysicsBody bodyB);           // Solve collision between two different types of shapes
static void SolvePolygonToPolygon(PhysicsManifold manifold);                                                // Solves collision between two polygons shape physics bodies
static void IntegratePhysicsForces(PhysicsState state, PhysicsBody body);                                                       // Integrates physics forces into velocity
static void InitializePhysicsManifold(PhysicsState state, PhysicsManifold manifold);                                           // Initializes physics manifolds to solve collisions
static void IntegratePhysicsImpulses(PhysicsManifold manifold);                                             // Integrates physics collisions impulses to solve collisions
static void IntegratePhysicsVelocity(PhysicsState state,PhysicsBody body);                                                     // Integrates physics velocity into position and forces
static void CorrectPhysicsPositions(PhysicsManifold manifold);                                              // Corrects physics bodies positions based on manifolds collision information
static float FindAxisLeastPenetration(int *faceIndex, PhysicsShape shapeA, PhysicsShape shapeB);            // Finds polygon shapes axis least penetration
static void FindIncidentFace(Vector2 *v0, Vector2 *v1, PhysicsShape ref, PhysicsShape inc, int index);      // Finds two polygon shapes incident face
static int Clip(Vector2 normal, float clip, Vector2 *faceA, Vector2 *faceB);                                // Calculates clipping based on a normal and two faces
static bool BiasGreaterThan(float valueA, float valueB);                                                    // Check if values are between bias range
static Vector2 TriangleBarycenter(Vector2 v1, Vector2 v2, Vector2 v3);                                      // Returns the barycenter of a triangle given by 3 points
                                                                      // Get current time measure in milliseconds

// Math functions
static Vector2 MathCross(float value, Vector2 vector);                                                      // Returns the cross product of a vector and a value
static float MathCrossVector2(Vector2 v1, Vector2 v2);                                                      // Returns the cross product of two vectors
static float MathLenSqr(Vector2 vector);                                                                    // Returns the len square root of a vector
static float MathDot(Vector2 v1, Vector2 v2);                                                               // Returns the dot product of two vectors
static inline float DistSqr(Vector2 v1, Vector2 v2);                                                        // Returns the square root of distance between two vectors
static void MathNormalize(Vector2 *vector);                                                                 // Returns the normalized values of a vector

static Vector2 Vector2Add(Vector2 v1, Vector2 v2);                                                          // Returns the sum of two given vectors
static Vector2 Vector2Subtract(Vector2 v1, Vector2 v2);                                                     // Returns the subtract of two given vectors


static Mat2 Mat2Radians(float radians);                                                                     // Creates a matrix 2x2 from a given radians value
static void Mat2Set(Mat2 *matrix, float radians);                                                           // Set values from radians to a created matrix 2x2
static inline Mat2 Mat2Transpose(Mat2 matrix);                                                              // Returns the transpose of a given matrix 2x2
static inline Vector2 Mat2MultiplyVector2(Mat2 matrix, Vector2 vector);                                     // Multiplies a vector by a matrix 2x2

//----------------------------------------------------------------------------------
// Module Functions Definition
//----------------------------------------------------------------------------------
// Initializes physics values, pointers and creates physics loop thread
BABAXDEF void InitPhysics(PhysicsState state)
{
    // Initialize high resolution timer
    state->deltaTime = 1.0/60.0/10.0 * 1000;
    //InitTimer(state);

    #if defined(BABAX_DEBUG)
        printf("[BABAX] physics module initialized successfully\n");
    #endif

    state->accumulator = 0.0;
}

// Returns true if physics thread is currently enabled
BABAXDEF bool IsPhysicsEnabled(PhysicsState state)
{
    return state->physicsThreadEnabled;
}

// Sets physics global gravity force
BABAXDEF void SetPhysicsGravity(PhysicsState state,float x, float y)
{
    state->gravityForce.x = x;
    state->gravityForce.y = y;
}

// Creates a new circle physics body with generic parameters
BABAXDEF PhysicsBody CreatePhysicsBodyCircle(PhysicsState state,Vector2 pos, float radius, float density)
{
    PhysicsBody newBody = (PhysicsBody)BABAX_MALLOC(sizeof(PhysicsBodyData));
    state->usedMemory += sizeof(PhysicsBodyData);

    int newId = FindAvailableBodyIndex(state);
    if (newId != -1)
    {
        // Initialize new body with generic values
        newBody->id = newId;
        newBody->enabled = true;
        newBody->position = pos;
        newBody->velocity = BABAX_VECTOR_ZERO;
        newBody->force = BABAX_VECTOR_ZERO;
        newBody->angularVelocity = 0.0f;
        newBody->torque = 0.0f;
        newBody->orient = 0.0f;
        newBody->shape.type = PHYSICS_CIRCLE;
        newBody->shape.body = newBody;
        newBody->shape.radius = radius;
        newBody->shape.transform = Mat2Radians(0.0f);
        newBody->shape.vertexData = (PolygonData) { 0 };

        newBody->mass = BABAX_PI*radius*radius*density;
        newBody->inverseMass = ((newBody->mass != 0.0f) ? 1.0f/newBody->mass : 0.0f);
        newBody->inertia = newBody->mass*radius*radius;
        newBody->inverseInertia = ((newBody->inertia != 0.0f) ? 1.0f/newBody->inertia : 0.0f);
        newBody->staticFriction = 0.4f;
        newBody->dynamicFriction = 0.2f;
        newBody->restitution = 0.0f;
        newBody->useGravity = true;
        newBody->isGrounded = false;
        newBody->freezeOrient = false;

        // Add new body to bodies pointers array and update bodies count
        state->bodies[state->physicsBodiesCount] = newBody;
        state->physicsBodiesCount++;

        #if defined(BABAX_DEBUG)
            printf("[BABAX] created polygon physics body id %i\n", newBody->id);
        #endif
    }
    #if defined(BABAX_DEBUG)
        else
            printf("[BABAX] new physics body creation failed because there is any available id to use\n");
    #endif

    return newBody;
}

// Creates a new rectangle physics body with generic parameters
BABAXDEF PhysicsBody CreatePhysicsBodyRectangle(PhysicsState state, Vector2 pos, float width, float height, float density)
{
    PhysicsBody newBody = (PhysicsBody)BABAX_MALLOC(sizeof(PhysicsBodyData));
    state->usedMemory += sizeof(PhysicsBodyData);

    int newId = FindAvailableBodyIndex(state);
    if (newId != -1)
    {
        // Initialize new body with generic values
        newBody->id = newId;
        newBody->enabled = true;
        newBody->position = pos;
        newBody->velocity = (Vector2){ 0.0f };
        newBody->force = (Vector2){ 0.0f };
        newBody->angularVelocity = 0.0f;
        newBody->torque = 0.0f;
        newBody->orient = 0.0f;
        newBody->shape.type = PHYSICS_POLYGON;
        newBody->shape.body = newBody;
        newBody->shape.radius = 0.0f;
        newBody->shape.transform = Mat2Radians(0.0f);
        newBody->shape.vertexData = CreateRectanglePolygon(pos, (Vector2){ width, height });

        // Calculate centroid and moment of inertia
        Vector2 center = { 0.0f, 0.0f };
        float area = 0.0f;
        float inertia = 0.0f;

        for (int i = 0; i < newBody->shape.vertexData.vertexCount; i++)
        {
            // Triangle vertices, third vertex implied as (0, 0)
            Vector2 p1 = newBody->shape.vertexData.positions[i];
            int nextIndex = (((i + 1) < newBody->shape.vertexData.vertexCount) ? (i + 1) : 0);
            Vector2 p2 = newBody->shape.vertexData.positions[nextIndex];

            float D = MathCrossVector2(p1, p2);
            float triangleArea = D/2;

            area += triangleArea;

            // Use area to weight the centroid average, not just vertex position
            center.x += triangleArea*BABAX_K*(p1.x + p2.x);
            center.y += triangleArea*BABAX_K*(p1.y + p2.y);

            float intx2 = p1.x*p1.x + p2.x*p1.x + p2.x*p2.x;
            float inty2 = p1.y*p1.y + p2.y*p1.y + p2.y*p2.y;
            inertia += (0.25f*BABAX_K*D)*(intx2 + inty2);
        }

        center.x *= 1.0f/area;
        center.y *= 1.0f/area;

        // Translate vertices to centroid (make the centroid (0, 0) for the polygon in model space)
        // Note: this is not really necessary
        for (int i = 0; i < newBody->shape.vertexData.vertexCount; i++)
        {
            newBody->shape.vertexData.positions[i].x -= center.x;
            newBody->shape.vertexData.positions[i].y -= center.y;
        }

        newBody->mass = density*area;
        newBody->inverseMass = ((newBody->mass != 0.0f) ? 1.0f/newBody->mass : 0.0f);
        newBody->inertia = density*inertia;
        newBody->inverseInertia = ((newBody->inertia != 0.0f) ? 1.0f/newBody->inertia : 0.0f);
        newBody->staticFriction = 0.4f;
        newBody->dynamicFriction = 0.2f;
        newBody->restitution = 0.0f;
        newBody->useGravity = true;
        newBody->isGrounded = false;
        newBody->freezeOrient = false;

        // Add new body to bodies pointers array and update bodies count
        state->bodies[state->physicsBodiesCount] = newBody;
        state->physicsBodiesCount++;

        #if defined(BABAX_DEBUG)
            printf("[BABAX] created polygon physics body id %i\n", newBody->id);
        #endif
    }
    #if defined(BABAX_DEBUG)
        else
            printf("[BABAX] new physics body creation failed because there is any available id to use\n");
    #endif

    return newBody;
}

// Creates a new polygon physics body with generic parameters
BABAXDEF PhysicsBody CreatePhysicsBodyPolygon(PhysicsState state, Vector2 pos, float radius, int sides, float density)
{
    PhysicsBody newBody = (PhysicsBody)BABAX_MALLOC(sizeof(PhysicsBodyData));
    state->usedMemory += sizeof(PhysicsBodyData);

    int newId = FindAvailableBodyIndex(state);
    if (newId != -1)
    {
        // Initialize new body with generic values
        newBody->id = newId;
        newBody->enabled = true;
        newBody->position = pos;
        newBody->velocity = BABAX_VECTOR_ZERO;
        newBody->force = BABAX_VECTOR_ZERO;
        newBody->angularVelocity = 0.0f;
        newBody->torque = 0.0f;
        newBody->orient = 0.0f;
        newBody->shape.type = PHYSICS_POLYGON;
        newBody->shape.body = newBody;
        newBody->shape.transform = Mat2Radians(0.0f);
        newBody->shape.vertexData = CreateRandomPolygon(radius, sides);

        // Calculate centroid and moment of inertia
        Vector2 center = { 0.0f, 0.0f };
        float area = 0.0f;
        float inertia = 0.0f;

        for (int i = 0; i < newBody->shape.vertexData.vertexCount; i++)
        {
            // Triangle vertices, third vertex implied as (0, 0)
            Vector2 position1 = newBody->shape.vertexData.positions[i];
            int nextIndex = (((i + 1) < newBody->shape.vertexData.vertexCount) ? (i + 1) : 0);
            Vector2 position2 = newBody->shape.vertexData.positions[nextIndex];

            float cross = MathCrossVector2(position1, position2);
            float triangleArea = cross/2;

            area += triangleArea;

            // Use area to weight the centroid average, not just vertex position
            center.x += triangleArea*BABAX_K*(position1.x + position2.x);
            center.y += triangleArea*BABAX_K*(position1.y + position2.y);

            float intx2 = position1.x*position1.x + position2.x*position1.x + position2.x*position2.x;
            float inty2 = position1.y*position1.y + position2.y*position1.y + position2.y*position2.y;
            inertia += (0.25f*BABAX_K*cross)*(intx2 + inty2);
        }

        center.x *= 1.0f/area;
        center.y *= 1.0f/area;

        // Translate vertices to centroid (make the centroid (0, 0) for the polygon in model space)
        // Note: this is not really necessary
        for (int i = 0; i < newBody->shape.vertexData.vertexCount; i++)
        {
            newBody->shape.vertexData.positions[i].x -= center.x;
            newBody->shape.vertexData.positions[i].y -= center.y;
        }

        newBody->mass = density*area;
        newBody->inverseMass = ((newBody->mass != 0.0f) ? 1.0f/newBody->mass : 0.0f);
        newBody->inertia = density*inertia;
        newBody->inverseInertia = ((newBody->inertia != 0.0f) ? 1.0f/newBody->inertia : 0.0f);
        newBody->staticFriction = 0.4f;
        newBody->dynamicFriction = 0.2f;
        newBody->restitution = 0.0f;
        newBody->useGravity = true;
        newBody->isGrounded = false;
        newBody->freezeOrient = false;

        // Add new body to bodies pointers array and update bodies count
        state->bodies[state->physicsBodiesCount] = newBody;
        state->physicsBodiesCount++;

        #if defined(BABAX_DEBUG)
            printf("[BABAX] created polygon physics body id %i\n", newBody->id);
        #endif
    }
    #if defined(BABAX_DEBUG)
        else
            printf("[BABAX] new physics body creation failed because there is any available id to use\n");
    #endif

    return newBody;
}

// Adds a force to a physics body
BABAXDEF void PhysicsAddForce(PhysicsBody body, Vector2 force)
{
    if (body != NULL)
        body->force = Vector2Add(body->force, force);
}

// Adds an angular force to a physics body
BABAXDEF void PhysicsAddTorque(PhysicsBody body, float amount)
{
    if (body != NULL)
        body->torque += amount;
}

// Shatters a polygon shape physics body to little physics bodies with explosion force
BABAXDEF void PhysicsShatter(PhysicsState state, PhysicsBody body, Vector2 position, float force)
{
    if (body != NULL)
    {
        if (body->shape.type == PHYSICS_POLYGON)
        {
            PolygonData vertexData = body->shape.vertexData;
            bool collision = false;

            for (int i = 0; i < vertexData.vertexCount; i++)
            {
                Vector2 positionA = body->position;
                Vector2 positionB = Mat2MultiplyVector2(body->shape.transform, Vector2Add(body->position, vertexData.positions[i]));
                int nextIndex = (((i + 1) < vertexData.vertexCount) ? (i + 1) : 0);
                Vector2 positionC = Mat2MultiplyVector2(body->shape.transform, Vector2Add(body->position, vertexData.positions[nextIndex]));

                // Check collision between each triangle
                float alpha = ((positionB.y - positionC.y)*(position.x - positionC.x) + (positionC.x - positionB.x)*(position.y - positionC.y))/
                              ((positionB.y - positionC.y)*(positionA.x - positionC.x) + (positionC.x - positionB.x)*(positionA.y - positionC.y));

                float beta = ((positionC.y - positionA.y)*(position.x - positionC.x) + (positionA.x - positionC.x)*(position.y - positionC.y))/
                             ((positionB.y - positionC.y)*(positionA.x - positionC.x) + (positionC.x - positionB.x)*(positionA.y - positionC.y));

                float gamma = 1.0f - alpha - beta;

                if ((alpha > 0.0f) && (beta > 0.0f) && (gamma > 0.0f))
                {
                    collision = true;
                    break;
                }
            }

            if (collision)
            {
                int count = vertexData.vertexCount;
                Vector2 bodyPos = body->position;
                Vector2 *vertices = (Vector2*)BABAX_MALLOC(sizeof(Vector2) * count);
                Mat2 trans = body->shape.transform;
                
                for (int i = 0; i < count; i++)
                    vertices[i] = vertexData.positions[i];

                // Destroy shattered physics body
                DestroyPhysicsBody(state, body);

                for (int i = 0; i < count; i++)
                {
                    int nextIndex = (((i + 1) < count) ? (i + 1) : 0);
                    Vector2 center = TriangleBarycenter(vertices[i], vertices[nextIndex], BABAX_VECTOR_ZERO);
                    center = Vector2Add(bodyPos, center);
                    Vector2 offset = Vector2Subtract(center, bodyPos);

                    PhysicsBody newBody = CreatePhysicsBodyPolygon(state, center, 10, 3, 10);     // Create polygon physics body with relevant values

                    PolygonData newData = { 0 };
                    newData.vertexCount = 3;

                    newData.positions[0] = Vector2Subtract(vertices[i], offset);
                    newData.positions[1] = Vector2Subtract(vertices[nextIndex], offset);
                    newData.positions[2] = Vector2Subtract(position, center);

                    // Separate vertices to avoid unnecessary physics collisions
                    newData.positions[0].x *= 0.95f;
                    newData.positions[0].y *= 0.95f;
                    newData.positions[1].x *= 0.95f;
                    newData.positions[1].y *= 0.95f;
                    newData.positions[2].x *= 0.95f;
                    newData.positions[2].y *= 0.95f;

                    // Calculate polygon faces normals
                    for (int j = 0; j < newData.vertexCount; j++)
                    {
                        int nextVertex = (((j + 1) < newData.vertexCount) ? (j + 1) : 0);
                        Vector2 face = Vector2Subtract(newData.positions[nextVertex], newData.positions[j]);

                        newData.normals[j] = (Vector2){ face.y, -face.x };
                        MathNormalize(&newData.normals[j]);
                    }

                    // Apply computed vertex data to new physics body shape
                    newBody->shape.vertexData = newData;
                    newBody->shape.transform = trans;

                    // Calculate centroid and moment of inertia
                    center = BABAX_VECTOR_ZERO;
                    float area = 0.0f;
                    float inertia = 0.0f;

                    for (int j = 0; j < newBody->shape.vertexData.vertexCount; j++)
                    {
                        // Triangle vertices, third vertex implied as (0, 0)
                        Vector2 p1 = newBody->shape.vertexData.positions[j];
                        int nextVertex = (((j + 1) < newBody->shape.vertexData.vertexCount) ? (j + 1) : 0);
                        Vector2 p2 = newBody->shape.vertexData.positions[nextVertex];

                        float D = MathCrossVector2(p1, p2);
                        float triangleArea = D/2;

                        area += triangleArea;

                        // Use area to weight the centroid average, not just vertex position
                        center.x += triangleArea*BABAX_K*(p1.x + p2.x);
                        center.y += triangleArea*BABAX_K*(p1.y + p2.y);

                        float intx2 = p1.x*p1.x + p2.x*p1.x + p2.x*p2.x;
                        float inty2 = p1.y*p1.y + p2.y*p1.y + p2.y*p2.y;
                        inertia += (0.25f*BABAX_K*D)*(intx2 + inty2);
                    }

                    center.x *= 1.0f/area;
                    center.y *= 1.0f/area;

                    newBody->mass = area;
                    newBody->inverseMass = ((newBody->mass != 0.0f) ? 1.0f/newBody->mass : 0.0f);
                    newBody->inertia = inertia;
                    newBody->inverseInertia = ((newBody->inertia != 0.0f) ? 1.0f/newBody->inertia : 0.0f);

                    // Calculate explosion force direction
                    Vector2 pointA = newBody->position;
                    Vector2 pointB = Vector2Subtract(newData.positions[1], newData.positions[0]);
                    pointB.x /= 2.0f;
                    pointB.y /= 2.0f;
                    Vector2 forceDirection = Vector2Subtract(Vector2Add(pointA, Vector2Add(newData.positions[0], pointB)), newBody->position);
                    MathNormalize(&forceDirection);
                    forceDirection.x *= force;
                    forceDirection.y *= force;

                    // Apply force to new physics body
                    PhysicsAddForce(newBody, forceDirection);
                }

                BABAX_FREE(vertices);
            }
        }
    }
    #if defined(BABAX_DEBUG)
        else
            printf("[BABAX] error when trying to shatter a null reference physics body");
    #endif
}

// Returns the current amount of created physics bodies
BABAXDEF int GetPhysicsBodiesCount(PhysicsState state)
{
    return state->physicsBodiesCount;
}

// Returns a physics body of the bodies pool at a specific index
BABAXDEF PhysicsBody GetPhysicsBody(PhysicsState state, int index)
{
    if (index < state->physicsBodiesCount)
    {
        if (state->bodies[index] == NULL)
        {
            #if defined(BABAX_DEBUG)
                printf("[BABAX] error when trying to get a null reference physics body");
            #endif
        }
    }
    #if defined(BABAX_DEBUG)
        else
            printf("[BABAX] physics body index is out of bounds");
    #endif

    return state->bodies[index];
}

// Returns the physics body shape type (PHYSICS_CIRCLE or PHYSICS_POLYGON)
BABAXDEF int GetPhysicsShapeType(PhysicsState state,int index)
{
    int result = -1;

    if (index < state->physicsBodiesCount)
    {
        if (state->bodies[index] != NULL) 
            result = state->bodies[index]->shape.type;

        #if defined(BABAX_DEBUG)
            else
                printf("[BABAX] error when trying to get a null reference physics body");
        #endif
    }
    #if defined(BABAX_DEBUG)
        else
            printf("[BABAX] physics body index is out of bounds");
    #endif

    return result;
}

// Returns the amount of vertices of a physics body shape
BABAXDEF int GetPhysicsShapeVerticesCount(PhysicsState state,int index)
{
    int result = 0;

    if (index < state->physicsBodiesCount)
    {
        if (state->bodies[index] != NULL)
        {
            switch (state, state->bodies[index]->shape.type)
            {
                case PHYSICS_CIRCLE: result = BABAX_CIRCLE_VERTICES; break;
                case PHYSICS_POLYGON: result = state->bodies[index]->shape.vertexData.vertexCount; break;
                default: break;
            }
        }
        #if defined(BABAX_DEBUG)
            else
                printf("[BABAX] error when trying to get a null reference physics body");
        #endif
    }
    #if defined(BABAX_DEBUG)
        else
            printf("[BABAX] physics body index is out of bounds");
    #endif

    return result;
}

// Returns transformed position of a body shape (body position + vertex transformed position)
BABAXDEF Vector2 GetPhysicsShapeVertex(PhysicsBody body, int vertex)
{
    Vector2 position = { 0.0f, 0.0f };

    if (body != NULL)
    {
        switch (body->shape.type)
        {
            case PHYSICS_CIRCLE:
            {
                position.x = body->position.x + cosf(360.0f/BABAX_CIRCLE_VERTICES*vertex*BABAX_DEG2RAD)*body->shape.radius;
                position.y = body->position.y + sinf(360.0f/BABAX_CIRCLE_VERTICES*vertex*BABAX_DEG2RAD)*body->shape.radius;
            } break;
            case PHYSICS_POLYGON:
            {
                PolygonData vertexData = body->shape.vertexData;
                position = Vector2Add(body->position, Mat2MultiplyVector2(body->shape.transform, vertexData.positions[vertex]));
            } break;
            default: break;
        }
    }
    #if defined(BABAX_DEBUG)
        else
            printf("[BABAX] error when trying to get a null reference physics body");
    #endif

    return position;
}

// Sets physics body shape transform based on radians parameter
BABAXDEF void SetPhysicsBodyRotation(PhysicsBody body, float radians)
{
    if (body != NULL)
    {
        body->orient = radians;

        if (body->shape.type == PHYSICS_POLYGON)
            body->shape.transform = Mat2Radians(radians);
    }
}

// Unitializes and destroys a physics body
BABAXDEF void DestroyPhysicsBody(PhysicsState state,PhysicsBody body)
{
    if (body != NULL)
    {
        int id = body->id;
        int index = -1;

        for (int i = 0; i < state->physicsBodiesCount; i++)
        {
            if (state->bodies[i]->id == id)
            {
                index = i;
                break;
            }
        }

        if (index == -1)
        {
            #if defined(BABAX_DEBUG)
                printf("[BABAX] Not possible to find body id %i in pointers array\n", id);
            #endif
            return;
        }

        // Free body allocated memory
        BABAX_FREE(body);
        state->usedMemory -= sizeof(PhysicsBodyData);
        state->bodies[index] = NULL;

        // Reorder physics bodies pointers array and its catched index
        for (int i = index; i < state->physicsBodiesCount; i++)
        {
            if ((i + 1) < state->physicsBodiesCount)
                state->bodies[i] = state->bodies[i + 1];
        }

        // Update physics bodies count
        state->physicsBodiesCount--;

        #if defined(BABAX_DEBUG)
            printf("[BABAX] destroyed physics body id %i\n", id);
        #endif
    }
    #if defined(BABAX_DEBUG)
        else
            printf("[BABAX] error trying to destroy a null referenced body\n");
    #endif
}

// Unitializes physics pointers and exits physics loop thread
BABAXDEF void ClosePhysics(PhysicsState state)
{
    // Exit physics loop thread
    state->physicsThreadEnabled = false;



    // Unitialize physics manifolds dynamic memory allocations
   
    // Unitialize physics bodies dynamic memory allocations
    for (int i = state->physicsBodiesCount - 1; i >= 0; i--)
        DestroyPhysicsBody(state,state->bodies[i]);

    #if defined(BABAX_DEBUG)
        if (state->physicsBodiesCount > 0 || state->usedMemory != 0)
            printf("[BABAX] physics module closed with %i still allocated bodies [MEMORY: %i bytes]\n", state->physicsBodiesCount, state->usedMemory);
        else
            printf("[BABAX] physics module closed successfully\n");
    #endif
}

//----------------------------------------------------------------------------------
// Module Internal Functions Definition
//----------------------------------------------------------------------------------
// Finds a valid index for a new physics body initialization
static int FindAvailableBodyIndex(PhysicsState state)
{
    int index = -1;
    for (int i = 0; i < BABAX_MAX_BODIES; i++)
    {
        int currentId = i;

        // Check if current id already exist in other physics body
        for (int k = 0; k < state->physicsBodiesCount; k++)
        {
            if (state->bodies[k]->id == currentId)
            {
                currentId++;
                break;
            }
        }

        // If it is not used, use it as new physics body id
        if (currentId == i)
        {
            index = i;
            break;
        }
    }

    return index;
}

// Creates a random polygon shape with max vertex distance from polygon pivot
static PolygonData CreateRandomPolygon(float radius, int sides)
{
    PolygonData data = { 0 };
    data.vertexCount = sides;

    // Calculate polygon vertices positions
    for (int i = 0; i < data.vertexCount; i++)
    {
        data.positions[i].x = cosf(360.0f/sides*i*BABAX_DEG2RAD)*radius;
        data.positions[i].y = sinf(360.0f/sides*i*BABAX_DEG2RAD)*radius;
    }

    // Calculate polygon faces normals
    for (int i = 0; i < data.vertexCount; i++)
    {
        int nextIndex = (((i + 1) < sides) ? (i + 1) : 0);
        Vector2 face = Vector2Subtract(data.positions[nextIndex], data.positions[i]);

        data.normals[i] = (Vector2){ face.y, -face.x };
        MathNormalize(&data.normals[i]);
    }

    return data;
}

// Creates a rectangle polygon shape based on a min and max positions
static PolygonData CreateRectanglePolygon(Vector2 pos, Vector2 size)
{
    PolygonData data = { 0 };
    data.vertexCount = 4;

    // Calculate polygon vertices positions
    data.positions[0] = (Vector2){ pos.x + size.x/2, pos.y - size.y/2 };
    data.positions[1] = (Vector2){ pos.x + size.x/2, pos.y + size.y/2 };
    data.positions[2] = (Vector2){ pos.x - size.x/2, pos.y + size.y/2 };
    data.positions[3] = (Vector2){ pos.x - size.x/2, pos.y - size.y/2 };

    // Calculate polygon faces normals
    for (int i = 0; i < data.vertexCount; i++)
    {
        int nextIndex = (((i + 1) < data.vertexCount) ? (i + 1) : 0);
        Vector2 face = Vector2Subtract(data.positions[nextIndex], data.positions[i]);

        data.normals[i] = (Vector2){ face.y, -face.x };
        MathNormalize(&data.normals[i]);
    }

    return data;
}

// Physics loop thread function
static void *PhysicsLoop(PhysicsState state, void *arg)
{
    #if defined(BABAX_DEBUG)
        printf("[BABAX] physics thread created successfully\n");
    #endif

    // Initialize physics loop thread values
    state->physicsThreadEnabled = true;

    // Physics update loop
    while (state->physicsThreadEnabled)
    {
        RunPhysicsStep(state);
    }

    return NULL;
}

inline static void ProcessManifold(PhysicsState state, PhysicsManifold manifold){
    SolvePhysicsManifold(manifold);
    InitializePhysicsManifold(state, manifold);
    CorrectPhysicsPositions(manifold);
    
    for (int i = 0; i < BABAX_COLLISION_ITERATIONS; i++)
    {
        IntegratePhysicsImpulses(manifold); 
    }
}
// Physics steps calculations (dynamics, collisions and position corrections)
static void PhysicsStep(PhysicsState state)
{
    // Update current steps count
    state->stepsCount++;

    // Clear previous generated collisions information

    // Reset physics bodies grounded state
    for (int i = 0; i < state->physicsBodiesCount; i++)
    {
        PhysicsBody body = state->bodies[i];
        body->isGrounded = false;
    }

    // Generate new collision information
    for (int i = 0; i < state->physicsBodiesCount; i++)
    {
        PhysicsBody bodyA = state->bodies[i];

        if (bodyA != NULL)
        {
            for (int j = i + 1; j < state->physicsBodiesCount; j++)
            {
                PhysicsBody bodyB = state->bodies[j];

                if (bodyB != NULL)
                {
                    if ((bodyA->inverseMass == 0) && (bodyB->inverseMass == 0))
                        continue;

                    PhysicsManifoldData manifold = CreatePhysicsManifold(bodyA, bodyB);
                    PhysicsManifoldData manifold2 = manifold;
                    bool flag = manifold.contactsCount > 0;
                    ProcessManifold(state, &manifold);
                    if(flag){
                        ProcessManifold(state, &manifold2);
                    }
                }
            }
        }
    }

    // Integrate forces to physics bodies
    for (int i = 0; i < state->physicsBodiesCount; i++)
    {
        PhysicsBody body = state->bodies[i];
        
        if (body != NULL)
            IntegratePhysicsForces(state,body);
    }



    // Integrate physics collisions impulses to solve collisions
    //

    // Integrate velocity to physics bodies
    for (int i = 0; i < state->physicsBodiesCount; i++)
    {
        PhysicsBody body = state->bodies[i];
        
        if (body != NULL)
            IntegratePhysicsVelocity(state,body);
    }

    // Correct physics bodies positions based on manifolds collision information

    // Clear physics bodies forces
    for (int i = 0; i < state->physicsBodiesCount; i++)
    {
        PhysicsBody body = state->bodies[i];
        
        if (body != NULL)
        {
            body->force = BABAX_VECTOR_ZERO;
            body->torque = 0.0f;
        }
    }
}

// Wrapper to ensure PhysicsStep is run with at a fixed time step
BABAXDEF void RunPhysicsStep(PhysicsState state)
{
    // Calculate current time
    //state->currentTime = GetCurrentTime(state);

    // Calculate current delta time
    const double delta = state->currentTime - state->startTime;

    // Store the time elapsed since the last frame began
    state->accumulator += delta;

    // Fixed time stepping loop
    while (state->accumulator >= state->deltaTime)
    {
        PhysicsStep(state);
        state->accumulator -= state->deltaTime;
    }

    // Record the starting of this frame
    state->startTime = state->currentTime;
}

BABAXDEF void SetPhysicsTimeStep(PhysicsState state, double delta)
{
    state->deltaTime = delta;
}

// Finds a valid index for a new manifold initialization

// Creates a new physics manifold to solve collision
static PhysicsManifoldData CreatePhysicsManifold(PhysicsBody a, PhysicsBody b)
{
    PhysicsManifoldData newManifold = {0};



        // Initialize new manifold with generic values
        newManifold.id = 0;
        newManifold.bodyA = a;
        newManifold.bodyB = b;
        newManifold.penetration = 0;
        newManifold.normal = BABAX_VECTOR_ZERO;
        newManifold.contacts[0] = BABAX_VECTOR_ZERO;
        newManifold.contacts[1] = BABAX_VECTOR_ZERO;
        newManifold.contactsCount = 0;
        newManifold.restitution = 0.0f;
        newManifold.dynamicFriction = 0.0f;
        newManifold.staticFriction = 0.0f;

        // Add new body to bodies pointers array and update bodies count
        //state->contacts[state->physicsManifoldsCount] = newManifold;
        //state->physicsManifoldsCount++;
    


    return newManifold;
}

// Unitializes and destroys a physics manifold


// Solves a created physics manifold between two physics bodies
static void SolvePhysicsManifold(PhysicsManifold manifold)
{
    switch (manifold->bodyA->shape.type)
    {
        case PHYSICS_CIRCLE:
        {
            switch (manifold->bodyB->shape.type)
            {
                case PHYSICS_CIRCLE: SolveCircleToCircle(manifold); break;
                case PHYSICS_POLYGON: SolveCircleToPolygon(manifold); break;
                default: break;
            }
        } break;
        case PHYSICS_POLYGON:
        {
            switch (manifold->bodyB->shape.type)
            {
                case PHYSICS_CIRCLE: SolvePolygonToCircle(manifold); break;
                case PHYSICS_POLYGON: SolvePolygonToPolygon(manifold); break;
                default: break;
            }
        } break;
        default: break;
    }

    // Update physics body grounded state if normal direction is down and grounded state is not set yet in previous manifolds
    if (!manifold->bodyB->isGrounded)
        manifold->bodyB->isGrounded = (manifold->normal.y < 0);
}

// Solves collision between two circle shape physics bodies
static void SolveCircleToCircle(PhysicsManifold manifold)
{
    PhysicsBody bodyA = manifold->bodyA;
    PhysicsBody bodyB = manifold->bodyB;

    if ((bodyA == NULL) || (bodyB == NULL))
        return;

    // Calculate translational vector, which is normal
    Vector2 normal = Vector2Subtract(bodyB->position, bodyA->position);

    float distSqr = MathLenSqr(normal);
    float radius = bodyA->shape.radius + bodyB->shape.radius;

    // Check if circles are not in contact
    if (distSqr >= radius*radius)
    {
        manifold->contactsCount = 0;
        return;
    }

    float distance = sqrtf(distSqr);
    manifold->contactsCount = 1;

    if (distance == 0.0f)
    {
        manifold->penetration = bodyA->shape.radius;
        manifold->normal = (Vector2){ 1.0f, 0.0f };
        manifold->contacts[0] = bodyA->position;
    }
    else
    {
        manifold->penetration = radius - distance;
        manifold->normal = (Vector2){ normal.x/distance, normal.y/distance }; // Faster than using MathNormalize() due to sqrt is already performed
        manifold->contacts[0] = (Vector2){ manifold->normal.x*bodyA->shape.radius + bodyA->position.x, manifold->normal.y*bodyA->shape.radius + bodyA->position.y };
    }

    // Update physics body grounded state if normal direction is down
    if (!bodyA->isGrounded)
        bodyA->isGrounded = (manifold->normal.y < 0);
}

// Solves collision between a circle to a polygon shape physics bodies
static void SolveCircleToPolygon(PhysicsManifold manifold)
{
    PhysicsBody bodyA = manifold->bodyA;
    PhysicsBody bodyB = manifold->bodyB;

    if ((bodyA == NULL) || (bodyB == NULL))
        return;

    SolveDifferentShapes(manifold, bodyA, bodyB);
}

// Solves collision between a circle to a polygon shape physics bodies
static void SolvePolygonToCircle(PhysicsManifold manifold)
{
    PhysicsBody bodyA = manifold->bodyA;
    PhysicsBody bodyB = manifold->bodyB;

    if ((bodyA == NULL) || (bodyB == NULL))
        return;

    SolveDifferentShapes(manifold, bodyB, bodyA);
    
    manifold->normal.x *= -1.0f;
    manifold->normal.y *= -1.0f;
}

// Solve collision between two different types of shapes
static void SolveDifferentShapes(PhysicsManifold manifold, PhysicsBody bodyA, PhysicsBody bodyB)
{
    manifold->contactsCount = 0;

    // Transform circle center to polygon transform space
    Vector2 center = bodyA->position;
    center = Mat2MultiplyVector2(Mat2Transpose(bodyB->shape.transform), Vector2Subtract(center, bodyB->position));

    // Find edge with minimum penetration
    // It is the same concept as using support points in SolvePolygonToPolygon
    float separation = -BABAX_FLT_MAX;
    int faceNormal = 0;
    PolygonData vertexData = bodyB->shape.vertexData;

    for (int i = 0; i < vertexData.vertexCount; i++)
    {
        float currentSeparation = MathDot(vertexData.normals[i], Vector2Subtract(center, vertexData.positions[i]));

        if (currentSeparation > bodyA->shape.radius)
            return;

        if (currentSeparation > separation)
        {
            separation = currentSeparation;
            faceNormal = i;
        }
    }

    // Grab face's vertices
    Vector2 v1 = vertexData.positions[faceNormal];
    int nextIndex = (((faceNormal + 1) < vertexData.vertexCount) ? (faceNormal + 1) : 0);
    Vector2 v2 = vertexData.positions[nextIndex];

    // Check to see if center is within polygon
    if (separation < BABAX_EPSILON)
    {
        manifold->contactsCount = 1;
        Vector2 normal = Mat2MultiplyVector2(bodyB->shape.transform, vertexData.normals[faceNormal]);
        manifold->normal = (Vector2){ -normal.x, -normal.y };
        manifold->contacts[0] = (Vector2){ manifold->normal.x*bodyA->shape.radius + bodyA->position.x, manifold->normal.y*bodyA->shape.radius + bodyA->position.y };
        manifold->penetration = bodyA->shape.radius;
        return;
    }

    // Determine which voronoi region of the edge center of circle lies within
    float dot1 = MathDot(Vector2Subtract(center, v1), Vector2Subtract(v2, v1));
    float dot2 = MathDot(Vector2Subtract(center, v2), Vector2Subtract(v1, v2));
    manifold->penetration = bodyA->shape.radius - separation;

    if (dot1 <= 0.0f) // Closest to v1
    {
        if (DistSqr(center, v1) > bodyA->shape.radius*bodyA->shape.radius)
            return;

        manifold->contactsCount = 1;
        Vector2 normal = Vector2Subtract(v1, center);
        normal = Mat2MultiplyVector2(bodyB->shape.transform, normal);
        MathNormalize(&normal);
        manifold->normal = normal;
        v1 = Mat2MultiplyVector2(bodyB->shape.transform, v1);
        v1 = Vector2Add(v1, bodyB->position);
        manifold->contacts[0] = v1;
    }
    else if (dot2 <= 0.0f) // Closest to v2
    {
        if (DistSqr(center, v2) > bodyA->shape.radius*bodyA->shape.radius)
            return;

        manifold->contactsCount = 1;
        Vector2 normal = Vector2Subtract(v2, center);
        v2 = Mat2MultiplyVector2(bodyB->shape.transform, v2);
        v2 = Vector2Add(v2, bodyB->position);
        manifold->contacts[0] = v2;
        normal = Mat2MultiplyVector2(bodyB->shape.transform, normal);
        MathNormalize(&normal);
        manifold->normal = normal;
    }
    else // Closest to face
    {
        Vector2 normal = vertexData.normals[faceNormal];

        if (MathDot(Vector2Subtract(center, v1), normal) > bodyA->shape.radius)
            return;

        normal = Mat2MultiplyVector2(bodyB->shape.transform, normal);
        manifold->normal = (Vector2){ -normal.x, -normal.y };
        manifold->contacts[0] = (Vector2){ manifold->normal.x*bodyA->shape.radius + bodyA->position.x, manifold->normal.y*bodyA->shape.radius + bodyA->position.y };
        manifold->contactsCount = 1;
    }
}

// Solves collision between two polygons shape physics bodies
static void SolvePolygonToPolygon(PhysicsManifold manifold)
{
    if ((manifold->bodyA == NULL) || (manifold->bodyB == NULL))
        return;

    PhysicsShape bodyA = manifold->bodyA->shape;
    PhysicsShape bodyB = manifold->bodyB->shape;
    manifold->contactsCount = 0;

    // Check for separating axis with A shape's face planes
    int faceA = 0;
    float penetrationA = FindAxisLeastPenetration(&faceA, bodyA, bodyB);
    
    if (penetrationA >= 0.0f)
        return;

    // Check for separating axis with B shape's face planes
    int faceB = 0;
    float penetrationB = FindAxisLeastPenetration(&faceB, bodyB, bodyA);
    
    if (penetrationB >= 0.0f)
        return;

    int referenceIndex = 0;
    bool flip = false;  // Always point from A shape to B shape

    PhysicsShape refPoly; // Reference
    PhysicsShape incPoly; // Incident

    // Determine which shape contains reference face
    if (BiasGreaterThan(penetrationA, penetrationB))
    {
        refPoly = bodyA;
        incPoly = bodyB;
        referenceIndex = faceA;
    }
    else
    {
        refPoly = bodyB;
        incPoly = bodyA;
        referenceIndex = faceB;
        flip = true;
    }

    // World space incident face
    Vector2 incidentFace[2];
    FindIncidentFace(&incidentFace[0], &incidentFace[1], refPoly, incPoly, referenceIndex);

    // Setup reference face vertices
    PolygonData refData = refPoly.vertexData;
    Vector2 v1 = refData.positions[referenceIndex];
    referenceIndex = (((referenceIndex + 1) < refData.vertexCount) ? (referenceIndex + 1) : 0);
    Vector2 v2 = refData.positions[referenceIndex];

    // Transform vertices to world space
    v1 = Mat2MultiplyVector2(refPoly.transform, v1);
    v1 = Vector2Add(v1, refPoly.body->position);
    v2 = Mat2MultiplyVector2(refPoly.transform, v2);
    v2 = Vector2Add(v2, refPoly.body->position);

    // Calculate reference face side normal in world space
    Vector2 sidePlaneNormal = Vector2Subtract(v2, v1);
    MathNormalize(&sidePlaneNormal);

    // Orthogonalize
    Vector2 refFaceNormal = { sidePlaneNormal.y, -sidePlaneNormal.x };
    float refC = MathDot(refFaceNormal, v1);
    float negSide = MathDot(sidePlaneNormal, v1)*-1;
    float posSide = MathDot(sidePlaneNormal, v2);

    // Clip incident face to reference face side planes (due to floating point error, possible to not have required points
    if (Clip((Vector2){ -sidePlaneNormal.x, -sidePlaneNormal.y }, negSide, &incidentFace[0], &incidentFace[1]) < 2)
        return;

    if (Clip(sidePlaneNormal, posSide, &incidentFace[0], &incidentFace[1]) < 2)
        return;

    // Flip normal if required
    manifold->normal = (flip ? (Vector2){ -refFaceNormal.x, -refFaceNormal.y } : refFaceNormal);

    // Keep points behind reference face
    int currentPoint = 0; // Clipped points behind reference face
    float separation = MathDot(refFaceNormal, incidentFace[0]) - refC;
    
    if (separation <= 0.0f)
    {
        manifold->contacts[currentPoint] = incidentFace[0];
        manifold->penetration = -separation;
        currentPoint++;
    }
    else
        manifold->penetration = 0.0f;

    separation = MathDot(refFaceNormal, incidentFace[1]) - refC;

    if (separation <= 0.0f)
    {
        manifold->contacts[currentPoint] = incidentFace[1];
        manifold->penetration += -separation;
        currentPoint++;

        // Calculate total penetration average
        manifold->penetration /= currentPoint;
    }

    manifold->contactsCount = currentPoint;
}

// Integrates physics forces into velocity
static void IntegratePhysicsForces(PhysicsState state, PhysicsBody body)
{
    if ((body == NULL) || (body->inverseMass == 0.0f) || !body->enabled)
        return;

    body->velocity.x += (body->force.x*body->inverseMass)*(state->deltaTime/2.0);
    body->velocity.y += (body->force.y*body->inverseMass)*(state->deltaTime/2.0);

    if (body->useGravity)
    {
        body->velocity.x += state->gravityForce.x*(state->deltaTime/1000/2.0);
        body->velocity.y += state->gravityForce.y*(state->deltaTime/1000/2.0);
    }

    if (!body->freezeOrient)
        body->angularVelocity += body->torque*body->inverseInertia*(state->deltaTime/2.0);
}

// Initializes physics manifolds to solve collisions
static void InitializePhysicsManifold(PhysicsState state,PhysicsManifold manifold)
{
    PhysicsBody bodyA = manifold->bodyA;
    PhysicsBody bodyB = manifold->bodyB;

    if ((bodyA == NULL) || (bodyB == NULL))
        return;

    // Calculate average restitution, static and dynamic friction
    manifold->restitution = sqrtf(bodyA->restitution*bodyB->restitution);
    manifold->staticFriction = sqrtf(bodyA->staticFriction*bodyB->staticFriction);
    manifold->dynamicFriction = sqrtf(bodyA->dynamicFriction*bodyB->dynamicFriction);

    for (int i = 0; i < manifold->contactsCount; i++)
    {
        // Caculate radius from center of mass to contact
        Vector2 radiusA = Vector2Subtract(manifold->contacts[i], bodyA->position);
        Vector2 radiusB = Vector2Subtract(manifold->contacts[i], bodyB->position);

        Vector2 crossA = MathCross(bodyA->angularVelocity, radiusA);
        Vector2 crossB = MathCross(bodyB->angularVelocity, radiusB);

        Vector2 radiusV = { 0.0f, 0.0f };
        radiusV.x = bodyB->velocity.x + crossB.x - bodyA->velocity.x - crossA.x;
        radiusV.y = bodyB->velocity.y + crossB.y - bodyA->velocity.y - crossA.y;

        // Determine if we should perform a resting collision or not;
        // The idea is if the only thing moving this object is gravity, then the collision should be performed without any restitution
        if (MathLenSqr(radiusV) < (MathLenSqr((Vector2){ state->gravityForce.x*state->deltaTime/1000, state->gravityForce.y*state->deltaTime/1000 }) + BABAX_EPSILON))
            manifold->restitution = 0;
    }
}

// Integrates physics collisions impulses to solve collisions
static void IntegratePhysicsImpulses(PhysicsManifold manifold)
{
    PhysicsBody bodyA = manifold->bodyA;
    PhysicsBody bodyB = manifold->bodyB;

    if ((bodyA == NULL) || (bodyB == NULL))
        return;

    // Early out and positional correct if both objects have infinite mass
    if (fabs(bodyA->inverseMass + bodyB->inverseMass) <= BABAX_EPSILON)
    {
        bodyA->velocity = BABAX_VECTOR_ZERO;
        bodyB->velocity = BABAX_VECTOR_ZERO;
        return;
    }

    for (int i = 0; i < manifold->contactsCount; i++)
    {
        // Calculate radius from center of mass to contact
        Vector2 radiusA = Vector2Subtract(manifold->contacts[i], bodyA->position);
        Vector2 radiusB = Vector2Subtract(manifold->contacts[i], bodyB->position);

        // Calculate relative velocity
        Vector2 radiusV = { 0.0f, 0.0f };
        radiusV.x = bodyB->velocity.x + MathCross(bodyB->angularVelocity, radiusB).x - bodyA->velocity.x - MathCross(bodyA->angularVelocity, radiusA).x;
        radiusV.y = bodyB->velocity.y + MathCross(bodyB->angularVelocity, radiusB).y - bodyA->velocity.y - MathCross(bodyA->angularVelocity, radiusA).y;

        // Relative velocity along the normal
        float contactVelocity = MathDot(radiusV, manifold->normal);

        // Do not resolve if velocities are separating
        if (contactVelocity > 0.0f)
            return;

        float raCrossN = MathCrossVector2(radiusA, manifold->normal);
        float rbCrossN = MathCrossVector2(radiusB, manifold->normal);

        float inverseMassSum = bodyA->inverseMass + bodyB->inverseMass + (raCrossN*raCrossN)*bodyA->inverseInertia + (rbCrossN*rbCrossN)*bodyB->inverseInertia;

        // Calculate impulse scalar value
        float impulse = -(1.0f + manifold->restitution)*contactVelocity;
        impulse /= inverseMassSum;
        impulse /= (float)manifold->contactsCount;

        // Apply impulse to each physics body
        Vector2 impulseV = { manifold->normal.x*impulse, manifold->normal.y*impulse };

        if (bodyA->enabled)
        {
            bodyA->velocity.x += bodyA->inverseMass*(-impulseV.x);
            bodyA->velocity.y += bodyA->inverseMass*(-impulseV.y);
            
            if (!bodyA->freezeOrient)
                bodyA->angularVelocity += bodyA->inverseInertia*MathCrossVector2(radiusA, (Vector2){ -impulseV.x, -impulseV.y });
        }

        if (bodyB->enabled)
        {
            bodyB->velocity.x += bodyB->inverseMass*(impulseV.x);
            bodyB->velocity.y += bodyB->inverseMass*(impulseV.y);
            
            if (!bodyB->freezeOrient)
                bodyB->angularVelocity += bodyB->inverseInertia*MathCrossVector2(radiusB, impulseV);
        }

        // Apply friction impulse to each physics body
        radiusV.x = bodyB->velocity.x + MathCross(bodyB->angularVelocity, radiusB).x - bodyA->velocity.x - MathCross(bodyA->angularVelocity, radiusA).x;
        radiusV.y = bodyB->velocity.y + MathCross(bodyB->angularVelocity, radiusB).y - bodyA->velocity.y - MathCross(bodyA->angularVelocity, radiusA).y;

        Vector2 tangent = { radiusV.x - (manifold->normal.x*MathDot(radiusV, manifold->normal)), radiusV.y - (manifold->normal.y*MathDot(radiusV, manifold->normal)) };
        MathNormalize(&tangent);

        // Calculate impulse tangent magnitude
        float impulseTangent = -MathDot(radiusV, tangent);
        impulseTangent /= inverseMassSum;
        impulseTangent /= (float)manifold->contactsCount;

        float absImpulseTangent = fabs(impulseTangent);

        // Don't apply tiny friction impulses
        if (absImpulseTangent <= BABAX_EPSILON)
            return;

        // Apply coulumb's law
        Vector2 tangentImpulse = { 0.0f, 0.0f };
        if (absImpulseTangent < impulse*manifold->staticFriction)
            tangentImpulse = (Vector2){ tangent.x*impulseTangent, tangent.y*impulseTangent };
        else
            tangentImpulse = (Vector2){ tangent.x*-impulse*manifold->dynamicFriction, tangent.y*-impulse*manifold->dynamicFriction };

        // Apply friction impulse
        if (bodyA->enabled)
        {
            bodyA->velocity.x += bodyA->inverseMass*(-tangentImpulse.x);
            bodyA->velocity.y += bodyA->inverseMass*(-tangentImpulse.y);

            if (!bodyA->freezeOrient)
                bodyA->angularVelocity += bodyA->inverseInertia*MathCrossVector2(radiusA, (Vector2){ -tangentImpulse.x, -tangentImpulse.y });
        }

        if (bodyB->enabled)
        {
            bodyB->velocity.x += bodyB->inverseMass*(tangentImpulse.x);
            bodyB->velocity.y += bodyB->inverseMass*(tangentImpulse.y);

            if (!bodyB->freezeOrient)
                bodyB->angularVelocity += bodyB->inverseInertia*MathCrossVector2(radiusB, tangentImpulse);
        }
    }
}

// Integrates physics velocity into position and forces
static void IntegratePhysicsVelocity(PhysicsState state,PhysicsBody body)
{
    if ((body == NULL) ||!body->enabled)
        return;

    body->position.x += body->velocity.x*state->deltaTime;
    body->position.y += body->velocity.y*state->deltaTime;

    if (!body->freezeOrient)
        body->orient += body->angularVelocity*state->deltaTime;

    Mat2Set(&body->shape.transform, body->orient);

    IntegratePhysicsForces(state,body);
}

// Corrects physics bodies positions based on manifolds collision information
static void CorrectPhysicsPositions(PhysicsManifold manifold)
{
    PhysicsBody bodyA = manifold->bodyA;
    PhysicsBody bodyB = manifold->bodyB;

    if ((bodyA == NULL) || (bodyB == NULL))
        return;

    Vector2 correction = { 0.0f, 0.0f };
    correction.x = (max(manifold->penetration - BABAX_PENETRATION_ALLOWANCE, 0.0f)/(bodyA->inverseMass + bodyB->inverseMass))*manifold->normal.x*BABAX_PENETRATION_CORRECTION;
    correction.y = (max(manifold->penetration - BABAX_PENETRATION_ALLOWANCE, 0.0f)/(bodyA->inverseMass + bodyB->inverseMass))*manifold->normal.y*BABAX_PENETRATION_CORRECTION;

    if (bodyA->enabled)
    {
        bodyA->position.x -= correction.x*bodyA->inverseMass;
        bodyA->position.y -= correction.y*bodyA->inverseMass;
    }

    if (bodyB->enabled)
    {
        bodyB->position.x += correction.x*bodyB->inverseMass;
        bodyB->position.y += correction.y*bodyB->inverseMass;
    }
}

// Returns the extreme point along a direction within a polygon
static Vector2 GetSupport(PhysicsShape shape, Vector2 dir)
{
    float bestProjection = -BABAX_FLT_MAX;
    Vector2 bestVertex = { 0.0f, 0.0f };
    PolygonData data = shape.vertexData;

    for (int i = 0; i < data.vertexCount; i++)
    {
        Vector2 vertex = data.positions[i];
        float projection = MathDot(vertex, dir);

        if (projection > bestProjection)
        {
            bestVertex = vertex;
            bestProjection = projection;
        }
    }

    return bestVertex;
}

// Finds polygon shapes axis least penetration
static float FindAxisLeastPenetration(int *faceIndex, PhysicsShape shapeA, PhysicsShape shapeB)
{
    float bestDistance = -BABAX_FLT_MAX;
    int bestIndex = 0;

    PolygonData dataA = shapeA.vertexData;

    for (int i = 0; i < dataA.vertexCount; i++)
    {
        // Retrieve a face normal from A shape
        Vector2 normal = dataA.normals[i];
        Vector2 transNormal = Mat2MultiplyVector2(shapeA.transform, normal);

        // Transform face normal into B shape's model space
        Mat2 buT = Mat2Transpose(shapeB.transform);
        normal = Mat2MultiplyVector2(buT, transNormal);

        // Retrieve support point from B shape along -n
        Vector2 support = GetSupport(shapeB, (Vector2){ -normal.x, -normal.y });

        // Retrieve vertex on face from A shape, transform into B shape's model space
        Vector2 vertex = dataA.positions[i];
        vertex = Mat2MultiplyVector2(shapeA.transform, vertex);
        vertex = Vector2Add(vertex, shapeA.body->position);
        vertex = Vector2Subtract(vertex, shapeB.body->position);
        vertex = Mat2MultiplyVector2(buT, vertex);

        // Compute penetration distance in B shape's model space
        float distance = MathDot(normal, Vector2Subtract(support, vertex));

        // Store greatest distance
        if (distance > bestDistance)
        {
            bestDistance = distance;
            bestIndex = i;
        }
    }

    *faceIndex = bestIndex;
    return bestDistance;
}

// Finds two polygon shapes incident face
static void FindIncidentFace(Vector2 *v0, Vector2 *v1, PhysicsShape ref, PhysicsShape inc, int index)
{
    PolygonData refData = ref.vertexData;
    PolygonData incData = inc.vertexData;

    Vector2 referenceNormal = refData.normals[index];

    // Calculate normal in incident's frame of reference
    referenceNormal = Mat2MultiplyVector2(ref.transform, referenceNormal); // To world space
    referenceNormal = Mat2MultiplyVector2(Mat2Transpose(inc.transform), referenceNormal); // To incident's model space

    // Find most anti-normal face on polygon
    int incidentFace = 0;
    float minDot = BABAX_FLT_MAX;

    for (int i = 0; i < incData.vertexCount; i++)
    {
        float dot = MathDot(referenceNormal, incData.normals[i]);

        if (dot < minDot)
        {
            minDot = dot;
            incidentFace = i;
        }
    }

    // Assign face vertices for incident face
    *v0 = Mat2MultiplyVector2(inc.transform, incData.positions[incidentFace]);
    *v0 = Vector2Add(*v0, inc.body->position);
    incidentFace = (((incidentFace + 1) < incData.vertexCount) ? (incidentFace + 1) : 0);
    *v1 = Mat2MultiplyVector2(inc.transform, incData.positions[incidentFace]);
    *v1 = Vector2Add(*v1, inc.body->position);
}

// Calculates clipping based on a normal and two faces
static int Clip(Vector2 normal, float clip, Vector2 *faceA, Vector2 *faceB)
{
    int sp = 0;
    Vector2 out[2] = { *faceA, *faceB };

    // Retrieve distances from each endpoint to the line
    float distanceA = MathDot(normal, *faceA) - clip;
    float distanceB = MathDot(normal, *faceB) - clip;

    // If negative (behind plane)
    if (distanceA <= 0.0f)
        out[sp++] = *faceA;

    if (distanceB <= 0.0f)
        out[sp++] = *faceB;

    // If the points are on different sides of the plane
    if ((distanceA*distanceB) < 0.0f)
    {
        // Push intersection point
        float alpha = distanceA/(distanceA - distanceB);
        out[sp] = *faceA;
        Vector2 delta = Vector2Subtract(*faceB, *faceA);
        delta.x *= alpha;
        delta.y *= alpha;
        out[sp] = Vector2Add(out[sp], delta);
        sp++;
    }

    // Assign the new converted values
    *faceA = out[0];
    *faceB = out[1];

    return sp;
}

// Check if values are between bias range
static bool BiasGreaterThan(float valueA, float valueB)
{
    return (valueA >= (valueB*0.95f + valueA*0.01f));
}

// Returns the barycenter of a triangle given by 3 points
static Vector2 TriangleBarycenter(Vector2 v1, Vector2 v2, Vector2 v3)
{
    Vector2 result = { 0.0f, 0.0f };

    result.x = (v1.x + v2.x + v3.x)/3;
    result.y = (v1.y + v2.y + v3.y)/3;

    return result;
}

// Initializes hi-resolution MONOTONIC timer


// Get hi-res MONOTONIC time measure in seconds

// Get current time in milliseconds

// Returns the cross product of a vector and a value
static inline Vector2 MathCross(float value, Vector2 vector)
{
    return (Vector2){ -value*vector.y, value*vector.x };
}

// Returns the cross product of two vectors
static inline float MathCrossVector2(Vector2 v1, Vector2 v2)
{
    return (v1.x*v2.y - v1.y*v2.x);
}

// Returns the len square root of a vector
static inline float MathLenSqr(Vector2 vector)
{
    return (vector.x*vector.x + vector.y*vector.y);
}

// Returns the dot product of two vectors
static inline float MathDot(Vector2 v1, Vector2 v2)
{
    return (v1.x*v2.x + v1.y*v2.y);
}

// Returns the square root of distance between two vectors
static inline float DistSqr(Vector2 v1, Vector2 v2)
{
    Vector2 dir = Vector2Subtract(v1, v2);
    return MathDot(dir, dir);
}

// Returns the normalized values of a vector
static void MathNormalize(Vector2 *vector)
{
    float length, ilength;

    Vector2 aux = *vector;
    length = sqrtf(aux.x*aux.x + aux.y*aux.y);

    if (length == 0)
        length = 1.0f;

    ilength = 1.0f/length;

    vector->x *= ilength;
    vector->y *= ilength;
}

#if defined(BABAX_STANDALONE)
// Returns the sum of two given vectors
static inline Vector2 Vector2Add(Vector2 v1, Vector2 v2)
{
    return (Vector2){ v1.x + v2.x, v1.y + v2.y };
}

// Returns the subtract of two given vectors
static inline Vector2 Vector2Subtract(Vector2 v1, Vector2 v2)
{
    return (Vector2){ v1.x - v2.x, v1.y - v2.y };
}
#endif

// Creates a matrix 2x2 from a given radians value
static Mat2 Mat2Radians(float radians)
{
    float c = cosf(radians);
    float s = sinf(radians);

    return (Mat2){ c, -s, s, c };
}

// Set values from radians to a created matrix 2x2
static void Mat2Set(Mat2 *matrix, float radians)
{
    float cos = cosf(radians);
    float sin = sinf(radians);

    matrix->m00 = cos;
    matrix->m01 = -sin;
    matrix->m10 = sin;
    matrix->m11 = cos;
}

// Returns the transpose of a given matrix 2x2
static inline Mat2 Mat2Transpose(Mat2 matrix)
{
    return (Mat2){ matrix.m00, matrix.m10, matrix.m01, matrix.m11 };
}

// Multiplies a vector by a matrix 2x2
static inline Vector2 Mat2MultiplyVector2(Mat2 matrix, Vector2 vector)
{
    return (Vector2){ matrix.m00*vector.x + matrix.m01*vector.y, matrix.m10*vector.x + matrix.m11*vector.y };
}

#endif  // BABAX_IMPLEMENTATION


#if defined(BABAX_BINDING)


#endif