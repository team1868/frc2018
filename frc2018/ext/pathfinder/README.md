# Pathfinder C Core Library
This is the C code for the core of the 'Pathfinder' motion profiling library. This library can be used in any C application for quick and easy generation of
motion profiles and trajectories.

## Using the Library
Full examples are provided under `examples/`

### Includes
```c
#include <pathfinder.h>
```

### Creating some Waypoints
```c
int POINT_LENGTH = 3;

Waypoint points[POINT_LENGTH];

Waypoint p1 = { -4, -1, d2r(45) };      // Waypoint @ x=-4, y=-1, exit angle=45 degrees
Waypoint p2 = { -1, 2, 0 };             // Waypoint @ x=-1, y= 2, exit angle= 0 radians
Waypoint p3 = {  2, 4, 0 };             // Waypoint @ x= 2, y= 4, exit angle= 0 radians
points[0] = p1;
points[1] = p2;
points[2] = p3;
```

### Generating the Trajectory
```c
TrajectoryCandidate candidate;

// Prepare the Trajectory for Generation.
//
// Arguments: 
// Fit Function:        FIT_HERMITE_CUBIC or FIT_HERMITE_QUINTIC
// Sample Count:        PATHFINDER_SAMPLES_HIGH (100 000)
//                      PATHFINDER_SAMPLES_LOW  (10 000)
//                      PATHFINDER_SAMPLES_FAST (1 000)
// Time Step:           0.001 Seconds
// Max Velocity:        15 m/s
// Max Acceleration:    10 m/s/s
// Max Jerk:            60 m/s/s/s
pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_HIGH, 0.001, 15.0, 10.0, 60.0, &candidate);

int length = candidate.length;

// Array of Segments (the trajectory points) to store the trajectory in
Segment *trajectory = malloc(length * sizeof(Segment));

// Generate the trajectory
int result = pathfinder_generate(&candidate, trajectory);
if (result < 0) {
    // An error occured
    printf("Uh-Oh! Trajectory could not be generated!\n");
}
```

### Using the Segments
```c
int i;
for (i = 0; i < length; i++) {
    Segment s = trajectory[i];
    printf("Time Step: %f\n", s.dt);
    printf("Coords: (%f, %f)\n", s.x, s.y);
    printf("Position (Distance): %f\n", s.position);
    printf("Velocity: %f\n", s.velocity);
    printf("Acceleration: %f\n", s.acceleration);
    printf("Jerk (Acceleration per Second): %f\n", s.jerk);
    printf("Heading (radians): %f\n", s.heading);
}
```

Don't forget to free the `trajectory`!

## Modifying your Trajectory
### Tank Drive
```c
Segment leftTrajectory[length];
Segment rightTrajectory[length];

// The distance between the left and right sides of the wheelbase is 0.6m
double wheelbase_width = 0.6;

// Generate the Left and Right trajectories of the wheelbase using the 
// originally generated trajectory
pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory, wheelbase_width);
```

### Swerve Drive
```c
// Output Trajectories
Segment frontLeft[length];
Segment frontRight[length];
Segment backLeft[length];
Segment backRight[length];

// The distance between the left and right sides of the wheelbase is 0.6m
double wheelbase_width = 0.6;

// The distance between the front and back sides of the wheelbase is 0.5m
double wheelbase_depth = 0.5;

// The swerve mode to generate will be the 'default' mode, where the robot
// will constantly be facing forward and 'sliding' sideways to follow a
// curved path.
SWERVE_MODE mode = SWERVE_DEFAULT;

// Generate the trajectories of each of the 4 swerve wheels using the 
// originally generated trajectory
pathfinder_modify_swerve(trajectory, length, frontLeft, frontRight, 
        backLeft, backRight, wheelbase_width, wheelbase_depth, mode);
```