# Balloon Circler Destroyer

This package implements destroying of balloons by circling around the arena and eliminating them one by one, narrowing the circle each time.

## Functionality

* Desired waypoints are loaded as a matrix from config file
* Service `go_closer` moves the drone closer to the balloon
* Service `circle around balloon` causes the UAV to start circling around the closest balloon
* Service `destroy_nearest` starts state machine to destroy nearest balloon
* Service `start_arena_destroy` starts state machine to destroy balloons on the arena


