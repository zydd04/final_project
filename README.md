## ECSE211 FINAL PROJECT
What it does in Phase 1:
1. Inputs: run it as python3 final.py s1x s1y s2x s2y d1x d1y d2x d2y. Writing the points like "(1,3)" "(1,2)" "(2,-1)" "(3,-1)" also works. Those four points are the only thing you give it, as the rules require.
2. Finding the entrance: the s1–s2 edge has a tile on each side. The robot drives to the one closer to the origin, with an L-shaped path, then enters the street from there. For the sample map that's tile (0,2), entering the street at (1,2) heading +x. If the first tile it enters has no walls on either side, it decides it guessed the wrong side and switches.
3. Mapping: it moves one tile at a time. On each tile it checks ahead and to the left with the ultrasonic sensor. It only checks the right when it needs to: on a straight stretch the right side must be a wall, since the street is one tile wide. It goes straight if it can, otherwise left, otherwise right.
   - It stops mapping when both sides are open (it has left the street), at a dead end, at the map edge, or after 75 s so there's time left to get home.
   - When it measures a wall, it uses that distance to correct its position estimate.
4. Return: it retraces its own path back to (0,0), skipping the stops on straight runs, and turns to face +y again.
5. Map file: it saves the street tiles in order, with their walls, the entrance and the exit, to street_map.json for Phase 2.
6. Pause: it waits for you to press Enter before Phase 2, which is still an empty placeholder.

Things you need to check on the robot:
- Ultrasonic sensor: the code assumes a front-facing one on port 2. I couldn't see one in the photos. If you're using a different sensor or port, tell me and I'll change it.
- US_OFFSET (8 cm): this is the distance from the wheel axle to the front of the sensor. Measure it, because the wall threshold depends on it.
- Tile size: I set TILE to 30.48 cm, as in the spec. Your old code had 32.48.
- Calibration: check RADIUS, TRACK_WIDTH and MOTOR_DIR = -1, which is kept from your old code where negative speed drives forward. Do one 360° turn test and one 1 m straight-line test.
- Timing: Phase 1 has a 2-minute limit. If it runs close, raise DRIVE_DPS or TURN_MAX_DPS.
- Starting position: put the robot on the origin facing +y.

I removed the old line-following code with the color sensor, along with the cube-delivery code, which used variables that were never defined (SEARCH_SPEED, WAYPOINTS, deliver_cube).
