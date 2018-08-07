# Parameters

I have extended the base `.argos` file with a set of new parameters, documented
below. Parameters *should* that you are not using should be able to be omitted
from the XML file; please open an issue if this is not the case.

## Controller

The following root XML tags are defined:

- `output` - Paramaters for simulation outputs across all runs.

- `occupancy_grid` - Parameters pertaining to a robotas discretization of the
                     continuous world into a grid, and the objects it tracks
                     within the grid.

- `task_executive` - Parameters pertaining to tasks/task allocation.

- `task_exec_estimates` - Parameters pertaining to initial tasks time estimates.

- `sensing` -  Parameters for robot sensors.

- `actuation` - Parameters for robot actuators.

### `output`

#### `sim`

- `output_root` - The root output directory in which the directories of
                    different simulation runs will be placed.

- `output_dir` - The output directory for the current simulation under
                 `output_root`. If you put the special field
                 `__current_date__` here, the simulation will get a unique
                 output directory in the form YYYY-MM-DD:HH-MM.

### `occupancy_grid`

#### `pheromone`

- `rho` How fast the relevance of information about a particular cell within a
        robot's 2D map of the world loses relevance.

- `repeat_deposit` - If TRUE, then repeated pheromone deposits for cells
                     containing blocks/caches a robot already knows about will
                     be enabled. `rho` should be possibly be updated
                     accordingly.

#### `grid`

- `resolution` - The size of the cells the arena is broken up (discretized)
                 into. Should probably be the same as whatever the block size
                 is, to make things easy.

- `size` - The size of the arena. This is a duplicate of the size that is
           passed to the loop functions; much easier to duplicate than to deal
           with searching through an XML tree in C++.

### `task_executive`

#### `estimation`

`alpha`- Parameter for exponential weighting of a moving estimate of the true
         execution time of a task.

#### `task_abort`
- `reactivty` - Once the `abort_offset` is tripped, this parameter controls how
                fast the probability a robot aborts its current task grows.

- `offset` - A positive proportition indicating what ratio of measured execution
             time to the robot's best estimate of the actual execution time of
             the task is considered to be the threshold for a task taking too
             long, and should be aborted.

#### `task_partition`

- `method` - If `pini2011`, then robots will use the method described in the
             corresponding paper to determine if a task should be partitioned or
             not when deciding on their next task allocation.

- `always_partition` - If `true`, then robots will always choose to partition a
                       task, given the chance. Has no effect if `false`.

- `never_partition` - If `true`, then robots will never choose to partition a
                       task, given the chance. Has no effect if `false`.

- `reactivty` - Once the partition offset is tripped, this parameter controls
                how fast the probability a robot will partition its next task
                allocation grows.

- `offset` - A positive proportition indicating what ratio of execution times
             for unpartitioned task to sum of subtasks is considered to be
             acceptable for continuing to partition/not partition the task.

#### `subtask_selection`

- `method` - If `random`, then if a robot choosing to employ partitioning for a
             given task it will select one of the subtasks randomly. If
             `brutcshy2014`, then it will use the method described in the
             corresponding paper, which using estimates of waiting time at the
             task interface for selection. If `harwell2018`, it will use the
             same method as `brutcshy2014`, but use estimates of overall
             execution time as input instead.

- `offset` - A positive proportition indicating what ratio of execution times
             for the two subtasks will cause the subtask switching probability
             to start to change drastically.

- `reactivty` - Once the offset is tripped, this parameter controls how fast the
                subtask switching probability grows.

- `gamma` - Multiplicative factor used by `brutcshy2014`. I think.

#### `task_exec_estimates`

- `enabled` - If `true`, then all estimates of task execution times are
              initialized randomly within the specified ranges, rather than with
              zero, in order to avoid any possibly weird behavior on system
              startup. Has no effect if `false`.

- `generalist_range` - Takes a pair like so: `100:200` specifying the range of
  the uniform random distribution over which a robots' initial estimation of the
  duration of the generalist task will be drawn. Only used if `enabled` is
  `true`.

- `collector_range` - Takes a pair like so: `100:200` specifying the range of
  the uniform random distribution over which a robots' initial estimation of the
  duration of the collector task will be drawn. Only used if `enabled` is
  `true`.

- `harvester_range` - Takes a pair like so: `100:200` specifying the range of
  the uniform random distribution over which a robots' initial estimation of the
  duration of the harvester task will be drawn. Only used if `enabled` is
  `true`.

### `sensing`

#### `proximity_sensor`

- `angle_range` - The angle range to the left/right of center (90 degrees on a
                  unit circle) in which obstacles are not ignored (outside of
                  this range they are ignored, assuming the robot will be able
                  to drive by them). Takes a pair like so: `-5:5` (for a 10
                  degree window). Specified in radians.

- `delta` - The longest distance away from the robot obstacles will be
            considered.

### `actuation`

#### `block_carry_throttle`

- `waveform` - Parameters defining the waveform of throttling. Can be no
  throttling.

#### `steering2D`

##### `avoidance_force`

- `lookahead` - How far ahead of the robot to look for obstacles. Currently
  unused, but may be used in the future.

- `max` - Max value for the force.

##### `arrival_force`

- `slowing_radius` - Radius around target inside which robots will slow down
  linearly to not overshoot their target.

- `slowing_speed_min` - The minimum speed robotics will linearly ramp down
  to. Should be > 0.

- `max` - Max value for the force.

##### `polar_force`

- `max` - Max value for the force.

##### `wander_force`

- `circle_distance` - Scaling factor for force; applied to current velocity.

- `circle_radius` - Displacement (i.e. wander) circle radius; placed at
                    `circle_distance` from the robot.

- `max_angle_delta` -  +/- Maximum amount of heading change for the wander angle
  (a random value is chosen in this range). Specified in degrees.

##### `phototaxis_force`

- `max` - Max value for the force.

#### `differential_drive`

- `soft_turn_max` - If actuators are told to change to a heading within a
                    difference greater than the one specified by this parameter
                    to the current heading, a hard turn is executed (spin in
                    place).

- `max_speed` - The maximimum speed of the robot.

### `visualization`

- `robot_id` - Set to `true` or `false`. If true, robot id is displayed above
               each robot during simulation.

- `robot_los` - Set to `true` or `false`. If true, each robot's approximate line
                of sight is displayed as a red wireframe square during
                simulation.

- `robot_task` - Set to `true` or `false`. If `true`, the current task each robot
              is executing is displayed above it.

- `block_id` - Set to `true` or `false`. If true, each block's id displayed
               above it during simulation.

## Loop Functions

The following root XML tags are defined:

- `output` - Parameters relating to logging simulation metrics/results.

- `arena_map` - Parameters relating to discretization of the arena.

### `output`

#### `sim`

- `sim_log_fname` - The name of the simulation log output file in
  `output_root`/`output_dir`.

- `output_root` - The root output directory in which the directories of
                  different simulation runs will be placed.

- `output_dir` - The output directory for the current simulation under
                 `output_root`. If you put the special field `__current_date__`
                 here, the simulation will get a unique output directory in the
                 form YYYY-MM-DD:HH-MM.

#### `metrics`

- `output_dir` - Name of directory within the output directory for the
  simulation run that metrics will be placed in.

- `collision_fname` - The filename that statistics collected about collision
                      avoidance will be logged to.

- `block_acquisition_fname` - Filename for statistics related to block
                              acquisition, such as vectoring/exploring, will be
                              logged to.

- `block_transport_fname` - Filename for statistics related to block
                            transportation, such as # carries, will be logged
                            to.

- `block_manipulation_fname` - Filename for statistics related to block
                               manipulation, such as pickup/drop penalties, will
                               be logged to.

- `cache_acquisition_fname` - Filename for statistics related to cache
                              acquisition, such as vectoring/exploring, will be
                              logged to.

- `cache_utilization_fname` - Filename for statistics related to cache size,
                              pickups/drops, will be logged to.

- `cache_lifecycle_fname` - Filename for statistics related to cache
                            creation/depletion, will be logged to.

- `distance_fname` - Filename for logging statistics for how far all robots have
                     cumulatively traveled.

- `task_execution_generalist_fname` - Filename for logging metrics about the
                                      generalist task as it is executed.

- `task_execution_harvester_fname` - Filename for logging metrics about the
                                     generalist task as it is executed.

- `task_execution_collector_fname` - Filename for logging metrics about the
                                     generalist task as it is executed.

- `task_generalist_tab_fname` - Filename for logging metrics collected at the
                                "meta" level of task allocation: how often tasks
                                are aborted, etc. for the TAB rooted at the
                                generalist task.

- `perception_world_model_fname` - Filename for logging metrics related to
                                   errors/inaccuracies in robots' perceived
                                   model of the world.

- `collect_interval` - The timestep interval after which statistics will be
                       reset. Gathering statistics on a single timestep of a
                       long simulation is generally not useful; hence this field.

### `arena_map`

#### `grid`

- `resolution` - The resolution that the arena will be represented at, in terms
                 of the size of grid cells. Must be the same as the value passed
                 to the robot controllers.

- `size` - The size of the arena.

#### `blocks`

##### `manipulation_penalty`

- `waveform` - Parameters defining the waveform of throttling. Should always be
              at least 5, and non-Null.

##### `distribution`

- `arena_resolution` - The size of the cells the arena is broken up
                       (discretized) into. Should probably be the same as
                       whatever the block size is, to make things easy.

- `dist_model` - The distribution model for the blocks. When blocks are
                 distributed to a new location in the arena and made available
                 for robots to pickup (either initially or after a block is
                 deposited in the nest), they are placed in the arena in one of
                 the following ways:

  - `random`: Placed in a random location in the arena.

  - `powerlaw`: Distributed according to a powerlaw.

  - `single_source` - Placed within a small arena opposite about 90% of the
                      way from the nest to the other side of the arena
                      (assumes horizontal, rectangular arena).

######  `powerlaw`

- `pwr_min` - Minimum power of 2 for cluster sizes.

- `pwr_max` - Maximum power of 2 for cluster sizes.

- `n_clusters` - Max # of clusters the arena.

###### `manifest`

- `n_cube`- # cube blocks in the arena

- `n_ramp`- # ramp blocks in the arena

- `unit_dim` - Size of one side of the a cube block. Ramp blocks are 2x1 in this
               dimension.


#### `static_caches`

- `enable` - If true, then a single static cache will be created in the center
             of the arena (assumed to be horizontal). The cache will be
             replenished by the loop functions if robots deplete it, under
             certain conditions.

- `size` - The number of blocks to use when (re)-creating the static cache. Must
           be >= 2.

- `respawn_scale_factor` - A scale factor controlling how quickly the
                           probability of static cache respawn will grow once
                           the conditions for respawning are met.

- `dimension` - The dimension of the cache. Should be greater than the dimension
                for blocks.

- `min_dist` - The minimum distance between two blocks to be considered for
               cache creation from said blocks.

- `usage_penalty` - Waveform params again.

#### `nest`

- `size` - The size of the nest. Must be specified in a tuple like so:
  `0.5, 0.5`. Note the space--parsing does not work if it is omitted.

- `center` - Location for center of the nest (nest is a square).  Must be
             specified in a tuple like so: `1.5, 1.5`. Note the space--parsing
             does not work if it is omitted.
