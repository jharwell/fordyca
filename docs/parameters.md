# Parameters

I have extended the base `.argos` file with a set of new parameters, documented
below. Unless otherwise stated, all parameters must be present in the XML file.

## Controller

The following root XML tags are defined:

- `output` - Paramaters for simulation outputs across all runs.

- `occupancy_grid` - Parameters pertaining to a robotas discretization of the
                     continuous world into a grid, and the objects it tracks
                     within the grid.

- `task_allocation` - Parameters pertaining to task allocation variables/methods.

- `sensors` -  Parameters for robot sensors.

- `actuators` - Parameters for robot actuators.

- `fsm` - Parameters for state machine controlling a robot's actions.

### `output`

- `robot`

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

### `task_allocation`

#### `executive`

- `estimation_alpha` - Parameter for exponential weighting of a moving estimate
                       of the true execution time of a task.

- `abort_reactivty` - Once the `abort_offset` is tripped, this parameter
                      controls how fast the probability a robot aborts its
                      current task grows.

- `abort_offset` - A positive proportition indicating what ratio of measured
                   execution time to the robot's best estimate of the actual
                   execution time of the task is considered to be the threshold
                   for a task taking too long, and should be aborted.

- `partition_reactivty` - Once the `partition_offset` is tripped, this parameter
                          controls how fast the probability a robot will
                          partition its next task allocation grows.

- `partition_offset` - A positive proportition indicating what ratio of
                       execution times for unpartitioned task to sum of subtasks
                       is considered to be acceptable for continuing to
                       partition/not partition the task.

- `subtask_selection_method` - If `random`, then if a robot choosing to employ
                               partitioning for a given task it will select one
                               of the subtasks randomly. If `brutcshy2014`, then
                               it will use the method described in the
                               corresponding paper, which using estimates of
                               waiting time at the task interface for
                               selection. If `harwell2018`, it will use the same
                               method as `brutcshy2014`, but use estimates of
                               overall execution time as input instead.

- `partition_method` - If `pini2011`, then robots will use the method described
                       in the corresponding paper to determine if a task should
                       be partitioned or not when deciding on their next task
                       allocation.

- `always_partition` - If `true`, then robots will always choose to partition a
                       task, given the chance. Has no effect if `false`.

- `never_partition` - If `true`, then robots will never choose to partition a
                       task, given the chance. Has no effect if `false`.

#### `init_estimates`

- `enabled` - If `true`, then all estimates of task execution times are
              initialized randomly within the specified ranges, rather than with
              zero, in order to avoid any possibly weird behavior on system
              startup. Has no effect if `false`.

- `generalist_range` - Takes a pair like so: `100:200` specifying the range of
  the uniform random distribution over which a robots' initial estimation of the
  duration of the generalist task will be drawn.

- `collector_range` - Takes a pair like so: `100:200` specifying the range of
  the uniform random distribution over which a robots' initial estimation of the
  duration of the collector task will be drawn.

- `harvester_range` - Takes a pair like so: `100:200` specifying the range of
  the uniform random distribution over which a robots' initial estimation of the
  duration of the harvester task will be drawn.

### `sensors`

#### `proximity`

- `go_straight_angle_range` - The angle range to the left/right of center (90
                              degrees on a unit circle) in which obstacles are
                              not ignored (outside of this range they are
                              ignored, assuming the robot will be able to
                              drive by them). Takes a pair like so: `-5:5`
                              (for a 10 degree window).

- `delta` - The longest distance away from the robot obstacles will be
            considered.

### `actuators`

#### `wheels`

- `hard_turn_angle_threshold` - If actuators are told to change to a heading
                                within a difference greater than the one
                                specified by this parameter to the current
                                heading, then a hard turn is executed (turn in
                                place/opposite wheel speeds).

- `soft_turn_angle_threshold` - If actuators are told to change to a heading
                                within a difference greater than the one
                                specified by this parameter to the current
                                heading, but less than
                                `hard_turn_angle_threshold`, then a soft turn
                                is executed (keep moving forward and turn
                                gradually).

- `no_turn_angle_threshold` - If actuators are told to change to a heading
                              within a difference less than the one specified
                              by this parameter to the current heading, the
                              heading change is ignored.

- `max_speed` - The maximimum speed of the robot.

### `fsm`

- `unsuccessful_explore_dir_change` - For robots executing an explore FSM to
                                      look for something, they will randomly
                                      change direction after this many steps if
                                      they are unsuccessful in finding what they
                                      are looking for.

- `frequent_collision_thresh` - The number of timesteps between subsequent
                                collisions for said collisions to be considered
                                frequent. If a collision is considered frequent,
                                then instead of heading in the opposite
                                direction after a collision, a robot will head
                                in a random direction, which should hopefully
                                help get it out of whatever situation it was in
                                that caused the frequent collision(s) in the
                                first place.

- `nest` - The location of the nest. Again, this is a duplicate of the location
           passed to the loop functions, but it was easier to do it this way
           rather than muck about with XML tree traversal.

- `speed_throttle_block_carry` - The percentage (specified between 0 and 1) by
  which a robot's speed will be decreased when it is carrying a block.

## Loop Functions

The following root XML tags are defined:

- `output` - Parameters relating to logging simulation metrics/results.

- `arena_map` - Parameters relating to discretization of the arena.

- `penalty` = Parameters relating to a temporal penalty function to simulate environmental changes.

- `visualization` - Parameters for simulation visualizations, for help in
                    debugging.

- `simulation` - Parameters relating to running experiments.

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

- `stateless_fname` - The filename that statistics collected for the
                      `stateless_foraging_controller` and all logically derived
                      controllers will be logged to.

- `stateful_fname` - The filename that statistics collected for the
                     `stateful_foraging_controller` and all logically derived
                     controllers will be logged to.

- `depth1_fname` - The filename that statistics collected for the
                   `depth1_foraging_controller` and all logically derived
                   controllers will be logged to.

- `block_fname` - Filename for statistics related to block collection, such as #
                  carries, will be logged to.

- `distance_fname` - Filename for logging statistics for how far all robots have
                     cumulatively traveled.

- `task_execution_fname` - Filename for logging metrics about partitioning,
                           etc. collected from tasks as they are executed.

- `task_management_fname` - Filename for logging metrics collected at the "meta"
                            level of task allocation: how often tasks are
                            aborted, etc.

- `cache_fname` - Filename that metrics collected from caches in the arena will
                  be collected in.

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

- `n_blocks` - # of blocks present in the arena.

- `dimension` - Since blocks are square, this the size of one side of the square.

- `dist_model` - The distribution model for the blocks. When blocks are
                 distributed to a new location in the arena and made available
                 for robots to pickup (either initially or after a block is
                 deposited in the nest), they are placed in the arena in one of
                 the following ways:

    - `random`: Placed in a random location in the arena.

    - `single_source` - Placed within a small arena opposite about 90% of the
                        way from the nest to the other side of the arena
                        (assumes horizontal, rectangular arena).

- `respawn` - Whether or not blocks should be re-distributed in the arena after
              they are brought to the nest.

#### `caches`

- `create_static` - If true, then a single static cache will be created in the
                    center of the arena (assumed to be horizontal). The cache will
                    be replenished by the loop functions if robots deplete it,
                    under certain conditions.

- `create_dynamic` - If true, then every timestep the loop functions will
                     create/remove caches as necessary as robots drop/pickup
                     blocks throughout the arena.

- `static_size` - The number of blocks to use when (re)-creating the static
                  cache. Must be >= 2.

- `static_respawn_scale_factor` - A scale factor controlling how quickly the
                                  probability of static cache respawn will grow
                                  once the conditions for respawning are met.

- `dimension` - The dimension of the cache. Should be greater than the dimension
                for blocks.

- `min_dist` - The minimum distance between two blocks to be considered for
               cache creation from said blocks.

- `usage_penalty` - How many timesteps a robot picking up a block from/dropping
                  a block into a cache will have to wait before the operation
                  will be allowed to proceed? Used to model the cost of actually
                  picking up/dropping physical blocks.

#### `nest`

- `size` - The size of the nest. Must be specified in a tuple like so:
  `0.5, 0.5`. Note the space--parsing does not work if it is omitted.

- `center` - Location for center of the nest (nest is a square).  Must be
             specified in a tuple like so: `1.5, 1.5`. Note the space--parsing
             does not work if it is omitted.

### `visualization`

- `robot_id` - Set to `true` or `false`. If true, robot id is displayed above
               each robot during simulation.

- `robot_los` - Set to `true` or `false`. If true, each robot's approximate line
                of sight is displayed as a red wireframe square during
                simulation.

- `block_id` - Set to `true` or `false`. If true, each block's id displayed
               above it during simulation.

- `task_id` - Set to `true` or `false`. If `true`, the current task each robot
              is executing is displayed above it.
