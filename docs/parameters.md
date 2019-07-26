# Parameters

I have extended the base `.argos` file with a set of new parameters, documented
below. Parameters *should* that you are not using should be able to be omitted
from the XML file; please open an issue if this is not the case.

## Controller

The following root XML tags are defined:

- `output` - Paramaters for simulation outputs across all runs.

- `perception` - Parameters pertaining to a robots discretization of the
                 continuous world into a grid, and/or the objects it tracks
                 within it.

- `task_executive` - Parameters pertaining to the task executive (the entitity
                     responsible for managing/running tasks after they have been
                     allocated).

- `task_alloc` - Parameters pertaining to task allocation.

- `block_sel_matrix` - Parameters used by robots when selecting which
                             block to acquire/obtain as part of the task they
                             are currently executing.

- `cache_sel_matrix` - Parameters used by robots when selecting which
                             cache to acquire/obtain as part of the task they
                             are currently executing.

- `sensing` -  Parameters for robot sensors.

- `actuation` - Parameters for robot actuators.

- `communication` - Parameters for inter-robot communication.

### `output`

#### `sim`

- `output_root` - The root output directory in which the directories of
                    different simulation runs will be placed.

- `output_dir` - The output directory for the current simulation under
                 `output_root`. If you put the special field
                 `__current_date__` here, the simulation will get a unique
                 output directory in the form YYYY-MM-DD:HH-MM.

### `perception`

#### `pheromone`

- `rho` How fast the relevance of information about a particular cell within a
        robot's 2D map of the world loses relevance.

- `repeat_deposit` - If TRUE, then repeated pheromone deposits for blocks/caches
                     a robot already knows about will be enabled. `rho` should
                     be updated accordingly, probably to a larger value to
                     enable faster decay.

#### `grid`

- `resolution` - The size of the cells the arena is broken up (discretized)
                 into. Should probably be the same as whatever the block size
                 is, to make things easy.

- `size` - The size of the arena. This is a duplicate of the size that is
           passed to the loop functions; much easier to duplicate than to deal
           with searching through an XML tree in C++.

### `task_executive`

- `update_exec_ests` - If TRUE, then the executive will use the elapsed time
                       since a task started to update the task time
                       estimate. Estimate is updated on both abort an
                       completion.

- `update_interface_ests` - If TRUE, then the executive will use the calculated
                            interface time for a task to update the interface
                            estimate for the task. Estimate is updated on both
                            abort an completion.

- `tab_init_method` - When performing initial task allocation, an active Task
                      Allocation Block (TAB), consisting of a root has and two
                      sequentially interdependent subtasks has to be
                      selected. This parameter controls the selection
                      method. Valid values are:

    - `root` - Use the root TAB as the initially active TAB.

    - `random` - Choose a random TAB as the initially active TAB.

    - `max_depth` - Choose a random TAB from among those at the greatest depth
                    within the task decomposition graph that is passed to the
                    executive.

    This parameter is current experimental, and only affects depth2
    simulations.

### `task_alloc`

Several subsections in this section sigmoid based functions for choosing between
alternatives, with the input src and sigmoid method varying.

#### `src_sigmoid_sel` - Sourced sigmoid activation function

- `input_src` - Can be `exec` or `interface`, indicating that estimates of
                execution/interface times should be used in the selection
                process.

##### `sigmoid_sel` - Sigmoid-based method for selecting SOMETHING

- `method` - Unused for task abort, but should still exist, as future
             implementations may define different sigmoid based methods.

###### `sigmoid` - Actual sigmoid parameters

- `reactivty` - Once the `offset` is tripped, this parameter controls how
                fast the probability a robot aborts its current task grows.

- `offset` - A positive proportition indicating what ratio of measured execution
             time to the robot's best estimate of the actual execution time of
             the task is considered to be the threshold for a task taking too
             long, and should be aborted.

- `gamma` - A scaling factor that is applied to the overall calculated
            probability.

#### `task_abort`

Uses `src_sigmoid_sel`, with an empty `method` tag to perform the stochastic
abort decision, which is calculated each timestep.

#### `task_partition`

- `always_partition` - If `true`, then robots will always choose to partition a
                       task, given the chance. Has no effect if `false`.

- `never_partition` - If `true`, then robots will never choose to partition a
                       task, given the chance. Has no effect if `false`.

Uses `src_sigmoid_sel` with a `method` tag that can be one of [`pini2011`] for
performing the stochastic partitioning decision. Calculated once upon each task
allocation, after the previous task is finished or aborted.

#### `subtask_sel`

Uses `src_sigmoid_sel` with a `method` tag that can be one of [`harwell2018`,
`random`] to perform stochastic subtask selection if partitioning is employed.

#### `task_exec_estimates`

- `seed_enabled` - If `true`, then all estimates of task execution times are
                   initialized randomly within the specified ranges, rather than
                   with zero, in order to avoid any possibly weird behavior on
                   system startup. Has no effect if `false`.

- `<task name>` - Takes a pair like so: `100:200` specifying the range of the
                  uniform random distribution over which a robots' initial
                  estimation of the duration of the specified task will be
                  drawn. Only used if `enabled` is `true`. Valid values for
                  `<task_namne>` are: [`generalist`, `collector`,
                  `harvester`, `cache_starter`, `cache_finisher`,
                  `cache_transferer` `cache_collector`].

##### `ema` - Exponential Moving Average

`alpha`- Parameter for exponential weighting of a moving time estimate of the
         true execution/interface time of a task.

#### `tab_sel`

Uses `src_sigmoid_sel` to select which TAB to switch to (if applicable) during
task allocation, with a `method` tag that can be one of [`harwell2019`].


### `block_sel_matrix`

`nest` - The location of the nest.

#### `block_priorities`

- `cube` - The priority value used as part of block utility calculation for cube
           blocks during block selection.

- `ramp` - The priority value used as part of block utility calculation for ramp
           blocks during block selection.

#### `pickup_policy`

- `policy` - The policy to use to restrict (1) the conditions under which robots
             can pick up a block that they encounter, (2) which blocks are
             considered valid for acquisition. Valid values are:

    - `cluster_proximity` - Only allow blocks which are within `prox_dist` from
      the average of the positions of the blocks currently known to a robot to
      be picked up. Only makes sense for object distributions in which objects
      are clumped into clusters; used to help depth2 robots not to pickup the
      blocks other robots have dropped in order to start caches.

    - `Null` - No policy--robots can pickup any blocks the know about/all known
      blocks are eligible for acquisition

- `prox_dist` - The minimum distance measure for usage with `cluster_proximity`
                pickup policy.

### `cache_sel_matrix`

- `cache_prox_dist` - When executing the Cache Finisher task, the constraint
                      applied to new cache selection for how close the chosen
                      new cache can be to known existing caches. Should be at
                      least twice the size of a cache for Cache Finisher robots
                      to behave properly and not get stuck in infinite loops
                      attempting to drop a block too close to a known cache.

- `block_prox_dist` - When executing the Cache Starter task, the constraint
                      applied to cache site selection for how close the chosen
                      cache site can be to known blocks.

- `nest_prox_dist` - When executing the Cache Starter task, the constraint
                     applied to cache site selection for how close the chosen
                     cache site can be to the nest.

- `site_xrange` - The valid X range for cache site selection (should be a subset
                  of the full arena X size, to avoid robots being able to select
                  locations by arena boundaries).

- `site_yrange` - The valid Y range for cache site selection (should be a subset
                  of the full arena Y size, to avoid robots being able to select
                  locations by arena boundaries).

#### `pickup_policy`

- `policy` - The policy to use to restrict (1) the conditions under which robots
             can pick up from a cache that they encounter, (2) which caches are
             considered valid for acquisition. Valid values are:

  - `cache_size` - Only allow robots to pickup from caches with at least
    `cache_size` blocks in them. Robots intending to drop blocks in caches are
    not restricted.

  - `timestep` - Only allow robots to pickup from caches after `timestep`
    timesteps have elapsed during simulation. Robots intending to drop blocks in
    caches are not restricted.

### `exploration`

- `block_strategy` - The strategy robots should use to located blocks when they
                     do not currently know of any and need to find one for the
                     task they are currently doing. Valid values are:
  - `CRW` - Correlated Random Walk

  - `likelihood_search` - Go to the location of the last known block and then
                          begin performing CRW there.

- `cache_strategy` - The strategy robots should use to located caches when they
                     do not currently know of any and need to find one for the
                     task they are currently doing.

  - `CRW` - Correlated Random Walk

  - `likelihood_search` - Go to the location of the last known block and then
                          begin performing CRW there.

  - `utility_search` - Use the average location of the known blocks/robot's
                       current location as input into the cache site selection
                       algorithm, then go to the location it returns and begin
                       performing CRW there.

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

### `communication`

- `on` - true/false that enables/disables the use of communication

- `mode` - utility/random that determines how cells will be selected for communicationS

- `prob_send` - The probability of sending a message. Specified as a decimal.

- `prob_receive` - The probability of receiving a message. Specified as a decimal.

## Loop Functions

The following root XML tags are defined:

- `output` - Parameters relating to logging simulation metrics/results.

- `convergence` - Parameters relating to computing swarm convergence.

- `arena_map` - Parameters relating to discretization of the arena.

- `oracle` - Parameters related to the all knowing oracle, which allows
             robots/swarms to make decisions based on perfect information, to
             provide an upper bound on performance.

- `temporal_variance` - Parameters relating to the various types of temporal
                        waveforms that can be applied to swarm/environmental
                        state during simulation.

- `caches` - Parameters related to the use of caches in the arena.

### `output`

#### `sim`

- `output_root` - The root output directory in which the directories of
                  different simulation runs will be placed.

- `output_dir` - The output directory for the current simulation under
                 `output_root`. If you put the special field `__current_date__`
                 here, the simulation will get a unique output directory in the
                 form YYYY-MM-DD:HH-MM.

#### `metrics`

- `output_dir` - Name of directory within the output root that metrics will be
                 placed in.

- `collect_interval` - The timestep interval after which statistics will be
                       reset. Gathering statistics on a single timestep of a
                       long simulation is generally not useful; hence this
                       field.

### `convergence`

- `n_threads` - How many threads will be used for convergence calculations
                during loop functions.

- `epsilon` - Threshold < 1.0 that a convergence measure will be considered
              to have converged when its normalized value is above.

- `epsilon_delta` - # of timesteps the swarm must maintain its normalized
                    convergence measure score of >= `epsilon` in order to be
                    considered converged.

#### `positional_entropy`

A measure of convergence using robot positions, Shannon's entropy definition,
and Balch2000's social entropy measure.

- `enable` - If this measure is enabled or not. Very expensive to compute in
             large swarms.

- `horizon` - A `min:max` pair of distances specifying the min and max spatail
              cluster size that will be used to compute the entropy of robot
              positions.

- `horizon_delta` - Step size for traversing the horizon from min to max.

#### `interactivity`

A measure of convergence using nearest neighbor distances.

- `enable` - If this measure is enabled or not. Relatively cheap to compute in
             large swarms.

#### `angular_order`

A measure of convergence using congruence of robot orientations.

- `enable` - If this measure is enabled or not. Relatively cheap to compute in
             large swarms.

### `oracle`

#### `tasking_oracle`

- `task_exec_est` - If enabled, then this will inject perfect estimates of task
                    execution time based on the performance of the entire swarm
                    into each robot when it performs task allocation.

- `task_interface_est` - If enabled, then this will inject perfect estimates of
                         task interface time based on the performance of the
                         entire swarm into each robot when it performs task
                         allocation.

#### `entities_oracle`

- `blocks_enabled` - Inject perfect knowledge of all block locations into the
                     swarm every timestep.

- `caches_enabled` - Inject perfect knowledge of all cache locations into the
                     swarm every timestep.

### `temporal_variance`

#### `blocks`

##### `manipulation_penalty`

- `waveform` - Parameters defining the waveform of block manipulation penalty
               (picking up/dropping that does not involve caches).

##### `carry_throttle`

- `waveform` - Parameters defining the waveform of block carry penalty
               (how much slower robots move when carrying a block).

### `caches`

##### `usage_penalty`

- `waveform` - Parameters defining the waveform of cache usage penalty (picking
               up/dropping).

### `arena_map`

#### `grid`

- `resolution` - The resolution that the arena will be represented at, in terms
                 of the size of grid cells. Must be the same as the value passed
                 to the robot controllers.

- `size` - The size of the arena.

#### `blocks`

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

  - `single_source` - Placed within an arena opposite about 90% of the way from
                      the nest to the other side of the arena (assumes
                      horizontal, rectangular arena).

  - `dual_source` - Placed in two sources on either side of a central nest
                    (assumes a horizontal, rectangular arena).

  - `quad_source` - Placed in 4 sources at each cardinal direction in the
                    arena. Assumes a square arena.

###### `manifest`

- `n_cube` - # Cube blocks that should be used.

- `n_ramp` - # Ramp blocks that should be used.

- `unit_dim` - Unit dimension of blocks. Cube are 1x1 of this, ramp are 2x1 of
               this.

######  `powerlaw`

- `pwr_min` - Minimum power of 2 for cluster sizes.

- `pwr_max` - Maximum power of 2 for cluster sizes.

- `n_clusters` - Max # of clusters the arena.

#### `nest`

- `size` - The size of the nest. Must be specified in a tuple like so:
  `0.5, 0.5`. Note the space--parsing does not work if it is omitted.

- `center` - Location for center of the nest (nest is a square).  Must be
             specified in a tuple like so: `1.5, 1.5`. Note the space--parsing
             does not work if it is omitted.

### `caches`

- `dimension` - The dimension of the cache. Should be greater than the dimension
                for blocks.

#### `static`

- `enable` - If true, then a single static cache will be created in the center
             of the arena (assumed to be horizontal). The cache will be
             replenished by the loop functions if robots deplete it, under
             certain conditions.

- `size` - The number of blocks to use when (re)-creating the static cache. Must
           be >= 2.

- `respawn_scale_factor` - A scale factor controlling how quickly the
                           probability of static cache respawn will grow once
                           the conditions for respawning are met.

#### `dynamic`

- `enable` - If `true`, then the creation of dynamic caches will be enabled
             (depth2 only).

- `min_dist` - The minimum distance between blocks to be considered for
               cache creation from said blocks.

- `min_blocks` - The minimum # of blocks that need to within `min_dist` from
                 each other to trigger dynamic cache creation.

- `robot_drop_only` - If TRUE, then caches will only be created by intential
                      robot block drops rather than drops due to abort/block
                      distribution after collection.

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
