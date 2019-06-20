# Parameters

I have extended the base `.argos` file with a set of new parameters, documented
below. Optional vs. mandatory parameters are documented as such.

## Controller

The following root XML tags are defined under the `<params>` tag for all
controller types:


| Root XML tag       | Mandatory For?      | Description                                                                                                                       |
|--------------------|---------------------|-----------------------------------------------------------------------------------------------------------------------------------|
| `output`           | All controllers     | Paramaters for simulation outputs across all runs.                                                                                |
| `perception`       | All but CRW         | Parameters pertaining to a robots discretization of the continuous world into a grid, and/or the objects it tracks within it.     |
| `task_executive`   | [`depth1`,`depth2`] | Parameters pertaining to the task executive (the entitity responsible for managing/running tasks after they have been allocated). |
| `task_alloc`       | [`depth1`,`depth2`] | Parameters pertaining to task allocation.                                                                                         |
| `block_sel_matrix` | All but CRW         | Parameters used by robots when selecting which block to acquire/obtain as part of the task they are currently executing.          |
| `cache_sel_matrix` | [`depth1`,`depth2`] | Parameters used by robots when selecting which cache to acquire/obtain as part of the task they are currently executing.          |
| `sensing`          | All controllers     | Parameters for robot sensors.                                                                                                     |
| `actuation`        | All controllers     | Parameters for robot actuators.                                                                                                   |

### `output`

All nested tags and attributes under this tag are required.

#### `sim`

- `output_root` - The root output directory in which the directories of
                    different simulation runs will be placed.

- `output_dir` - The output directory for the current simulation under
                 `output_root`. If you put the special field
                 `__current_date__` here, the simulation will get a unique
                 output directory in the form YYYY-MM-DD:HH-MM.

### `perception`

This tag is optional, but must be defined for all non-CRW controllers. If this
tag is defined, then all nested tags and attributes are required.

#### `pheromone`

- `rho` How fast the relevance of information about a particular cell within a
        robot's 2D map of the world loses relevance.

- `repeat_deposit` - If TRUE, then repeated pheromone deposits for blocks/caches
                     a robot already knows about will be enabled. `rho` should
                     be updated accordingly, probably to a larger value to
                     enable faster decay.

#### `grid`

This tag is optional, but must be defined for all controllers that use MDPO
perception. If this tag is defined, then all nested tags and attributes are
required.

- `resolution` - The size of the cells the arena is broken up (discretized)
                 into. Should probably be the same as whatever the block size
                 is, to make things easy.

- `size` - The size of the arena. This is a duplicate of the size that is
           passed to the loop functions; much easier to duplicate than to deal
           with searching through an XML tree in C++.

### `task_executive`

This tag is optional, but must be defined for all controllers that use the task
executive [`depth1`,`depth2`].

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
                      method. This tag is only required for depth2
                      controllers. Valid values are:

    - `root` - Use the root TAB as the initially active TAB.

    - `random` - Choose a random TAB as the initially active TAB.

    - `max_depth` - Choose a random TAB from among those at the greatest depth
                    within the task decomposition graph that is passed to the
                    executive.

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

This tag is required if the task executive is used.

#### `task_abort`

Uses `src_sigmoid_sel`, with an empty `method` tag to perform the stochastic
abort decision, which is calculated each timestep. This tag is required if the
task executive is used.

#### `task_partition`

This tag is required if the task executive is used.

- `always_partition` - If `true`, then robots will always choose to partition a
                       task, given the chance. Has no effect if `false`.

- `never_partition` - If `true`, then robots will never choose to partition a
                       task, given the chance. Has no effect if `false`.

Uses `src_sigmoid_sel` with a `method` tag that can be one of [`pini2011`] for
performing the stochastic partitioning decision. Calculated once upon each task
allocation, after the previous task is finished or aborted.

#### `subtask_sel`

Uses `src_sigmoid_sel` with a `method` tag that can be one of [`harwell2018`,
`random`] to perform stochastic subtask selection if partitioning is
employed. This tag is required if the task executive is used.

#### `task_exec_estimates`

This tag is optional if the task executive is used.

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

This tag is required if `task_exec_estimates` is present.

`alpha`- Parameter for exponential weighting of a moving time estimate of the
         true execution/interface time of a task.

#### `tab_sel`

Uses `src_sigmoid_sel` to select which TAB to switch to (if applicable) during
task allocation, with a `method` tag that can be one of [`harwell2019`]. It is
required if the task executive is used.

### `block_sel_matrix`

This tag is optional, but must be defined for all non-CRW controllers.

`nest` - The location of the nest.

#### `block_priorities`

This tag is optional. If a priority is omitted, it defaults to 1.0.

- `cube` - The priority value used as part of block utility calculation for cube
           blocks during block selection.

- `ramp` - The priority value used as part of block utility calculation for ramp
           blocks during block selection.

#### `pickup_policy`

This tag is optional, but if it is included `policy` must be specified.

- `policy` - The policy to use to restrict (1) the conditions under which robots
             can pick up a block that they encounter, (2) which blocks are
             considered valid for acquisition. Valid values are:

    - `cluster_proximity` - Only allow blocks which are within `prox_dist` from
      the average of the positions of the blocks currently known to a robot to
      be picked up. Only makes sense for object distributions in which objects
      are clumped into clusters; used to help depth2 robots not to pickup the
      blocks other robots have dropped in order to start caches.

- `prox_dist` - The minimum distance measure for usage with `cluster_proximity`
                pickup policy.

### `cache_sel_matrix`

This tag is optional, but must be defined for [`depth1`, `depth2`]
controllers. If this tag is defined, then all nested tags and attributes are
required.

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

This tag is optional, but must be defined for all non-CRW controllers.

- `block_strategy` - The strategy robots should use to located blocks when they
                     do not currently know of any and need to find one for the
                     task they are currently doing. Valid values are:

  - `CRW` - Correlated Random Walk

  - `likelihood_search` - Go to the location of the last known block and then
                          begin performing CRW there.

- `cache_strategy` - The strategy robots should use to located caches when they
                     do not currently know of any and need to find one for the
                     task they are currently doing. Required for [`depth1,
                     depth2`] controllers, ignored otherwise.

  - `CRW` - Correlated Random Walk

  - `likelihood_search` - Go to the location of the last known block and then
                          begin performing CRW there.

  - `utility_search` - Use the average location of the known blocks/robot's
                       current location as input into the cache site selection
                       algorithm, then go to the location it returns and begin
                       performing CRW there.

### `sensing`

This tag, and all nested tags and attributes are required for all controllers.

#### `proximity_sensor`

- `angle_range` - The angle range to the left/right of center (90 degrees on a
                  unit circle) in which obstacles are not ignored (outside of
                  this range they are ignored, assuming the robot will be able
                  to drive by them). Takes a pair like so: `-5:5` (for a 10
                  degree window). Specified in radians.

- `delta` - The longest distance away from the robot obstacles will be
            considered.

### `actuation`

This tag, and all nested tags and attributes are required for all controllers.

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

## Loop Functions

The following root XML tags are defined:

| Root XML tag        | Mandatory For? | Description                                                                                                                              |
|---------------------|----------------|------------------------------------------------------------------------------------------------------------------------------------------|
| `output`            |                | Parameters for logging simulation metrics/results.                                                                                       |
| `convergence`       |                | Parameters for computing swarm convergence.                                                                                              |
| `arena_map`         |                | Parameters for discretization of the arena.                                                                                              |
| `oracle`            |                | Parameters for the oracle, which allows swarms to make decisions based on perfect information, to provide an upper bound on performance. |
| `temporal_variance` |                | Parameters for the various types of temporal waveforms that can be applied to swarm/environmental state during simulation.               |
| `caches`            |                | Parameters for the use of caches in the arena.                                                                                           |

### `output`

All nested tags and attributes under this tag are required.

#### `sim`

- `output_root` - The root output directory in which the directories of
                  different simulation runs will be placed.

- `output_dir` - The output directory for the current simulation under
                 `output_root`. If you put the special field `__current_date__`
                 here, the simulation will get a unique output directory in the
                 form YYYY-MM-DD:HH-MM.

#### `metrics`

The following attributes under this tag are required:

- `output_dir` - Name of directory within the output root that metrics will be
                 placed in.

- `collect_interval` - The timestep interval after which statistics will be
                       reset. Gathering statistics on a single timestep of a
                       long simulation is generally not useful; hence this
                       field.

The following attributes under this tag are optional. Not defining them does not
disable metric collection of the given type, but redirects it to a `NULL.csv`
file in the `output_dir` (all excluded tags can overwrite each other in this
file). This is a temporary fix until I have time to do this properly with
/dev/null.

- `fsm_collision_counts_fname`
- `fsm_collision_locs_fname`
- `fsm_movement_fname`
- `block_acq_counts_fname`
- `block_acq_locs_fname`
- `block_acq_explore_locs_fname`
- `block_acq_vector_locs_fname`
- `block_transport_fname`
- `block_manipulation_fname`
- `cache_acq_counts_fname`
- `cache_acq_locs_fname`
- `cache_acq_explore_locs_fname`
- `cache_acq_vector_locs_fname`
- `cache_utilization_fname`
- `cache_lifecycle_fname`
- `cache_locations_fname`
- `task_execution_generalist_fname`
- `task_execution_collector_fname`
- `task_execution_harvester_fname`
- `task_execution_cache_starter_fname`
- `task_execution_cache_finisher_fname`
- `task_execution_cache_transferer_fname`
- `task_execution_cache_collector_fname`
- `task_tab_generalist_fname`
- `task_tab_collector_fname`
- `task_tab_harvester_fname`
- `task_distribution_fname`
- `perception_dpo_fname`
- `perception_mdpo_fname`
- `swarm_dist_pos2D_fname`
- `swarm_convergence_fname`
- `temporal_variance_fname`

### `convergence`

This tag is optional.

- `n_threads` - How many threads will be used for convergence calculations
                during loop functions.

- `epsilon` - Threshold < 1.0 that a convergence measure will be considered
              to have converged when its normalized value is above.

- `epsilon_delta` - # of timesteps the swarm must maintain its normalized
                    convergence measure score of >= `epsilon` in order to be
                    considered converged.

#### `positional_entropy`

A measure of convergence using robot positions, Shannon's entropy definition,
and Balch2000's social entropy measure. This tag is optional, but if it is
defined, only the `enable` attribute is required. All other attributes are
parsed iff `enable` is `true`.

- `enable` - If this measure is enabled or not. Very expensive to compute in
             large swarms.

- `horizon` - A `min:max` pair of distances specifying the min and max spatail
              cluster size that will be used to compute the entropy of robot
              positions.

- `horizon_delta` - Step size for traversing the horizon from min to max.

#### `interactivity`

A measure of convergence using nearest neighbor distances. This tag is optional.

- `enable` - If this measure is enabled or not. Relatively cheap to compute in
             large swarms.

#### `angular_order`

A measure of convergence using congruence of robot orientations. This tag is
optional.

- `enable` - If this measure is enabled or not. Relatively cheap to compute in
             large swarms.

### `oracle`

This tag is optional.

#### `tasking_oracle`

This tag is optional. All attributes default to `false` if they are omitted.

- `task_exec_est` - If enabled, then this will inject perfect estimates of task
                    execution time based on the performance of the entire swarm
                    into each robot when it performs task allocation.

- `task_interface_est` - If enabled, then this will inject perfect estimates of
                         task interface time based on the performance of the
                         entire swarm into each robot when it performs task
                         allocation.

#### `entities_oracle`

This tag is optional, but if it is defined, all nested tags and attributes are
required.

- `blocks_enabled` - Inject perfect knowledge of all block locations into the
                     swarm every timestep.

- `caches_enabled` - Inject perfect knowledge of all cache locations into the
                     swarm every timestep.

### `temporal_variance`

This tag is optional.

#### `blocks`

This tag is optional.

##### `manipulation_penalty`

This tag is optional, but if it is defined, then all nested tags and attributes
are required.

- `waveform` - Parameters defining the waveform of block manipulation penalty
               (picking up/dropping that does not involve caches).

##### `carry_throttle`

This tag is optional, but if it is defined, then all nested tags and attributes
are required.

- `waveform` - Parameters defining the waveform of block carry penalty
               (how much slower robots move when carrying a block).

### `caches`

This tag is optional.

##### `usage_penalty`

This tag is optional, but if it is defined, then all nested tags and attributes
are required.

- `waveform` - Parameters defining the waveform of cache usage penalty (picking
               up/dropping).

### `arena_map`

This tag is required. Some nested tags and/or attributes can be omitted,
depending.

#### `grid`

- `resolution` - The resolution that the arena will be represented at, in terms
                 of the size of grid cells. Must be the same as the value passed
                 to the robot controllers.

- `size` - The size of the arena.

#### `blocks`

This tag and all nested tags and attributes are required.

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

###### `redist_governor`

This tag is optional. If it is defined, then the [`trigger`,
`recurrence_policy`] attributes are required. Other attributes are only required
depending on configuration.

- `trigger` - The trigger for (possibly) stopping block redistribution:

  - `timestep` - Blocks will be redistributed until the specified timestep. This
                 trigger type can be used with the [`single`] recurrence policy.

  - `block_count` - Blocks will be redistributed until the specified # of blocks
                    have been collected. This trigger type can be used with the
                    [`single`] recurrence policy.

  - `convergence` - Blocks will be redistributed until the swarm has
                    converged. This trigger type can be used with the
                    [`single`,`multi`] recurrence policies.

- `recurrence_policy` - The policy for determining how block redistribution
                        status can change as the simulation progresses.

  - `single` - Once the specified trigger is tripped, then block redistribution
               will stop permanently.

  - `multi` - Blocks will be redistributed as long as the specified trigger has
              not been tripped. Once it has been tripped, block distribution
              will stop until the trigger is no longer tripped, in which case it
              will resume.

- `timestep` - The timestep to stop block redistribution at. Only required if
               `trigger` is `timestep`.

- `block_count`- The collection count to stop block redistribution at. Only
                 required if `trigger` is `block_count`.

###### `manifest`

This tag is required. At least one of [`n_cube`, `n_ramp`] must be defined. The
`unit_dim` attribute is required.

- `n_cube` - # Cube blocks that should be used.

- `n_ramp` - # Ramp blocks that should be used.

- `unit_dim` - Unit dimension of blocks. Cube are 1x1 of this, ramp are 2x1 of
               this.

######  `powerlaw`

This tag is only required if the `dist_model` is `powerlaw`.

- `pwr_min` - Minimum power of 2 for cluster sizes.

- `pwr_max` - Maximum power of 2 for cluster sizes.

- `n_clusters` - Max # of clusters the arena.

#### `nest`

This tag and all nested tags and attributes are required.

- `size` - The size of the nest. Must be specified in a tuple like so:
  `0.5, 0.5`. Note the space--parsing does not work if it is omitted.

- `center` - Location for center of the nest (nest is a square).  Must be
             specified in a tuple like so: `1.5, 1.5`. Note the space--parsing
             does not work if it is omitted.

### `caches`

This tag is required for [`depth1`, `depth2`] loop functions.

- `dimension` - The dimension of the cache. Should be greater than the dimension
                for blocks.

#### `static`

This tag is required for [`depth1`] loop functions. If the tag is present, only
the `enable` attribute is required; all other attributes are parsed iff `enable`
is `true`.

- `enable` - If true, then a single static cache will be created in the center
             of the arena. The cache will be replenished by the loop functions
             if robots deplete it, under certain conditions.

- `size` - The number of blocks to use when (re)-creating the static cache. Must
           be >= 2.

- `respawn_scale_factor` - A scale factor controlling how quickly the
                           probability of static cache respawn will grow once
                           the conditions for respawning are met.

#### `dynamic`

This tag is required for [`depth2`] loop functions. If the tag is present, only
the `enable` attribute is required; all other attributes are parsed iff `enable`
is `true`.

- `enable` - If `true`, then the creation of dynamic caches will be enabled.

- `min_dist` - The minimum distance between blocks to be considered for
               cache creation from said blocks.

- `min_blocks` - The minimum # of blocks that need to within `min_dist` from
                 each other to trigger dynamic cache creation.

- `robot_drop_only` - If TRUE, then caches will only be created by intential
                      robot block drops rather than drops due to abort/block
                      distribution after collection.

### `visualization`

This tag and its attributes are optional (attributes default to `false` if they
are omitted).

- `robot_id` - Set to `true` or `false`. If true, robot id is displayed above
               each robot during simulation.

- `robot_los` - Set to `true` or `false`. If true, each robot's approximate line
                of sight is displayed as a red wireframe square during
                simulation.

- `robot_task` - Set to `true` or `false`. If `true`, the current task each robot
              is executing is displayed above it.

- `block_id` - Set to `true` or `false`. If true, each block's id displayed
               above it during simulation.
