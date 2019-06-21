# Loop Functions XML Configuration

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
