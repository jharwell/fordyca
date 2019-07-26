# Controller XML Configuration

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

In the subsections below, multiple choices for an XML attribute value are
separated by a `|` in example XML, and attributes that should be floating point
are specified as `FLOAT` (acceptable range, if applicable, is documented for
each individual attribute).

## `output`

- Required by: all controllers.
- Required child attributes if present: all.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

```xml
<params>
...
<output>
    <output_root="output">
    <output_dir="__current_date__">
</output
...
</params>
```

- `output_root` - The root output directory in which the directories of
                  different simulation runs will be placed. The path specified
                  can be relative or absolute, and will be created if it does
                  not exist.

- `output_dir` - The output directory for the current simulation under
                 `output_root`. If you put the special field
                 `__current_date__` here, the simulation will get a unique
                 output directory in the form YYYY-MM-DD:HH-MM.

## `perception`

- Required by: All but the CRW controller.
- Required child attributes if present: none.
- Required child tags if present: `pheromone`.
- Optional child tags: `grid`. Required by [`MDPO`, `GP-MDPO`, `GRP-MDPO`]
  controllers.
- Optional child attributes: none.

XML configuration:

```xml
<params>
...
<perception>
    <pheromone>
    ...
    <pheromone/>
    <grid>
    ...
    <grid/>
</perception>
...
</params>
```

### `pheromone`

- Required child attributes if present: `rho`.
- Required child tags if present: none.
- Optional child attributes: `repeat_deposit`.
- Optional child tags: none.

XML configuration:

```xml
<perception>
    ...
    <pheromone rho="FLOAT"
               repeat_deposit="false"/>
    ...
</perception>
```

- `rho` How fast the relevance of information about a particular cell within a
        robot's 2D map of the world loses relevance. Should be < 1.0.

- `repeat_deposit` - If `true`, then repeated pheromone deposits for
                     blocks/caches a robot already knows about will be
                     enabled. `rho` should be updated accordingly, probably to a
                     larger value to enable faster decay. Default=`false` (can
                     be omitted).


### `grid`

- Required by: [`MDPO`, `GP-MDPO`, `GRP-MDPO`] controllers.
- Required child attributes if present: [`resolution`, `size`]
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: none.

```xml
<perception>
    ...
    <grid resolution="0.2"
          size="12, 6, 2"/>
    ...
</perception>
```

- `resolution` - The size of the cells the arena is broken up (discretized)
                 into. Should probably be the same as whatever the block size
                 is, to make things easy.

- `size` - The size of the arena, specified as "X, Y, Z" (the spaces are
           mandatory).

## `task_executive`

- Required by: none. Used by all [depth1, depth2] controllers with the default
               values shown below if it is omitted.
- Required child attributes if present: none.
- Required child tags if present: none.
- Optional child attributes: [`update_exec_ests`, `update_interface_ests`,
  `tab_init_method`]
- Optional child tags: none.

XML configuration:

```xml
<params>
...
    <task_executive
        update_exec_ests="false",
        update_interface_ests="false",
        tab_init_method="root"/>
...
</params>
```

- `update_exec_ests` - If `true`, then the executive will use the elapsed time
                       since a task started to update the task time
                       estimate. Estimate is updated on both abort an
                       completion.

- `update_interface_ests` - If `true`, then the executive will use the
                            calculated interface time for a task to update the
                            interface estimate for the task. Estimate is updated
                            on both abort and completion.

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

## `task_alloc`

XML configuration:

- Required by: [depth1, depth2] controllers.
- Required child attributes if present: none.
- Required child tags if present: [`task_abort`, `task_partition`, `subtask_sel`].
- Optional child attributes: none.
- Optional child tags: [`task_exec_estimates`, `tab_sel` (required by depth2
  controllers)].

XML configuration:

```xml
<params>
...
<task_alloc>
    <task_abort>
    ...
    </task_abort>
    <task_partition>
    ...
    </task_partition>
    <subtask_sel>
    ...
    </subtask_sel>
    <task_exec_estimates>
    ...
    </task_exec_estimates>
    <tab_sel>
    ...
    </tab_sel>

</task_alloc>
...
</params>
```
Many child tags in `<task_alloc>` use sigmoid- based functions for choosing
between alternatives, with the input src and sigmoid method varying. For such
tags, all child attributes and tags are required.

```xml
...
<src_sigmoid_sel
    input_src="exec|interface">
    <sigmoid_sel
    method="harwell2018">
        <sigmoid reactivity="FLOAT"
                 offset="FLOAT"
                 gamma="FLOAT"/>
    <sigmoid_sel/>
</src_sigmoid_sel>
...
```

- `input_src` - Can be `exec` or `interface`, indicating that estimates of
                execution/interface times should be used in the selection
                process.

- `method` - The method used to calculate a probability using the selected input
             source.

- `reactivty` - Once the `offset` is tripped, this parameter controls how
                fast the probability a robot aborts its current task grows.

- `offset` - A positive proportition indicating what ratio of measured execution
             time to the robot's best estimate of the actual execution time of
             the task is considered to be the threshold for a task taking too
             long, and should be aborted.

- `gamma` - A scaling factor that is applied to the overall calculated
            probability.


### `task_abort`

- Required by: [depth1, depth2] controllers.
- Required child attributes if present: none.
- Required child tags if present: `src_sigmoid_sel`.
- Optional child attributes: none.
- Optional child tags: none.

`method=harwell2018` is required.

XML configuration:

```xml

<task_alloc>
    ...
    <task_abort>
        <src_sigmoid_sel
            input_src="exec|interface">
            <sigmoid_sel
                method="harwell2018">
                <sigmoid reactivity="FLOAT"
                         offset="FLOAT"
                         gamma="FLOAT"/>
            <sigmoid_sel/>
        </src_sigmoid_sel>
    </task_abort>
</task_alloc>
...
</params>
```

### `task_partition`

- Required by: [depth1, depth2[] controllers.
- Required child attributes if present: none.
- Required child tags if present: `src_sigmoid_sel`.
- Optional child attributes: [`always_partition`, `never_partition`].
- Optional child tags: none.

XML configuration:

```xml
<task_alloc>
    ...
    <task_partition
        always_partition="false"
        never_partition="false">
        <src_sigmoid_sel
            input_src="exec|interface">
            <sigmoid_sel
                method="pini2011">
                <sigmoid reactivity="FLOAT"
                         offset="FLOAT"
                         gamma="FLOAT"/>
            <sigmoid_sel/>
        </src_sigmoid_sel>
    </task_partition>
</task_alloc>
```

- `always_partition` - If `true`, then robots will always choose to partition a
                       task, given the chance. Default=`false`.

- `never_partition` - If `true`, then robots will never choose to partition a
                       task, given the chance. Default=`false`.

`method` tag can be one of [`pini2011`] for performing the stochastic
partitioning decision. Calculated once upon each task allocation, after the
previous task is finished or aborted.

### `subtask_sel`

- Required by: [depth1, depth2] controllers.
- Required child attributes if present: none.
- Required child tags if present: `src_sigmoid_sel`.
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

```xml
<task_alloc>
    ...
    <subtask_sel>
        <src_sigmoid_sel
            input_src="exec|interface">
            <sigmoid_sel
                method="harwell2018|random">
                <sigmoid reactivity="FLOAT"
                         offset="FLOAT"
                         gamma="FLOAT"/>
            <sigmoid_sel/>
        </src_sigmoid_sel>
    </subtask_sel>
    ...
</task_alloc>
```
`method` tag can be one of [`harwell2018`,`random`] to perform stochastic
subtask selection if partitioning is employed.

### `task_exec_estimates`

- Required by: none.
- Required child attributes if present: None.
- Required child tags if present: `ema` (only if `seed_enabled` is `true`).
- Optional child attributes: all. Only the task names used by the loaded task
  decomposition graph are required; others are ignored.
- Optional child tags: none.

```xml
<task_alloc>
    ...
    <task_exec_estimates
        seed_enabled="false"
        task_name="2000:4000"
        <ema alpha="FLOAT"/>
    </task_exec_estimates>
    ...
</task_alloc>
```

- `seed_enabled` - If `true`, then all estimates of task execution times are
                   initialized randomly within the specified ranges, rather than
                   with zero, in order to avoid any possibly weird behavior on
                   system startup. Default=`false`.

- `<task name>` - Takes a pair like so: `100:200` specifying the range of the
                  uniform random distribution over which a robots' initial
                  estimation of the duration of the specified task will be
                  drawn. Only used if `seed_enabled` is `true`. Valid values for
                  `<task_task_name>` are: [`generalist`, `collector`, `harvester`,
                  `cache_starter`, `cache_finisher`, `cache_transferer`
                  `cache_collector`].


- `alpha`- Parameter for exponential weighting of a moving time estimate of the
           true execution/interface time of a task. Must be < 1.0.

### `tab_sel`

- Required by: Depth2 controllers.
- Required child attributes if present: `src_sigmoid_sel`.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

```xml
<task_alloc>
    ...
    <tab_sel>
        <src_sigmoid_sel
            input_src="exec|interface">
            <sigmoid_sel
                method="harwell2019">
                <sigmoid reactivity="FLOAT"
                         offset="FLOAT"
                         gamma="FLOAT"/>
            <sigmoid_sel/>
        </src_sigmoid_sel>
    </tab_sel>
    ...
</task_alloc>
```

`method` tag that can be one of [`harwell2019`].

## `block_sel_matrix`

- Required by: all but CRW controller.
- Required child attributes if present: `nest`.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: [`block_priorities`, `pickup_policy`].

XML configuration:

```xml
<params>
...
<block_sel_matrix
    nest="6, 3">
    <block_priorities>
    ...
    </block_priorities>
    <pickup_policy>
    ...
    </pickup_policy>
</block_sel_matrix>
...
</params>
```

`nest` - The location of the nest.

### `block_priorities`

- Required by: none.
- Required child attributes if present: `nest`.
- Required child tags if present: none.
- Optional child attributes: [`cube`, `ramp`].
- Optional child tags: none.

XML configuration:

```xml
<block_sel_matrix>
    ...
    <block_priorities
    cube="1.0"
    ramp=1.0/>
    ...
</block_sel_matrix>
```

- `cube` - The priority value used as part of block utility calculation for cube
           blocks during block selection. Default=1.0.

- `ramp` - The priority value used as part of block utility calculation for ramp
           blocks during block selection. Default = 1.0

### `pickup_policy`

- Required by: none.
- Required child attributes if present: `policy`.
- Required child tags if present: none.
- Optional child attributes: [`cluster_proximity`, `prox_dist`].
- Optional child tags: none.

XML configuration:

```xml
<block_sel_matrix>
    ...
    <pickup_policy
    policy=""
    prox_dist="FLOAT"/>
    ...
</block_sel_matrix>
```

- `policy` - The policy to use to restrict (1) the conditions under which robots
             can pick up a block that they encounter, (2) which blocks are
             considered valid for acquisition. Valid values are:

    - `cluster_proximity` - Only allow blocks which are within `prox_dist` from
                            the average of the positions of the blocks currently
                            known to a robot to be picked up. Only makes sense
                            for object distributions in which objects are
                            clumped into clusters; used to help depth2 robots
                            not to pickup the blocks other robots have dropped
                            in order to start caches.

    - Can also be an empty string to disable if the the tag `pickup_policy`is
      present.

- `prox_dist` - The minimum distance measure for usage with `cluster_proximity`
                pickup policy.

## `cache_sel_matrix`

- Required by: [depth1, depth2] controllers.
- Required child attributes if present: all.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: `pickup_policy`.

XML configuration:

```xml
<params>
...
<cache_sel_matrix>
    cache_prox_dist="FLOAT"
    nest_prox_dist="FLOAT"
    block_prox_dist="FLOAT"
    site_xrange_dist="FLOAT:FLOAT"
    cache_prox_dist="FLOAT:FLOAT"
    <pickup_policy>
    ...
    </pickup_policy>
</cache_sel_matrix>
...
</params>
```
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

### `pickup_policy`

- Required by: [depth1, depth2] controllers.
- Required child attributes if present: `policy`.
- Required child tags if present: none.
- Optional child attributes: [`timestep`, `cache_size`].
- Optional child tags: none.

XML configuration:

```xml
<cache_sel_matrix>
    ...
    <pickup_policy
        policy=""
        timestep="INTEGER"
        cache_size="INTEGER"/>
    ...
</cache_sel_matrix>
...
</params>
```

- `policy` - The policy to use to restrict (1) the conditions under which robots
             can pick up from a cache that they encounter, (2) which caches are
             considered valid for acquisition. Valid values are:

  - `cache_size` - Only allow robots to pickup from caches with at least
    `cache_size` blocks in them. Robots intending to drop blocks in caches are
    not restricted.

  - `timestep` - Only allow robots to pickup from caches after `timestep`
    timesteps have elapsed during simulation. Robots intending to drop blocks in
    caches are not restricted.

  - Can also be an empty string to disable the cache pickup policy if the
    `pickup_policy` tag is present.

## `exploration`

- Required by: all but CRW controller.
- Required child attributes if present: `block_strategy`.
- Required child tags if present: none.
- Optional child attributes: [`cache_strategy`].
- Optional child tags: none.

XML configuration:

```xml
<params>
...
<exploration>
    block_strategy="CRW|likelihood_search"
    cache_strategy="CRW|likelihood_search|utility_search|ledtaxis_search"
</exploration>
...
</params>
```

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

  - `ledtaxis_search` - Use the sensing information given off by a cache to
                        perform LEDtaxis towards it, and then perform CRW once a
                        robot is sufficiently close.

## `sensing`

- Required by: all.
- Required child attributes if present: `los_dim`.
- Required child tags if present: `proximity_sensor`
- Optional child attributes: none.
- Optional child tags: none.

```xml
<params>
...
<sensing
    los_dim="0.4">
    <proximity_sensor>
    ...
    </proximity_sensor>
</sensing>
...
</params>
```

### `proximity_sensor`

- Required by: all.
- Required child attributes if present: `los_dim`.
- Required child tags if present: `proximity_sensor`
- Optional child attributes: none.
- Optional child tags: none.

```xml
<sensing>
    ...
    <proximity_sensor
        angle_range="FLOAT:FLOAT"
        delta="FLOAT"/>
</sensing>
```
- `angle_range` - The angle range to the left/right of center (90 degrees on a
                  unit circle) in which obstacles are not ignored (outside of
                  this range they are ignored, assuming the robot will be able
                  to drive by them). Takes a pair like so: `-1:1`. Specified in
                  radians.

- `delta` - Tripping threshold for exponential distance calculations for
            obstacle detection.

## `actuation`

- Required by: all.
- Required child attributes if present: none.
- Required child tags if present: [`steering2D`, `differential_drive`]
- Optional child attributes: none.
- Optional child tags: none.

```xml
<params>
...
<actuation>
    <steering2D>
    ...
    </steering2D>
    <differential_drive>
    ...
    </differential_drive>
</actuation>
...
</params>
```

### `steering2D`

- Required by: all.
- Required child attributes if present: none.
- Required child tags if present: [`avoidance_force`, `arrival_force`,
  `wander_force`, `phototaxis_force`].
- Optional child attributes: none.
- Optional child tags: none.

```xml
<actuation>
    ...
    <steering2D>
    <avoidance_force lookahead="FLOAT"
                     max="FLOAT"/>
    <arrival_force slowing_radius="FLOAT"
                   slowing_speed_min="FLOAT"
                   max="FLOAT"/>
    <wander_force circle_distance="FLOAT"
                  circle_radius_min="FLOAT"
                  max_angle_delta="FLOAT"
                  max="FLOAT"
                  interval="INTEGER"
                  normal_dist="false"/>
    <phototaxis_force max="FLOAT"/>
    </steering2D>
    ...
</actuation>

```
#### `avoidance_force`

- `lookahead` - How far ahead of the robot to look for obstacles. Currently
  unused, but may be used in the future.

- `max` - Max value for the force.

#### `arrival_force`

- `slowing_radius` - Radius around target inside which robots will slow down
  linearly to not overshoot their target.

- `slowing_speed_min` - The minimum speed robotics will linearly ramp down
  to. Should be > 0.

- `max` - Max value for the force.

#### `wander_force`

- `circle_distance` - Scaling factor for force; applied to current velocity.

- `circle_radius` - Displacement (i.e. wander) circle radius; placed at
                    `circle_distance` from the robot.

- `max_angle_delta` -  +/- Maximum amount of heading change for the wander angle
  (a random value is chosen in this range). Specified in degrees.

- `max` - Max value for the force.

- `interval` - How many timesteps to skip between applying the force.

- `normal_dist` - Should the deviations be drawn from a uniform distribution
  (default), or from a normal distribution?

#### `phototaxis_force`

- `max` - Max value for the force.

### `differential_drive`

- Required by: all.
- Required child attributes if present: none.
- Required child tags if present: [`avoidance_force`, `arrival_force`,
  `wander_force`, `phototaxis_force`].
- Optional child attributes: none.
- Optional child tags: none.

```xml
<actuation>
    ...
    <differential_drive soft_turn_max="FLOAT"
                        max_speed="FLOAT"/>
    ...
</actuation>
```

- `soft_turn_max` - If actuators are told to change to a heading within a
                    difference greater than the one specified by this parameter
                    to the current heading, a hard turn is executed (spin in
                    place). Specified in degrees.

- `max_speed` - The maximimum speed of the robot.
