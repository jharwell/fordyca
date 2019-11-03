Loop Functions XML Configuration
================================

The following root XML tags are defined under ``<loop_functions>``:

+------------------------+----------------------------+------------------------------------------------------------------------------------------------------------------------------+
| Root XML tag           | Mandatory For?             | Description                                                                                                                  |
+------------------------+----------------------------+------------------------------------------------------------------------------------------------------------------------------+
| ``output``             | All controllers            | Parameters for logging simulation metrics/results.                                                                           |
+------------------------+----------------------------+------------------------------------------------------------------------------------------------------------------------------+
| ``convergence``        | None                       | Parameters for computing swarm convergence.                                                                                  |
+------------------------+----------------------------+------------------------------------------------------------------------------------------------------------------------------+
| ``arena_map``          | All controllers            | Parameters for discretization of the arena.                                                                                  |
+------------------------+----------------------------+------------------------------------------------------------------------------------------------------------------------------+
| ``oracle_manager``     | None                       | Parameters for the oracle, which allows swarms to make decisions based on perfect information (performance upper bound).     |
+------------------------+----------------------------+------------------------------------------------------------------------------------------------------------------------------+
| ``temporal_variance``  | None                       | Parameters for the various types of temporal waveforms that can be applied to swarm/environmental state during simulation.   |
+------------------------+----------------------------+------------------------------------------------------------------------------------------------------------------------------+
| ``caches``             | Depth1, depth2 controllers | Parameters for the use of caches in the arena.                                                                               |
+------------------------+----------------------------+------------------------------------------------------------------------------------------------------------------------------+

``output``
----------

- Required by: all controllers.
- Required child attributes if present: all.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <output
       output_root="output"
       output_dir="__current_date__">
       <metrics>
           ...
       </metrics>
   </output>

- ``output_root`` - The root output directory in which the directories of
  different simulation runs will be placed. The path specified can be relative
  or absolute, and will be created if it does not exist.

- ``output_dir`` - The output directory for the current simulation under
  ``output_root``. If you put the special field ``__current_date__`` here, the
  simulation will get a unique output directory in the form
  ``YYYY-MM-DD:HH-MM``.


``output/metrics``
^^^^^^^^^^^^^^^^^^

- Required by: all controllers.
- Required child attributes if present: [ ``output_dir``, ``collect_interval`` ].
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <output>
       ...
       <metrics
           output_dir="metrics"
           output_interval="INTEGER">
           ...
           ...
           ...
           ...
       </metrics>
       ...
   </output>

- ``output_dir`` - Name of directory within the output root that metrics will be
  placed in.

- ``output_interval`` - The timestep interval after which statistics will be
  reset. Gathering statistics on a single timestep of a long simulation is
  generally not useful; hence this field.

Any of the attributes can be added under the ``metrics`` tag in place of one of
the ``<...>`` above. Not defining them disables metric collection of the given
type.

+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| XML attribute                                  | Description                                                                   | Additional Notes                                 |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``fsm_collision_counts``                       | Counts of robots entering, are in, and exiting the collision avoidance state. |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``fsm_collision_locs``                         | Spatial distribution of collision avoidance locations in the arena.           |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``fsm_movement``                               | Swarm average distance traveled/velocity.                                     |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``block_acq_counts``                           | Counts of robots exploring for, vectoring to, and acquiring blocks.           |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``block_acq_locs``                             | Spatial distribution of where robots acquire blocks.                          |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``block_acq_explore_locs``                     | Spatial distribution of robots exploring for blocks.                          |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``block_acq_vector_locs``                      | Spatial distribution of robots vectoring to blocks.                           |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``block_transport``                            | # blocks collected/ # transporters.                                           |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``block_manipulation``                         | Free block pickup/drop counts/penalties.                                      |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``cache_acq_counts``                           | Counts of robots exploring for, vectoring to, and acquiring caches.           |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``cache_acq_locs``                             | Spatial distribution of where robots acquire caches.                          |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``cache_acq_explore_locs``                     | Spatial distribution of robots exploring for caches.                          |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``cache_acq_vector_locs``                      | Spatial distribution of robots vectoring to caches.                           |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``cache_utilization``                          | Average block pickup/drop rates within caches.                                |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``cache_lifecycle``                            | Depletion/creation rates of caches in the arena.                              |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``cache_locations``                            | Spatial distribution of the locations of caches in the arena.                 |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``cache_site_selection``                       | Cache site selection counts, NLOpt insights.                                  |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``task_execution_generalist``                  | Execution time/estimate, interface time/estimate, completion/abort counts.    |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``task_execution_collector``                   | Execution time/estimate, interface time/estimate, completion/abort counts.    |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``task_execution_harvester``                   | Execution time/estimate, interface time/estimate, completion/abort counts.    |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``task_execution_cache_starter``               | Execution time/estimate, interface time/estimate, completion/abort counts.    |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``task_execution_cache_finisher``              | Execution time/estimate, interface time/estimate, completion/abort counts.    |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``task_execution_cache_transferer``            | Execution time/estimate, interface time/estimate, completion/abort counts.    |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``task_execution_cache_collector``             | Execution time/estimate, interface time/estimate, completion/abort counts.    |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``task_tab_generalist``                        | TAB task allocation probabilities/counts.                                     |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``task_tab_collector``                         | TAB task allocation probabilities/counts.                                     |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``task_tab_harvester``                         | TAB task allocation probabilities/counts.                                     |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``task_distribution``                          | TAB task allocation probabilities/counts.                                     |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``perception_dpo``                             | Metrics from each robots' decaying pheromone store.                           |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``perception_mdpo``                            | Metrics from each robot's internal map of the arena.                          |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``swarm_dist_pos2D``                           | Swarm distribution in 2D space.                                               |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``swarm_convergence``                          | Results of swarm convergence calculations.                                    | Requires convergence calculations to be enabled. |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``loop_temporal_variance``                     | Waveforms of the penalties applied to the swarm.                              | Output every timestep.                           |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+

``convergence``
---------------

- Required by: none.
- Required child attributes if present: all.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: [ ``postional_entropy``, ``task_dist_entropy``,
  ``interactivity``, ``angular_order`` ].

XML configuration:

.. code-block:: XML

   <convergence>
       <postional_entropy>
       ...
       </positional_entropy>
       <task_dist_entropy>
       ...
       </task_dist_entropy>
       <interactivity>
       ...
       </interactivity>
       <angular_order>
       ...
       </angular_order>
   </convergence>

- ``n_threads`` - How many threads will be used for convergence calculations
  during loop functions.

- ``epsilon`` - Threshold < 1.0 that a convergence measure will be considered
  to have converged when its normalized value is above.

``convergence/positional_entropy``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A measure of convergence using robot positions, Shannon's entropy definition,
and Balch2000's social entropy measure. If it is defined, only the ``enable``
attribute is required. All other attributes are parsed iff ``enable`` is `true`.

- Required by: none.
- Required child attributes if present: ``enable``.
- Required child tags if present: none.
- Optional child attributes: [ ``horizon``, ``horizon_delta`` ].
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <convergence>
       ...
       <postional_entropy
           enable="false"
           horizon="FLOAT:FLOAT"
           horizon_delta="FLOAT:FLOAT"/>
       ...
   </convergence>


- ``enable`` - If this measure is enabled or not. Very expensive to compute in
  large swarms.

- ``horizon`` - A ``min:max`` pair of distances specifying the min and max
  spatial cluster size that will be used to compute the entropy of robot
  positions. Should be <= arena X,Y dimensions. Only required if ``enable`` is `true`.

- ``horizon_delta`` - Step size for traversing the horizon from min to max. Only
  required if ``enable`` is `true`.


``convergence/interactivity``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A measure of convergence using nearest neighbor distances.

- Required by: none.
- Required child attributes if present: ``enable``.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <convergence>
       ...
       <interactivity
           enable="false"/>
       ...
   </convergence>

- ``enable`` - If this measure is enabled or not. Relatively cheap to compute in
  large swarms.

### ``angular_order``

A measure of convergence using congruence of robot orientations.

- Required by: none.
- Required child attributes if present: ``enable``.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <convergence>
       ...
       <angular_order
           enable="false"/>
       ...
   </convergence>

- ``enable`` - If this measure is enabled or not. Relatively cheap to compute in
  large swarms.

``convergence/angular_order``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A measure of convergence using stability of robot task allocations over time.

- Required by: none.
- Required child attributes if present: ``enable``.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <convergence>
       ...
       <task_dist_entropy
           enable="false"/>
       ...
   </convergence>

- ``enable`` - If this measure is enabled or not. Relatively cheap to compute in
  large swarms.

``oracle_manager``
------------------

- Required by: none.
- Required child attributes if present: none.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: [ ``tasking_oracle``, ``entities_oracle`` ].

XML configuration:

.. code-block:: XML

   <oracle_manager>
       <tasking_oracle>
       ...
       </tasking_oracle>
       <entities_oracle>
       ...
       </entities_oracle>
   </oracle_manager>


``oracle_manager/tasking_oracle``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Required by: none.
- Required child attributes if present: none.
- Required child tags if present: none.
- Optional child attributes: [ ``task_exec_ests``, ``task_interface_ests`` ].
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <oracle_manager>
       ...
       <tasking_oracle
           task_exec_ests="false"
           task_interface_ests="false"/>
       ...
   </oracle_manager>


All attributes default as shown above if omitted.

- ``task_exec_ests`` - If enabled, then this will inject perfect estimates of
  task execution time based on the performance of the entire swarm into each
  robot when it performs task allocation.

- ``task_interface_ests`` - If enabled, then this will inject perfect estimates
  of task interface time based on the performance of the entire swarm into each
  robot when it performs task allocation.

``oracle_manager/entities_oracle``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Required by: none.
- Required child attributes if present: none.
- Required child tags if present: none.
- Optional child attributes: [ ``blocks``, ``caches`` ].
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <oracle_manager>
       ...
       <entities_oracle
           blocks="false"
           caches="false"/>
       ...
   </oracle_manager>

- ``blocks`` - Inject perfect knowledge of all block locations into the
  swarm every timestep.

- ``caches`` - Inject perfect knowledge of all cache locations into the
  swarm every timestep.

``temporal_variance``
---------------------

- Required by: none.
- Required child attributes if present: none.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: [ ``blocks``, ``caches`` ].

XML configuration:

.. code-block:: XML

   <temporal_variance>
       <blocks>
       ...
       </blocks>
       <caches>
       ...
       </caches>
   </temporal_variance>

Subsections in this section make use of the ``waveform`` XML configuration block:

.. code-block:: XML

   <waveform
       type="Null|Sine|Square|Sawtooth|Constant"
       frequency="FLOAT"
       amplitude="FLOAT"
       offset="FLOAT"
       phase="FLOAT"/>


- ``type`` - The type of the waveform. ``Null`` disables the waveform.

Other parameters are self explanatory. ``phase`` is specified in radians.

``temporal_variance/blocks``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Required by: none.
- Required child attributes if present: none.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: [ ``manipulation_penalty``, ``carry_throttle`` ].

XML configuration:

.. code-block:: XML

   <temporal_variance>
       ...
       <blocks>
           <manipulation_penalty>
           ...
           </manipulation_penalty>
           <carry_throttle>
           ...
           </carry_throttle>
           </blocks>
       ...
   </temporal_variance>

``temporal_variance/blocks/manipulation_penalty``
"""""""""""""""""""""""""""""""""""""""""""""""""

- Required by: none.
- Required child attributes if present: none.
- Required child tags if present: ``waveform``.
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <blocks>
       ...
       <manipulation_penalty>
       <waveform>
           ...
       </waveform>
       </manipulation_penalty>
       ...
   </blocks>

- ``waveform`` - Parameters defining the waveform of block manipulation penalty
  (picking up/dropping that does not involve caches).

``temporal_variance/blocks/carry_throttle``
"""""""""""""""""""""""""""""""""""""""""""

- Required by: none.
- Required child attributes if present: none.
- Required child tags if present: ``waveform``.
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <blocks>
       ...
       <carry_throttle>
       <waveform>
       ...
       </waveform>
       </carry_throttle>
       ...
   </blocks>

- ``waveform`` - Parameters defining the waveform of block carry penalty (how
  much slower robots move when carrying a block).

``temporal_variance/caches``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Required by: none.
- Required child attributes if present: none.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: [ ``usage_penalty`` ].

XML configuration:

.. code-block:: XML

   <temporal_variance>
       ...
       <caches>
           <usage_penalty>
           ...
           </usage_penalty>
       </caches>
       ...
   </temporal_variance>

``temporal_variance/caches/usage_penalty``
""""""""""""""""""""""""""""""""""""""""""

- Required by: none.
- Required child attributes if present: none.
- Required child tags if present: ``waveform``.
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <caches>
       ...
       <usage_penalty>
       <waveform>
           ...
       </waveform>
       </usage_penalty>
       ...
   </caches>


- ``waveform`` - Parameters defining the waveform of cache usage penalty (picking
  up/dropping).

``arena_map``
-------------

- Required by: all.
- Required child attributes if present: none.
- Required child tags if present: [ ``grid``, ``blocks``, ``nest`` ].
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <arena_map>
       <grid>
       ...
       </grid>
       <blocks>
       ...
       </blocks>
       <nest>
       ...
       </nest>
   </arena_map>

``arena_map/grid``
^^^^^^^^^^^^^^^^^^

- Required by: all.
- Required child attributes if present: [ ``resolution``, ``size`` ].
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <arena_map>
       ...
       <grid
           resolution="FLOAT"
           size="X, Y, Z"/>
       ...
   </arena_map>

- ``resolution`` - The resolution that the arena will be represented at, in
  terms of the size of grid cells. Must be the same as the value passed to the
  robot controllers.

- ``size`` - The size of the arena.

``arena_map/blocks``
^^^^^^^^^^^^^^^^^^^^

- Required by: all.
- Required child attributes if present: none.
- Required child tags if present: [ ``distribution``, ``manifest`` ].
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <arena_map>
       ...
       <blocks>
           <distribution>
           ...
           </distribution>
           <manifest>
           ...
           </manifest>
       </blocks>
       ...
   </arena_map>

``arena_map/blocks/distribution``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Required by: all.
- Required child attributes if present: ``dist_type``.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: [ ``redist_governor``, ``powerlaw`` ].

XML configuration:

.. code-block:: XML

   <blocks>
       ...
       <distribution
       dist_type="random|powerlaw|single_source|dual_source|quad_source">
       ...
       </distribution>
       ...
   </blocks>

- ``dist_type`` - The distribution model for the blocks. When blocks are
  distributed to a new location in the arena and made available for robots to
  pickup (either initially or after a block is deposited in the nest), they are
  placed in the arena in one of the following ways:

  - ``random``: Placed in a random location in the arena.

  - ``powerlaw``: Distributed according to a powerlaw.

  - ``single_source`` - Placed within an arena opposite about 90% of the way
    from the nest to the other side of the arena (assumes horizontal,
    rectangular arena).

  - ``dual_source`` - Placed in two sources on either side of a central nest
    (assumes a horizontal, rectangular arena).

  - ``quad_source`` - Placed in 4 sources at each cardinal direction in the
    arena. Assumes a square arena.

``arena_map/blocks/distribution/redist_governor``
#################################################

- Required by: none.
- Required child attributes if present: ``trigger``.
- Required child tags if present: none.
- Optional child attributes: [ ``recurrence_policy``, ``timestep``, ``block_count`` ].
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <distribution>
       ...
       <redist_governor
           trigger="Null"
           recurrence_policy="mult|single"
           timestep="INTEGER"
           block_count="INTEGER"/>
       ...
   </distribution>


- ``trigger`` - The trigger for (possibly) stopping block redistribution:

  - ``Null`` - Disables the governor.

  - ``timestep`` - Blocks will be redistributed until the specified timestep. This
                 trigger type can be used with the [ ``single`` ] recurrence policy.

  - ``block_count`` - Blocks will be redistributed until the specified # of
    blocks have been collected. This trigger type can be used with the
    ``single`` recurrence policy.

  - ``convergence`` - Blocks will be redistributed until the swarm has
    converged. This trigger type can be used with the ``single``, ``multi``
    recurrence policies.

- ``recurrence_policy`` - The policy for determining how block redistribution
  status can change as the simulation progresses.

  - ``single`` - Once the specified trigger is tripped, then block
    redistribution will stop permanently.

  - ``multi`` - Blocks will be redistributed as long as the specified trigger
    has not been tripped. Once it has been tripped, block distribution will stop
    until the trigger is no longer tripped, in which case it will resume.

- ``timestep`` - The timestep to stop block redistribution at. Only required if
  ``trigger`` is ``timestep``.

- ``block_count`` - The collection count to stop block redistribution at. Only
  required if ``trigger`` is ``block_count``.

``arena_map/blocks/distribution/manifest``
##########################################

- Required by: all.
- Required child attributes if present: At least one of [ ``n_cube``, ``n_ramp`` ],
  ``unit_dimm``.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: At most one of [ ``n_cube``, ``n_ramp`` ].

XML configuration:

.. code-block:: XML

    <distribution>
        ...
        <manifest
            n_cube="INTEGER"
            n_ramp="INTEGER"
            unit_dim="FLOAT"/>
        ...
    </distribution>


- ``n_cube`` - # Cube blocks that should be used.

- ``n_ramp`` - # Ramp blocks that should be used.

- ``unit_dim`` - Unit dimension of blocks. Cubes are 1x1 of this, ramps are 2x1 of
  this.

``arena_map/blocks/distribution/powerlaw``
##########################################

- Required by: all iff ``dist_type`` is ``powerlaw``.
- Required child attributes if present: [ ``pwr_min``, ``pwr_max``, ``n_clusters`` ].
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <distribution>
       ...
       <powerlaw
           pwr_min="INTEGER"
           pwr_max="INTEGER"
           n_clusters="INTEGER"/>
       ...
   </distribution>

- ``pwr_min`` - Minimum power of 2 for cluster sizes.

- ``pwr_max`` - Maximum power of 2 for cluster sizes.

- ``n_clusters`` - Max # of clusters the arena.

``arena_map/nest``
^^^^^^^^^^^^^^^^^^

- Required by: all.
- Required child attributes if present: [ ``size``, ``center`` ].
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <arena_map>
       ...
       <nest
       size="X, Y"
       center="X, Y"/>
       ...
   </arena_map>


- ``size`` - The size of the nest. Must be specified in a tuple like so:
  ``0.5, 0.5``. Note the space--parsing does not work if it is omitted.

- ``center`` - Location for center of the nest (nest is a square).  Must be
  specified in a tuple like so: ``1.5, 1.5``. Note the space--parsing does not
  work if it is omitted.

``arena_map/caches``
^^^^^^^^^^^^^^^^^^^^

- Required by: [depth1, depth2 controllers].
- Required child attributes if present: [ ``dimension`` ].
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: [ ``static``, ``dynamic`` ].

XML configuration:

.. code-block:: XML

   <arena_map>
       ...
       <caches
           dimension="FLOAT">
           <static>
               ...
           </static>
           <dynamic>
               ...
           </dynamic>
       </caches>
       ...
   </arena_map>

- ``dimension`` - The dimension of the cache. Should be greater than the dimension
  for blocks.

``arena_map/caches/static``
"""""""""""""""""""""""""""

- Required by: [depth1 controllers].
- Required child attributes if present: [ ``enable`` ].
- Required child tags if present: none.
- Optional child attributes: [ ``size``, ``respawn_scale_factor`` ].
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <caches>
       ...
       <static
           enable="false"
           size="INTEGER"
           respawn_scale_factor="FLOAT"/>
       ...
   </caches>


This tag is required for ``depth1`` loop functions. If the tag is present, only
the ``enable`` attribute is required; all other attributes are parsed iff
``enable`` is `true`.

- ``enable`` - If true, then a single static cache will be created in the center
  of the arena. The cache will be replenished by the loop functions if robots
  deplete it, under certain conditions.

- ``size`` - The number of blocks to use when (re)-creating the static
  cache. Must be >= 2.

- ``respawn_scale_factor`` - A scale factor controlling how quickly the
  probability of static cache respawn will grow once the conditions for
  respawning are met.

``arena_map/caches/dynamic``
""""""""""""""""""""""""""""

- Required by: [depth2 controllers].
- Required child attributes if present: ``enable``.
- Required child tags if present: none.
- Optional child attributes: [ ``min_dist``, ``min_blocks``, ``robot_drop_only`` ].
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <caches>
       ...
       <dynamic
           enable="false"
           min_dist="FLOAT"
           min_blocks="INTEGER"
           robot_drop_only="false"/>
       ...
   </caches>

- ``enable`` - If `true`, then the creation of dynamic caches will be enabled.

- ``min_dist`` - The minimum distance between blocks to be considered for
  cache creation from said blocks.

- ``min_blocks`` - The minimum # of blocks that need to within ``min_dist`` from
  each other to trigger dynamic cache creation.

- ``robot_drop_only`` - If `true`, then caches will only be created by intential
  robot block drops rather than drops due to abort/block distribution after
  collection. Default if omitted: `false`.

``visualization``
-----------------

- Required by: none.
- Required child attributes if present: none.
- Required child tags if present: none.
- Optional child attributes: [ ``robot_id``, ``robot_los``, ``robot_task``, ``block_id`` ].
- Optional child tags: none.

XML configuration:

.. code-block:: XML

    <visulation
        robot_id="false"
        robot_los="false"
        robot_task="false"
        block_id="false"/>


Omitted attributes default to the values shown above.

- ``robot_id`` - If `true`, robot id is displayed above each robot during
  simulation. Default if omitted: `false`.

- ``robot_los`` - If `true`, each robot's approximate line of sight is displayed
  as a red wireframe square during simulation. Only applicable to MDPO
  controllers. Default if omitted: `false`.

- ``robot_task`` - If `true`, the current task each robot is executing is
  displayed above it. Default if omitted: `false`.

- ``block_id`` - If `true`, each block's id displayed above it during
  simulation. Default if omitted: `false`.
