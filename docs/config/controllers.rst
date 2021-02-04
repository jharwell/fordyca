Controller XML Configuration
============================

The following controllers are available:

.. list-table::
   :widths: 25,50,50
   :header-rows: 1

   * - Controller

     - Required Loop Functions

     - Notes

   * - crw

     - d0

     - CRW = Correlated Random Walk.

   * - dpo

     - d0

     - DPO = Mapped Decaying Pheromone Object. Uses pheromones to track objects within the arena.


   * - mdpo

     - d0

     - MDPO = Mapped Decaying Pheromone Object. DPO + mapped extent of the arena
       tracking relevance of individual cells within it.

   * -  odpo

     - d0

     - ODPO = Oracular DPO. Has perfect information about blocks in thye arena.

   * -  omdpo

     - d0

     - OMDPO = Oracular MDPO. Has perfect information about blocks in the arena.

   * -  bitd\_dpo

     - d1

     - Greedy task partitioning + DPO. Requires static caches to also be enabled.

   * -  bitd\_odpo

     - d1

     - Greedy task partitioning + DPO + oracle (perfect knowledge, as
       configured). Requires static caches, oracle to be enabled.

   * -  bitd\_mdpo

     - d1

     - Greedy task partitioning + MDPO. Requires static caches, oracle to be
       enabled.

   * -  bitd\_omdpo

     - d1

     - Greedy task partitioning + MDPO + oracle (perfect knowledge, as
       configured). Requires static caches, oracle to be enabled.

   * -  birtd\_dpo

     - d2

     - Recursive greedy task partitioning + DPO. Requires dynamic caches to be
       enabled.

   * -  birtd\_mdpo

     - d2

     - Recursive greedy task partitioning + MDPO. Requires dynamic caches to be
       enabled.

   * -  birtd\_odpo

     - d2

     - Recursive greedy task partitioning + DPO + oracle (perfect knowledge, as
       configured). Requires dynamic caches, oracle to be enabled.

   * -  birtd\_omdpo

     - d2

     - Recursive greedy task partitioning + MDPO + oracle (perfect knowledge, as
       configured). Requires dynamic caches, oracle to be enabled.



The following root XML tags are defined under ``<params>`` Parameters for robot exploration, collision avoidance,
                                                           Parameters for robot
                                                           exploration,
                                                           collision avoidance,

.. list-table::
   :widths: 25,50,50
   :header-rows: 1

   * - Root XML Tag

     - Mantory For ?

     - Notes

   * - ``perception``

     - All but CRW

     - See :xref:`COSM` docs also; only augmented slightly here.

   * - ``block_sel_matrix``

     - All but CRW

     - Parameters used by robots when selecting which block to acquire
       as part of the task they are currently executing.

   * - ``cache_sel_matrix``

     - All d1, d2 controllers

     - Parameters used by robots when selecting which cache to acquire as part
       of the task they are currently executing.

   * - ``sensing_subsystemQ3D``

     - All controllers

     - See :xref:`COSM` docs.

   * - ``actuation_subsystem2D``

     - All controllers

     - See :xref:`COSM` docs.

   * - ``strategy``

     - All controllers

     - Parameters for robot exploration, collision avoidance, etc. strategies.

   * - ``task_executive``

     - d1, d2 controllers

     - See :xref:`COSM` docs.

   * - ``task_alloc``

     - d1, d2 controllers

     - See :xref:`COSM` docs.


``block_sel_matrix``
--------------------

- Required child attributes if present: ``nest``.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: [``block_priorities``, ``pickup_policy`` ].

XML configuration:

.. code-block:: XML

   <block_sel_matrix
       nest="6, 3">
       <block_priorities>
       ...
       </block_priorities>
       <pickup_policy>
       ...
       </pickup_policy>
   </block_sel_matrix>

``nest`` - The location of the nest.

``block_sel_matrix/block_priorities``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Required by: None. If omitted, the default priority values shown below are
  used.
- Required child attributes if present: ``nest``.
- Required child tags if present: none.
- Optional child attributes: [``cube``, ``ramp`` ].
- Optional child tags: none.

XML configuration:

.. code-block:: XML

    <block_sel_matrix>
        ...
        <block_priorities
        cube="1.0"
        ramp=1.0/>
        ...
    </block_sel_matrix>


- ``cube`` - The priority value used as part of block utility calculation for cube
  blocks during block selection. Default if omitted: 1.0

- ``ramp`` - The priority value used as part of block utility calculation for
  ramp blocks during block selection. Default if omitted: 1.0

``block_sel_matrix/pickup_policy``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Required by: None.
- Required child attributes if present: ``policy``.
- Required child tags if present: none.
- Optional child attributes: [``cluster_proximity``, ``prox_dist`` ].
- Optional child tags: none.

XML configuration:

.. code-block:: XML

    <block_sel_matrix>
        ...
        <pickup_policy
        policy=""
        prox_dist="FLOAT"/>
        ...
    </block_sel_matrix>


- ``policy`` - The policy to use to restrict (1) the conditions under which
  robots can pick up a block that they encounter, (2) which blocks are
  considered valid for acquisition. Valid values are:

    - ``cluster_proximity`` - Only allow blocks which are within ``prox_dist``
      from the average of the positions of the blocks currently known to a robot
      to be picked up. Only makes sense for object distributions in which
      objects are clumped into clusters; used to help d2 robots not to
      pickup the blocks other robots have dropped in order to start caches.

    - ``""`` - An empty string to disable if the the tag ``pickup_policy`` is
      present.

- ``prox_dist`` - The minimum distance measure for usage with
  ``cluster_proximity`` pickup policy.

``cache_sel_matrix``
--------------------

- Required by: [d1, d2] controllers.
- Required child attributes if present: all.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: ``pickup_policy``.

XML configuration:

.. code-block:: XML

   <cache_sel_matrix
       cache_prox_dist="FLOAT"
       nest_prox_dist="FLOAT"
       block_prox_dist="FLOAT"
       site_xrange_dist="FLOAT:FLOAT"
       cache_prox_dist="FLOAT:FLOAT">
           <pickup_policy>
           ...
           </pickup_policy>
   </cache_sel_matrix>

- ``cache_prox_dist`` - When executing the Cache Finisher task, the constraint
  applied to new cache selection for how close the chosen new cache can be to
  known existing caches. Should be at least twice the size of a cache for Cache
  Finisher robots to behave properly and not get stuck in infinite loops
  attempting to drop a block too close to a known cache.

- ``block_prox_dist`` - When executing the Cache Starter task, the constraint
  applied to cache site selection for how close the chosen cache site can be to
  known blocks.

- ``nest_prox_dist`` - When executing the Cache Starter task, the constraint
  applied to cache site selection for how close the chosen cache site can be to
  the nest.

- ``site_xrange`` - The valid X range for cache site selection (should be a
  subset of the full arena X size, to avoid robots being able to select
  locations by arena boundaries).

- ``site_yrange`` - The valid Y range for cache site selection (should be a
  subset of the full arena Y size, to avoid robots being able to select
  locations by arena boundaries).

``cache_sel_matrix/pickup_policy``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Required by: [d1, d2] controllers.
- Required child attributes if present: ``policy``.
- Required child tags if present: none.
- Optional child attributes: [``timestep``, ``cache_size`` ].
- Optional child tags: none.

XML configuration:

.. code-block:: XML

    <cache_sel_matrix>
        ...
        <pickup_policy
            policy=""
            timestep="INTEGER"
            cache_size="INTEGER"/>
        ...
    </cache_sel_matrix>

- ``policy`` - The policy to use to restrict (1) the conditions under which
  robots can pick up from a cache that they encounter, (2) which caches are
  considered valid for acquisition. Valid values are:

  - ``cache_size`` - Only allow robots to pickup from caches with at least
    ``cache_size`` blocks in them. Robots intending to drop blocks in caches are
    not restricted.

  - ``timestep`` - Only allow robots to pickup from caches after ``timestep``
    timesteps have elapsed during simulation. Robots intending to drop blocks in
    caches are not restricted.

  - Can also be an empty string to disable the cache pickup policy if the
    ``pickup_policy`` tag is present.

``strategy``
-------------

- Required by: All controllers.
- Required child attributes if present: None.
- Required child tags if present: [ ``explore``, ``nest_acq`` ].
- Optional child attributes: None.
- Optional child tags: None.

XML configuration:

.. code-block:: XML

   <strategy>
       <explore>
       ...
       </explore>
       <nest_acq>
       ...
       </nest_cq>
   </strategy>

``strategy/explore``
^^^^^^^^^^^^^^^^^^^^

- Required by: All but CRW.
- Required child attributes if present: ``block_strategy``.
- Required child tags if present: None.
- Optional child attributes: ``cache_strategy``.
- Optional child tags: None.

XML configuration:

.. code-block:: XML

   <strategy>
       <explore
          block_strategy="CRW|likelihood_search"
          cache_strategy="CRW|likelihood_search|utility_search|ledtaxis_search"
       </explore>
       ...
   </strategy>

``strategy/nest_acq``
^^^^^^^^^^^^^^^^^^^^^

- Required by: All controllers.
- Required child attributes if present: ``strategy``.
- Required child tags if present: None.
- Optional child attributes: None.
- Optional child tags: None.

XML configuration:

.. code-block:: XML

   <strategy>
       <nest_acq
          strategy="wander|random_thresh|wander_random_thresh"
       </nest_acq>
       ...
   </strategy>


- ``strategy`` - The strategy robots should use once they have entered the nest
  with an object to choose a precise location to drop it at. Valid values are:

  - ``wander`` - Perform phototaxis+wander, avoiding collisions as needed, for a
    random number of timesteps before acquiring the nest.

  - ``random_thresh`` - Perform phototaxis+collision avoidance, choosing a
    random point along the vector pointing from where the robot enters the nest
    to the center to treat as the center/use as the nest acquisition point, and
    phototaxis to that point.

  - ``wander_random_thresh`` - ``random_thresh`` + ``wander``.

Additional notes to :xref:`COSM` controller docs
================================================

``perception``
--------------

- ``grid`` child tag required by [``MDPO``, ``BITD-MDPO``, ``BIRTD-MDPO`` ]

``task_alloc/stoch_nbhd1``
---------------------------------

- ``tab_sel`` child tag required by d2 controllers

``task_alloc/task_exec_estimates``
----------------------------------

Valid values for ``<task_name>`` are:

  - ``generalist``
  - ``collector``
  - ``harvester``
  - ``cache_starter``
  - ``cache_finisher``
  - ``cache_transferer``
  - ``cache_collector``
