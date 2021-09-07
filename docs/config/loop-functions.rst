================================
Loop Functions XML Configuration
================================

The following root XML tags are defined under ``<loop_functions>``:

.. list-table::
   :widths: 25,50,50
   :header-rows: 1

   * - Root XML Tag

     - Mandatory For?

     - Description

   * - ``output``

     - All

     - See :xref:`COSM` docs.

   * - ``convergence``

     - None

     - See :xref:`COSM` docs.

   * - ``arena_map``

     - All

     - See :xref:`COSM` docs.

   * - ``temporal_variance``

     - None

     - See :xref:`COSM` docs.

   * - ``visualization``

     - None

     - See :xref:`COSM` docs.

   * - ``oracle_manager``

     - None

     - See :xref:`COSM` docs.

   * - ``caches``

     - Depth1, depth2 controllers

     - Parameters for the use of caches in the arena.

Any of the following attributes can be added under the ``metrics`` tag in place
of one of the ``<append>,<create>,<truncate>`` tags, in addition to the ones
specified in :xref:`COSM`. Not defining them disables metric collection of the
given type.

.. list-table::
   :widths: 25,50,50
   :header-rows: 1

   * - XML Attribute
     - Description
     - Allowable Output Modes
     - Notes


   * - ``fsm_interference_counts``

     - See :xref:`COSM` docs.

   * - ``fsm_interference_locs2D``

     - See :xref:`COSM` docs.

   * - ``fsm_movement``

     - See :xref:`COSM` docs.

   * - ``block_acq_counts``

     - See :xref:`COSM` docs.

   * - ``block_acq_locs2D``

     - See :xref:`COSM` docs.

   * - ``block_acq_explore_locs2D``

     - See :xref:`COSM` docs.

   * - ``block_acq_vector_locs2D``

     - See :xref:`COSM` docs.

   * - ``block_transport``

     - See :xref:`COSM` docs.

   * - ``task_distribution``

     - See :xref:`COSM` docs.

   * - ``swarm_dist_pos2D``

     - See :xref:`COSM` docs.

   * - ``swarm_convergence``

     - See :xref:`COSM` docs.

   * - ``tv_population``

     - See :xref:`COSM` docs.

   * - ``oracle_manager``

     - See :xref:`COSM` docs.

   * - ``block_manipulation``

     - Free block pickup/drop counts/penalties.

     - append

   * - ``cache_acq_counts``

     - Counts of robots exploring for, vectoring to, and acquiring caches.

     - append

   * - ``cache_acq_locs2D``

     - Spatial distribution of where robots acquire caches.

     - create/truncate

   * - ``cache_acq_explore_locs2D``

     - Spatial distribution of robots exploring for caches.

     - create/truncate

   * - ``cache_acq_vector_locs2D``

     - Spatial distribution of robots vectoring to caches.

     - create/truncate

   * - ``cache_utilization``

     - Average block pickup/drop rates within caches.

     - append

   * - ``cache_lifecycle``

     - Depletion/creation rates of caches in the arena.

     - append

   * - ``cache_locations``

     - Spatial distribution of the locations of caches in the arena.

     - create/truncate

   * - ``cache_site_selection``

     - Cache site selection counts, NLOpt insights.

     - append

   * - ``task_execution_generalist``

     - Execution time/estimate, interface time/estimate, completion/abort
       counts.

     - append

   * - ``task_execution_collector``

     - Execution time/estimate, interface time/estimate, completion/abort
       counts.

     - append

   * - ``task_execution_harvester``

     - Execution time/estimate, interface time/estimate, completion/abort
       counts.

     - append

   * - ``task_execution_cache_starter``

     - Execution time/estimate, interface time/estimate, completion/abort
       counts.

     - append

   * - ``task_execution_cache_finisher``

     - Execution time/estimate, interface time/estimate, completion/abort
       counts.

     - append

   * - ``task_execution_cache_transferer``

     - Execution time/estimate, interface time/estimate, completion/abort
       counts.

     - append

   * - ``task_execution_cache_collector``

     - Execution time/estimate, interface time/estimate, completion/abort
       counts.

     - append

   * - ``task_tab_generalist``

     - TAB task allocation probabilities/counts.

     - append

   * - ``task_tab_collector``

     - TAB task allocation probabilities/counts.

     - append

   * - ``task_tab_harvester``

     - TAB task allocation probabilities/counts.

     - append

   * - ``task_distribution``

     - TAB task allocation probabilities/counts.

     - append

   * - ``perception_dpo``

     - Metrics from each robots' decaying pheromone store.

     - append

   * - ``perception_mdpo``

     - Metrics from each robot's internal map of the arena.

     - append

   * - ``tv_environment``

     - Waveforms of the penalties applied to the swarm.

     - append


Extend the temporal variance capabilities in :xref:`COSM` with caches:

``temporal_variance/env_dynamics/caches``
"""""""""""""""""""""""""""""""""""""""""

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


- ``waveform`` - Parameters defining the waveform of cache usage penalty
  (picking up/dropping).

Extend the arena map capabilities in :xref:`COSM` with caches:

``arena_map/caches``
--------------------

- Required by: [depth1, depth2 controllers].
- Required child attributes if present: [ ``dimension``, ``strict_constraints`` ].
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: [ ``static``, ``dynamic`` ].

XML configuration:

.. code-block:: XML

   <arena_map>
       ...
       <caches
           dimension="FLOAT"
           strict_constraints="true">
           <static>
               ...
           </static>
           <dynamic>
               ...
           </dynamic>
       </caches>
       ...
   </arena_map>

- ``dimension`` - The dimension of the cache. Should be greater than the
  dimension for blocks.

- ``strict_constraints`` - If `true`, then created caches will not be checked
  for overlap with block clusters in the arena after creation (this happens in
  non-error contexts with static caches and RN block distributions, for
  example). Other sanity checks will still be performed and appropriate error
  messages issued; however, an "OK" return code will always be returned.

  For dynamic cache creation, if `true`, cache creation will be strict, meaning
  that any caches that fail validation after creation will be discarded. This
  can happen because when robots select cache sites they only consider the
  distance between the `center` of existing caches/blocks/nests/etc, and do not
  take the extent into consideration. Depending on what the values of the
  various proximity constraints robots use when searching for a cache site,
  validation can fail after cache creation.

  For dynamic cache creation, if `false`, then dynamically created caches will
  be kept regardless if they violate constraints or not, which MIGHT be OK, or
  MIGHT cause issues/segfaults. Provided as an option so that it will be
  possible to more precisely duplicate the results of papers run with earlier
  versions of FORDYCA which had more bugs.

  For static cache creation, caches are never discarded; however if one or more
  caches fail validation after creation, an assert will be triggered if set to
  `true`.

  Default if omitted: `true`.


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
           robot_drop_only="false" />
       ...
   </caches>

- ``enable`` - If `true`, then the creation of dynamic caches will be enabled.

- ``min_dist`` - The minimum distance between blocks to be considered for
  cache creation from said blocks.

- ``min_blocks`` - The minimum # of blocks that need to within ``min_dist`` from
  each other to trigger dynamic cache creation.

- ``robot_drop_only`` - If `true`, then caches will only be created by
  intentional robot block drops rather than drops due to abort/block
  distribution after collection. Default if omitted: `false`.
