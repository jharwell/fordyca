Loop Functions XML Configuration
================================

The following root XML tags are defined under ``<loop_functions>`` in addition to the ones specified in :xref:`COSM`.

+------------------------+----------------------------+------------------------------------------------------------------------------------------------------------------------------+
| Root XML tag           | Mandatory For?             | Description                                                                                                                  |
+------------------------+----------------------------+------------------------------------------------------------------------------------------------------------------------------+
| ``caches``             | Depth1, depth2 controllers | Parameters for the use of caches in the arena.                                                                               |
+------------------------+----------------------------+------------------------------------------------------------------------------------------------------------------------------+

Any of the attributes can be added under the ``metrics`` tag in place of one of
the ``<...>`` above. Not defining them disables metric collection of the given
type.

+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| XML attribute                                  | Description                                                                   | Additional Notes                                 |
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
| ``tv_environment``                             | Waveforms of the penalties applied to the swarm.                              | Output every timestep.                           |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+

Extend the temporal variance capabilities in :xref:`COSM` with caches:

``temporal_variance/env_dynamics/caches``
#########################################

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
