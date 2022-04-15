/**
 * \file specs.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * This file is part of FORDYCA.
 *
 * FORDYCA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * FORDYCA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * FORDYCA.  If not, see <http://www.gnu.org/licenses/
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/metrics/specs.hpp"

#include "fordyca/tasks/d0/foraging_task.hpp"
#include "fordyca/tasks/d1/foraging_task.hpp"
#include "fordyca/tasks/d2/foraging_task.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, metrics, specs);

NS_START(perception);

cmspecs::name_spec kDPO = {
  "perception_dpo",
  "perception/dpo",
};
cmspecs::name_spec kMDPO = {
  "perception_mdpo",
  "perception/mdpo",
};

NS_END(perception);

NS_START(blocks);

cmspecs::name_spec kManipulation = { "block_manipulation",
                                     "blocks/manipulation" };

NS_END(blocks);

NS_START(caches);

cmspecs::name_spec kAcqCounts = { "cache_acq_counts", "caches/acq_counts" };

cmspecs::name_spec kAcqLocs2D = { "cache_acq_locs2D", "caches/acq/locs2D" };
cmspecs::name_spec kAcqExploreLocs2D = { "cache_acq_explore_locs2D",
                                         "caches/acq/explore_locs2D" };
cmspecs::name_spec kAcqVectorLocs2D = { "cache_acq_vector_locs2D",
                                        "caches/acq/vector_locs2D" };

cmspecs::name_spec kSiteSelection = { "cache_site_selection",
                                      "caches/site_selection" };
cmspecs::name_spec kUtilization = { "cache_utilization", "caches/utilization" };
cmspecs::name_spec kLocations = { "cache_locations", "caches/locations" };
cmspecs::name_spec kLifecycle = { "cache_lifecycle", "caches/lifecycle" };

NS_END(caches);

NS_START(tasks, exec);

cmspecs::name_spec kGeneralist = { "task_execution_generalist",
                                   "tasks/execution/" +
                                       ftd0::foraging_task::kGeneralistName };
cmspecs::name_spec kHarvester = { "task_execution_harvester",
                                  "tasks/execution/" +
                                      ftd1::foraging_task::kHarvesterName };
cmspecs::name_spec kCollector = { "task_execution_collector",
                                  "tasks/execution/" +
                                      ftd1::foraging_task::kCollectorName };
cmspecs::name_spec kCacheStarter = { "task_execution_cache_starter",
                                     "tasks/execution/" +
                                         ftd2::foraging_task::kCacheStarterName };
cmspecs::name_spec kCacheFinisher = {
  "task_execution_cache_finisher",
  "tasks/execution/" + ftd2::foraging_task::kCacheFinisherName
};
cmspecs::name_spec kCacheTransferer = {
  "task_execution_cache_transferer",
  "tasks/execution/" + ftd2::foraging_task::kCacheTransfererName
};
cmspecs::name_spec kCacheCollector = {
  "task_execution_cache_collector",
  "tasks/execution/" + ftd2::foraging_task::kCacheCollectorName
};

NS_END(exec);
NS_START(tab);

cmspecs::name_spec kGeneralist = { "task_tab_generalist",
                                   "tasks/tab/generalist" };
cmspecs::name_spec kHarvester = { "task_tab_harvester", "tasks/tab/harvester" };
cmspecs::name_spec kCollector = { "task_tab_collector", "tasks/tab/collector" };

NS_END(tab);

NS_END(tasks);

NS_END(specs, metrics, fordyca);
