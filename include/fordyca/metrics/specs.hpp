/**
 * \file specs.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/metrics/name_spec.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, metrics, specs);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
NS_START(perception);

extern cmspecs::name_spec kDPO;
extern cmspecs::name_spec kMDPO;

NS_END(perception);

NS_START(blocks);

extern cmspecs::name_spec kManipulation;

NS_END(blocks);

NS_START(caches);

extern cmspecs::name_spec kAcqCounts;
extern cmspecs::name_spec kAcqLocs2D;
extern cmspecs::name_spec kAcqExploreLocs2D;
extern cmspecs::name_spec kAcqVectorLocs2D;
extern cmspecs::name_spec kSiteSelection;
extern cmspecs::name_spec kUtilization;
extern cmspecs::name_spec kLocations;
extern cmspecs::name_spec kLifecycle;

NS_END(caches);

NS_START(tasks, exec);


extern cmspecs::name_spec kGeneralist;
extern cmspecs::name_spec kHarvester;
extern cmspecs::name_spec kCollector;
extern cmspecs::name_spec kCacheStarter;
extern cmspecs::name_spec kCacheFinisher;
extern cmspecs::name_spec kCacheTransferer;
extern cmspecs::name_spec kCacheCollector;

NS_END(exec);

NS_START(tab);

extern cmspecs::name_spec kGeneralist;
extern cmspecs::name_spec kHarvester;
extern cmspecs::name_spec kCollector;

NS_END(tab, tasks);

NS_END(specs, metrics, fordyca);
