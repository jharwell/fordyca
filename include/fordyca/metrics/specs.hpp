/**
 * \file specs.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
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
