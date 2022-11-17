/**
 * \file perception_fwd.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, subsystem, perception);

/*******************************************************************************
 * Forward Decls
 ******************************************************************************/
class foraging_perception_subsystem;
class dpo_perception_subsystem;
class mdpo_perception_subsystem;
class oracular_info_receptor;

namespace ds {
class dpo_store;
class dpo_semantic_map;
class occupancy_grid;
class dp_block_map;
class dp_cache_map;
} /* namespace ds */

NS_END(perception, subsystem, fordyca);

