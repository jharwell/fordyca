/**
 * \file perception_fwd.hpp
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

#ifndef INCLUDE_FORDYCA_SUBSYSTEM_PERCEPTION_PERCEPTION_FWD_HPP_
#define INCLUDE_FORDYCA_SUBSYSTEM_PERCEPTION_PERCEPTION_FWD_HPP_

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

#endif /* INCLUDE_FORDYCA_SUBSYSTEM_PERCEPTION_PERCEPTION_FWD_HPP_ */
