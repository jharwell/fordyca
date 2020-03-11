/**
 * \file dp_block_map.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_DS_DP_BLOCK_MAP_HPP_
#define INCLUDE_FORDYCA_DS_DP_BLOCK_MAP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/types/type_uuid.hpp"

#include "cosm/repr/base_block2D.hpp"

#include "fordyca/ds/dpo_map.hpp"
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, ds);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class dp_block_map
 * \ingroup ds
 *
 * \brief The block map is a repr of the robot's perception of blocks
 * in the arena. It uses integers as keys, because blocks are mobile (i.e. can
 * move between instants of time where the robot sees them), and
 * inserting/removing blocks from the map using location comparison will not
 * give correct results.
 */
class dp_block_map : public dpo_map<rtypes::type_uuid, crepr::base_block2D> {
 public:
  using dpo_map<rtypes::type_uuid, crepr::base_block2D>::dpo_map;

  /**
   * \brief Build a string from the list of DP blocks that a robot is tracking
   * for logging.
   */
  std::string to_str(void) const;
};

NS_END(ds, fordyca);

#endif /* INCLUDE_FORDYCA_DS_DP_BLOCK_MAP_HPP_ */
