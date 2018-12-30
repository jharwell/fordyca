/**
 * @file dp_block_set.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_DS_DP_BLOCK_SET_HPP_
#define INCLUDE_FORDYCA_DS_DP_BLOCK_SET_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/common/common.hpp"
#include "fordyca/ds/dpo_set.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace representation { class base_block; }
NS_START(ds);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class dp_block_set : public dpo_set<representation::base_block> {
 public:
  using dpo_set<representation::base_block>::dpo_set;
  using value_type = dpo_set<representation::base_block>::value_type;

  /**
   * @brief Build a string from the list of DP blocks that a robot is tracking
   * for logging.
   */
  std::string to_str(void) const;
};

class const_dp_block_set : public dpo_set<const representation::base_block> {
 public:
  using dpo_set<const representation::base_block>::dpo_set;
  using value_type = dpo_set<representation::base_block>::value_type;
};

NS_END(ds, fordyca);

#endif /* INCLUDE_FORDYCA_DS_DP_BLOCK_SET_HPP_ */
