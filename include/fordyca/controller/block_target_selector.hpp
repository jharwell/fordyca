/**
 * @file block_target_selector.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
 *
 * This file is part of RCPPSW.
 *
 * RCPPSW is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * RCPPSW is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * RCPPSW.  If not, see <http://www.gnu.org/licenses/
 */

#ifndef INCLUDE_FORDYCA_CONTROLLER_BLOCK_TARGET_SELECTOR_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_BLOCK_TARGET_SELECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include "rcppsw/common/common.hpp"
#include "fordyca/representation/block.hpp"
#include "fordyca/representation/discrete_coord.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class block_target_selector {
 public:
  block_target_selector(representation::discrete_coord nest_loc) :
      m_nest_loc(nest_loc) {}

  representation::block* calc_best(
      const std::list<std::pair<representation::block*, double>> blocks,
      argos::CVector2 robot_loc);


 private:
  representation::discrete_coord m_nest_loc;
};

NS_END(fordyca, controller);

#endif /* INCLUDE_FORDYCA_CONTROLLER_BLOCK_TARGET_SELECTOR_HPP_ */
