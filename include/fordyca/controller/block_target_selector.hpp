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
#include <utility>

#include "rcppsw/common/er_client.hpp"
#include "fordyca/representation/block.hpp"
#include "fordyca/representation/discrete_coord.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class block_target_selector: public rcppsw::common::er_client {
 public:
  block_target_selector(const std::shared_ptr<rcppsw::common::er_server>& server,
                        argos::CVector2 nest_loc);

  std::pair<const representation::block*, double> calc_best(
      const std::list<std::pair<const representation::block*, double>> blocks,
      argos::CVector2 robot_loc);

 private:
  argos::CVector2 m_nest_loc;
};

NS_END(fordyca, controller);

#endif /* INCLUDE_FORDYCA_CONTROLLER_BLOCK_TARGET_SELECTOR_HPP_ */
