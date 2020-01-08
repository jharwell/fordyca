/**
 * \file visualization_config.hpp
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

#ifndef INCLUDE_FORDYCA_CONFIG_VISUALIZATION_CONFIG_HPP_
#define INCLUDE_FORDYCA_CONFIG_VISUALIZATION_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct visualization_config
 * \ingroup config
 *
 * \brief Configuration for the extended ARGoS visualizations possible within
 * FORDYCA.
 */
struct visualization_config final : public rconfig::base_config {
  bool robot_id{false};
  bool robot_los{false};
  bool robot_task{false};
  bool block_id{false};
};

NS_END(config, fordyca);

#endif /* INCLUDE_FORDYCA_CONFIG_VISUALIZATION_CONFIG_HPP_ */
