/**
 * @file sensing_config.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_CONFIG_SENSING_CONFIG_HPP_
#define INCLUDE_FORDYCA_CONFIG_SENSING_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/config/proximity_sensor_config.hpp"
#include "fordyca/nsalias.hpp"
#include "rcppsw/config/base_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @struct sensing_config
 * @ingroup fordyca config
 */
struct sensing_config : public rconfig::base_config {
  /**
   * @brief The dimension of a robot's line of sight, specified in meters.
   */
  double los_dim{0.0};
  struct proximity_sensor_config proximity {};
};

NS_END(config, fordyca);

#endif /* INCLUDE_FORDYCA_CONFIG_SENSING_CONFIG_HPP_ */
