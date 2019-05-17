/**
 * @file steering_force2D_config.hpp
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

#ifndef INCLUDE_FORDYCA_CONFIG_STEERING_FORCE2D_CONFIG_HPP_
#define INCLUDE_FORDYCA_CONFIG_STEERING_FORCE2D_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/config/phototaxis_force_config.hpp"
#include "fordyca/nsalias.hpp"
#include "rcppsw/config/base_config.hpp"
#include "rcppsw/robotics/steer2D/config/force_calculator_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config);
namespace steering = rcppsw::robotics::steer2D;

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @struct steering_force2D_config
 * @ingroup fordyca config
 */
struct steering_force2D_config
    : public steering::config::force_calculator_config {
  struct phototaxis_force_config phototaxis {};
};

NS_END(config, fordyca);

#endif /* INCLUDE_FORDYCA_CONFIG_STEERING_FORCE2D_CONFIG_HPP_ */
