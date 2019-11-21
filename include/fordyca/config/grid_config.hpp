/**
 * \file grid_config.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_CONFIG_GRID_CONFIG_HPP_
#define INCLUDE_FORDYCA_CONFIG_GRID_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/types/discretize_ratio.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct grid_config
 * \ingroup fordyca config perception
 *
 * \brief Configuration for the 2D grid used to represent the arena by both loop
 * functions and robots.
 */
struct grid_config final : public rconfig::base_config {
  rtypes::discretize_ratio resolution{0.0};
  rmath::vector2d upper{};
  rmath::vector2d lower{};
};

NS_END(config, fordyca);

#endif /* INCLUDE_FORDYCA_CONFIG_GRID_CONFIG_HPP_ */
