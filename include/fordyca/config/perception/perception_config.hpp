/**
 * @file perception_config.hpp
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

#ifndef INCLUDE_FORDYCA_CONFIG_PERCEPTION_PERCEPTION_CONFIG_HPP_
#define INCLUDE_FORDYCA_CONFIG_PERCEPTION_PERCEPTION_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/config/grid_config.hpp"
#include "fordyca/config/perception/pheromone_config.hpp"
#include "rcppsw/config/base_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, perception);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @struct perception_grid_config
 * @ingroup fordyca config perception
 */
struct perception_config final : public rconfig::base_config {
  double los_dim{-1};
  struct grid_config occupancy_grid {};
  struct pheromone_config pheromone {};
};

NS_END(perception, config, fordyca);

#endif /* INCLUDE_FORDYCA_CONFIG_PERCEPTION_PERCEPTION_CONFIG_HPP_ */
