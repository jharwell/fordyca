/**
 * @file grid_params.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_GRID_PARAMS_HPP_
#define INCLUDE_FORDYCA_PARAMS_GRID_PARAMS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/math/vector2.h>
#include "fordyca/params/base_params.hpp"
#include "fordyca/params/block_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
struct grid_params : public base_params {
  grid_params(double resolution_ = 0.0,
              argos::CVector2 upper_ = argos::CVector2(),
              argos::CVector2 lower_ = argos::CVector2(),
              struct block_params block_ = block_params()) :
      resolution(resolution_), upper(upper_), lower(lower_), block(block_) {}
  double resolution;
  argos::CVector2 upper;
  argos::CVector2 lower;
  struct block_params block;
};

struct perceived_grid_params : public base_params {
  perceived_grid_params(void) : grid(), pheromone_rho(0.0) {}
  struct grid_params grid;
  double pheromone_rho;
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_GRID_PARAMS_HPP_ */
