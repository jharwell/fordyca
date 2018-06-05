/**
 * @file block_distribution_params.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_BLOCK_DISTRIBUTION_PARAMS_HPP_
#define INCLUDE_FORDYCA_PARAMS_BLOCK_DISTRIBUTION_PARAMS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "fordyca/params/block_params.hpp"
#include "fordyca/representation/extent_model.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @struct block_distribution_params
 * @ingroup params
 */
struct block_distribution_params : public rcppsw::params::base_params {
  uint n_blocks{0};
  std::string dist_type{""};
  representation::extent_model arena_model{};
  representation::extent_model nest_model{};
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_BLOCK_DISTRIBUTION_PARAMS_HPP_ */
