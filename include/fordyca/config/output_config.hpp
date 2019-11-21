/**
 * \file output_config.hpp
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

#ifndef INCLUDE_FORDYCA_CONFIG_OUTPUT_CONFIG_HPP_
#define INCLUDE_FORDYCA_CONFIG_OUTPUT_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/config/base_config.hpp"

#include "fordyca/config/metrics_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct output_config
 * \ingroup fordyca config
 *
 * \brief Configuration for robot/loop function logging/metrics.
 */
struct output_config final : public rconfig::base_config {
  std::string output_root{};
  std::string output_dir{};
  struct metrics_config metrics {};
};

NS_END(config, fordyca);

#endif /* INCLUDE_FORDYCA_CONFIG_OUTPUT_CONFIG_HPP_ */
