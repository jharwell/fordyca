/**
 * @file perceived_grid_parser.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_PERCEIVED_GRID_PARSER_HPP_
#define INCLUDE_FORDYCA_PARAMS_PERCEIVED_GRID_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"
#include "fordyca/params/grid_params.hpp"
#include "fordyca/params/base_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class perceived_grid_parser: public base_parser {
 public:
  perceived_grid_parser(void): m_params() {}

  void parse(argos::TConfigurationNode& node);
  const struct perceived_grid_params* get_results(void) {
    return m_params.get();
  }
  void show(std::ostream& stream);

 private:
  std::unique_ptr<struct perceived_grid_params> m_params;
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_PERCEIVED_GRID_PARSER_HPP_ */
