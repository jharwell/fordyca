/**
 * @file exec_estimates_parser.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_DEPTH1_EXEC_ESTIMATES_PARSER_HPP_
#define INCLUDE_FORDYCA_PARAMS_DEPTH1_EXEC_ESTIMATES_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/configuration/argos_configuration.h>

#include "rcppsw/common/common.hpp"
#include "rcppsw/common/xml_param_parser.hpp"
#include "fordyca/params/depth1/exec_estimates_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class exec_estimates_parser
 * @ingroup params depth1
 *
 * @brief Parses XML parameters used for estimation of task execution at the
 * start of simulation.
 */
class exec_estimates_parser: public rcppsw::common::xml_param_parser {
 public:
  exec_estimates_parser(void) : m_params() {}
  void parse(argos::TConfigurationNode& node) override;
  const struct exec_estimates_params* get_results(void) override {
    return m_params.get();
  }
  void show(std::ostream& stream) override;

 private:
  std::unique_ptr<struct exec_estimates_params> m_params;
};

NS_END(params, fordyca, depth1);

#endif /* INCLUDE_FORDYCA_PARAMS_DEPTH1_EXEC_ESTIMATES_PARSER_HPP_ */
