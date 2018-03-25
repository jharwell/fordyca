/**
 * @file task_allocation_parser.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_DEPTH1_TASK_ALLOCATION_PARSER_HPP_
#define INCLUDE_FORDYCA_PARAMS_DEPTH1_TASK_ALLOCATION_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/configuration/argos_configuration.h>

#include "rcppsw/common/common.hpp"
#include "rcppsw/params/xml_param_parser.hpp"
#include "fordyca/params/depth1/task_allocation_params.hpp"
#include "fordyca/params/depth1/executive_parser.hpp"
#include "fordyca/params/depth1/exec_estimates_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class task_allocation_parser
 * @ingroup params depth1
 *
 * @brief Parses XML parameters for relating to the task allocation methods used
 * in FORDYCA into \ref task_allocation_params.
 */
class task_allocation_parser: public rcppsw::params::xml_param_parser {
 public:
  explicit task_allocation_parser(uint level)
      : xml_param_parser(level),
        m_exec_parser(level + 1),
        m_estimate_parser(level + 1) {}

  /**
   * @brief The root tag that all task allocation parameters should lie under
   * in the XML tree.
   */
  static constexpr char kXMLRoot[] = "task_allocation";

  void show(std::ostream& stream) const override;
  bool validate(void) const override;
  void parse(const ticpp::Element& node) override;

  std::string xml_root(void) const override { return kXMLRoot; }
  const struct task_allocation_params* parse_results(void) const override {
    return &m_params;
  }

 private:
  // clang-format off
  struct task_allocation_params m_params{};
  executive_parser              m_exec_parser;
  exec_estimates_parser         m_estimate_parser;
  // clang-format on
};

NS_END(params, fordyca, depth1);

#endif /* INCLUDE_FORDYCA_PARAMS_DEPTH1_TASK_ALLOCATION_PARSER_HPP_ */
