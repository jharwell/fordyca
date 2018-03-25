/**
 * @file executive_parser.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_DEPTH1_EXECUTIVE_PARSER_HPP_
#define INCLUDE_FORDYCA_PARAMS_DEPTH1_EXECUTIVE_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/common/common.hpp"
#include "rcppsw/params/xml_param_parser.hpp"
#include "rcppsw/task_allocation/partitionable_task_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, depth1);
namespace task_allocation = rcppsw::task_allocation;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class executive_parser
 * @ingroup params depth1
 *
 * @brief Parses XML parameters for relating to the task executive and the tasks
 * it runs.
 */
class executive_parser: public rcppsw::params::xml_param_parser {
 public:
  explicit executive_parser(uint level) : xml_param_parser(level) {}

  /**
   * @brief The root tag that all task executive parameters should lie under in
   * the XML tree.
   */
  static constexpr char kXMLRoot[] = "task_allocation";

  void show(std::ostream& stream) const override;
  bool validate(void) const override;
  void parse(const ticpp::Element& node) override;

  std::string xml_root(void) const override { return kXMLRoot; }
  const struct task_allocation::partitionable_task_params* parse_results(void) const override {
    return &m_params;
  }

 private:
  struct task_allocation::partitionable_task_params m_params{};
};

NS_END(params, fordyca, depth1);

#endif /* INCLUDE_FORDYCA_PARAMS_DEPTH1_EXECUTIVE_PARSER_HPP_ */
