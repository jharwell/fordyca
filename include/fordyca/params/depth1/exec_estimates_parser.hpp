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
#include <string>

#include "fordyca/params/depth0/exec_estimates_parser.hpp"
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
 * @brief Parses XML parameters used for estimation of depth1 task execution
 * times at the start of simulation.
 */
class exec_estimates_parser: public depth0::exec_estimates_parser {
 public:
  explicit exec_estimates_parser(uint level)
      : depth0::exec_estimates_parser(level) {}

  /**
   * @brief The root tag that all cache parameters should lie under in the
   * XML tree.
   */
  static constexpr char kXMLRoot[] = "task_exec_estimates";

  void parse(const ticpp::Element& node) override;
  void show(std::ostream& stream) const override;

  std::string xml_root(void) const override { return kXMLRoot; }
  std::shared_ptr<exec_estimates_params> parse_results(void) const {
    return m_params;
  }

 private:
  std::shared_ptr<rcppsw::params::base_params> parse_results_impl(void) const override {
    return m_params;
  }

  // clang-format off
  std::shared_ptr<exec_estimates_params> m_params{nullptr};
  // clang-format on
};

NS_END(params, fordyca, depth1);

#endif /* INCLUDE_FORDYCA_PARAMS_DEPTH1_EXEC_ESTIMATES_PARSER_HPP_ */
