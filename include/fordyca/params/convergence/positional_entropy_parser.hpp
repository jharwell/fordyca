/**
 * @file positional_entropy_parser.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_CONVERGENCE_POSITIONAL_ENTROPY_PARSER_HPP_
#define INCLUDE_FORDYCA_PARAMS_CONVERGENCE_POSITIONAL_ENTROPY_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/params/convergence/positional_entropy_params.hpp"
#include "rcppsw/common/common.hpp"
#include "rcppsw/params/xml_param_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, convergence);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class positional_entropy_parser
 * @ingroup params
 *
 * @brief Parses XML parameters related the calculation of swarm positional
 * entropy into \ref positional_entropy_params.
 */
class positional_entropy_parser : public rcppsw::params::xml_param_parser {
 public:
  explicit positional_entropy_parser(uint level) :
      xml_param_parser(level),
      m_params(std::make_shared<std::remove_reference<decltype(*m_params)>::type>()) {}

  /**
   * @brief The root tag that all loop functions relating to positional_entropy
   * parameters should lie under in the XML tree.
   */
  static constexpr char kXMLRoot[] = "positional_entropy";

  void parse(const ticpp::Element& node) override;

  std::string xml_root(void) const override { return kXMLRoot; }

  std::shared_ptr<positional_entropy_params> parse_results(void) const { return m_params; }

 private:
  std::shared_ptr<rcppsw::params::base_params> parse_results_impl(
      void) const override {
    return m_params;
  }

  // clang-format off
  std::shared_ptr<positional_entropy_params> m_params;
  // clang-format on
};

NS_END(convergence, params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_CONVERGENCE_POSITIONAL_ENTROPY_PARSER_HPP_ */
