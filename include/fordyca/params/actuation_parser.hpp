/**
 * @file actuation_parser.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_ACTUATION_PARSER_HPP_
#define INCLUDE_FORDYCA_PARAMS_ACTUATION_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/params/actuation_params.hpp"
#include "fordyca/params/wheel_parser.hpp"
#include "rcppsw/control/kinematics2D_xml_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class actuation_parser
 * @ingroup params
 *
 * @brief Parses XML parameters for \ref actuation_subsystem into
 * \ref actuation_params.
 */
class actuation_parser : public rcppsw::params::xml_param_parser {
 public:
  explicit actuation_parser(uint level)
      : xml_param_parser(level), m_wheels(level + 1), m_kinematics(level + 1) {}

  /**
   * @brief The root tag that all actuation parameters should lie under in the
   * XML tree.
   */
  static constexpr char kXMLRoot[] = "actuations";

  void show(std::ostream& stream) const override;
  bool validate(void) const override;
  void parse(const ticpp::Element& node) override;

  std::string xml_root(void) const override { return kXMLRoot; }
  const struct actuation_params* parse_results(void) const override {
    return &m_params;
  }

 private:
  // clang-format off
  struct actuation_params                  m_params{};
  wheel_parser                             m_wheels;
  rcppsw::control::kinematics2D_xml_parser m_kinematics;
  // clang-format on
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_ACTUATION_PARSER_HPP_ */
