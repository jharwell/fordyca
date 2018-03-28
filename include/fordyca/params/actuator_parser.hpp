/**
 * @file actuator_parser.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_ACTUATOR_PARSER_HPP_
#define INCLUDE_FORDYCA_PARAMS_ACTUATOR_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/params/actuator_params.hpp"
#include "fordyca/params/wheel_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class actuator_parser
 * @ingroup params
 *
 * @brief Parses XML parameters for \ref actuator_manager into
 * \ref actuator_params.
 */
class actuator_parser : public rcppsw::params::xml_param_parser {
 public:
  explicit actuator_parser(uint level)
      : xml_param_parser(level), m_wheels(level + 1) {}

  /**
   * @brief The root tag that all actuator parameters should lie under in the
   * XML tree.
   */
  static constexpr char kXMLRoot[] = "actuators";

  void show(std::ostream& stream) const override;
  bool validate(void) const override;
  void parse(const ticpp::Element& node) override;

  std::string xml_root(void) const override { return kXMLRoot; }
  const struct actuator_params* parse_results(void) const override {
    return &m_params;
  }

 private:
  struct actuator_params m_params {};
  wheel_parser m_wheels;
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_ACTUATOR_PARSER_HPP_ */
