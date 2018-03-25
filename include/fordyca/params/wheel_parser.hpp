/**
 * @file wheel_parser.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_WHEEL_PARSER_HPP_
#define INCLUDE_FORDYCA_PARAMS_WHEEL_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/params/wheel_params.hpp"
#include "rcppsw/common/common.hpp"
#include "rcppsw/params/xml_param_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class wheel_parser
 * @ingroup params
 *
 * @brief Parses XML parameters relating to the wheels into \ref wheel_params.
 */
class wheel_parser : public rcppsw::params::xml_param_parser {
 public:
  /**
   * @brief The root tag that all wheel parameters should lie under in the
   * XML tree.
   */
  static constexpr char kXMLRoot[] = "wheels";

  explicit wheel_parser(uint level) : xml_param_parser(level) {}

  void parse(const ticpp::Element& node) override;
  void show(std::ostream& stream) const override;
  bool validate(void) const override;

  std::string xml_root(void) const override { return kXMLRoot; }
  const struct wheel_params* parse_results(void) const override {
    return &m_params;
  }

 private:
  struct wheel_params m_params{};
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_WHEEL_PARSER_HPP_ */
