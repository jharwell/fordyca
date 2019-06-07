/**
 * @file actuation_parser.hpp
 *
 * @copyright 2019 Nathan White, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_PARAMS_COMMUNICATION_PARSER_HPP_
#define INCLUDE_FORDYCA_PARAMS_COMMUNICATION_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/config/communication_config.hpp"
#include "fordyca/nsalias.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"
/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class communication_parser
 * @ingroup config
 */
class communication_parser : public rconfig::xml::xml_config_parser {
 public:
  explicit communication_parser(uint level) : xml_config_parser(level) {}

  /**
   * @brief The root tag that all actuation parameters should lie under in the
   * XML tree.
   */
  static constexpr char kXMLRoot[] = "communication";

  bool validate(void) const override;
  void parse(const ticpp::Element &node) override;

  std::string xml_root(void) const override { return kXMLRoot; }
  std::shared_ptr<communication_config> config_get(void) const {
    return m_params;
  }

private:
  std::shared_ptr<rcppsw::config::base_config> config_get_impl(void) const override {
    return m_params;
  }

  // clang-format off
  std::shared_ptr<communication_config>           m_params{nullptr};
  // clang-format on
};

NS_END(config, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_COMMUNICATION_PARSER_HPP_ */
