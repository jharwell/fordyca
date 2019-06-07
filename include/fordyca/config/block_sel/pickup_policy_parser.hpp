/**
 * @file pickup_policy_parser.hpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_CONFIG_BLOCK_SEL_PICKUP_POLICY_PARSER_HPP_
#define INCLUDE_FORDYCA_CONFIG_BLOCK_SEL_PICKUP_POLICY_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "fordyca/config/block_sel/pickup_policy_config.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, block_sel);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class pickup_policy_parser
 * @ingroup fordyca config
 *
 * @brief Parses XML parameters for \ref pickup_policy_config at the
 * start of simulation.
 */
class pickup_policy_parser : public rconfig::xml::xml_config_parser {
 public:
  explicit pickup_policy_parser(uint level) : xml_config_parser(level) {}

  /**
   * @brief The root tag that all block sel matrix parameters should lie
   * under in the XML tree.
   */
  static constexpr char kXMLRoot[] = "pickup_policy";

  void parse(const ticpp::Element& node) override;

  std::string xml_root(void) const override { return kXMLRoot; }
  std::shared_ptr<pickup_policy_config> config_get(void) const {
    return m_config;
  }

 private:
  std::shared_ptr<rconfig::base_config> config_get_impl(
      void) const override {
    return m_config;
  }

  /* clang-format off */
  std::shared_ptr<pickup_policy_config> m_config{nullptr};
  /* clang-format on */
};

NS_END(block_sel, config, fordyca);

#endif /* INCLUDE_FORDYCA_CONFIG_BLOCK_SEL_PICKUP_POLICY_PARSER_HPP_ */
