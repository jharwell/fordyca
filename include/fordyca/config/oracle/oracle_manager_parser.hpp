/**
 * @file oracle_manager_parser.hpp
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

#ifndef INCLUDE_FORDYCA_CONFIG_ORACLE_ORACLE_MANAGER_PARSER_HPP_
#define INCLUDE_FORDYCA_CONFIG_ORACLE_ORACLE_MANAGER_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/config/oracle/oracle_manager_config.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"
#include "fordyca/config/oracle/entities_oracle_parser.hpp"
#include "fordyca/config/oracle/tasking_oracle_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, oracle);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class oracle_manager_parser
 * @ingroup fordyca config oracle
 *
 * @brief Parses XML parameters used for oracles at the start of simulation.
 */
class oracle_manager_parser final : public rconfig::xml::xml_config_parser {
 public:
  explicit oracle_manager_parser(uint level)
      : xml_config_parser(level),
        m_entities(level + 1),
        m_tasking(level + 1) {}

  /**
   * @brief The root tag that all cache parameters should lie under in the
   * XML tree.
   */
  static constexpr char kXMLRoot[] = "oracle_manager";

  void parse(const ticpp::Element& node) override;

  std::string xml_root(void) const override { return kXMLRoot; }
  std::shared_ptr<oracle_manager_config> config_get(void) const {
    return m_config;
  }

 private:
  std::shared_ptr<rconfig::base_config> config_get_impl(
      void) const override {
    return m_config;
  }

  /* clang-format off */
  std::shared_ptr<oracle_manager_config> m_config{nullptr};
  entities_oracle_parser                 m_entities;
  tasking_oracle_parser                  m_tasking;
  /* clang-format on */
};

NS_END(oracle, config, fordyca);

#endif /* INCLUDE_FORDYCA_CONFIG_ORACLE_ORACLE_MANAGER_PARSER_HPP_ */
