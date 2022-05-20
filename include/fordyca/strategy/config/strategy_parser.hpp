/**
 * \file strategy_parser.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>

#include "rcppsw/config/xml/xml_config_parser.hpp"

#include "fordyca/strategy/config/strategy_config.hpp"
#include "fordyca/strategy/config/blocks_parser.hpp"
#include "fordyca/strategy/config/caches_parser.hpp"
#include "fordyca/strategy/config/nest_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, strategy, config);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class strategy_parser
 * \ingroup strategy config
 *
 * \brief Parses XML configuration for how robots should DO things into \ref
 * strategy_config.
 */
class strategy_parser final : public rer::client<strategy_parser>,
                              public rconfig::xml::xml_config_parser {
 public:
  using config_type = strategy_config;

  strategy_parser(void)
      : ER_CLIENT_INIT("fordyca.strategy.config.strategy_parser") {}


  /**
   * \brief The root tag that all XML configuration for strategy should lie
   * under in the XML tree.
   */
  static inline const std::string kXMLRoot = "strategy";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  std::string xml_root(void) const override { return kXMLRoot; }

 private:
  const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  blocks_parser                m_blocks{};
  caches_parser                m_caches{};
  nest_parser                  m_nest{};
  /* clang-format on */
};

NS_END(config, strategy, fordyca);
