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

#ifndef INCLUDE_FORDYCA_STRATEGY_CONFIG_STRATEGY_PARSER_HPP_
#define INCLUDE_FORDYCA_STRATEGY_CONFIG_STRATEGY_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>

#include "rcppsw/config/xml/xml_config_parser.hpp"

#include "cosm/spatial/strategy/config/xml/nest_acq_parser.hpp"

#include "fordyca/strategy/config/strategy_config.hpp"
#include "fordyca/strategy/config/explore_parser.hpp"

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

  strategy_parser(void) : ER_CLIENT_INIT("fordyca.strategy.config.strategy_parser") {}


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
  std::unique_ptr<config_type>             m_config{nullptr};
  csstrategy::config::xml::nest_acq_parser m_nest_acq{};
  explore_parser                           m_explore{};
  /* clang-format on */
};

NS_END(config, strategy, fordyca);

#endif /* INCLUDE_FORDYCA_STRATEGY_CONFIG_STRATEGY_PARSER_HPP_ */
