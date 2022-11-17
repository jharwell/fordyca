/**
 * \file strategy_parser.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
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
