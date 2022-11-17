/**
 * \file blocks_parser.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#include "cosm/spatial/strategy/blocks/config/xml/drop_parser.hpp"
#include "cosm/spatial/strategy/explore/config/xml/explore_parser.hpp"

#include "fordyca/strategy/config/blocks_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, strategy, config);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class blocks_parser
 * \ingroup strategy config
 *
 * \brief Parses XML configuration for how robots can do things to/with blocks
 * into \ref blocks_config.
 */
class blocks_parser final : public rer::client<blocks_parser>,
                             public rconfig::xml::xml_config_parser {
 public:
  using config_type = blocks_config;

  blocks_parser(void)
      : ER_CLIENT_INIT("fordyca.strategy.blocks.config.blocks_parser") {}

  /**
   * \brief The root tag that all XML configuration for blocks should lie
   * under in the XML tree.
   */
  static inline const std::string kXMLRoot = "blocks";

  void parse(const ticpp::Element& node) override;
  bool validate(void) const override RCPPSW_ATTR(const, cold);

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type>            m_config{nullptr};
  cssblocks::config::xml::drop_parser     m_drop{};
  cssexplore::config::xml::explore_parser m_explore{};
  /* clang-format on */
};

NS_END(config, strategy, fordyca);
