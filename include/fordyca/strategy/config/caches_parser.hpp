/**
 * \file caches_parser.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#include "cosm/spatial/strategy/explore/config/xml/explore_parser.hpp"

#include "fordyca/strategy/config/caches_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, strategy, config);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class caches_parser
 * \ingroup strategy config
 *
 * \brief Parses XML configuration for how robots can do things to/with caches
 * into \ref caches_config.
 */
class caches_parser final : public rer::client<caches_parser>,
                             public rconfig::xml::xml_config_parser {
 public:
  using config_type = caches_config;

  caches_parser(void)
      : ER_CLIENT_INIT("fordyca.strategy.caches.config.caches_parser") {}

  /**
   * \brief The root tag that all XML configuration for caches should lie
   * under in the XML tree.
   */
  static inline const std::string kXMLRoot = "caches";

  void parse(const ticpp::Element& node) override;
  bool validate(void) const override RCPPSW_ATTR(pure, cold);

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type>            m_config{nullptr};
  cssexplore::config::xml::explore_parser m_explore{};
  /* clang-format on */
};

NS_END(config, strategy, fordyca);
