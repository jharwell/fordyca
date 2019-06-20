/**
 * @file block_dist_parser.hpp
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

#ifndef INCLUDE_FORDYCA_CONFIG_ARENA_BLOCK_DIST_PARSER_HPP_
#define INCLUDE_FORDYCA_CONFIG_ARENA_BLOCK_DIST_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "fordyca/config/arena/block_dist_config.hpp"
#include "fordyca/config/arena/powerlaw_dist_parser.hpp"
#include "fordyca/config/arena/block_manifest_parser.hpp"
#include "fordyca/config/arena/block_redist_governor_parser.hpp"

#include "fordyca/nsalias.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, arena);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class block_dist_parser
 * @ingroup fordyca config arena
 *
 * @brief Parses XML parameters related to block distribution
 * into \ref block_dist_config.
 */
class block_dist_parser : public rconfig::xml::xml_config_parser {
 public:
  using config_type = block_dist_config;

  /**
   * @brief The root tag that all block distribution parameters should lie under
   * in the XML tree.
   */
  static constexpr char kXMLRoot[] = "distribution";

  void parse(const ticpp::Element& node) override;
  bool validate(void) const override;

  std::string xml_root(void) const override { return kXMLRoot; }

 private:
    const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  block_manifest_parser        m_manifest{};
  powerlaw_dist_parser         m_powerlaw{};
  block_redist_governor_parser m_redist_governor{};
  /* clang-format on */
};

NS_END(arena, config, fordyca);

#endif /* INCLUDE_FORDYCA_CONFIG_ARENA_BLOCK_DIST_PARSER_HPP_ */
