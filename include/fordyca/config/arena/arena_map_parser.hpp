/**
 * @file arena_map_parser.hpp
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

#ifndef INCLUDE_FORDYCA_CONFIG_ARENA_ARENA_MAP_PARSER_HPP_
#define INCLUDE_FORDYCA_CONFIG_ARENA_ARENA_MAP_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "fordyca/config/arena/arena_map_config.hpp"
#include "fordyca/config/arena/blocks_parser.hpp"
#include "fordyca/config/grid_parser.hpp"
#include "fordyca/config/arena/nest_parser.hpp"

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
 * @class arena_map_parser
 * @ingroup fordyca config arena
 *
 * @brief Parses XML parameters for \ref arena_map into \ref arena_map_config.
 */
class arena_map_parser final : public rconfig::xml::xml_config_parser {
 public:
  using config_type = arena_map_config;

  /**
   * @brief The root tag that all arena map parameters should lie under in the
   * XML tree.
   */
  static constexpr char kXMLRoot[] = "arena_map";

  void parse(const ticpp::Element& node) override;
  bool validate(void) const override;

  std::string xml_root(void) const override { return kXMLRoot; }

 private:
  const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::shared_ptr<config_type> m_config{nullptr};
  grid_parser                  m_grid{};
  blocks_parser                m_blocks{};
  nest_parser                  m_nest{};
  /* clang-format on */
};

NS_END(arena, config, fordyca);

#endif /* INCLUDE_FORDYCA_CONFIG_ARENA_ARENA_MAP_PARSER_HPP_ */
