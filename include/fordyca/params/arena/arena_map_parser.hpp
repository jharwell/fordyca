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

#ifndef INCLUDE_FORDYCA_PARAMS_ARENA_ARENA_MAP_PARSER_HPP_
#define INCLUDE_FORDYCA_PARAMS_ARENA_ARENA_MAP_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "fordyca/params/arena/arena_map_params.hpp"
#include "fordyca/params/arena/blocks_parser.hpp"
#include "fordyca/params/grid_parser.hpp"
#include "fordyca/params/arena/nest_parser.hpp"

#include "rcppsw/common/common.hpp"
#include "rcppsw/params/xml_param_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, arena);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class arena_map_parser
 * @ingroup fordyca params arena
 *
 * @brief Parses XML parameters for \ref arena_map into \ref arena_map_params.
 */
class arena_map_parser : public rcppsw::params::xml_param_parser {
 public:
  explicit arena_map_parser(uint level)
      : xml_param_parser(level),
        m_grid(level + 1),
        m_blocks(level + 1),
        m_nest(level + 1) {}

  /**
   * @brief The root tag that all arena map parameters should lie under in the
   * XML tree.
   */
  static constexpr char kXMLRoot[] = "arena_map";

  void parse(const ticpp::Element& node) override;
  void show(std::ostream& stream) const override;
  bool validate(void) const override;

  std::string xml_root(void) const override { return kXMLRoot; }

  std::shared_ptr<arena_map_params> parse_results(void) const {
    return m_params;
  }

 private:
  std::shared_ptr<rcppsw::params::base_params> parse_results_impl(
      void) const override {
    return m_params;
  }

  /* clang-format off */
  std::shared_ptr<arena_map_params> m_params{nullptr};
  grid_parser                       m_grid;
  blocks_parser                     m_blocks;
  nest_parser                       m_nest;
  /* clang-format on */
};

NS_END(arena, params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_ARENA_ARENA_MAP_PARSER_HPP_ */
