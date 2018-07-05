/**
 * @file arena_map_parser.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_PARAMS_ARENA_MAP_PARSER_HPP_
#define INCLUDE_FORDYCA_PARAMS_ARENA_MAP_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/configuration/argos_configuration.h>

#include "fordyca/params/arena_map_params.hpp"
#include "fordyca/params/block_distribution_parser.hpp"
#include "fordyca/params/block_parser.hpp"
#include "fordyca/params/depth1/static_cache_parser.hpp"
#include "fordyca/params/grid_parser.hpp"
#include "fordyca/params/nest_parser.hpp"

#include "rcppsw/common/common.hpp"
#include "rcppsw/params/xml_param_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class arena_map_parser
 * @ingroup params
 *
 * @brief Parses XML parameters for \ref arena_map into \ref arena_map_params.
 */
class arena_map_parser : public rcppsw::params::xml_param_parser {
 public:
  arena_map_parser(const std::shared_ptr<rcppsw::er::server>& server, uint level)
      : xml_param_parser(server, level),
        m_grid_parser(server, level + 1),
        m_block_parser(server, level + 1),
        m_block_dist_parser(server, level + 1),
        m_cache_parser(server, level + 1),
        m_nest_parser(server, level + 1) {}

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
  std::shared_ptr<rcppsw::params::base_params> parse_results_impl(void) const override {
    return m_params;
  }

  // clang-format off
  std::shared_ptr<arena_map_params> m_params{nullptr};
  grid_parser                       m_grid_parser;
  block_parser                      m_block_parser;
  block_distribution_parser         m_block_dist_parser;
  depth1::static_cache_parser       m_cache_parser;
  nest_parser                       m_nest_parser;
  // clang-format on
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_ARENA_MAP_PARSER_HPP_ */
