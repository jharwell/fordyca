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

#include "rcppsw/common/common.hpp"
#include "fordyca/params/arena_map_params.hpp"
#include "rcppsw/common/xml_param_parser.hpp"
#include "fordyca/params/block_parser.hpp"
#include "fordyca/params/grid_parser.hpp"
#include "fordyca/params/depth1/cache_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class arena_map_parser: public rcppsw::common::xml_param_parser {
 public:
  arena_map_parser(void): m_params(), m_grid_parser(), m_block_parser(),
                          m_cache_parser() {}

  void parse(argos::TConfigurationNode& node) override;
  const struct arena_map_params* get_results(void) override { return m_params.get(); }
  void show(std::ostream& stream) override;

 private:
  std::unique_ptr<struct arena_map_params> m_params;
  grid_parser m_grid_parser;
  block_parser m_block_parser;
  depth1::cache_parser m_cache_parser;
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_ARENA_MAP_PARSER_HPP_ */
