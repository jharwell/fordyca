/**
 * @file occupancy_grid_parser.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_OCCUPANCY_GRID_PARSER_HPP_
#define INCLUDE_FORDYCA_PARAMS_OCCUPANCY_GRID_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/common/common.hpp"
#include "fordyca/params/occupancy_grid_params.hpp"
#include "rcppsw/params/xml_param_parser.hpp"
#include "fordyca/params/grid_parser.hpp"
#include "fordyca/params/pheromone_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class occupancy_grid_parser
 * @ingroup params depth0
 *
 * @brief Parses XML parameters for \ref occupancy_grid into
 * \ref occupancy_grid_params.
 */
class occupancy_grid_parser: public rcppsw::params::xml_param_parser {
 public:
  explicit occupancy_grid_parser(uint level):
      xml_param_parser(level),
      m_grid_parser(level + 1),
      m_pheromone_parser(level + 1) {}

  /**
   * @brief The root tag that all occupancy grid parameters should lie under in
   * the XML tree.
   */
  static constexpr char kXMLRoot[] = "occupancy_grid";

  void show(std::ostream& stream) const override;
  bool validate(void) const override;
  void parse(const ticpp::Element& node) override;

  std::string xml_root(void) const override { return kXMLRoot; }
  const struct occupancy_grid_params* parse_results(void) const override {
    return &m_params;
  }

 private:
  // clang-format off
  occupancy_grid_params m_params{};
  grid_parser           m_grid_parser;
  pheromone_parser      m_pheromone_parser;
  // clang-format on
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_OCCUPANCY_GRID_PARSER_HPP_ */