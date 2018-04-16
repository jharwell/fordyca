/**
 * @file pheromone_parser.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_PHEROMONE_PARSER_HPP_
#define INCLUDE_FORDYCA_PARAMS_PHEROMONE_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/common/common.hpp"
#include "fordyca/params/pheromone_params.hpp"
#include "rcppsw/params/xml_param_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class pheromone_parser
 * @ingroup params depth0
 *
 * @brief Parses XML parameters relating to pheromones into
 * \ref pheromone_params.
 */
class pheromone_parser: public rcppsw::params::xml_param_parser {
 public:
  explicit pheromone_parser(uint level) : xml_param_parser(level) {}

  /**
   * @brief The root tag that all pheromone parameters should lie under in the
   * XML tree.
   */
  static constexpr char kXMLRoot[] = "pheromone";

  void show(std::ostream& stream) const override;
  bool validate(void) const override;
  void parse(const ticpp::Element& node) override;

  std::string xml_root(void) const override { return kXMLRoot; }
  const struct pheromone_params* parse_results(void) const override {
    return &m_params;
  }

 private:
  struct pheromone_params m_params{};
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_PHEROMONE_PARSER_HPP_ */
