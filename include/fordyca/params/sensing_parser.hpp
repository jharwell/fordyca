/**
 * @file sensing_parser.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_SENSING_PARSER_HPP_
#define INCLUDE_FORDYCA_PARAMS_SENSING_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/nsalias.hpp"
#include "fordyca/params/proximity_sensor_parser.hpp"
#include "fordyca/params/sensing_params.hpp"
#include "rcppsw/params/xml_param_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class sensing_parser
 * @ingroup fordyca params
 *
 * @brief Parses XML parameters relating to sensings into \ref sensing_params.
 */
class sensing_parser final : public rparams::xml_param_parser {
 public:
  explicit sensing_parser(uint level)
      : xml_param_parser(level), m_proximity_parser(level + 1) {}

  ~sensing_parser(void) override = default;

  /**
   * @brief The root tag that all robot sensing parameters should lie under in
   * the XML tree.
   */
  static constexpr char kXMLRoot[] = "sensing";

  void show(std::ostream& stream) const override;
  bool validate(void) const override;
  void parse(const ticpp::Element& node) override;

  std::string xml_root(void) const override { return kXMLRoot; }

  std::shared_ptr<sensing_params> parse_results(void) const { return m_params; }

 private:
  std::shared_ptr<rparams::base_params> parse_results_impl(void) const override {
    return m_params;
  }

  /* clang-format off */
  std::shared_ptr<sensing_params> m_params{nullptr};
  proximity_sensor_parser         m_proximity_parser;
  /* clang-format on */
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_SENSING_PARSER_HPP_ */
