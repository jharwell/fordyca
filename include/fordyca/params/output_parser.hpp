/**
 * @file output_parser.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_OUTPUT_PARSER_HPP_
#define INCLUDE_FORDYCA_PARAMS_OUTPUT_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/nsalias.hpp"
#include "fordyca/params/metrics_parser.hpp"
#include "fordyca/params/output_params.hpp"
#include "rcppsw/params/xml_param_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class output_parser
 * @ingroup fordyca params
 *
 * @brief Parses XML parameters relating to simulation output into
 * \ref output_params. This parser is used by both loop functions and robots,
 * and so its logic is slighly more complex in order to handle the needs of
 * both.
 */
class output_parser : public rparams::xml_param_parser {
 public:
  explicit output_parser(uint level)
      : xml_param_parser(level), m_metrics_parser(level + 1) {}

  /**
   * @brief The root tag that all output loop functions parameters should lie
   * under in the XML tree.
   */
  static constexpr char kXMLRoot[] = "output";

  void show(std::ostream& stream) const override;
  bool validate(void) const override;
  void parse(const ticpp::Element& node) override;

  std::string xml_root(void) const override { return kXMLRoot; }

  std::shared_ptr<output_params> parse_results(void) const { return m_params; }

 private:
  std::shared_ptr<rparams::base_params> parse_results_impl(void) const override {
    return m_params;
  }

  /* clang-format off */
  std::shared_ptr<output_params> m_params{nullptr};
  metrics_parser m_metrics_parser;
  /* clang-format on */
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_OUTPUT_PARSER_HPP_ */
