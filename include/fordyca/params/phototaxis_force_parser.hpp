/**
 * @file phototaxis_force_parser.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_PHOTOTAXIS_FORCE_PARSER_HPP_
#define INCLUDE_FORDYCA_PARAMS_PHOTOTAXIS_FORCE_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/params/phototaxis_force_params.hpp"
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
 * @class phototaxis_force_parser
 * @ingroup params
 *
 * @brief Parses XML parameters for related to \ref phototaxis_force objects
 * into \ref phototaxis_force_params.
 */
class phototaxis_force_parser : public rcppsw::params::xml_param_parser {
 public:
  phototaxis_force_parser(const std::shared_ptr<rcppsw::er::server>& server,
                          uint level)
      : xml_param_parser(server, level) {}

  /**
   * @brief The root tag that all phototaxis_force parameters should lie under
   * in the XML tree.
   */
  static constexpr char kXMLRoot[] = "phototaxis_force";

  void parse(const ticpp::Element& node) override;
  void show(std::ostream& stream) const override;
  bool validate(void) const override;

  std::string xml_root(void) const override { return kXMLRoot; }
  std::shared_ptr<phototaxis_force_params> parse_results(void) const {
    return m_params;
  }

 private:
  std::shared_ptr<rcppsw::params::base_params> parse_results_impl(void) const override {
    return m_params;
  }


  // clang-format on
  bool                                     m_parsed{false};
  std::shared_ptr<phototaxis_force_params> m_params{nullptr};
  // clang-format off
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_PHOTOTAXIS_FORCE_PARSER_HPP_ */
