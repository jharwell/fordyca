/**
 * @file nest_parser.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_NEST_PARSER_HPP_
#define INCLUDE_FORDYCA_PARAMS_NEST_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/params/nest_params.hpp"
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
 * @class nest_parser
 * @ingroup params
 *
 * @brief Parses XML parameters for related to \ref nest objects into
 * \ref nest_params.
 */
class nest_parser : public rcppsw::params::xml_param_parser {
 public:
  nest_parser(const std::shared_ptr<rcppsw::er::server>& server, uint level)
      : xml_param_parser(server, level) {}

  /**
   * @brief The root tag that all nest parameters should lie under in the
   * XML tree.
   */
  static constexpr char kXMLRoot[] = "nest";

  void parse(const ticpp::Element& node) override;
  void show(std::ostream& stream) const override;
  bool validate(void) const override;

  std::string xml_root(void) const override { return kXMLRoot; }

  std::shared_ptr<nest_params> parse_results(void) const {
    return m_params;
  }

 private:
  std::shared_ptr<rcppsw::params::base_params> parse_results_impl(void) const override {
    return m_params;
  }

  std::shared_ptr<nest_params> m_params{nullptr};
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_NEST_PARSER_HPP_ */
