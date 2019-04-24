/**
 * @file entities_oracle_parser.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_ORACLE_ENTITIES_ORACLE_PARSER_HPP_
#define INCLUDE_FORDYCA_PARAMS_ORACLE_ENTITIES_ORACLE_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/params/oracle/entities_oracle_params.hpp"
#include "rcppsw/params/xml_param_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, oracle);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class entities_oracle_parser
 * @ingroup fordyca params oracle
 *
 * @brief Parses XML parameters used for the \ref entities_oracle at the start
 * of simulation.
 */
class entities_oracle_parser : public rparams::xml_param_parser {
 public:
  explicit entities_oracle_parser(uint level) : xml_param_parser(level) {}

  /**
   * @brief The root tag that all entity oracle parameters should lie under in
   * the XML tree.
   */
  static constexpr char kXMLRoot[] = "entities_oracle";

  void parse(const ticpp::Element& node) override;

  std::string xml_root(void) const override { return kXMLRoot; }
  std::shared_ptr<entities_oracle_params> parse_results(void) const {
    return m_params;
  }

 private:
  std::shared_ptr<rparams::base_params> parse_results_impl(
      void) const override {
    return m_params;
  }

  /* clang-format off */
  std::shared_ptr<entities_oracle_params> m_params{nullptr};
  /* clang-format on */
};

NS_END(oracle, params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_ORACLE_ENTITIES_ORACLE_PARSER_HPP_ */
