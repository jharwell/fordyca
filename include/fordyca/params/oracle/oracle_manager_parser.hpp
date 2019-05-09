/**
 * @file oracle_manager_parser.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_ORACLE_ORACLE_MANAGER_PARSER_HPP_
#define INCLUDE_FORDYCA_PARAMS_ORACLE_ORACLE_MANAGER_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/params/oracle/oracle_manager_params.hpp"
#include "rcppsw/params/xml_param_parser.hpp"
#include "fordyca/params/oracle/entities_oracle_parser.hpp"
#include "fordyca/params/oracle/tasking_oracle_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, oracle);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class oracle_manager_parser
 * @ingroup fordyca params oracle
 *
 * @brief Parses XML parameters used for oracles at the start of simulation.
 */
class oracle_manager_parser : public rparams::xml_param_parser {
 public:
  explicit oracle_manager_parser(uint level)
      : xml_param_parser(level),
        m_entities(level + 1),
        m_tasking(level + 1) {}

  /**
   * @brief The root tag that all cache parameters should lie under in the
   * XML tree.
   */
  static constexpr char kXMLRoot[] = "oracle_manager";

  void parse(const ticpp::Element& node) override;

  std::string xml_root(void) const override { return kXMLRoot; }
  std::shared_ptr<oracle_manager_params> parse_results(void) const {
    return m_params;
  }

 private:
  std::shared_ptr<rparams::base_params> parse_results_impl(
      void) const override {
    return m_params;
  }

  /* clang-format off */
  std::shared_ptr<oracle_manager_params> m_params{nullptr};
  entities_oracle_parser                 m_entities;
  tasking_oracle_parser                  m_tasking;
  /* clang-format on */
};

NS_END(oracle, params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_ORACLE_ORACLE_MANAGER_PARSER_HPP_ */