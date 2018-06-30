/**
 * @file steering_force2D_parser.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_STEERING_FORCE2D_PARSER_HPP_
#define INCLUDE_FORDYCA_PARAMS_STEERING_FORCE2D_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/params/phototaxis_force_parser.hpp"
#include "fordyca/params/steering_force2D_params.hpp"
#include "rcppsw/common/common.hpp"
#include "rcppsw/robotics/steering2D/force_calculator_xml_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);
namespace steering = rcppsw::robotics::steering2D;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class steering_force2D_parser
 * @ingroup params
 *
 * @brief Parses XML parameters related to \ref steering_force2D objects
 * into \ref steering_force2D_params.
 */
class steering_force2D_parser : public steering::force_calculator_xml_parser {
 public:
  steering_force2D_parser(const std::shared_ptr<rcppsw::er::server>& server,
                          uint level)
      : force_calculator_xml_parser(server, level),
        m_phototaxis(server, level + 1) {}

  void parse(const ticpp::Element& node) override;
  void show(std::ostream& stream) const override;
  bool validate(void) const override;

  std::string xml_root(void) const override { return kXMLRoot; }

  std::shared_ptr<steering_force2D_params> parse_results(void) const {
    return m_params;
  }

 private:
  std::shared_ptr<rcppsw::params::base_params> parse_results_impl(void) const override {
    return m_params;
  }

  // clang-format off
  std::shared_ptr<steering_force2D_params> m_params{};
  phototaxis_force_parser                  m_phototaxis;
  // clang-format on
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_STEERING_FORCE2D_PARSER_HPP_ */
