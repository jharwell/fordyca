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

#ifndef INCLUDE_FORDYCA_CONFIG_STEERING_FORCE2D_PARSER_HPP_
#define INCLUDE_FORDYCA_CONFIG_STEERING_FORCE2D_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>

#include "fordyca/config/phototaxis_force_parser.hpp"
#include "fordyca/config/steering_force2D_config.hpp"
#include "fordyca/nsalias.hpp"
#include "rcppsw/robotics/steer2D/config/xml/force_calculator_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config);
namespace steer2D = rcppsw::robotics::steer2D;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class steering_force2D_parser
 * @ingroup fordyca config
 *
 * @brief Parses XML parameters related to \ref steering_force2D objects
 * into \ref steering_force2D_config.
 */
class steering_force2D_parser
    : public steer2D::config::xml::force_calculator_parser {
 public:
  explicit steering_force2D_parser(uint level)
      : force_calculator_parser(level), m_phototaxis(level + 1) {}

  void parse(const ticpp::Element& node) override;
  void show(std::ostream& stream) const override;
  bool validate(void) const override;

  std::string xml_root(void) const override { return kXMLRoot; }

  std::shared_ptr<steering_force2D_config> config_get(void) const {
    return m_config;
  }

 private:
  std::shared_ptr<rconfig::base_config> config_get_impl(void) const override {
    return m_config;
  }

  /* clang-format off */
  std::shared_ptr<steering_force2D_config> m_config{};
  phototaxis_force_parser                  m_phototaxis;
  /* clang-format on */
};

NS_END(config, fordyca);

#endif /* INCLUDE_FORDYCA_CONFIG_STEERING_FORCE2D_PARSER_HPP_ */
