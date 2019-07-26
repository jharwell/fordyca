/**
 * @file actuation_parser.hpp
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

#ifndef INCLUDE_FORDYCA_CONFIG_ACTUATION_PARSER_HPP_
#define INCLUDE_FORDYCA_CONFIG_ACTUATION_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>

#include "fordyca/config/actuation_config.hpp"
#include "rcppsw/robotics/kin2D/config/xml/differential_drive_parser.hpp"
#include "rcppsw/robotics/steer2D/config/xml/force_calculator_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class actuation_parser
 * @ingroup fordyca config
 *
 * @brief Parses XML parameters for \ref actuation_subsystem into
 * \ref actuation_config.
 */
class actuation_parser final : public rconfig::xml::xml_config_parser {
 public:
  using config_type = actuation_config;

  /**
   * @brief The root tag that all actuation parameters should lie under in the
   * XML tree.
   */
  static constexpr char kXMLRoot[] = "actuation";

  bool validate(void) const override RCSW_PURE;
  void parse(const ticpp::Element& node) override;

  std::string xml_root(void) const override { return kXMLRoot; }

 private:
  const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type>                    m_config{nullptr};
  rrkin2D::config::xml::differential_drive_parser m_differential_drive{};
  rrsteer2D::config::xml::force_calculator_parser m_steering{};
  /* clang-format on */
};

NS_END(config, fordyca);

#endif /* INCLUDE_FORDYCA_CONFIG_ACTUATION_PARSER_HPP_ */
