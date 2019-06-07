/**
 * @file proximity_sensor_parser.hpp
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

#ifndef INCLUDE_FORDYCA_CONFIG_PROXIMITY_SENSOR_PARSER_HPP_
#define INCLUDE_FORDYCA_CONFIG_PROXIMITY_SENSOR_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/config/proximity_sensor_config.hpp"
#include "fordyca/nsalias.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class proximity_sensor_parser
 * @ingroup fordyca config
 *
 * @brief Parses XML parameters relating to proximity_sensors into \ref proximity_sensor_config.
 */
class proximity_sensor_parser : public rconfig::xml::xml_config_parser {
 public:
  explicit proximity_sensor_parser(uint level) : xml_config_parser(level) {}

  ~proximity_sensor_parser(void) override = default;

  /**
   * @brief The root tag that all robot proximity sensor parameters should lie
   * under in the XML tree.
   */
  static constexpr char kXMLRoot[] = "proximity_sensor";

  void show(std::ostream& stream) const override;
  bool validate(void) const override;
  void parse(const ticpp::Element& node) override;

  std::string xml_root(void) const override { return kXMLRoot; }

  std::shared_ptr<proximity_sensor_config> config_get(void) const {
    return m_config;
  }

 private:
  std::shared_ptr<rconfig::base_config> config_get_impl(void) const override {
    return m_config;
  }

  std::shared_ptr<proximity_sensor_config> m_config{nullptr};
};

NS_END(config, fordyca);

#endif /* INCLUDE_FORDYCA_CONFIG_PROXIMITY_SENSOR_PARSER_HPP_ */
