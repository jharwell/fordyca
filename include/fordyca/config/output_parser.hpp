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

#ifndef INCLUDE_FORDYCA_CONFIG_OUTPUT_PARSER_HPP_
#define INCLUDE_FORDYCA_CONFIG_OUTPUT_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>

#include "rcppsw/config/xml/xml_config_parser.hpp"

#include "fordyca/config/metrics_parser.hpp"
#include "fordyca/config/output_config.hpp"
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class output_parser
 * @ingroup fordyca config
 *
 * @brief Parses XML parameters relating to simulation output into
 * \ref output_config. This parser is used by both loop functions and robots,
 * and so its logic is slighly more complex in order to handle the needs of
 * both.
 */
class output_parser final : public rconfig::xml::xml_config_parser {
 public:
  using config_type = output_config;

  /**
   * @brief The root tag that all output loop functions parameters should lie
   * under in the XML tree.
   */
  static constexpr char kXMLRoot[] = "output";

  bool validate(void) const override RCSW_ATTR(pure, cold);
  void parse(const ticpp::Element& node) override RCSW_COLD;

  RCSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<output_config> m_config{nullptr};
  metrics_parser                 m_metrics_parser{};
  /* clang-format on */
};

NS_END(config, fordyca);

#endif /* INCLUDE_FORDYCA_CONFIG_OUTPUT_PARSER_HPP_ */
