/**
 * @file cache_sel_matrix_parser.hpp
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

#ifndef INCLUDE_FORDYCA_CONFIG_CACHE_SEL_CACHE_SEL_MATRIX_PARSER_HPP_
#define INCLUDE_FORDYCA_CONFIG_CACHE_SEL_CACHE_SEL_MATRIX_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "fordyca/config/cache_sel/cache_sel_matrix_config.hpp"
#include "fordyca/config/cache_sel/pickup_policy_parser.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, cache_sel);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cache_sel_matrix_parser
 * @ingroup fordyca config cache_sel
 *
 * @brief Parses XML parameters for the \ref cache_sel_matrix at the start
 * of simulation.
 */
class cache_sel_matrix_parser final : public rconfig::xml::xml_config_parser {
 public:
  explicit cache_sel_matrix_parser(uint level)
      : xml_config_parser(level),
        m_pickup_policy(level + 1) {}

  /**
   * @brief The root tag that all cache sel matrix parameters should lie
   * under in the XML tree.
   */
  static constexpr char kXMLRoot[] = "cache_sel_matrix";

  void parse(const ticpp::Element& node) override;

  std::string xml_root(void) const override { return kXMLRoot; }
  std::shared_ptr<cache_sel_matrix_config> config_get(void) const {
    return m_config;
  }
  bool validate(void) const override;

 private:
  std::shared_ptr<rconfig::base_config> config_get_impl(
      void) const override {
    return m_config;
  }

  /* clang-format off */
  std::shared_ptr<cache_sel_matrix_config> m_config{nullptr};
  pickup_policy_parser                     m_pickup_policy;
  /* clang-format on */
};

NS_END(cache_sel, config, fordyca);

#endif /* INCLUDE_FORDYCA_CONFIG_CACHE_SEL_CACHE_SEL_MATRIX_PARSER_HPP_ */