/**
 * \file dynamic_cache_parser.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_CONFIG_CACHES_DYNAMIC_CACHE_PARSER_HPP_
#define INCLUDE_FORDYCA_CONFIG_CACHES_DYNAMIC_CACHE_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "fordyca/fordyca.hpp"
#include "fordyca/config/caches/dynamic_cache_config.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, caches);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class dynamic_cache_parser
 * \ingroup config caches
 *
 * \brief Parses XML parameters for relating to dynamic caches into \ref
 * dynamic_cache_config.
 */
class dynamic_cache_parser final: public rconfig::xml::xml_config_parser {
 public:
  using config_type = dynamic_cache_config;

  /**
   * \brief The root tag that all dynamic cache parameters should lie under in
   * the XML tree.
   */
  inline static const std::string kXMLRoot = "dynamic";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_ATTR(const, cold);

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  /* clang-format on */
};

NS_END(caches, config, fordyca);

#endif /* INCLUDE_FORDYCA_CONFIG_CACHES_DYNAMIC_CACHE_PARSER_HPP_ */
