/**
 * \file static_cache_parser.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "rcppsw/config/xml/xml_config_parser.hpp"

#include "fordyca/fordyca.hpp"
#include "fordyca/argos/support/caches/config/static_cache_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, caches, config);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class static_cache_parser
 * \ingroup argos support caches config
 *
 * \brief Parses XML parameters for relating to cache into \ref caches_config.
 */
class static_cache_parser final: public rer::client<static_cache_parser>,
                                 public rconfig::xml::xml_config_parser {
 public:
  using config_type = static_cache_config;

  static_cache_parser(void)
      : ER_CLIENT_INIT("fordyca.argos.support.caches.config.static_cache_parser") {}

  /**
   * \brief The root tag that all static cache parameters should lie under in
   * the XML tree.
   */
  static inline const std::string kXMLRoot = "static";

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

NS_END(config, caches, support, argos, fordyca);

