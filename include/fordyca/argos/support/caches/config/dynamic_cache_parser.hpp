/**
 * \file dynamic_cache_parser.hpp
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
#include "fordyca/argos/support/caches/config/dynamic_cache_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, caches, config);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class dynamic_cache_parser
 * \ingroup argos support caches config
 *
 * \brief Parses XML parameters for relating to dynamic caches into \ref
 * dynamic_cache_config.
 */
class dynamic_cache_parser final: public rer::client<dynamic_cache_parser>,
                                  public rconfig::xml::xml_config_parser {
 public:
  using config_type = dynamic_cache_config;

  dynamic_cache_parser(void)
      : ER_CLIENT_INIT("fordyca.argos.support.caches.config.dynamic_cache_parser") {}

  /**
   * \brief The root tag that all dynamic cache parameters should lie under in
   * the XML tree.
   */
  static inline const std::string kXMLRoot = "dynamic";

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

