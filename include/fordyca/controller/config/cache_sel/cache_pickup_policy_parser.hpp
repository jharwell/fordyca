/**
 * \file cache_pickup_policy_parser.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#include "fordyca/controller/config/cache_sel/cache_pickup_policy_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, config, cache_sel);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cache_pickup_policy_parser
 * \ingroup controller config cache_sel
 *
 * \brief Parses XML parameters for \ref cache_pickup_policy_config at the
 * start of simulation.
 */
class cache_pickup_policy_parser : public rer::client<cache_pickup_policy_parser>,
                                   public rconfig::xml::xml_config_parser {
 public:
  using config_type = cache_pickup_policy_config;

  cache_pickup_policy_parser(void)
      : ER_CLIENT_INIT("fordyca.controller.config.cache_sela.cache_pickup_policy_parser") {}

  /**
   * \brief The root tag that all cache sel matrix parameters should lie
   * under in the XML tree.
   */
  static inline const std::string kXMLRoot = "pickup_policy";

  void parse(const ticpp::Element& node) override;

  std::string xml_root(void) const override { return kXMLRoot; }

 private:
  const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  /* clang-format on */
};

NS_END(cache_sel, config, controller, fordyca);

