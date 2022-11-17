/**
 * \file tv_manager_parser.hpp
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

#include "cosm/tv/config/xml/population_dynamics_parser.hpp"

#include "fordyca/argos/support/tv/config/tv_manager_config.hpp"
#include "fordyca/argos/support/tv/config/env_dynamics_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, tv, config);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class tv_manager_parser
 * \ingroup argos support tv config
 *
 * \brief Parses XML parameters for \ref tv_manager into \ref tv_manager_config.
 */
class tv_manager_parser final : public rer::client<tv_manager_parser>,
                                public rconfig::xml::xml_config_parser {
 public:
  using config_type = tv_manager_config;

  tv_manager_parser(void)
      : ER_CLIENT_INIT("fordyca.argos.support.config.tv_manager_parser") {}

  /**
   * \brief The root tag that all temporal variance parameters should lie under
   * in the XML tree.
   */
  static inline const std::string kXMLRoot = "temporal_variance";

  void parse(const ticpp::Element& node) override;
  bool validate(void) const override RCPPSW_ATTR(const, cold);

  std::string xml_root(void) const override { return kXMLRoot; }

 private:
  const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type>                 m_config{nullptr};
  ctv::config::xml::population_dynamics_parser m_popd{};
  env_dynamics_parser                          m_envd{};
  /* clang-format on */
};

NS_END(config, tv, support, argos, fordyca);

