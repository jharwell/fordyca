/**
 * \file env_dynamics_parser.hpp
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

#include "cosm/tv/config/xml/temporal_penalty_parser.hpp"

#include "fordyca/argos/support/tv/config/env_dynamics_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, tv, config);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class env_dynamics_parser
 * \ingroup argos support tv config
 *
 * \brief Parses XML parameters for \ref env_dynamics into \ref
 * env_dynamics_config.
 */
class env_dynamics_parser final : public rer::client<env_dynamics_parser>,
                                  public rconfig::xml::xml_config_parser {
 public:
  using config_type = env_dynamics_config;

  env_dynamics_parser(void);

  /**
   * \brief The root tag that all temporal variance parameters should lie under
   * in the XML tree.
   */
  static inline const std::string kXMLRoot = "env_dynamics";

  void parse(const ticpp::Element& node) override;
  bool validate(void) const override RCPPSW_ATTR(const, cold);

  std::string xml_root(void) const override { return kXMLRoot; }

 private:
  const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type>              m_config{nullptr};
  ctv::config::xml::temporal_penalty_parser m_motion{};
  ctv::config::xml::temporal_penalty_parser m_block_manip{};
  ctv::config::xml::temporal_penalty_parser m_block_carry{};
  ctv::config::xml::temporal_penalty_parser m_cache_usage{};
  /* clang-format on */
};

NS_END(config, tv, support, argos, fordyca);

