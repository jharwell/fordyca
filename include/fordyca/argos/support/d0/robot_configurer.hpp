/**
 * \file d0/robot_configurer.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant/static_visitor.hpp>
#include <memory>

#include "cosm/argos/vis/config/visualization_config.hpp"

#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/subsystem/perception/oracular_info_receptor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, argos, support, d0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct robot_configurer
 * \ingroup support d0
 *
 * \brief Functor to perform controller configuration during initialization.
 */
template<typename TController>
class robot_configurer : public boost::static_visitor<void> {
 public:
  using controller_type = TController;
  robot_configurer(const cavis::config::visualization_config* const config,
                   const cforacle::foraging_oracle* const oracle)
      : mc_config(config),
        mc_oracle(oracle) {}

  template<typename U = TController,
           RCPPSW_SFINAE_TYPELIST_REQUIRE(fcontroller::d0::reactive_typelist,
                                         U)>
  void operator()(U* const c) const {
    if (nullptr != mc_config) {
      c->display_id(mc_config->robot_id);
      c->display_apf2D(mc_config->robot_apf2D);
    }
  }
  template<typename U = TController,
           typename V = TController,
           RCPPSW_SFINAE_TYPELIST_REJECT(fcontroller::d0::oracular_typelist,
                                         U),
           RCPPSW_SFINAE_TYPELIST_REQUIRE(fcontroller::d0::cognitive_typelist,
                                         V)>
  void operator()(U* const c) const {
    if (nullptr != mc_config) {
      c->display_los(mc_config->robot_los);
      c->display_id(mc_config->robot_id);
      c->display_apf2D(mc_config->robot_apf2D);
    }
  }
  template<typename U = TController,
           typename V = TController,
           RCPPSW_SFINAE_TYPELIST_REQUIRE(fcontroller::d0::oracular_typelist,
                                          U),
           RCPPSW_SFINAE_TYPELIST_REQUIRE(fcontroller::d0::cognitive_typelist,
                                          V)>
  void operator()(U* const c) const {
    if (nullptr != mc_config) {
      c->display_los(mc_config->robot_los);
      c->display_id(mc_config->robot_id);
      c->display_apf2D(mc_config->robot_apf2D);
    }
    if (nullptr != mc_oracle) {
      auto receptor = std::make_unique<fsperception::oracular_info_receptor>(mc_oracle);
      c->oracle_init(std::move(receptor));
    }
  }

 private:
  /* clang-format off */
  const cavis::config::visualization_config * const mc_config;
  const cforacle::foraging_oracle *                 mc_oracle;
  /* clang-format on */
};

NS_END(d0, support, argos, fordyca);

