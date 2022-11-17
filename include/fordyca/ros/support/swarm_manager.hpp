/**
 * \file swarm_manager.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/config/rng_config.hpp"

#include "cosm/pal/ros/swarm_manager_adaptor.hpp"
#include "cosm/pal/ros/config/xml/swarm_manager_repository.hpp"
#include "cosm/ros/config/sierra_config.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::pal::config {
struct output_config;
} /* namespace cosm::metrics::config */

NS_START(fordyca, ros, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class swarm_manager
 * \ingroup ros support
 *
 * \brief The base loop functions in FORDYCA that all other loop functions
 * inherit from when FORDYCA is built for ROS. This code runs on the central ROS
 * node.
 */
class swarm_manager : public rer::client<swarm_manager>,
                      public cpros::swarm_manager_adaptor {
 public:
  swarm_manager(const cros::config::sierra_config* config) RCPPSW_COLD;
  ~swarm_manager(void) override RCPPSW_COLD;

  /* Not copy constructible/assignable by default */
  swarm_manager(const swarm_manager& s) = delete;
  swarm_manager& operator=(const swarm_manager& s) = delete;

  /* swarm manager overrides */
  void init(ticpp::Element&) override RCPPSW_COLD;
  void reset(void) override {}
  void pre_step(void) override;
  void post_step(void) override {}

  /* swarm_manager_adaptor overrides */
  bool experiment_finished(void) const override;

 protected:
  const cpros::config::xml::swarm_manager_repository* config(void) const {
    return &m_config;
  }
  cpros::config::xml::swarm_manager_repository* config(void) {
    return &m_config;
  }
  void config_parse(ticpp::Element& node) RCPPSW_COLD;

 private:
  /* clang-format off */
  const cros::config::sierra_config            mc_sierra;

  cpros::config::xml::swarm_manager_repository m_config{};
  /* clang-format on */
};

NS_END(support, ros, fordyca);
