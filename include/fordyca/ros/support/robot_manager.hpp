/**
 * \file robot_manager.hpp
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

#include "cosm/ros/robot_manager_adaptor.hpp"
#include "cosm/ros/config/xml/robot_manager_repository.hpp"
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
 * \class robot_manager
 * \ingroup ros support
 *
 * \brief The base loop functions in FORDYCA that all other loop functions
 * inherit from when FORDYCA is built for ROS.
 */
class robot_manager : public cros::robot_manager_adaptor,
                      public rer::client<robot_manager> {
 public:
  robot_manager(const cros::config::sierra_config* config) RCPPSW_COLD;
  ~robot_manager(void) override RCPPSW_COLD;

  /* Not copy constructible/assignable by default */
  robot_manager(const robot_manager& s) = delete;
  robot_manager& operator=(const robot_manager& s) = delete;

  /* swarm manager overrides */
  void init(ticpp::Element&) override RCPPSW_COLD;
  void reset(void) override {}
  void pre_step(void) override;
  void post_step(void) override {}

  /* robot manager adaptor overrides */
  bool experiment_finished(void) const override;

 protected:
  const cros::config::xml::robot_manager_repository* config(void) const {
    return &m_config;
  }
  cros::config::xml::robot_manager_repository* config(void) {
    return &m_config;
  }
  void config_parse(ticpp::Element& node) RCPPSW_COLD;

 private:
  /* clang-format off */
  const cros::config::sierra_config           mc_sierra;
  cros::config::xml::robot_manager_repository m_config{};
  /* clang-format on */
};

NS_END(support, ros, fordyca);
