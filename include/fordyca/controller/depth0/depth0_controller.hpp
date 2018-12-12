/**
 * @file depth0_controller.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH0_DEPTH0_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH0_DEPTH0_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"
#include "fordyca/controller/base_controller.hpp"
#include "fordyca/fsm/block_transporter.hpp"
#include "fordyca/metrics/blocks/manipulation_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class depth0_controller : public base_controller,
                          public metrics::blocks::manipulation_metrics,
                          public fsm::block_transporter {
 public:
  depth0_controller(void) = default;
  ~depth0_controller(void) override = default;

  /* block manipulation metrics */
  bool free_pickup_event(void) const override { return m_free_pickup_event; }
  bool free_drop_event(void) const override { return m_free_drop_event; }
  bool cache_pickup_event(void) const override { return false; }
  bool cache_drop_event(void) const override { return false; }

  /* block manipulation metrics */
  uint penalty_served(void) const override { return m_penalty; }

  void free_pickup_event(bool b) { m_free_pickup_event = b; }
  void free_drop_event(bool b) { m_free_drop_event = b; }
  void penalty_served(uint penalty) { m_penalty = penalty; }

 private:
  // clang-format off
  uint m_penalty{false};
  bool m_free_pickup_event{false};
  bool m_free_drop_event{false};
  // clang-format on
};

NS_END(depth0, controller, fordyca);


#endif /* INCLUDE_FORDYCA_CONTROLLER_DEPTH0_DEPTH0_CONTROLLER_HPP_ */
