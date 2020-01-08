/**
 * \file block_manip_collator.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_BLOCK_MANIP_COLLATOR_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_BLOCK_MANIP_COLLATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/types/timestep.hpp"

#include "fordyca/controller/base_controller.hpp"
#include "fordyca/fordyca.hpp"
#include "fordyca/fsm/block_transporter.hpp"
#include "fordyca/metrics/blocks/manipulation_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_manip_collator
 * \ingroup controller
 *
 * \brief Collates block manipulation metrics.
 */
class block_manip_collator : public metrics::blocks::manipulation_metrics {
 public:
  block_manip_collator(void) = default;
  ~block_manip_collator(void) override = default;

  /* block manipulation metrics */
  bool free_pickup_event(void) const override { return m_free_pickup_event; }
  bool free_drop_event(void) const override { return m_free_drop_event; }
  bool cache_pickup_event(void) const override { return m_cache_pickup_event; }
  bool cache_drop_event(void) const override { return m_cache_drop_event; }
  rtypes::timestep penalty_served(void) const override { return m_penalty; }

  void penalty_served(rtypes::timestep penalty) { m_penalty = penalty; }
  void free_pickup_event(bool b) { m_free_pickup_event = b; }
  void free_drop_event(bool b) { m_free_drop_event = b; }
  void cache_pickup_event(bool b) { m_cache_pickup_event = b; }
  void cache_drop_event(bool b) { m_cache_drop_event = b; }

  void reset(void) {
    m_free_pickup_event = false;
    m_free_drop_event = false;
    m_cache_pickup_event = false;
    m_cache_drop_event = false;
  }

 private:
  /* clang-format off */
  rtypes::timestep m_penalty{0};
  bool m_free_pickup_event{false};
  bool m_free_drop_event{false};
  bool m_cache_pickup_event{false};
  bool m_cache_drop_event{false};
  /* clang-format on */
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_BLOCK_MANIP_COLLATOR_HPP_ */
