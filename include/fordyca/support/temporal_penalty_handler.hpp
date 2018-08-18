/**
 * @file temporal_penalty_handler.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_TEMPORAL_PENALTY_HANDLER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_TEMPORAL_PENALTY_HANDLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>

#include "fordyca/support/temporal_penalty.hpp"
#include "fordyca/support/loop_functions_utils.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/control/waveform_generator.hpp"
#include "rcppsw/control/periodic_waveform.hpp"
#include "rcppsw/control/waveform_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace ct = rcppsw::control;
NS_START(fordyca, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * @class temporal_penalty_handler
 * @ingroup support depth1
 *
 * @brief The penalty handler for penalties for robots (e.g. how long they have
 * to wait when they pickup/drop a block).
 *
 * Does not do much more than provide the penalty list, and functions for
 * manipulating it to derived classes.
 */
template <typename T>
class temporal_penalty_handler : public rcppsw::er::client {
 public:
  /**
   * @Brief Initialize the penalty handler.
   *
   * @param server Server for debugging.
   * @param params Parameters for penalty waveform generation.
   */
  temporal_penalty_handler(std::shared_ptr<rcppsw::er::server> server,
                       const ct::waveform_params* const params)
      : client(server),
        m_penalty_list(),
        m_penalty(ct::waveform_generator()(params->type, params)) {
    insmod("temporal_penalty_handler",
           rcppsw::er::er_lvl::DIAG,
           rcppsw::er::er_lvl::NOM);
  }

  ~temporal_penalty_handler(void) override { client::rmmod(); }

  /**
   * @brief Determine if a robot has satisfied the \ref temporal_penalty
   * it is currently serving yet.
   *
   * @param robot The robot to check. If the robot is not currently serving a
   * penalty, \c FALSE will be returned.
   * @param timestep The current timestep.
   *
   * @return \c TRUE If the robot is currently waiting AND it has satisfied its
   * penalty.
   */
  __rcsw_pure bool penalty_satisfied(const T& controller, uint timestep) const {
    auto it = std::find_if(m_penalty_list.begin(), m_penalty_list.end(),
                           [&](const temporal_penalty<T>& p) {
                             return p.controller() == &controller;
                           });
    if (it != m_penalty_list.end()) {
      return it->penalty_satisfied(timestep);
    }
    return false;
  }
  /**
   * @brief Get the next robot that will satisfy its penalty from the list.
   */
  const temporal_penalty<T>& next(void) const { return m_penalty_list.front(); }

  /**
   * @brief Remove the specified penalty from the list once the robot it
   * corresponds to has served its penalty.
   *
   * @param victim The penalty to remove.
   */
  void remove(const temporal_penalty<T>& victim) {
    return m_penalty_list.remove(victim);
  }

  /**
   * @brief Abort a robot's serving of its penalty.
   *
   * This should only be done if the robot aborts its task WHILE also serving a
   * penalty.
   *
   * @param controller The robot to abort the penalty for.
   */
  void penalty_abort(const T& controller) {
    auto it = find(controller);
    if (it != m_penalty_list.end()) {
      m_penalty_list.remove(*it);
    }
    ER_NOM("fb%d", utils::robot_id(controller));
    ER_ASSERT(!is_serving_penalty(controller),
              "FATAL: Robot still serving penalty after abort?!");
  }

  typename std::list<temporal_penalty<T>>::iterator find(const T& controller) {
    return std::find_if(m_penalty_list.begin(),
                        m_penalty_list.end(),
                        [&](const temporal_penalty<T>& p) {
                          return p.controller() == &controller;
                        });
  }
  /**
   * @brief If \c TRUE, then the specified robot is currently serving a cache
   * penalty.
   */
  __rcsw_pure bool is_serving_penalty(const T& controller) const {
    auto it = std::find_if(m_penalty_list.begin(), m_penalty_list.end(),
                           [&](const temporal_penalty<T>& p) {
                             return p.controller() == &controller; });
    return it != m_penalty_list.end();
  }

  size_t list_size(void) const { return m_penalty_list.size(); }

 protected:
  std::list<temporal_penalty<T>>& penalty_list(void) {
    return m_penalty_list;
  }
  const std::list<temporal_penalty<T>>& penalty_list(void) const {
    return m_penalty_list;
  }

  /*
   * @brief Deconflict penalties such that at most 1 robot finishes
   * serving their penalty per block/cache operation.
   *
   * If the penalty for the robot was zero, should we still need to make the
   * robot serve a 1 timestep penalty. Not needed for block ops (but doesn't
   * really hurt), but IS needed for cache ops, so that if two robots that enter
   * a cache on the same timestep and will serve 0 duration penalties things are
   * still handled properly. You can't rely on just checking the list in that
   * case, because 0 duration penalties are marked as served and removed from
   * the list the SAME timestep they are added, so the handler incorrectly
   * thinks that there is no conflict.
   *
   * @param timestep The current timestep.
   */
  uint deconflict_penalty_finish(uint timestep) const {
    uint penalty = m_penalty->value(timestep);
    if (0 == penalty) {
      ++penalty;
    }
    m_orig_penalty = penalty;
    for (auto it = m_penalty_list.begin(); it != m_penalty_list.end(); ++it) {
      if (it->start_time() + it->penalty() == timestep + penalty) {
        ++penalty;
        it = m_penalty_list.begin();
      }
    } /* for(i..) */
    return penalty;
  }

  uint original_penalty(void) const { return m_orig_penalty; }

  // clang-format off
  mutable uint                   m_orig_penalty{0};
  std::list<temporal_penalty<T>> m_penalty_list;
  std::unique_ptr<ct::waveform>  m_penalty;
  // clang-format on
};
NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_TEMPORAL_PENALTY_HANDLER_HPP_ */
