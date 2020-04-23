/**
 * \file ledtaxis_cache_search.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fsm/expstrat/ledtaxis_cache_search.hpp"

#include "cosm/robots/footbot/footbot_saa_subsystem2D.hpp"
#include "cosm/robots/footbot/footbot_sensing_subsystem.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, fsm, expstrat);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void ledtaxis_cache_search::task_execute(void) {
  /*
   * If we are in a cache and executing this explore behavior, then the cache we
   * are currently in must be unsuitable for some reason. Don't try to taxis to
   * a cache signal in this cache, as it will not serve any purpose. Instead,
   * wander until the cache becomes suitable or something else changes.
   */
  if (saa()->sensing()->ground()->detect("cache")) {
    if (m_taxis.task_running()) {
      m_taxis.task_reset();
    }
  } else {
    /* Otherwise, we are not inside a cache, so taxis to one */
    if (m_taxis.task_running()) {
      m_taxis.task_execute();
      return;
    }
  }
  /*
   * If we get here, then we are probably inside an unsuitable cache, so just
   * fall back to wandering.
   */
  if (!m_crw.task_running()) {
    m_crw.task_reset();
    m_crw.task_start(nullptr);
  }
  m_crw.task_execute();
} /* task_execute() */

void ledtaxis_cache_search::task_start(const cta::taskable_argument*) {
  saa()->sensing()->blobs()->enable();
  m_taxis.task_start(nullptr);
} /* task_start() */

void ledtaxis_cache_search::task_reset(void) {
  saa()->sensing()->blobs()->disable();
  m_taxis.task_reset();
  m_crw.task_reset();
} /* task_reset() */

/*******************************************************************************
 * Collision Metrics
 ******************************************************************************/
bool ledtaxis_cache_search::in_collision_avoidance(void) const {
  return (m_taxis.task_running() && m_taxis.in_collision_avoidance()) ||
         (m_crw.task_running() && m_crw.in_collision_avoidance());
} /* in_collision_avoidance() */

bool ledtaxis_cache_search::entered_collision_avoidance(void) const {
  return (m_taxis.task_running() && m_taxis.entered_collision_avoidance()) ||
         (m_crw.task_running() && m_crw.entered_collision_avoidance());
} /* entered_collision_avoidance() */

bool ledtaxis_cache_search::exited_collision_avoidance(void) const {
  return (m_taxis.task_running() && m_taxis.exited_collision_avoidance()) ||
         (m_crw.task_running() && m_crw.exited_collision_avoidance());
} /* exited_collision_avoidance() */

rtypes::timestep ledtaxis_cache_search::collision_avoidance_duration(void) const {
  if (m_taxis.task_running()) {
    return m_taxis.collision_avoidance_duration();
  } else if (m_crw.task_running()) {
    return m_crw.collision_avoidance_duration();
  } else {
    return rtypes::timestep(0);
  }
} /* collision_avoidance_duration() */

rmath::vector2z ledtaxis_cache_search::avoidance_loc(void) const {
  ER_ASSERT(m_taxis.task_running() || m_crw.task_running(),
            "In collision avoidance without running task?");
  if (m_taxis.task_running()) {
    return m_taxis.avoidance_loc();
  } else {
    return m_crw.avoidance_loc();
  }
} /* collision_avoidance_duration() */

NS_END(expstrat, fsm, fordyca);
