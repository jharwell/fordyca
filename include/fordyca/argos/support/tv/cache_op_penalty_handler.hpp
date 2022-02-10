/**
 * \file cache_op_penalty_handler.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "cosm/tv/temporal_penalty_handler.hpp"

#include "fordyca/argos/support/tv/cache_op_filter.hpp"
#include "fordyca/argos/support/tv/cache_op_src.hpp"
#include "fordyca/argos/support/tv/cache_op_penalty_id_calculator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, tv);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class cache_op_penalty_handler
 * \ingroup argos support tv
 *
 * \brief The handler for block operation penalties for robots (e.g. picking
 * up, dropping in places that do not involve existing caches.
 */
class cache_op_penalty_handler final
    : public ctv::temporal_penalty_handler,
      public rer::client<cache_op_penalty_handler> {
 public:
  cache_op_penalty_handler(carena::caching_arena_map* const map,
                           const ctv::config::temporal_penalty_config* const config,
                           const std::string& name)
      : temporal_penalty_handler(config, name),
        ER_CLIENT_INIT("fordyca.support.tv.cache_op_penalty_handler"),
        m_map(map),
        m_filter(m_map) {}

  ~cache_op_penalty_handler(void) override = default;
  cache_op_penalty_handler& operator=(const cache_op_penalty_handler& other) =
      delete;
  cache_op_penalty_handler(const cache_op_penalty_handler&) = delete;

  /**
   * \brief Check if a robot has acquired a block or is in the nest, and is
   * trying to drop/pickup a block. If so, create a \ref temporal_penalty object
   * and associate it with the robot.
   *
   * \tparam TControllerType The type of the controller. Must be a template
   * parameter, rather than \ref controller::foraging_controller, because of the
   * goal acquisition determination done by \ref cache_op_filter.

   * \param controller The robot to check.
   * \param src The penalty source (i.e. what event caused this penalty to be
   *            applied).
   * \param t The current timestep.
  */
  template<typename TController>
  op_filter_status penalty_init(const TController& controller,
                                cache_op_src src,
                                const rtypes::timestep& t) {
    /*
     * Check if we have satisfied the conditions for a cache operation, which
     * involves querying the area map.
     *
     * NO LOCKING IS PERFORMED.
     *
     * There really can't be any locking here, because if there was, the
     * simulation would get REALLY slow for large swarms, because every robot
     * has to make this check every timestep.
     */
    auto filter = m_filter(controller, src);
    if (filter.status != op_filter_status::ekSATISFIED) {
      return filter.status;
    }

    ER_ASSERT(!is_serving_penalty(controller),
              "%s already serving cache penalty?",
              controller.GetId().c_str());

    rtypes::timestep orig_duration = penalty_calc(t);
    rtypes::type_uuid id = m_id_calc(src, filter);
    rtypes::timestep RCPPSW_UNUSED duration = penalty_add(&controller,
                                                        id,
                                                        orig_duration,
                                                        t);
    ER_INFO("%s: cache%d start=%zu, penalty=%zu, adjusted penalty=%zu src=%d",
            controller.GetId().c_str(),
            id.v(),
            t.v(),
            orig_duration.v(),
            duration.v(),
            static_cast<int>(src));

    return filter.status;
  }

 private:
  /* clang-format off */
  carena::caching_arena_map*     m_map;

  cache_op_filter                m_filter;
  cache_op_penalty_id_calculator m_id_calc{};
  /* clang-format on */
};
NS_END(tv, support, argos, fordyca);

