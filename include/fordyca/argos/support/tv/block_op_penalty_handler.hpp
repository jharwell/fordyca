/**
 * \file block_op_penalty_handler.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <boost/optional.hpp>

#include "rcppsw/types/type_uuid.hpp"

#include "cosm/tv/temporal_penalty_handler.hpp"

#include "fordyca/argos/support/tv/block_op_filter.hpp"
#include "fordyca/argos/support/tv/block_op_penalty_id_calculator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, tv);

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * \class block_op_penalty_handler
 * \ingroup argos support tv
 *
 * \brief The handler for block operation penalties for robots (e.g. picking
 * up, dropping in places that do not involve existing caches).
 */
class block_op_penalty_handler final : public ctv::temporal_penalty_handler,
                                       public rer::client<block_op_penalty_handler> {
 public:
  block_op_penalty_handler(carena::caching_arena_map* const map,
                           const ctv::config::temporal_penalty_config* const config,
                           const std::string& name)
      : temporal_penalty_handler(config, name),
        ER_CLIENT_INIT("fordyca.argos.support.tv.block_op_penalty_handler"),
        m_map(map),
        m_filter(m_map),
        m_id_calc(m_map) {}

  ~block_op_penalty_handler(void) override = default;
  block_op_penalty_handler& operator=(const block_op_penalty_handler& other) =
      delete;
  block_op_penalty_handler(const block_op_penalty_handler&) = delete;

  /**
   * \brief Check if a robot has acquired a block or is in the nest, and is
   * trying to drop/pickup a block. If so, create a \ref temporal_penalty object
   * and associate it with the robot.
   *
   * \tparam TController The type of the controller. Must be a template
   * parameter, rather than \ref controller::foraging_controller, because of the
   * goal acquisition determination done by \ref block_op_filter.
   *
   * \param controller The robot to check.
   * \param src The penalty source (i.e. what event caused this penalty to be
   *            applied).
   * \param t The current timestep.
   * \param prox_dist The minimum distance that the cache site needs to be from
   *                  all caches in the arena.
   */
  op_filter_status penalty_init(const controller::foraging_controller& controller,
                                const rtypes::timestep& t,
                                block_op_src src,
                                const boost::optional<rspatial::euclidean_dist>& cache_prox) {
    /*
     * Check if we have satisfied the conditions for a block operation, which
     * involves querying the area map.
     *
     * NO LOCKING IS PERFORMED.
     *
     * There really can't be any locking here, because if there was, the
     * simulation would get REALLY slow for large swarms, because every robot
     * has to make this check every timestep. This makes the resulting block ID
     * we get on some successful operations potentially invalid; that's OK, as
     * validity is thoroughly checked after the penalty is served before
     * executing the action.
     *
     * See FORDYCA#668.
     */
    auto filter = m_filter(controller, src, cache_prox);
    if (filter.status != op_filter_status::ekSATISFIED) {
      return filter.status;
    }
    ER_ASSERT(!is_serving_penalty(controller),
              "%s already serving block penalty?",
              controller.GetId().c_str());

    /*
     * Because NO LOCKING IS PERFORMED around the first check and this one, we
     * may no longer meet the conditions for free block pickup, and the ID we
     * already have of the block we are on/might be on might not be valid
     * anymore.
     *
     * See FORDYCA#668.
     */
    rtypes::type_uuid id = m_id_calc(controller, src, filter);

    rtypes::timestep orig = penalty_calc(t);
    rtypes::timestep RCPPSW_UNUSED adjusted = penalty_add(&controller,
                                                          id,
                                                          orig,
                                                          t);

    ER_INFO("%s: block%d start=%zu, penalty=%zu, adjusted penalty=%zu src=%d",
            controller.GetId().c_str(),
            id.v(),
            t.v(),
            orig.v(),
            adjusted.v(),
            static_cast<int>(src));

    return filter.status;
  }

 private:
  /* clang-format off */
  carena::caching_arena_map*     m_map;

  block_op_filter                m_filter;
  block_op_penalty_id_calculator m_id_calc;
  /* clang-format on */
};
NS_END(tv, support, argos, fordyca);

