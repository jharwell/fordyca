/**
 * \file block_op_penalty_handler.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_TV_BLOCK_OP_PENALTY_HANDLER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_TV_BLOCK_OP_PENALTY_HANDLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/types/type_uuid.hpp"

#include "fordyca/fsm/block_transporter.hpp"
#include "fordyca/support/tv/block_op_filter.hpp"
#include "fordyca/support/utils/loop_utils.hpp"
#include "fordyca/support/tv/temporal_penalty_handler.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, tv);

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * \class block_op_penalty_handler
 * \ingroup support tv
 *
 * \brief The handler for block operation penalties for robots (e.g. picking
 * up, dropping in places that do not involve existing caches.
 */
class block_op_penalty_handler final : public temporal_penalty_handler,
                                       public rer::client<block_op_penalty_handler> {
 public:
  block_op_penalty_handler(cfds::arena_map* const map,
                           const rct::config::waveform_config* const config,
                           const std::string& name)
      : temporal_penalty_handler(config, name),
        ER_CLIENT_INIT("fordyca.support.block_op_penalty_handler"),
        mc_map(map) {}

  ~block_op_penalty_handler(void) override = default;
  block_op_penalty_handler& operator=(const block_op_penalty_handler& other) =
      delete;
  block_op_penalty_handler(const block_op_penalty_handler&) = delete;

  /**
   * \brief Check if a robot has acquired a block or is in the nest, and is
   * trying to drop/pickup a block. If so, create a \ref temporal_penalty object
   * and associate it with the robot.
   *
   * \tparam TControllerType The type of the controller. Must be a template
   * parameter, rather than \ref controller::base_controller, because of the
   * goal acquisition determination done by \ref block_op_filter.
   *
   * \param controller The robot to check.
   * \param src The penalty source (i.e. what event caused this penalty to be
   *            applied).
   * \param t The current timestep.
   * \param prox_dist The minimum distance that the cache site needs to be from
   *                  all caches in the arena.
   */
  template<typename TControllerType>
  op_filter_status penalty_init(const TControllerType& controller,
                                block_op_src src,
                                const rtypes::timestep& t,
                                rtypes::spatial_dist cache_prox = rtypes::spatial_dist(-1)) {
    auto filter = block_op_filter<TControllerType>(
        mc_map)(controller, src, cache_prox);
    if (filter != op_filter_status::ekSATISFIED) {
      return filter;
    }
    ER_ASSERT(!is_serving_penalty(controller),
              "%s already serving block penalty?",
              controller.GetId().c_str());

    rtypes::type_uuid id = penalty_id_calc(controller, src, cache_prox);
    rtypes::timestep orig_duration = penalty_calc(t);
    auto duration RCSW_UNUSED = penalty_add(&controller,
                                            id,
                                            orig_duration,
                                            t);

    ER_INFO("%s: block%d start=%u, penalty=%u, adjusted penalty=%u src=%d",
            controller.GetId().c_str(),
            id.v(),
            t.v(),
            orig_duration.v(),
            duration.v(),
            static_cast<int>(src));

    return filter;
  }

 private:
  template<typename TControllerType>
  rtypes::type_uuid penalty_id_calc(const TControllerType& controller,
                      block_op_src src,
                      rtypes::spatial_dist cache_prox) const {
    rtypes::type_uuid id(-1);
    switch (src) {
      case block_op_src::ekFREE_PICKUP:
        id = utils::robot_on_block(controller, *mc_map);
        ER_ASSERT(rtypes::constants::kNoUUID != id, "Robot not on block?");
        break;
      case block_op_src::ekNEST_DROP:
        ER_ASSERT(nullptr != controller.block() &&
                  rtypes::constants::kNoUUID != controller.block()->id(),
                  "Robot not carrying block?");
        id = controller.block()->id();
        break;
      case block_op_src::ekCACHE_SITE_DROP:
        ER_ASSERT(nullptr != controller.block() &&
                      rtypes::constants::kNoUUID != controller.block()->id(),
                  "Robot not carrying block?");
        id = controller.block()->id();
        break;
      case block_op_src::ekNEW_CACHE_DROP:
        ER_ASSERT(nullptr != controller.block() &&
                      rtypes::constants::kNoUUID != controller.block()->id(),
                  "Robot not carrying block?");
        ER_ASSERT(cache_prox > 0.0,
                  "Cache proximity distance not specified for new cache drop");
        id = controller.block()->id();
        break;
      default:
        break;
    }
    return id;
  } /* penalty_id_calc() */

  /* clang-format off */
  const cfds::arena_map* const mc_map;
  /* clang-format on */
};
NS_END(tv, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_TV_BLOCK_OP_PENALTY_HANDLER_HPP_ */
