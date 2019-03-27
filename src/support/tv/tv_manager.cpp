/**
 * @file tv_manager.cpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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
/*
 * This is needed because without it boost instantiates static assertions that
 * verify that every possible handler<controller> instantiation is valid, which
 * includes checking for depth1 controllers being valid for new cache drop/cache
 * site drop events. These will not happen in reality (or shouldn't), and if
 * they do it's 100% OK to crash with an exception.
 */
#define BOOST_VARIANT_USE_RELAXED_GET_BY_DEFAULT

#include "fordyca/support/tv/tv_manager.hpp"
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include "fordyca/controller/base_controller.hpp"
#include "fordyca/controller/depth0/crw_controller.hpp"
#include "fordyca/controller/depth0/dpo_controller.hpp"
#include "fordyca/controller/depth0/mdpo_controller.hpp"
#include "fordyca/controller/depth1/gp_dpo_controller.hpp"
#include "fordyca/controller/depth1/gp_mdpo_controller.hpp"
#include "fordyca/controller/depth2/grp_dpo_controller.hpp"
#include "fordyca/controller/depth2/grp_mdpo_controller.hpp"
#include "fordyca/params/tv/tv_manager_params.hpp"
#include "fordyca/support/depth0/depth0_loop_functions.hpp"
#include "fordyca/support/depth1/depth1_loop_functions.hpp"

/*******************************************************************************
 * Namespaces/decls
 ******************************************************************************/
NS_START(fordyca, support, tv);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
tv_manager::tv_manager(const params::tv::tv_manager_params* params,
                       const support::base_loop_functions* const lf,
                       ds::arena_map* const map)
    : ER_CLIENT_INIT("fordyca.support.tv.tv_manager"),
      mc_lf(lf),
      mc_motion_throttle_params(params->block_carry_throttle) {
  nest_drop_init(params, map);
  fb_pickup_init(params, map);
  existing_cache_init(params, map);
  cache_site_init(params, map);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void tv_manager::nest_drop_init(const params::tv::tv_manager_params* params,
                                ds::arena_map* const map) {
  m_nest_drop.emplace(
      typeid(controller::depth0::crw_controller),
      rcppsw::make_unique<
          block_op_penalty_handler<controller::depth0::crw_controller>>(
          map, &params->block_manipulation_penalty, "Nest block drop"));
  m_nest_drop.emplace(
      typeid(controller::depth0::dpo_controller),
      rcppsw::make_unique<
          block_op_penalty_handler<controller::depth0::dpo_controller>>(
          map, &params->block_manipulation_penalty, "Nest block drop"));
  m_nest_drop.emplace(
      typeid(controller::depth0::mdpo_controller),
      rcppsw::make_unique<
          block_op_penalty_handler<controller::depth0::mdpo_controller>>(
          map, &params->block_manipulation_penalty, "Nest block drop"));
  m_nest_drop.emplace(
      typeid(controller::depth1::gp_dpo_controller),
      rcppsw::make_unique<
          block_op_penalty_handler<controller::depth1::gp_dpo_controller>>(
          map, &params->block_manipulation_penalty, "Nest block drop"));
  m_nest_drop.emplace(
      typeid(controller::depth1::gp_mdpo_controller),
      rcppsw::make_unique<
          block_op_penalty_handler<controller::depth1::gp_mdpo_controller>>(
          map, &params->block_manipulation_penalty, "Nest block drop"));
  m_nest_drop.emplace(
      typeid(controller::depth2::grp_dpo_controller),
      rcppsw::make_unique<
          block_op_penalty_handler<controller::depth2::grp_dpo_controller>>(
          map, &params->block_manipulation_penalty, "Nest block drop"));
  m_nest_drop.emplace(
      typeid(controller::depth2::grp_mdpo_controller),
      rcppsw::make_unique<
          block_op_penalty_handler<controller::depth2::grp_mdpo_controller>>(
          map, &params->block_manipulation_penalty, "Nest block drop"));
} /* nest_drop_init() */

void tv_manager::fb_pickup_init(const params::tv::tv_manager_params* params,
                                ds::arena_map* const map) {
  m_fb_pickup.emplace(
      typeid(controller::depth0::crw_controller),
      rcppsw::make_unique<
          block_op_penalty_handler<controller::depth0::crw_controller>>(
          map, &params->block_manipulation_penalty, "Free block pickup"));
  m_fb_pickup.emplace(
      typeid(controller::depth0::dpo_controller),
      rcppsw::make_unique<
          block_op_penalty_handler<controller::depth0::dpo_controller>>(
          map, &params->block_manipulation_penalty, "Free block pickup"));
  m_fb_pickup.emplace(
      typeid(controller::depth0::mdpo_controller),
      rcppsw::make_unique<
          block_op_penalty_handler<controller::depth0::mdpo_controller>>(
          map, &params->block_manipulation_penalty, "Free block pickup"));
  m_fb_pickup.emplace(
      typeid(controller::depth1::gp_dpo_controller),
      rcppsw::make_unique<
          block_op_penalty_handler<controller::depth1::gp_dpo_controller>>(
          map, &params->block_manipulation_penalty, "Free block pickup"));
  m_fb_pickup.emplace(
      typeid(controller::depth1::gp_mdpo_controller),
      rcppsw::make_unique<
          block_op_penalty_handler<controller::depth1::gp_mdpo_controller>>(
          map, &params->block_manipulation_penalty, "Free block pickup"));
  m_fb_pickup.emplace(
      typeid(controller::depth2::grp_dpo_controller),
      rcppsw::make_unique<
          block_op_penalty_handler<controller::depth2::grp_dpo_controller>>(
          map, &params->block_manipulation_penalty, "nFree block pickup"));
  m_fb_pickup.emplace(
      typeid(controller::depth2::grp_mdpo_controller),
      rcppsw::make_unique<
          block_op_penalty_handler<controller::depth2::grp_mdpo_controller>>(
          map, &params->block_manipulation_penalty, "Free block pickup"));
} /* fb_pickup_init() */

void tv_manager::existing_cache_init(const params::tv::tv_manager_params* params,
                                     ds::arena_map* const map) {
  m_existing_cache.emplace(
      typeid(controller::depth1::gp_dpo_controller),
      rcppsw::make_unique<
          cache_op_penalty_handler<controller::depth1::gp_dpo_controller>>(
          map, &params->cache_usage_penalty, "Cache"));
  m_existing_cache.emplace(
      typeid(controller::depth1::gp_mdpo_controller),
      rcppsw::make_unique<
          cache_op_penalty_handler<controller::depth1::gp_mdpo_controller>>(
          map, &params->cache_usage_penalty, "Cache"));

  m_existing_cache.emplace(
      typeid(controller::depth2::grp_dpo_controller),
      rcppsw::make_unique<
          cache_op_penalty_handler<controller::depth2::grp_dpo_controller>>(
          map, &params->block_manipulation_penalty, "Cache"));

  m_existing_cache.emplace(
      typeid(controller::depth2::grp_mdpo_controller),
      rcppsw::make_unique<
          cache_op_penalty_handler<controller::depth2::grp_mdpo_controller>>(
          map, &params->block_manipulation_penalty, "Cache"));
} /* existing_cache_init() */

void tv_manager::cache_site_init(const params::tv::tv_manager_params* params,
                                 ds::arena_map* const map) {
  m_cache_site.emplace(
      typeid(controller::depth2::grp_dpo_controller),
      rcppsw::make_unique<
          block_op_penalty_handler<controller::depth2::grp_dpo_controller>>(
          map, &params->block_manipulation_penalty, "Cache site block drop"));

  m_cache_site.emplace(
      typeid(controller::depth2::grp_mdpo_controller),
      rcppsw::make_unique<
          block_op_penalty_handler<controller::depth2::grp_mdpo_controller>>(
          map, &params->block_manipulation_penalty, "Cache site block drop"));

  m_new_cache.emplace(
      typeid(controller::depth2::grp_dpo_controller),
      rcppsw::make_unique<
          block_op_penalty_handler<controller::depth2::grp_dpo_controller>>(
          map, &params->block_manipulation_penalty, "Cache site block drop"));

  m_new_cache.emplace(
      typeid(controller::depth2::grp_mdpo_controller),
      rcppsw::make_unique<
          block_op_penalty_handler<controller::depth2::grp_mdpo_controller>>(
          map, &params->block_manipulation_penalty, "Cache site block drop"));
} /* cache_site_init() */

double tv_manager::swarm_motion_throttle(void) const {
  double accum = 0.0;
  auto& robots = const_cast<support::base_loop_functions*>(mc_lf)
                     ->GetSpace()
                     .GetEntitiesByType("foot-bot");

  for (auto& entity_pair : robots) {
    auto* robot = argos::any_cast<argos::CFootBotEntity*>(entity_pair.second);
    auto& controller = dynamic_cast<controller::base_controller&>(
        robot->GetControllableEntity().GetController());
    accum += controller.applied_motion_throttle();
  } /* for(&entity..) */
  return accum / robots.size();
} /* swarm_motion_throttle() */

double tv_manager::env_block_manipulation(void) const {
  uint timestep = const_cast<support::base_loop_functions*>(mc_lf)
                      ->GetSpace()
                      .GetSimulationClock();

  return penalty_handler<controller::depth0::crw_controller>(
             block_op_src::kSrcNestDrop)
      ->timestep_penalty(timestep);
} /* env_block_manipulation() */

double tv_manager::env_cache_usage(void) const {
  uint timestep = const_cast<support::base_loop_functions*>(mc_lf)
                      ->GetSpace()
                      .GetSimulationClock();
  return penalty_handler<controller::depth1::gp_dpo_controller>(
             cache_op_src::kSrcExistingCachePickup)
      ->timestep_penalty(timestep);
} /* env_cache_usage() */

void tv_manager::register_controller(int robot_id) {
  m_motion_throttling.emplace(std::piecewise_construct,
                              std::forward_as_tuple(robot_id),
                              std::forward_as_tuple(&mc_motion_throttle_params));
} /* register_controller() */

void tv_manager::update(void) {
  auto& robots = const_cast<support::base_loop_functions*>(mc_lf)
                     ->GetSpace()
                     .GetEntitiesByType("foot-bot");
  uint timestep = const_cast<support::base_loop_functions*>(mc_lf)
                      ->GetSpace()
                      .GetSimulationClock();

  for (auto& entity_pair : robots) {
    auto* robot = argos::any_cast<argos::CFootBotEntity*>(entity_pair.second);
    auto& controller = dynamic_cast<controller::base_controller&>(
        robot->GetControllableEntity().GetController());
    m_motion_throttling.at(controller.entity_id())
        .toggle(controller.is_carrying_block());
    m_motion_throttling.at(controller.entity_id()).update(timestep);
  } /* for(&entity..) */
} /* update() */

NS_END(tv, support, fordyca);
