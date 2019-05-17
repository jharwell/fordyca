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
#include <boost/mpl/for_each.hpp>

#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include "fordyca/config/tv/tv_manager_config.hpp"
#include "fordyca/controller/base_controller.hpp"
#include "fordyca/controller/depth0/crw_controller.hpp"
#include "fordyca/controller/depth0/dpo_controller.hpp"
#include "fordyca/controller/depth0/mdpo_controller.hpp"
#include "fordyca/controller/depth0/odpo_controller.hpp"
#include "fordyca/controller/depth0/omdpo_controller.hpp"
#include "fordyca/controller/depth1/gp_dpo_controller.hpp"
#include "fordyca/controller/depth1/gp_mdpo_controller.hpp"
#include "fordyca/controller/depth1/gp_odpo_controller.hpp"
#include "fordyca/controller/depth1/gp_omdpo_controller.hpp"
#include "fordyca/controller/depth2/grp_dpo_controller.hpp"
#include "fordyca/controller/depth2/grp_mdpo_controller.hpp"
#include "fordyca/controller/depth2/grp_odpo_controller.hpp"
#include "fordyca/controller/depth2/grp_omdpo_controller.hpp"
#include "fordyca/support/base_loop_functions.hpp"
#include "fordyca/support/tv/tv_manager.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
template <typename Typelist,
          template <class> class PenaltyHandlerType,
          class PenaltyHandlerParamType>
class penalty_handler_initializer : public boost::static_visitor<void> {
 public:
  penalty_handler_initializer(const PenaltyHandlerParamType* const config,
                              ds::arena_map* const arena_map,
                              rds::type_map<Typelist>* type_map,
                              const std::string& handler_name)
      : mc_handler_name(handler_name),
        mc_config(config),
        m_arena_map(arena_map),
        m_type_map(type_map) {}

  /*
   * @todo Ideally these would be deleted, but emplace() does not seem to do
   * what I think it does (i.e. construct an object in place without a need for
   * a copy constructor), so it is defaulted instead.
   */
  penalty_handler_initializer(const penalty_handler_initializer&) = default;
  penalty_handler_initializer& operator=(const penalty_handler_initializer&) =
      delete;

  template <typename ControllerType>
  void operator()(const ControllerType& controller) {
    m_type_map->emplace(
        typeid(controller),
        rcppsw::make_unique<class PenaltyHandlerType<ControllerType>>(
            m_arena_map, mc_config, mc_handler_name));
  }

 private:
  /* clang-format off */
  const std::string                    mc_handler_name;
  const PenaltyHandlerParamType* const mc_config;
  ds::arena_map* const                 m_arena_map;
  rds::type_map<Typelist>*             m_type_map;
  /* clang-format on */
};

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
tv_manager::tv_manager(const config::tv::tv_manager_config* config,
                       const support::base_loop_functions* const lf,
                       ds::arena_map* const map)
    : ER_CLIENT_INIT("fordyca.support.tv.tv_manager"),
      mc_lf(lf),
      mc_motion_throttle_config(config->block_carry_throttle) {
  /* All controllers can drop blocks in the nest */
  boost::mpl::for_each<controller::typelist>(
      penalty_handler_initializer<block_handler_typelist,
                                  block_op_penalty_handler,
                                  decltype(config->block_manipulation_penalty)>(
          &config->block_manipulation_penalty,
          map,
          &m_nest_drop,
          "Nest Block Drop"));

  /* all controllers can pickup blocks */
  boost::mpl::for_each<controller::typelist>(
      penalty_handler_initializer<block_handler_typelist,
                                  block_op_penalty_handler,
                                  decltype(config->block_manipulation_penalty)>(
          &config->block_manipulation_penalty,
          map,
          &m_fb_pickup,
          "Free Block Pickup"));

  /* Only D1/D2 controllers interact with existing caches */
  boost::mpl::for_each<controller::d1d2_typelist>(
      penalty_handler_initializer<existing_cache_handler_typelist,
                                  cache_op_penalty_handler,
                                  decltype(config->cache_usage_penalty)>(
          &config->cache_usage_penalty,
          map,
          &m_existing_cache,
          "Existing Cache"));

  /* Only D2 controllers deal with new caches */
  boost::mpl::for_each<controller::depth2::typelist>(
      penalty_handler_initializer<fb_drop_handler_typelist,
                                  block_op_penalty_handler,
                                  decltype(config->block_manipulation_penalty)>(
          &config->block_manipulation_penalty, map, &m_new_cache, "New Cache"));

  /* Only D2 controllers deal with cache sites */
  boost::mpl::for_each<controller::depth2::typelist>(
      penalty_handler_initializer<fb_drop_handler_typelist,
                                  block_op_penalty_handler,
                                  decltype(config->block_manipulation_penalty)>(
          &config->block_manipulation_penalty,
          map,
          &m_cache_site,
          "Cache Site"));
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
double tv_manager::swarm_motion_throttle(void) const {
  double accum = 0.0;
  auto& robots = mc_lf->GetSpace().GetEntitiesByType("foot-bot");

  for (auto& entity_pair : robots) {
    auto* robot = argos::any_cast<argos::CFootBotEntity*>(entity_pair.second);
    auto& controller = dynamic_cast<controller::base_controller&>(
        robot->GetControllableEntity().GetController());
    accum += controller.applied_motion_throttle();
  } /* for(&entity..) */
  return accum / robots.size();
} /* swarm_motion_throttle() */

double tv_manager::env_block_manipulation(void) const {
  uint timestep = mc_lf->GetSpace().GetSimulationClock();
  return penalty_handler<controller::depth0::crw_controller>(
             block_op_src::ekNEST_DROP)
      ->timestep_penalty(timestep);
} /* env_block_manipulation() */

double tv_manager::env_cache_usage(void) const {
  uint timestep = mc_lf->GetSpace().GetSimulationClock();
  return penalty_handler<controller::depth1::gp_dpo_controller>(
             cache_op_src::ekEXISTING_CACHE_PICKUP)
      ->timestep_penalty(timestep);
} /* env_cache_usage() */

void tv_manager::register_controller(int robot_id) {
  m_motion_throttling.emplace(std::piecewise_construct,
                              std::forward_as_tuple(robot_id),
                              std::forward_as_tuple(&mc_motion_throttle_config));
} /* register_controller() */

void tv_manager::update(void) {
  auto& robots = mc_lf->GetSpace().GetEntitiesByType("foot-bot");
  uint timestep = mc_lf->GetSpace().GetSimulationClock();

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
