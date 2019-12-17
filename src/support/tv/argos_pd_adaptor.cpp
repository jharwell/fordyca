/**
 * \file argos_pd_adaptor.cpp
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
#include "fordyca/support/tv/argos_pd_adaptor.hpp"

#include "fordyca/ds/arena_map.hpp"
#include "fordyca/support/swarm_iterator.hpp"
#include "fordyca/support/tv/env_dynamics.hpp"

#include "cosm/tv/config/population_dynamics_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, tv);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
argos_pd_adaptor::argos_pd_adaptor(
    const ctv::config::population_dynamics_config* config,
    support::base_loop_functions* const lf,
    ds::arena_map* map,
    env_dynamics* envd,
    const std::string& entity_prefix,
    const std::string& controller_xml_id,
    rmath::rng* rng)
    : ER_CLIENT_INIT("fordyca.support.tv.argos_pd_adaptor"),
      population_dynamics(config,
                          lf->GetSpace().GetEntitiesByType("foot-bot").size(),
                          rng),
      mc_entity_prefix(entity_prefix),
      mc_controller_xml_id(controller_xml_id),
      mc_lf(lf),
      m_map(map),
      m_envd(envd),
      m_rng(rng),
      m_lf(lf) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
argos_pd_adaptor::op_result argos_pd_adaptor::robot_kill(void) {
  auto range =
      rmath::rangei(0, m_lf->GetSpace().GetEntitiesByType("foot-bot").size());
  int id = m_rng->uniform(range);
  std::string name = mc_entity_prefix + rcppsw::to_string(id);

  auto* victim = argos::any_cast<controller::base_controller*>(
      m_lf->GetSpace().GetEntitiesByType("foot-bot").find(name)->second);

  /*
   * If the robot is carrying a block, drop/distribute it in the arena to avoid
   * it getting permanently lost when the it is removed.
   */
  if (victim->is_carrying_block()) {
    ER_INFO("Victim robot %s is carrying block%d",
            victim->GetId().c_str(),
            victim->block()->id().v());
    auto it = std::find_if(m_map->blocks().begin(),
                           m_map->blocks().end(),
                           [&](const auto& b) {
                             return victim->block()->id() == b->id();
                           });
    events::free_block_drop_visitor drop_op(
        *it,
        rmath::dvec2uvec(victim->position2D(), m_map->grid_resolution().v()),
        m_map->grid_resolution(),
        true);

    bool conflict =
        utils::free_block_drop_conflict(*m_map, it->get(), victim->position2D());
    utils::handle_arena_free_block_drop(drop_op, *m_map, conflict);
  }

  /* remove controller from any applied environmental variances */
  m_envd->unregister_controller(*victim);

  /* remove robot from ARGoS */
  m_lf->RemoveEntity(mc_entity_prefix + rcppsw::to_string(id));

  return {rtypes::type_uuid(id),
          m_lf->GetSpace().GetEntitiesByType("foot-bot").size()};
} /* robot_kill() */

argos_pd_adaptor::op_result argos_pd_adaptor::robot_add(
    size_t max_pop,
    const rtypes::type_uuid& id) {
  if (m_lf->GetSpace().GetEntitiesByType("foot-bot").size() >= max_pop) {
    ER_INFO("Not adding new robot %s%d: max_pop=%zu reached",
            mc_entity_prefix.c_str(),
            id.v(),
            max_pop);
    return {rtypes::constants::kNoUUID,
            m_lf->GetSpace().GetEntitiesByType("foot-bot").size()};
  }
  auto* fb = new argos::CFootBotEntity(mc_entity_prefix + rcppsw::to_string(id),
                                       mc_controller_xml_id,
                                       argos::CVector3());
  rmath::ranged xrange(2.0, mc_lf->arena_map()->xrsize() - 1.0);
  rmath::ranged yrange(2.0, mc_lf->arena_map()->yrsize() - 1.0);

  for (size_t i = 0; i < kMaxDistributeAttempts; ++i) {
    auto x = m_rng->uniform(xrange);
    auto y = m_rng->uniform(yrange);
    fb->GetEmbodiedEntity().GetOriginAnchor().Position =
        argos::CVector3(x, y, 0.0);
    try {
      m_lf->AddEntity(*fb);
      ER_INFO("Placed new robot %s at %s",
              fb->GetId().c_str(),
              rmath::vector2d(x, y).to_str().c_str());
      return {id, m_lf->GetSpace().GetEntitiesByType("foot-bot").size()};
    } catch (argos::CARGoSException& e) {
      ER_TRACE("Failed to place new robot %s at %s",
               fb->GetId().c_str(),
               rmath::vector2d(x, y).to_str().c_str());
    }
  } /* for(i..) */
  ER_FATAL_SENTINEL("Unable to add new robot to simulation");
} /* robot_add() */

argos_pd_adaptor::op_result argos_pd_adaptor::robot_malfunction(void) {
  /* TODO implement */
  return {rtypes::constants::kNoUUID,
          m_lf->GetSpace().GetEntitiesByType("foot-bot").size()};
} /* robot_malfunction() */

argos_pd_adaptor::op_result argos_pd_adaptor::robot_repair(
    const rtypes::type_uuid&) {
  /* TODO implement */
  return {rtypes::constants::kNoUUID,
          m_lf->GetSpace().GetEntitiesByType("foot-bot").size()};
} /* robot_repair() */

NS_END(tv, support, fordyca);
