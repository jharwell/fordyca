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

#include "cosm/tv/config/population_dynamics_config.hpp"

#include "fordyca/ds/arena_map.hpp"
#include "fordyca/support/swarm_iterator.hpp"
#include "fordyca/support/tv/env_dynamics.hpp"

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
  size_t current_pop = swarm_population();
  rtypes::type_uuid id = rtypes::constants::kNoUUID;

  if (0 == current_pop) {
    ER_WARN("Not killing robot: pop_size=%zu", current_pop);
    return {id, current_pop};
  }
  auto range = rmath::rangei(0, current_pop - 1);

  argos::CFootBotEntity* entity;
  controller::base_controller* controller;
  for (size_t i = 0; i < kMaxOperationAttempts; ++i) {
    auto it = m_lf->GetSpace().GetEntitiesByType("foot-bot").begin();
    std::advance(it, m_rng->uniform(range));
    entity = argos::any_cast<argos::CFootBotEntity*>(it->second);
    controller = dynamic_cast<controller::base_controller*>(
        &entity->GetControllableEntity().GetController());
    if (!already_killed(controller->entity_id())) {
      id = controller->entity_id();
      break;
    }
  } /* for(i..) */

  if (rtypes::constants::kNoUUID == id) {
    ER_WARN("Unable to find victim: pop_size=%zu", current_pop);
    return {id, current_pop};
  }
  ER_INFO("Found victim with ID=%d", id.v());

  std::string name = mc_entity_prefix + rcppsw::to_string(id);
  /*
   * If the robot is carrying a block, drop/distribute it in the arena to avoid
   * it getting permanently lost when the it is removed.
   */
  if (controller->is_carrying_block()) {
    ER_INFO("Victim robot %s is carrying block%d",
            controller->GetId().c_str(),
            controller->block()->id().v());
    auto it = std::find_if(m_map->blocks().begin(),
                           m_map->blocks().end(),
                           [&](const auto& b) {
                             return controller->block()->id() == b->id();
                           });
    events::free_block_drop_visitor drop_op(
        *it,
        rmath::dvec2uvec(controller->position2D(), m_map->grid_resolution().v()),
        m_map->grid_resolution(),
        true);

    bool conflict = utils::free_block_drop_conflict(*m_map,
                                                    it->get(),
                                                    controller->position2D());
    utils::handle_arena_free_block_drop(drop_op, *m_map, conflict);
  }

  /* remove controller from any applied environmental variances */
  m_envd->unregister_controller(*controller);

  /* remove robot from ARGoS */
  ER_INFO("Remove entity %s", name.c_str());
  m_lf->RemoveEntity(name);

  return {rtypes::type_uuid(id),
          m_lf->GetSpace().GetEntitiesByType("foot-bot").size()};
} /* robot_kill() */

argos_pd_adaptor::op_result argos_pd_adaptor::robot_add(
    int max_pop,
    const rtypes::type_uuid& id) {
  size_t current_pop = m_lf->GetSpace().GetEntitiesByType("foot-bot").size();
  if (max_pop != -1 && current_pop >= static_cast<size_t>(max_pop)) {
    ER_INFO("Not adding new robot %s%d: max_pop=%d reached",
            mc_entity_prefix.c_str(),
            id.v(),
            max_pop);
    return {rtypes::constants::kNoUUID, current_pop};
  }

  /*
   * Give 2.0 buffer around the edges of the arena so that robots are not too
   * close to the boundaries of physics engines, which can cause "no engine can
   * house entity" exceptions in rare cases otherwise.
   */
  rmath::ranged xrange(2.0, mc_lf->arena_map()->xrsize() - 2.0);
  rmath::ranged yrange(2.0, mc_lf->arena_map()->yrsize() - 2.0);
  argos::CFootBotEntity* fb = nullptr;

  /*
   * You CANNOT first create the entity, then attempt to move it to a
   * collision free location within ARGoS when there are multiple physics
   * engines used--you always get an exception thrown. The only way to ensure
   * correct operation is to pass the desired location to the entity constructor
   * BEFORE calling AddEntity(). This is suboptimal, because it involves
   * potentially a lot of dynamic memory management that can slow things down,
   * but it is required. See #623.
   *
   * This is a @bug in ARGoS, and so this code can be reverted to something like
   * what is was originally once this is fixed in the ARGoS master. Diffing the
   * branch for #623 against the previous commit should show the changes.
   */
  for (size_t i = 0; i < kMaxOperationAttempts; ++i) {
    auto x = m_rng->uniform(xrange);
    auto y = m_rng->uniform(yrange);
    try {
      /* ick raw pointers--thanks ARGoS... */
      fb = new argos::CFootBotEntity(mc_entity_prefix + rcppsw::to_string(id),
                                           mc_controller_xml_id,
                                           argos::CVector3(x, y, 0.0));
      m_lf->AddEntity(*fb);
      ER_INFO("Added entity %s attached to physics engine %s at %s",
              fb->GetId().c_str(),
              fb->GetEmbodiedEntity().GetPhysicsModel(0).GetEngine().GetId().c_str(),
              rmath::vector2d(x, y).to_str().c_str());

      /* Register controller for environmental variances */
      auto* controller = dynamic_cast<controller::base_controller*>(
          &fb->GetControllableEntity().GetController());

      m_envd->register_controller(*controller);

      return {id, m_lf->GetSpace().GetEntitiesByType("foot-bot").size()};
    } catch (argos::CARGoSException& e) {
      if (nullptr != fb) {
        delete fb; /* ick raw pointers--thanks ARGoS... */
      }
      ER_TRACE("Failed to place new robot %s at %s",
               fb->GetId().c_str(),
               rmath::vector2d(x, y).to_str().c_str());
    }
  } /* for(i..) */
  ER_FATAL_SENTINEL("Unable to add new robot to simulation");
  return {rtypes::constants::kNoUUID, current_pop};
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
