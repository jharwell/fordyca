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

#include "cosm/arena/arena_map.hpp"
#include "cosm/arena/operations/free_block_drop.hpp"
#include "cosm/tv/config/population_dynamics_config.hpp"

#include "fordyca/support/swarm_iterator.hpp"
#include "fordyca/support/tv/env_dynamics.hpp"
#include "fordyca/events/mech_malfunction.hpp"
#include "fordyca/events/mech_repair.hpp"

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
    carena::arena_map* map,
    env_dynamics* envd,
    const std::string& entity_prefix,
    const std::string& controller_xml_id,
    rmath::rng* rng)
    : ER_CLIENT_INIT("fordyca.support.tv.argos_pd_adaptor"),
      population_dynamics(config,
                          lf->GetSpace().GetEntitiesByType("foot-bot").size(),
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
  size_t active_pop = swarm_active_population();
  size_t total_pop = swarm_total_population();

  if (0 == total_pop) {
    ER_WARN("Not killing robot: total_pop=%zu active_pop=%zu",
            total_pop,
            active_pop);
    return {rtypes::constants::kNoUUID, total_pop, active_pop};
  }
  controller::foraging_controller* controller = kill_victim_locate(total_pop);

  if (nullptr == controller) {
    ER_WARN("Unable to find kill victim: total_pop=%zu, active_pop=%zu",
            total_pop,
            active_pop);
    return {rtypes::constants::kNoUUID, total_pop, active_pop};
  }
  rtypes::type_uuid id = controller->entity_id();
  ER_INFO("Found kill victim with ID=%d", id.v());

  std::string name = mc_entity_prefix + rcppsw::to_string(id);
  /*
   * If the robot is carrying a block, drop/distribute it in the arena to avoid
   * it getting permanently lost when the it is removed.
   */
  if (controller->is_carrying_block()) {
    ER_INFO("Kill victim robot %s is carrying block%d",
            controller->GetId().c_str(),
            controller->block()->id().v());
    auto it = std::find_if(m_map->blocks().begin(),
                           m_map->blocks().end(),
                           [&](const auto& b) {
                             return controller->block()->id() == b->id();
                           });
    /*
     * We are not REALLY holding all the arena map locks, but since population
     * dynamics are always applied AFTER all robots have had their control steps
     * run, we are in a non-concurrent context, so no reason to grab them.
     */
    caops::free_block_drop_visitor adrop_op(
        *it,
        rmath::dvec2uvec(controller->pos2D(), m_map->grid_resolution().v()),
        m_map->grid_resolution(),
        carena::arena_map_locking::ekALL_HELD);

    adrop_op.visit(*m_map);
  }

  /* remove controller from any applied environmental variances */
  m_envd->unregister_controller(*controller);

  /* remove robot from ARGoS */
  ER_INFO("Remove entity %s", name.c_str());
  m_lf->RemoveEntity(name);

  /* killing a robot reduces both the active and total populations */
  return {id,
        m_lf->GetSpace().GetEntitiesByType("foot-bot").size(),
        m_lf->GetSpace().GetEntitiesByType("foot-bot").size()};
} /* robot_kill() */

argos_pd_adaptor::op_result argos_pd_adaptor::robot_add(int max_pop,
                                                        const rtypes::type_uuid& id) {
  size_t total_pop = swarm_total_population();
  size_t active_pop = swarm_active_population();

  if (max_pop != -1 && total_pop >= static_cast<size_t>(max_pop)) {
    ER_INFO("Not adding new robot %s%d: max_pop=%d reached",
            mc_entity_prefix.c_str(),
            id.v(),
            max_pop);
    return {rtypes::constants::kNoUUID, total_pop, active_pop};
  }

  for (size_t i = 0; i < kMaxOperationAttempts; ++i) {
    if (robot_attempt_add(id)) {
      /* adding a new robot increases both the total and active populations */
      return {id,
            m_lf->GetSpace().GetEntitiesByType("foot-bot").size(),
            m_lf->GetSpace().GetEntitiesByType("foot-bot").size()};
    }
  } /* for(i..) */
  ER_FATAL_SENTINEL("Unable to add new robot to simulation");
  return {rtypes::constants::kNoUUID, total_pop, active_pop};
} /* robot_add() */

argos_pd_adaptor::op_result argos_pd_adaptor::robot_malfunction(void) {
  size_t total_pop = swarm_total_population();
  size_t active_pop = swarm_active_population();

  if (0 == total_pop || 0 == active_pop) {
    ER_WARN("Cannot force robot malfunction: total_pop=%zu, active_pop=%zu",
            total_pop, active_pop);
    return {rtypes::constants::kNoUUID, total_pop, active_pop};
  }

  auto* controller = malfunction_victim_locate(total_pop);

  if (nullptr == controller) {
    ER_WARN("Unable to find malfunction victim: total_pop=%zu, active_pop=%zu",
            total_pop,
            active_pop);
    return {rtypes::constants::kNoUUID, total_pop, active_pop};
  }
  ER_INFO("Found malfunction victim with ID=%d,n_repairing=%zu",
          controller->entity_id().v(),
          repair_queue_status().size);

  events::mech_malfunction_visitor visitor;
  visitor.visit(*controller);

  /*
   * @todo: Once the ability to temporarily remove robots from ARGoS by removing
   * it from physics engines is added, do that here.
   */
  return {controller->entity_id(), total_pop, active_pop - 1};
} /* robot_malfunction() */

argos_pd_adaptor::op_result argos_pd_adaptor::robot_repair(
    const rtypes::type_uuid& id) {

  ER_INFO("Robot with ID=%d repaired, n_repairing=%zu",
          id.v(),
          repair_queue_status().size);
  std::string name = mc_entity_prefix + rcppsw::to_string(id);
  events::mech_repair_visitor visitor;
  auto* entity = dynamic_cast<argos::CFootBotEntity*>(
      &m_lf->GetSpace().GetEntity(name));
  auto* controller = dynamic_cast<controller::foraging_controller*>(
      &entity->GetControllableEntity().GetController());
  visitor.visit(*controller);

  size_t total_pop = swarm_total_population();
  size_t active_pop = swarm_active_population();

  /*
   * @todo: Once the ability to restore temporarily removed robots from
   * simulation is added, do that here.
   */
  return {controller->entity_id(), total_pop, active_pop + 1};
} /* robot_repair() */

controller::foraging_controller* argos_pd_adaptor::malfunction_victim_locate(size_t total_pop) const {
  auto range = rmath::rangei(0, total_pop - 1);

  for (size_t i = 0; i < kMaxOperationAttempts; ++i) {
    auto it = m_lf->GetSpace().GetEntitiesByType("foot-bot").begin();
    std::advance(it, m_rng->uniform(range));
    auto* entity = argos::any_cast<argos::CFootBotEntity*>(it->second);
    auto* controller = dynamic_cast<controller::foraging_controller*>(
        &entity->GetControllableEntity().GetController());
    if (!currently_repairing(controller->entity_id())) {
      return controller;
    }
  } /* for(i..) */
  return nullptr;
} /* malfunction_victim_locate() */

controller::foraging_controller* argos_pd_adaptor::kill_victim_locate(size_t total_pop) const {
    auto range = rmath::rangei(0, total_pop - 1);
  argos::CFootBotEntity* entity;
  for (size_t i = 0; i < kMaxOperationAttempts; ++i) {
    auto it = m_lf->GetSpace().GetEntitiesByType("foot-bot").begin();
    std::advance(it, m_rng->uniform(range));
    entity = argos::any_cast<argos::CFootBotEntity*>(it->second);
    auto* controller = dynamic_cast<controller::foraging_controller*>(
        &entity->GetControllableEntity().GetController());
    if (!already_killed(controller->entity_id())) {
      return controller;
      break;
    }
  } /* for(i..) */
  return nullptr;
} /* kill_victim_locate() */

bool argos_pd_adaptor::robot_attempt_add(const rtypes::type_uuid& id) {
  /*
   * Give 2.0 buffer around the edges of the arena so that robots are not too
   * close to the boundaries of physics engines, which can cause "no engine can
   * house entity" exceptions in rare cases otherwise.
   */
  rmath::ranged xrange(2.0, mc_lf->arena_map()->xrsize() - 2.0);
  rmath::ranged yrange(2.0, mc_lf->arena_map()->yrsize() - 2.0);
  argos::CFootBotEntity* fb = nullptr;

  auto x = m_rng->uniform(xrange);
  auto y = m_rng->uniform(yrange);

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
  try {
    /* ick raw pointers--thanks ARGoS... */
    fb = new argos::CFootBotEntity(mc_entity_prefix + rcppsw::to_string(id),
                                   mc_controller_xml_id,
                                   argos::CVector3(x, y, 0.0));
    m_lf->AddEntity(*fb);
    ER_INFO(
        "Added entity %s attached to physics engine %s at %s",
        fb->GetId().c_str(),
        fb->GetEmbodiedEntity().GetPhysicsModel(0).GetEngine().GetId().c_str(),
        rmath::vector2d(x, y).to_str().c_str());

    /* Register controller for environmental variances */
    auto* controller = dynamic_cast<controller::foraging_controller*>(
        &fb->GetControllableEntity().GetController());

    m_envd->register_controller(*controller);

    return true;
  } catch (argos::CARGoSException& e) {
    if (nullptr != fb) {
      delete fb; /* ick raw pointers--thanks ARGoS... */
    }
    ER_TRACE("Failed to place new robot %s at %s",
             fb->GetId().c_str(),
             rmath::vector2d(x, y).to_str().c_str());
    return false;
  }
} /* robot_attempt_add() */

NS_END(tv, support, fordyca);
