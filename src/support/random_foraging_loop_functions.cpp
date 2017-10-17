/**
 * @file random_foraging_loop_functions.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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
#include <limits>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include "fordyca/support/random_foraging_loop_functions.hpp"
#include "fordyca/controller/random_foraging_controller.hpp"
#include "fordyca/events/block_drop.hpp"
#include "fordyca/events/block_pickup.hpp"
#include "fordyca/params/loop_functions_params.hpp"
#include "fordyca/params/logging_params.hpp"
#include "fordyca/params/arena_map_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
random_foraging_loop_functions::random_foraging_loop_functions(void) :
    er_client(rcppsw::common::g_server),
    m_nest_x(),
    m_nest_y(),
    m_floor(NULL),
    m_sim_type(),
    m_repo(new params::loop_function_repository),
    m_robot_collector(),
    m_block_collector(),
    m_map() {
  insmod("loop_functions",
         rcppsw::common::er_lvl::DIAG,
         rcppsw::common::er_lvl::NOM);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void random_foraging_loop_functions::Init(argos::TConfigurationNode& node) {
  rcppsw::common::g_server->change_logfile("loop-functions.txt");
  rcppsw::common::g_server->dbglvl(rcppsw::common::er_lvl::NOM);
  rcppsw::common::g_server->loglvl(rcppsw::common::er_lvl::DIAG);
  ER_NOM("Initializing random foraging loop functions");

  m_floor = &GetSpace().GetFloorEntity();

  /* parse all environment parameters */
  m_repo->parse_all(node);

  /* Capture parsed parameters in logfile */
  m_repo->show_all(rcppsw::common::g_server->log_stream());

  const struct params::loop_functions_params * l_params =
      static_cast<const struct params::loop_functions_params*>(
      m_repo->get_params("loop_functions"));
  m_nest_x = l_params->nest_x;
  m_nest_y = l_params->nest_y;
  m_sim_type = l_params->simulation_type;

  /* initialize arena map and distribute blocks */
  const struct params::arena_map_params * arena_params =
      static_cast<const struct params::arena_map_params*>(
          m_repo->get_params("arena_map"));
  m_map.reset(new representation::arena_map(arena_params, m_nest_x, m_nest_y));
  m_map->distribute_blocks(true);
  for (size_t i = 0; i < m_map->blocks().size(); ++i) {
    m_map->blocks()[i].display_id(l_params->display_block_id);
  } /* for(i..) */

  /* initialize stat collecting */
  m_robot_collector.reset(new robot_stat_collector(
      static_cast<const struct params::logging_params*>(
          m_repo->get_params("logging"))->robot_stats));
  m_block_collector.reset(new block_stat_collector(
      static_cast<const struct params::logging_params*>(
          m_repo->get_params("logging"))->block_stats));
  m_block_collector->reset();
  m_robot_collector->reset();

  /* configure robots */
  argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
  for (argos::CSpace::TMapPerType::iterator it = footbots.begin();
       it != footbots.end();
       ++it) {
    argos::CFootBotEntity& robot = *argos::any_cast<argos::CFootBotEntity*>(
        it->second);
    controller::random_foraging_controller& controller =
        dynamic_cast<controller::random_foraging_controller&>(
            robot.GetControllableEntity().GetController());
    controller.display_id(l_params->display_robot_id);
  } /* for(it..) */
  ER_NOM("Random foraging loop functions initialization finished");
}

void random_foraging_loop_functions::Reset() {
  m_block_collector->reset();
  m_robot_collector->reset();
  m_map->distribute_blocks(true);
}

void random_foraging_loop_functions::Destroy() {
  m_block_collector->finalize();
  m_robot_collector->finalize();
}

argos::CColor random_foraging_loop_functions::GetFloorColor(
    const argos::CVector2& plane_pos) {

  /* The nest is a light gray */
  if (m_nest_x.WithinMinBoundIncludedMaxBoundIncluded(plane_pos.GetX()) &&
      m_nest_y.WithinMinBoundIncludedMaxBoundIncluded(plane_pos.GetY())) {
    return argos::CColor::GRAY30;
  }
  /* blocks are black */
  for (size_t i = 0; i < m_map->blocks().size(); ++i) {
    if (m_map->blocks()[i].contains_point(plane_pos)) {
      return argos::CColor::BLACK;
    }
  } /* for(i..) */

  for (size_t i = 0; i < m_map->blocks().size(); ++i) {
    if (m_map->blocks()[i].contains_point(plane_pos)) {
      return argos::CColor::GRAY70;
    }
  } /* for(i..) */
  return argos::CColor::WHITE;
} /* GetFloorColor() */

void random_foraging_loop_functions::pre_step_iter(argos::CFootBotEntity& robot) {
    controller::random_foraging_controller& controller =
        dynamic_cast<controller::random_foraging_controller&>(
        robot.GetControllableEntity().GetController());

    /* get stats from this robot before its state changes */
    m_robot_collector->collect(controller);

    if (controller.is_carrying_block()) {
      if (controller.in_nest()) {
        /*
         * Get stats from carried block before it's dropped and its state
         * changes.
         */
        m_block_collector->collect(*controller.block());

        /* Update arena map state due to a block nest drop */
        events::block_drop drop_op(rcppsw::common::g_server,
                                       controller.block());
        m_map->accept(drop_op);

        /* Actually drop the block */
        controller.accept(drop_op);

        /* The floor texture must be updated */
        m_floor->SetChanged();
      }
    } else { /* The foot-bot has no block item */
      if (!controller.in_nest() && controller.is_exploring() &&
          controller.block_detected()) {
        /* Check whether the foot-bot is actually on a block */
        int block = robot_on_block(robot);
        if (-1 != block) {
          events::block_pickup pickup_op(rcppsw::common::g_server,
                                             &m_map->blocks()[block],
                                             robot_id(robot));
          controller.accept(pickup_op);
          m_map->accept(pickup_op);

          /* The floor texture must be updated */
          m_floor->SetChanged();
        }
      }
    }
} /* pre_step_iter() */

void random_foraging_loop_functions::pre_step_final(void) {
  m_block_collector->csv_line_write(GetSpace().GetSimulationClock());
  m_robot_collector->csv_line_write(GetSpace().GetSimulationClock());
  m_robot_collector->reset_on_timestep();
} /* pre_step_final() */

void random_foraging_loop_functions::PreStep() {
  argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");

  for (argos::CSpace::TMapPerType::iterator it = footbots.begin();
       it != footbots.end();
       ++it) {
    argos::CFootBotEntity& robot = *argos::any_cast<argos::CFootBotEntity*>(
        it->second);
    pre_step_iter(robot);
  } /* for(it..) */
  pre_step_final();
} /* PreStep() */

int random_foraging_loop_functions::robot_id(const argos::CFootBotEntity& robot) {
  /* +2 because the ID string starts with 'fb' */
  return std::atoi(robot.GetId().c_str()+2);
} /* robot_id() */

int random_foraging_loop_functions::robot_on_block(const argos::CFootBotEntity& robot) {
  argos::CVector2 pos;
  pos.Set(const_cast<argos::CFootBotEntity&>(robot).GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
          const_cast<argos::CFootBotEntity&>(robot).GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
  return m_map->robot_on_block(pos);
} /* robot_on_block() */

bool random_foraging_loop_functions::IsExperimentFinished(void) {
  /*
   * If we are not respawning blocks and all blocks have been collected, signal
   * the end of the experiment. If respawn is enabled, then the experiment will
   * run until I cancel it.
   */
  if (!m_map->respawn_enabled() &&
      m_block_collector->total_collected() == m_map->n_blocks()) {
    return true;
  }
  return false;
} /* IsExperimentFinished() */

void random_foraging_loop_functions::PostExperiment(void) {
  if (m_sim_type == "scripted") {
    std::exit(0);
  }
} /* PostExperiment() */

using namespace argos;
REGISTER_LOOP_FUNCTIONS(random_foraging_loop_functions, "random_foraging_loop_functions")

NS_END(support, fordyca);
