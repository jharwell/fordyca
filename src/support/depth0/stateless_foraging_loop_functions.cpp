/**
 * @file stateless_foraging_loop_functions.cpp
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
#include "fordyca/support/depth0/stateless_foraging_loop_functions.hpp"
#include <argos3/core/simulator/simulator.h>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "fordyca/controller/depth0/stateless_foraging_controller.hpp"
#include "fordyca/params/arena/arena_map_params.hpp"
#include "fordyca/params/loop_function_repository.hpp"
#include "fordyca/params/output_params.hpp"
#include "fordyca/params/visualization_params.hpp"
#include "fordyca/representation/arena_map.hpp"
#include "fordyca/support/depth0/stateless_metrics_aggregator.hpp"
#include "rcppsw/er/server.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
stateless_foraging_loop_functions::stateless_foraging_loop_functions(void)
    : client(rcppsw::er::g_server), m_arena_map(nullptr) {
  insmod("loop_functions", rcppsw::er::er_lvl::DIAG, rcppsw::er::er_lvl::NOM);
}

stateless_foraging_loop_functions::~stateless_foraging_loop_functions(void) =
    default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void stateless_foraging_loop_functions::Init(ticpp::Element& node) {
  base_foraging_loop_functions::Init(node);

  rcppsw::er::g_server->dbglvl(rcppsw::er::er_lvl::NOM);
  rcppsw::er::g_server->loglvl(rcppsw::er::er_lvl::DIAG);
  ER_NOM("Initializing stateless foraging loop functions");

  /* parse all environment parameters and capture in logfile */
  params::loop_function_repository repo(server_ref());
  repo.parse_all(node);

  /* initialize output and metrics collection */
  auto* p_output = repo.parse_results<params::output_params>();
  output_init(p_output);

  rcppsw::er::g_server->change_logfile(m_output_root + "/" +
                                       p_output->log_fname);
  rcppsw::er::g_server->log_stream() << repo;

  /* setup logging timestamp calculator */
  rcppsw::er::g_server->log_ts_calculator(
      std::bind(&stateless_foraging_loop_functions::log_timestamp_calc, this));

  /* initialize arena map and distribute blocks */
  arena_map_init(repo);

  auto* arenap = repo.parse_results<params::arena::arena_map_params>();
  m_interactor =
      rcppsw::make_unique<interactor>(rcppsw::er::g_server,
                                      arena_map(),
                                      m_metrics_agg.get(),
                                      floor(),
                                      &arenap->blocks.manipulation_penalty);

  /* configure robots */
  for (auto& entity_pair : GetSpace().GetEntitiesByType("foot-bot")) {
    argos::CFootBotEntity& robot =
        *argos::any_cast<argos::CFootBotEntity*>(entity_pair.second);
    auto& controller = static_cast<controller::base_foraging_controller&>(
        robot.GetControllableEntity().GetController());

    /*
     * If NULL, then visualization has been disabled.
     */
    auto* vparams = repo.parse_results<struct params::visualization_params>();
    if (nullptr != vparams) {
      controller.display_id(vparams->robot_id);
    }
  } /* for(&robot..) */
  ER_NOM("Stateless foraging loop functions initialization finished");
}

void stateless_foraging_loop_functions::Reset() {
  m_metrics_agg->reset_all();
  m_arena_map->distribute_all_blocks();
}

void stateless_foraging_loop_functions::Destroy() {
  m_metrics_agg->finalize_all();
}

__rcsw_pure argos::CColor stateless_foraging_loop_functions::GetFloorColor(
    const argos::CVector2& plane_pos) {
  if (m_arena_map->nest().contains_point(plane_pos)) {
    return argos::CColor(m_arena_map->nest().color().red(),
                         m_arena_map->nest().color().green(),
                         m_arena_map->nest().color().blue());
  }

  for (auto& block : m_arena_map->blocks()) {
    /*
     * Even though each block type has a unique color, the only distinction
     * that robots can make to determine if they are on a block or not is
     * between shades of black/white. So, all blocks must appear as black, even
     * when they are not actually (when blocks are picked up their correct color
     * is shown through visualization).
     */
    if (block->contains_point(plane_pos)) {
      return argos::CColor::BLACK;
    }
  } /* for(&block..) */

  return argos::CColor::WHITE;
} /* GetFloorColor() */

void stateless_foraging_loop_functions::pre_step_iter(
    argos::CFootBotEntity& robot) {
  auto& controller =
      static_cast<controller::depth0::stateless_foraging_controller&>(
          robot.GetControllableEntity().GetController());

  /* get stats from this robot before its state changes */
  m_metrics_agg->collect_from_controller(
      static_cast<rcppsw::metrics::base_metrics*>(&controller));
  controller.free_pickup_event(false);
  controller.free_drop_event(false);

  /* Send the robot its current position */
  set_robot_tick<controller::depth0::stateless_foraging_controller>(robot);
  utils::set_robot_pos<controller::depth0::stateless_foraging_controller>(robot);

  /* Now watch it react to the environment */
  (*m_interactor)(controller, GetSpace().GetSimulationClock());
} /* pre_step_iter() */

void stateless_foraging_loop_functions::pre_step_final(void) {
  m_metrics_agg->metrics_write_all(GetSpace().GetSimulationClock());
  m_metrics_agg->timestep_reset_all();
  m_metrics_agg->interval_reset_all();
  m_metrics_agg->timestep_inc_all();
} /* pre_step_final() */

void stateless_foraging_loop_functions::PreStep() {
  for (auto& entity_pair : GetSpace().GetEntitiesByType("foot-bot")) {
    argos::CFootBotEntity& robot =
        *argos::any_cast<argos::CFootBotEntity*>(entity_pair.second);
    pre_step_iter(robot);
  } /* for(&entity..) */
  pre_step_final();
} /* PreStep() */

void stateless_foraging_loop_functions::arena_map_init(
    params::loop_function_repository& repo) {
  auto* aparams = repo.parse_results<struct params::arena::arena_map_params>();
  auto* vparams = repo.parse_results<struct params::visualization_params>();

  m_arena_map.reset(new representation::arena_map(aparams));
  if (!m_arena_map->initialize()) {
    ER_ERR("FATAL: Could not initialize arena map");
    std::exit(EXIT_FAILURE);
  }
  m_arena_map->distribute_all_blocks();

  /*
   * If null, visualization has been disabled.
   */
  if (nullptr != vparams) {
    for (auto& block : m_arena_map->blocks()) {
      block->display_id(vparams->block_id);
    } /* for(&block..) */
  }
} /* arena_map_init() */

void stateless_foraging_loop_functions::output_init(
    const struct params::output_params* params) {
  if ("__current_date__" == params->output_dir) {
    boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
    m_output_root = params->output_root + "/" +
                    std::to_string(now.date().year()) + "-" +
                    std::to_string(now.date().month()) + "-" +
                    std::to_string(now.date().day()) + ":" +
                    std::to_string(now.time_of_day().hours()) + "-" +
                    std::to_string(now.time_of_day().minutes());
  } else {
    m_output_root = params->output_root + "/" + params->output_dir;
  }
  m_metrics_agg = rcppsw::make_unique<stateless_metrics_aggregator>(
      rcppsw::er::g_server, &params->metrics, m_output_root);
  m_metrics_agg->reset_all();
} /* output_init() */

std::string stateless_foraging_loop_functions::log_timestamp_calc(void) {
  return "[t=" + std::to_string(GetSpace().GetSimulationClock()) + "]";
} /* log_timestamp_calc() */

using namespace argos;
REGISTER_LOOP_FUNCTIONS(stateless_foraging_loop_functions,
                        "stateless_foraging_loop_functions"); // NOLINT

NS_END(depth0, support, fordyca);
