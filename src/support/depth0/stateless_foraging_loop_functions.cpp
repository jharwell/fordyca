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
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <experimental/filesystem>

#include "fordyca/controller/depth0/stateless_foraging_controller.hpp"
#include "fordyca/events/free_block_pickup.hpp"
#include "fordyca/events/nest_block_drop.hpp"
#include "fordyca/metrics/collectors/block_metrics_collector.hpp"
#include "fordyca/metrics/collectors/fsm/distance_metrics_collector.hpp"
#include "fordyca/metrics/collectors/fsm/stateless_metrics_collector.hpp"
#include "fordyca/params/arena_map_params.hpp"
#include "fordyca/params/loop_function_repository.hpp"
#include "fordyca/params/loop_functions_params.hpp"
#include "fordyca/params/output_params.hpp"
#include "fordyca/representation/cell2D.hpp"
#include "rcppsw/er/server.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth0);
namespace fs = std::experimental::filesystem;
namespace rmetrics = metrics::collectible_metrics::fsm;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
stateless_foraging_loop_functions::stateless_foraging_loop_functions(void)
    : client(rcppsw::er::g_server),
      m_nest_x(),
      m_nest_y(),
      m_output_root(),
      m_metrics_path(),
      m_stateless_collector(),
      m_distance_collector(),
      m_block_collector(),
      m_map() {
  insmod("loop_functions", rcppsw::er::er_lvl::DIAG, rcppsw::er::er_lvl::NOM);
}

stateless_foraging_loop_functions::~stateless_foraging_loop_functions(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void stateless_foraging_loop_functions::Init(argos::TConfigurationNode &node) {
  base_foraging_loop_functions::Init(node);

  rcppsw::er::g_server->dbglvl(rcppsw::er::er_lvl::NOM);
  rcppsw::er::g_server->loglvl(rcppsw::er::er_lvl::DIAG);
  ER_NOM("Initializing stateless foraging loop functions");

  /* parse all environment parameters and capture in logfile */
  params::loop_function_repository repo;
  repo.parse_all(node);

  auto *p_output = static_cast<const struct params::output_params *>(
      repo.get_params("output"));
  auto *p_arena = static_cast<const struct params::arena_map_params *>(
      repo.get_params("arena_map"));

  output_init(p_output);

  rcppsw::er::g_server->change_logfile(m_output_root + "/" +
                                       p_output->sim_log_fname);
  repo.show_all(rcppsw::er::g_server->log_stream());

  /* setup logging timestamp calculator */
  rcppsw::er::g_server->log_ts_calculator(
      std::bind(&stateless_foraging_loop_functions::log_timestamp_calc, this));

  auto *l_params = static_cast<const struct params::loop_functions_params *>(
      repo.get_params("loop_functions"));
  m_nest_x = p_arena->nest_x;
  m_nest_y = p_arena->nest_y;

  /* initialize arena map and distribute blocks */
  arena_map_init(repo);

  /* initialize metric collecting */
  metric_collecting_init(p_output);

  /* configure robots */
  for (auto &entity_pair : GetSpace().GetEntitiesByType("foot-bot")) {
    argos::CFootBotEntity &robot =
        *argos::any_cast<argos::CFootBotEntity *>(entity_pair.second);
    auto &controller = static_cast<controller::base_foraging_controller &>(
        robot.GetControllableEntity().GetController());
    controller.display_id(l_params->display_robot_id);
  } /* for(&robot..) */
  ER_NOM("Stateless foraging loop functions initialization finished");
}

void stateless_foraging_loop_functions::Reset() {
  m_block_collector->reset();
  m_stateless_collector->reset();
  m_distance_collector->reset();
  m_map->distribute_blocks();
}

void stateless_foraging_loop_functions::Destroy() {
  m_block_collector->finalize();
  m_stateless_collector->finalize();
  m_distance_collector->finalize();
}

argos::CColor stateless_foraging_loop_functions::GetFloorColor(
    const argos::CVector2 &plane_pos) {

  /* The nest is a light gray */
  if (m_nest_x.WithinMinBoundIncludedMaxBoundIncluded(plane_pos.GetX()) &&
      m_nest_y.WithinMinBoundIncludedMaxBoundIncluded(plane_pos.GetY())) {
    return argos::CColor::GRAY70;
  }

  for (auto &block : map()->blocks()) {
    if (block.contains_point(plane_pos)) {
      return block.color();
    }
  } /* for(&block..) */

  return argos::CColor::WHITE;
} /* GetFloorColor() */

void stateless_foraging_loop_functions::pre_step_iter(
    argos::CFootBotEntity &robot) {
  auto &controller =
      static_cast<controller::depth0::stateless_foraging_controller &>(
          robot.GetControllableEntity().GetController());

  /* get stats from this robot before its state changes */
  m_stateless_collector->collect(
      static_cast<rmetrics::stateless_metrics &>(controller));
  m_distance_collector->collect(
      static_cast<rmetrics::distance_metrics &>(controller));

  set_robot_tick<controller::depth0::stateless_foraging_controller>(robot);

  if (!handle_nest_block_drop<controller::depth0::stateless_foraging_controller>(
          robot, *m_map, *m_block_collector)) {
    if (!controller.in_nest() && controller.is_exploring_for_block() &&
        controller.block_detected()) {
      /* Check whether the foot-bot is actually on a block */
      int block = utils::robot_on_block(robot, *map());
      if (-1 != block) {
        events::free_block_pickup pickup_op(rcppsw::er::g_server,
                                            &m_map->blocks()[block],
                                            utils::robot_id(robot));
        controller.accept(pickup_op);
        m_map->accept(pickup_op);

        /* The floor texture must be updated */
        floor()->SetChanged();
      }
    }
  }
} /* pre_step_iter() */

void stateless_foraging_loop_functions::pre_step_final(void) {
  m_block_collector->csv_line_write(GetSpace().GetSimulationClock());
  m_stateless_collector->csv_line_write(GetSpace().GetSimulationClock());
  m_distance_collector->csv_line_write(GetSpace().GetSimulationClock());

  m_stateless_collector->timestep_reset();
  m_stateless_collector->interval_reset();
  m_stateless_collector->timestep_inc();

  m_block_collector->timestep_reset();
  m_block_collector->interval_reset();
  m_block_collector->timestep_inc();
} /* pre_step_final() */

void stateless_foraging_loop_functions::PreStep() {
  for (auto &entity_pair : GetSpace().GetEntitiesByType("foot-bot")) {
    argos::CFootBotEntity &robot =
        *argos::any_cast<argos::CFootBotEntity *>(entity_pair.second);
    pre_step_iter(robot);
  } /* for(&entity..) */
  pre_step_final();
} /* PreStep() */

void stateless_foraging_loop_functions::metric_collecting_init(
    const struct params::output_params *p_output) {
  m_metrics_path = m_output_root + "/" + p_output->metrics.output_dir;
  if (fs::exists(m_metrics_path)) {
    fs::remove_all(m_metrics_path);
  }
  fs::create_directories(m_metrics_path);

  m_stateless_collector = rcppsw::make_unique<robot_collectors::stateless_metrics_collector>(
      m_metrics_path + "/" + p_output->metrics.stateless_fname,
      p_output->metrics.collect_cum,
      p_output->metrics.collect_interval);

  m_block_collector = rcppsw::make_unique<collectors::block_metrics_collector>(
      m_metrics_path + "/" + p_output->metrics.block_fname,
      p_output->metrics.collect_interval);

  m_distance_collector = rcppsw::make_unique<robot_collectors::distance_metrics_collector>(
      m_metrics_path + "/" + p_output->metrics.distance_fname,
      p_output->metrics.n_robots);

  m_stateless_collector->reset();
  m_distance_collector->reset();
  m_block_collector->reset();
} /* metric_collecting_init() */

void stateless_foraging_loop_functions::arena_map_init(
    params::loop_function_repository &repo) {
  auto *arena_params = static_cast<const struct params::arena_map_params *>(
      repo.get_params("arena_map"));
  auto *l_params = static_cast<const struct params::loop_functions_params *>(
      repo.get_params("loop_functions"));

  m_map.reset(new representation::arena_map(arena_params));
  m_map->distribute_blocks();
  for (auto &block : m_map->blocks()) {
    block.display_id(l_params->display_block_id);
  } /* for(&block..) */
} /* arena_map_init() */

void stateless_foraging_loop_functions::output_init(
    const struct params::output_params *params) {
  if ("__current_date__" == params->output_dir) {
    boost::posix_time::ptime now =
        boost::posix_time::second_clock::local_time();
    m_output_root = params->output_root + "/" +
                    std::to_string(now.date().year()) + "-" +
                    std::to_string(now.date().month()) + "-" +
                    std::to_string(now.date().day()) + ":" +
                    std::to_string(now.time_of_day().hours()) + "-" +
                    std::to_string(now.time_of_day().minutes());
  } else {
    m_output_root = params->output_root + "/" + params->output_dir;
  }
} /* output_init() */

__pure collectors::block_metrics_collector *
stateless_foraging_loop_functions::block_collector(void) const {
  return m_block_collector.get();
} /* block_collector() */

__pure robot_collectors::distance_metrics_collector *
stateless_foraging_loop_functions::distance_collector(void) const {
  return m_distance_collector.get();
} /* distance_collector() */

__pure robot_collectors::stateless_metrics_collector *
stateless_foraging_loop_functions::stateless_collector(void) const {
  return m_stateless_collector.get();
} /* stateless_collector() */

std::string stateless_foraging_loop_functions::log_timestamp_calc(void) {
  return "[t=" + std::to_string(GetSpace().GetSimulationClock()) + "]";
} /* log_timestamp_calc() */

using namespace argos;
REGISTER_LOOP_FUNCTIONS(stateless_foraging_loop_functions,
                        "stateless_foraging_loop_functions")

NS_END(depth0, support, fordyca);
