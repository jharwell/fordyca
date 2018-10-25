/**
 * @file stateful_foraging_controller.cpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/controller/depth0/stateful_foraging_controller.hpp"
#include <fstream>

#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/base_perception_subsystem.hpp"
#include "fordyca/controller/block_selection_matrix.hpp"
#include "fordyca/controller/depth0/sensing_subsystem.hpp"
#include "fordyca/controller/depth0/stateful_tasking_initializer.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/params/depth0/stateful_controller_repository.hpp"
#include "fordyca/params/sensing_params.hpp"

#include "rcppsw/task_allocation/bifurcating_tdgraph_executive.hpp"
#include "rcppsw/task_allocation/executive_params.hpp"
#include "rcppsw/task_allocation/task_params.hpp"

#include "fordyca/representation/base_cell_entity.hpp"
#include "fordyca/representation/ramp_block.hpp"
#include "fordyca/representation/cube_block.hpp"
#include "fordyca/representation/base_cache.hpp"
#include "fordyca/metrics/blocks/transport_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth0);
namespace ta = rcppsw::task_allocation;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
stateful_foraging_controller::stateful_foraging_controller(void)
    : stateless_foraging_controller(),
      ER_CLIENT_INIT("fordyca.controller.stateful"),
      m_light_loc(),
      m_block_sel_matrix(),
      m_perception(),
      m_executive(),
      m_communication_params(),
      m_arena_x(0),
      m_arena_y(0) {}

stateful_foraging_controller::~stateful_foraging_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
__rcsw_pure const ta::bifurcating_tab* stateful_foraging_controller::active_tab(
    void) const {
  return m_executive->active_tab();
}

void stateful_foraging_controller::block_sel_matrix(
    std::unique_ptr<block_selection_matrix> m) {
  m_block_sel_matrix = std::move(m);
}
void stateful_foraging_controller::executive(
    std::unique_ptr<ta::bifurcating_tdgraph_executive> executive) {
  m_executive = std::move(executive);
}

void stateful_foraging_controller::perception(
    std::unique_ptr<base_perception_subsystem> perception) {
  m_perception = std::move(perception);
}
__rcsw_pure tasks::base_foraging_task* stateful_foraging_controller::current_task(
    void) {
  return dynamic_cast<tasks::base_foraging_task*>(
      m_executive.get()->current_task());
} /* current_task() */

__rcsw_pure const tasks::base_foraging_task* stateful_foraging_controller::
    current_task(void) const {
  return const_cast<stateful_foraging_controller*>(this)->current_task();
} /* current_task() */

__rcsw_pure const representation::line_of_sight* stateful_foraging_controller::los(
    void) const {
  return stateful_sensors()->los();
}
void stateful_foraging_controller::los(
    std::unique_ptr<representation::line_of_sight>& new_los) {
  stateful_sensors()->los(new_los);
}

__rcsw_pure const depth0::sensing_subsystem* stateful_foraging_controller::
    stateful_sensors(void) const {
  return static_cast<const depth0::sensing_subsystem*>(
      saa_subsystem()->sensing().get());
}

__rcsw_pure depth0::sensing_subsystem* stateful_foraging_controller::stateful_sensors(
    void) {
  return static_cast<depth0::sensing_subsystem*>(
      saa_subsystem()->sensing().get());
}

void stateful_foraging_controller::ControlStep(void) {
  ndc_pusht();
  /*
   * Update the robot's model of the world with the current line-of-sight, and
   * update the relevance of information within it. Then, you can run the main
   * FSM loop.
   */
  m_perception->update(stateful_sensors()->los());

  saa_subsystem()->actuation()->block_carry_throttle(is_carrying_block());
  saa_subsystem()->actuation()->throttling_update(stateful_sensors()->tick());

  if(m_communication_params.mode > 0) {
    perform_communication();
  }

  m_executive->run();
  ndc_pop();
} /* ControlStep() */

void stateful_foraging_controller::Init(ticpp::Element& node) {
  /*
   * Note that we do not call \ref stateless_foraging_controller::Init()--there
   * is nothing in there that we need.
   */
  base_foraging_controller::Init(node);

  ndc_push();
  ER_INFO("Initializing...");

  /* parse and validate parameters */
  params::depth0::stateful_controller_repository param_repo;
  param_repo.parse_all(node);

  if (!param_repo.validate_all()) {
    ER_FATAL_SENTINEL("Not all parameters were validated");
    std::exit(EXIT_FAILURE);
  }

  /* initialize subsystems and perception */
  m_perception = rcppsw::make_unique<base_perception_subsystem>(
      param_repo.parse_results<params::perception_params>(), GetId());

  saa_subsystem()->sensing(std::make_shared<depth0::sensing_subsystem>(
      param_repo.parse_results<struct params::sensing_params>(),
      &saa_subsystem()->sensing()->sensor_list()));

  auto* ogrid = param_repo.parse_results<params::occupancy_grid_params>();
  m_block_sel_matrix =
      rcppsw::make_unique<block_selection_matrix>(ogrid->nest,
                                                  &ogrid->priorities);

  auto* comm_params = param_repo.parse_results<params::communication_params>();
  m_communication_params = *comm_params;

  /* initialize tasking */
  m_executive = stateful_tasking_initializer(m_block_sel_matrix.get(),
                                             saa_subsystem(),
                                             perception())(&param_repo);

  ER_INFO("Initialization finished");
  ndc_pop();
} /* Init() */

void stateful_foraging_controller::Reset(void) {
  stateless_foraging_controller::Reset();
  m_perception->reset();
} /* Reset() */

void stateful_foraging_controller::perform_communication(void) {
  m_arena_x = perception()->map()->grid().xdsize();
  m_arena_y = perception()->map()->grid().ydsize();
  std::vector<uint8_t> recieved_packet_data = saa_subsystem()->sensing()->recieve_message();
  float probability = static_cast <float> (rand()) /
      static_cast <float> (RAND_MAX);

  hal::wifi_packet packet = hal::wifi_packet();
  if (!recieved_packet_data.empty() && probability >=
      (1 - m_communication_params.chance_to_pass_on)) {
    packet.data = recieved_packet_data;
    integrate_recieved_packet(packet);
    saa_subsystem()->actuation()->start_sending_message(packet);
  } else if (probability >= (1 - m_communication_params.chance_to_start)) {
    int x_coord;
    int y_coord;

    // Random Mode
    if(m_communication_params.mode == 1) {
      x_coord = static_cast <int> (rand()) % m_arena_x;
      y_coord = static_cast <int> (rand()) % m_arena_y;
    // Utility function
    } else if (m_communication_params.mode == 2) {
      argos::CVector2 cell = get_most_valuable_cell();
      x_coord = cell.GetX();
      y_coord = cell.GetY();
    } else {
      x_coord = 2;
      y_coord = 2;
    }

    ds::cell2D cell = m_perception->map()->
      access<ds::occupancy_grid::kCell>(x_coord, y_coord);
    packet.data.push_back(static_cast<uint8_t>(x_coord)); // X Coord of cell
    packet.data.push_back(static_cast<uint8_t>(y_coord)); // Y Coord of cell

    // The state is what the cell contains (nothing, block, or cache)
    int state = 1;
    if (cell.state_is_empty()) {
      state = 2;
    } else if (cell.state_has_block()) {
      state = 3;
    } else if (cell.state_has_cache()) {
      state = 4;
    }
    // Type of entity (block / cache) (will be 1 if the cell state is unknown)
    packet.data.push_back(static_cast<uint8_t>(state));

    auto entity = cell.entity();
    int id = 0;

    // Type is specific to blocks as there are ramp and cube blocks.
    int type = 1;
    if (entity) {
      id = entity->id();

      // Block
      if (state == 3) {
        // Ramp block
        if (cell.block()->type() == metrics::blocks::transport_metrics::kRamp) {
          type = 2;
        // Cube block
        } else {
          type = 3;
        } /* if block type */
      } /* if state */
    } /* if entity */
    packet.data.push_back(static_cast<uint8_t>(id)); // Entity ID (will be 0 if the cell is unknown)
    packet.data.push_back(static_cast<uint8_t>(type)); // Type of block

    rcppsw::swarm::pheromone_density& density = m_perception->map()->
      access<ds::occupancy_grid::kPheromone>(x_coord, y_coord);
    packet.data.push_back(static_cast<uint8_t>(density.last_result()));

    saa_subsystem()->actuation()->start_sending_message(packet);
  } else {
    saa_subsystem()->actuation()->stop_sending_message();
  } /* if !recieved_packet_data.empty() */
} /* perform_communication */

void stateful_foraging_controller::integrate_recieved_packet(hal::wifi_packet packet) {
  // Data extraction
  int x_coord = static_cast<int>(packet.data[0]);
  int y_coord = static_cast<int>(packet.data[1]);
  int state = static_cast<int>(packet.data[2]);
  int ent_id = static_cast<int>(packet.data[3]);
  // type of block (if it's not a block it will be -1)
  int type = static_cast<int>(packet.data[4]);
  int pheromone_density = static_cast<int>(packet.data[5]);

  if (state != 1) {
    rcppsw::swarm::pheromone_density& density = m_perception->map()->
      access<ds::occupancy_grid::kPheromone>(x_coord, y_coord);

      // If the recieved pheromone density is less than the known, don't
      // integrate the recieved information.
      if (density.last_result() > pheromone_density) {
        return;
      }

    density.pheromone_set(static_cast<double>(pheromone_density));

    // blocks
    if (state == 3) {
      // ramp block
      if (type == 2) {
        std::shared_ptr<representation::ramp_block> block_ptr (new
          representation::ramp_block(rcppsw::math::vector2d(x_coord, y_coord),
          ent_id));

        m_perception->map()->block_add(block_ptr);
      // cube block
    } else if (type == 3) {
        std::shared_ptr<representation::cube_block> block_ptr (new
          representation::cube_block(rcppsw::math::vector2d(x_coord, y_coord),
          ent_id));

        m_perception->map()->block_add(block_ptr);
      } /* if type */
    // caches
    } else {
      // TODO: Add caches to percieved_arena_map

    } /* if (state == 1) / else */
  } /* if (state != -1) */
} /* integrate_recieved_packet */

argos::CVector2 stateful_foraging_controller::get_most_valuable_cell(void) {
  argos::CVector2 cell_coords;

  // TODO: Get information on where the nest is!!!!
  int nest_x_coord = 2;
  int nest_y_coord = 3;

  int communicated_cell_value = 99999;
  // int communicated_cell_value = 0;
  int current_cell_value = 0;

  int arena_x_coord = static_cast<int>(m_arena_x);
  int arena_y_coord = static_cast<int>(m_arena_y);
  int cell_x = 0;
  int cell_y = 0;

  for (int i = 0; i < arena_x_coord; ++i) {
    for (int j = 0; j < arena_y_coord; ++j) {
      ds::cell2D current_cell = m_perception->map()->
        access<ds::occupancy_grid::kCell>(i, j);
      if (current_cell.state_has_block()) {
        current_cell_value = ((((i - nest_x_coord)^2) +
          ((j - nest_y_coord)^2))^(1/2)) / current_cell.block_count();
        if (current_cell_value < communicated_cell_value) {
          communicated_cell_value = current_cell_value;
          cell_x = i;
          cell_y = j;
        } /* if value */
      } /* if cell has block */
    } /* for(j..) */
  } /* for(i..) */
  if (cell_x != 0 || cell_y != 0) {
    std::cout << "Arena x: " << arena_x_coord << " and y: " << arena_y_coord << std::endl;
    std::cout << "Most Valuable X coord: " << cell_x << " and the most valuable y: " << cell_y << std::endl;
  }
  cell_coords.SetX(cell_x);
  cell_coords.SetY(cell_y);
  return cell_coords;
} /* get_most_valuable_cell */

FSM_WRAPPER_DEFINE_PTR(transport_goal_type,
                       stateful_foraging_controller,
                       block_transport_goal,
                       current_task());

FSM_WRAPPER_DEFINE_PTR(acquisition_goal_type,
                       stateful_foraging_controller,
                       acquisition_goal,
                       current_task());

FSM_WRAPPER_DEFINE_PTR(bool,
                       stateful_foraging_controller,
                       goal_acquired,
                       current_task());

/*******************************************************************************
 * World Model Metrics
 ******************************************************************************/
uint stateful_foraging_controller::cell_state_inaccuracies(uint state) const {
  return m_perception->cell_state_inaccuracies(state);
} /* cell_state_inaccuracies() */

using namespace argos;
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-variable-declarations"
#pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wglobal-constructors"
REGISTER_CONTROLLER(stateful_foraging_controller,
                    "stateful_foraging_controller"); // NOLINT
#pragma clang diagnostic pop

NS_END(depth0, controller, fordyca);
