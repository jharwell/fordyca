/**
 * @file stateful_controller.cpp
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
#include "fordyca/controller/depth0/stateful_controller.hpp"
#include <fstream>

#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/base_perception_subsystem.hpp"
#include "fordyca/controller/block_sel_matrix.hpp"
#include "fordyca/controller/sensing_subsystem.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/fsm/depth0/stateful_fsm.hpp"
#include "fordyca/params/block_sel_matrix_params.hpp"
#include "fordyca/params/depth0/stateful_controller_repository.hpp"
#include "fordyca/params/sensing_params.hpp"
#include "fordyca/representation/base_block.hpp"

#include "fordyca/representation/base_cell_entity.hpp"
#include "fordyca/representation/ramp_block.hpp"
#include "fordyca/representation/cube_block.hpp"
#include "fordyca/representation/base_cache.hpp"
#include "fordyca/metrics/blocks/transport_metrics.hpp"
#include "fordyca/events/block_found.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth0);
namespace ta = rcppsw::task_allocation;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
stateful_controller::stateful_controller(void)
    : crw_controller(),
      ER_CLIENT_INIT("fordyca.controller.depth0.stateful"),
      m_light_loc(),
      m_block_sel_matrix(),
      m_perception(),
      m_fsm(),
      m_communication_params(),
      m_arena_x(0),
      m_arena_y(0) {}

stateful_controller::~stateful_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void stateful_controller::block_sel_matrix(
    std::unique_ptr<class block_sel_matrix> m) {
  m_block_sel_matrix = std::move(m);
}

void stateful_controller::perception(
    std::unique_ptr<base_perception_subsystem> perception) {
  m_perception = std::move(perception);
}

__rcsw_pure const representation::line_of_sight* stateful_controller::los(
    void) const {
  return m_perception->los();
}
void stateful_controller::los(
    std::unique_ptr<representation::line_of_sight>& new_los) {
  m_perception->los(new_los);
}

double stateful_controller::los_dim(void) const {
  return saa_subsystem()->sensing()->los_dim();
} /* los_dim() */


void stateful_controller::ControlStep(void) {
  ndc_pusht();
  if (nullptr != block()) {
    ER_ASSERT(-1 != block()->robot_id(),
              "Carried block%d has robot id=%d",
              block()->id(),
              block()->robot_id());
  }

  /*
   * Update the robot's model of the world with the current line-of-sight, and
   * update the relevance of information within it. Then, you can run the main
   * FSM loop.
   */
  m_perception->update(m_perception->los());

  saa_subsystem()->actuation()->block_carry_throttle(is_carrying_block());
  saa_subsystem()->actuation()->throttling_update(saa_subsystem()->sensing()->tick());
  m_fsm->run();

  if(m_communication_params.on) {
    perform_communication();
  }

  m_fsm->run();
  ndc_pop();
} /* ControlStep() */

void stateful_controller::Init(ticpp::Element& node) {
  /*
   * Note that we do not call \ref crw_controller::Init()--there
   * is nothing in there that we need.
   */
  base_controller::Init(node);

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

  auto* block_mat = param_repo.parse_results<params::block_sel_matrix_params>();
  m_block_sel_matrix = rcppsw::make_unique<class block_sel_matrix>(block_mat);

  m_fsm = rcppsw::make_unique<fsm::depth0::stateful_fsm>(
      m_block_sel_matrix.get(),
      base_controller::saa_subsystem(),
      m_perception->map());

  auto* comm_params = param_repo.parse_results<params::communication_params>();
  m_communication_params = *comm_params;

  ER_INFO("Initialization finished");
  ndc_pop();


  m_arena_x = perception()->map()->xdsize();
  m_arena_y = perception()->map()->ydsize();
} /* Init() */

void stateful_controller::Reset(void) {
  crw_controller::Reset();
  m_perception->reset();
} /* Reset() */

void stateful_controller::perform_communication(void) {
  std::vector<uint8_t> recieved_packet_data =
    saa_subsystem()->sensing()->recieve_message();
  float probability = static_cast <float> (rand()) /
      static_cast <float> (RAND_MAX);

  hal::wifi_packet packet = hal::wifi_packet();
  if (!recieved_packet_data.empty() && probability >=
      (1 - m_communication_params.chance_to_continue_communication)) {
    packet.data = recieved_packet_data;
    integrate_recieved_packet(packet);
    saa_subsystem()->actuation()->start_sending_message(packet);
  } else if (probability >= (1 -
      m_communication_params.chance_to_start_communication)) {
    int x_coord;
    int y_coord;

    // Random Mode
    if(m_communication_params.mode == 1) {
      x_coord = static_cast <int> (rand()) % m_arena_x;
      y_coord = static_cast <int> (rand()) % m_arena_y;
    // Utility function
    } else if (m_communication_params.mode == 2) {
      rcppsw::math::vector2u cell = get_most_valuable_cell();
      x_coord = cell.x();
      y_coord = cell.y();
    } else {
      x_coord = 2;
      y_coord = 2;
    }

    ds::cell2D cell = perception()->map()->
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

    // Entity ID (will be 0 if the cell is unknown)
    packet.data.push_back(static_cast<uint8_t>(id));

    // Type of block
    packet.data.push_back(static_cast<uint8_t>(type));

    rcppsw::swarm::pheromone_density& density = perception()->map()->
      access<ds::occupancy_grid::kPheromone>(x_coord, y_coord);
    packet.data.push_back(static_cast<uint8_t>(density.last_result() * 10));

    saa_subsystem()->actuation()->start_sending_message(packet);
  } else {
    saa_subsystem()->actuation()->stop_sending_message();
  } /* if !recieved_packet_data.empty() */
} /* perform_communication */

void stateful_controller::integrate_recieved_packet(hal::wifi_packet packet) {
  // Data extraction
  int x_coord = static_cast<int>(packet.data[0]);
  int y_coord = static_cast<int>(packet.data[1]);
  int state = static_cast<int>(packet.data[2]);
  int ent_id = static_cast<int>(packet.data[3]);
  // type of block (if it's not a block it will be -1)

  rcppsw::math::vector2u disc_loc = rcppsw::math::vector2u(x_coord, y_coord);

  auto rcoord_vector = uvec2dvec(disc_loc,
    perception()->map()->grid_resolution());


  int type = static_cast<int>(packet.data[4]);
  double pheromone_density = static_cast<double>(packet.data[5]) / 10;

  if (state > 2) {
    rcppsw::swarm::pheromone_density& density = perception()->map()->
      access<ds::occupancy_grid::kPheromone>(x_coord, y_coord);

    // If the recieved pheromone density is less than the known, don't
    // integrate the recieved information.
    if (density.last_result() + 0.001 >= pheromone_density) {
      return;
    }

    // blocks
    if (state == 3) {
      // ramp block
      if (type == 2) {
        std::shared_ptr<representation::ramp_block> block_ptr (new
          representation::ramp_block(rcppsw::math::vector2d(2, 1), ent_id));
        block_ptr->real_loc(rcoord_vector);
        block_ptr->discrete_loc(disc_loc);

        perception()->map()->accept(*(
          new fordyca::events::block_found(block_ptr)));
      // cube block
    } else if (type == 3) {
        std::shared_ptr<representation::cube_block> block_ptr (new
          representation::cube_block(rcppsw::math::vector2d(1, 1), ent_id));
        block_ptr->real_loc(rcoord_vector);
        block_ptr->discrete_loc(disc_loc);

        perception()->map()->accept(*(
          new fordyca::events::block_found(block_ptr)));
      } /* if type */

      density.pheromone_set(pheromone_density);
    // caches
    } else {
      // TODO: Add caches to percieved_arena_map

    } /* if (state == 3) / else */
  } /* if (state > 2) */
} /* integrate_recieved_packet */

rcppsw::math::vector2u stateful_controller::get_most_valuable_cell(void) {
  rcppsw::math::vector2u cell_coords;

  // Get information regarding the location of the nest
  rcppsw::math::vector2d nest_loc = boost::get<rcppsw::math::vector2d>(
    m_block_sel_matrix->find("nest_loc")->second);
  int nest_x_coord = nest_loc.x();
  int nest_y_coord = nest_loc.y();

  // Initialize variables for keeping track of most valuable location
  int communicated_cell_value = 0;
  int current_cell_value = 0;

  int arena_x_coord = static_cast<int>(m_arena_x);
  int arena_y_coord = static_cast<int>(m_arena_y);
  int cell_x = 0;
  int cell_y = 0;

  for (int i = 0; i < arena_x_coord; ++i) {
    for (int j = 0; j < arena_y_coord; ++j) {
      // Current cell
      ds::cell2D current_cell = perception()->map()->
        access<ds::occupancy_grid::kCell>(i, j);

      if (current_cell.state_has_block()) {
        // Current cell density
        rcppsw::swarm::pheromone_density& density = perception()->map()->
          access<ds::occupancy_grid::kPheromone>(i, j);

        // Distance from the nest * number of blocks * pheromone density
        current_cell_value = ((((i - nest_x_coord)^2) +
          ((j - nest_y_coord)^2))^(1/2)) * current_cell.block_count() *
           density.last_result();

        // Update variables
        if (current_cell_value > communicated_cell_value) {
          communicated_cell_value = current_cell_value;
          cell_x = i;
          cell_y = j;
        } /* update value and cell location */
      } /* if cell has block */
    } /* for(j..) */
  } /* for(i..) */

  cell_coords.set(cell_x, cell_y);
  return cell_coords;
} /* get_most_valuable_cell */

FSM_WRAPPER_DEFINEC_PTR(transport_goal_type,
                       stateful_controller,
                       block_transport_goal,
                       m_fsm);

FSM_WRAPPER_DEFINEC_PTR(acquisition_goal_type,
                        stateful_controller,
                        acquisition_goal,
                        m_fsm);

FSM_WRAPPER_DEFINEC_PTR(bool, stateful_controller, goal_acquired, m_fsm);

/*******************************************************************************
 * World Model Metrics
 ******************************************************************************/
uint stateful_controller::cell_state_inaccuracies(uint state) const {
  return m_perception->cell_state_inaccuracies(state);
} /* cell_state_inaccuracies() */

double stateful_controller::known_percentage(void) const {
  return m_perception->known_percentage();
} /* known_percentage() */

double stateful_controller::unknown_percentage(void) const {
  return m_perception->unknown_percentage();
} /* unknown_percentage() */

using namespace argos; // NOLINT
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-variable-declarations"
#pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wglobal-constructors"
REGISTER_CONTROLLER(stateful_controller, "stateful_controller");
#pragma clang diagnostic pop

NS_END(depth0, controller, fordyca);
