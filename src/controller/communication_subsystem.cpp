/**
 * @file communication_subsystem.cpp
 *
 * @copyright 2019 Nathan White, All rights reserved.
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
#include "fordyca/controller/communication_subsystem.hpp"
#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/sensing_subsystem.hpp"
#include "fordyca/ds/dpo_semantic_map.hpp"
#include "fordyca/events/block_found.hpp"
#include "fordyca/repr/base_block.hpp"
#include "fordyca/repr/cube_block.hpp"
#include "fordyca/repr/ramp_block.hpp"
#include "rcppsw/robotics/hal/wifi_packet.hpp"

/*
VERIFY IT WORKS:
changed "nest_loc" to kNestLoc
*/

// NATETODO: do I need these

// #include "fordyca/fsm/depth0/dpo_fsm.hpp"
/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
NS_START(controller);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
communication_subsystem::communication_subsystem(
    controller::base_perception_subsystem* perception,
    controller::saa_subsystem* saa,
    class block_sel_matrix* block_sel_matrix)
    : m_perception(perception),
      m_saa(saa),
      m_block_sel_matrix(block_sel_matrix) {}
/*******************************************************************************
 * Member Functions
 ******************************************************************************/

//  This is how we got the params in Init from MDPO.
// void mdpo_controller::Init(ticpp::Element &node)
// {
//   auto *comm_params = param_repo.config_get<config::communication_config>();
//   m_communication_params = *comm_params;
// }

void communication_subsystem::communication_check(void) {
  // If communication is off, do nothing and return;
  if (!m_communication_params.on) {
    return;
  }

  // Receive vector of all messages available
  std::vector<std::vector<uint8_t>> recieved_packet_data =
      validate_messages(saa_subsystem()->sensing()->recieve_message());

  // Calculate probabilities for sending and receiving a message
  float prob_receive = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
  float prob_send = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

  // Make sure there are available message to integrate and that the probability
  // is in an acceptable range.
  if (!recieved_packet_data.empty() &&
      prob_receive >= (1 - m_communication_params.prob_receive)) {
    rcppsw::robotics::hal::wifi_packet packet =
        rcppsw::robotics::hal::wifi_packet();

    // Iterate over every message and integrate its contents with the robot's
    // internal mapping
    for (std::vector<uint8_t> individual_message : recieved_packet_data) {
      integrate_recieved_packet(individual_message);
    }
  }

  // If prob_send is acceptable, then fill the packet's contents and begin
  // communication.
  if (prob_send >= (1 - m_communication_params.prob_send)) {
    fill_packet();
  } else {
    // Only stop sending messages when the probabilty for sending falls below
    // an acceptable number.
    saa_subsystem()->actuation()->stop_sending_message();
  } /* if !recieved_packet_data.empty() */
} /* perform_communication */

void communication_subsystem::fill_packet(void) {
  rrhal::wifi_packet packet = rrhal::wifi_packet();
  int x_coord;
  int y_coord;

  // Random Mode
  if (m_communication_params.mode.compare(kRANDOM) == 0) {
    x_coord = static_cast<int>(rand()) % mdpo_perception()->map()->xdsize();
    y_coord = static_cast<int>(rand()) % mdpo_perception()->map()->ydsize();
    // Utility function
  } else if (m_communication_params.mode.compare(kUTILITY) == 0) {
    rcppsw::math::vector2u cell = get_most_valuable_cell();
    x_coord = cell.x();
    y_coord = cell.y();
  } else {
    // Fail safe coords
    x_coord = 2;
    y_coord = 2;
  }

  ds::cell2D cell =
      mdpo_perception()->map()->access<ds::occupancy_grid::kCell>(x_coord,
                                                                  y_coord);
  packet.data.push_back(static_cast<uint8_t>(x_coord)); // X Coord of cell
  packet.data.push_back(static_cast<uint8_t>(y_coord)); // Y Coord of cell

  // The state is what the cell contains (nothing, block, or cache)
  int state = fsm::cell2D_fsm::ekST_UNKNOWN;
  if (cell.state_is_empty()) {
    state = fsm::cell2D_fsm::ekST_EMPTY;
  } else if (cell.state_has_block()) {
    state = fsm::cell2D_fsm::ekST_HAS_BLOCK;
  } else if (cell.state_has_cache()) {
    state = fsm::cell2D_fsm::ekST_HAS_CACHE;
  }
  // Type of entity (block / cache) (will be 1 if the cell state is unknown)
  packet.data.push_back(static_cast<uint8_t>(state));

  auto entity = cell.entity();
  int id = 0;

  // Type is specific to blocks as there are ramp and cube blocks.
  int type = -1;
  if (entity) {
    id = entity->id();

    // Block
    if (state == fsm::cell2D_fsm::ekST_HAS_BLOCK) {
      // NATETODO: make sure this doesn't result in an error
      type = static_cast<int>(cell.block()->type());
    } /* if state */
  }   /* if entity */

  // Entity ID (will be 0 if the cell is unknown)
  packet.data.push_back(static_cast<uint8_t>(id));

  // Type of block
  packet.data.push_back(static_cast<uint8_t>(type));

  rcppsw::swarm::pheromone_density& density =
      mdpo_perception()->map()->access<ds::occupancy_grid::kPheromone>(x_coord,
                                                                       y_coord);
  packet.data.push_back(static_cast<uint8_t>(density.v() * 10));

  saa_subsystem()->actuation()->start_sending_message(packet);
}

void communication_subsystem::integrate_recieved_packet(
    std::vector<uint8_t> packet_data) {
  // Data extraction
  int x_coord = static_cast<int>(packet_data[0]);
  int y_coord = static_cast<int>(packet_data[1]);
  int state = static_cast<int>(packet_data[2]);
  int ent_id = static_cast<int>(packet_data[3]);

  rcppsw::math::vector2u disc_loc = rcppsw::math::vector2u(x_coord, y_coord);

  auto rcoord_vector =
      uvec2dvec(disc_loc, mdpo_perception()->map()->resolution().v());

  // type of block (if it's not a bl->acceock it will be -1)
  int type = static_cast<int>(packet_data[4]);

  double pheromone_density = static_cast<double>(packet_data[5]) / 10;

  if (state != fsm::cell2D_fsm::ekST_UNKNOWN) {
    rcppsw::swarm::pheromone_density& density =
        mdpo_perception()->map()->access<ds::occupancy_grid::kPheromone>(
            x_coord, y_coord);

    // If the recieved pheromone density is less than the known, don't
    // integrate the recieved information.
    if (pheromone_density <= 0.1 || density.v() >= pheromone_density) {
      return;
    }

    // blocks
    if (state == fsm::cell2D_fsm::ekST_HAS_BLOCK) {
      // size = 1 for cube
      int size = 1;
      if (type == static_cast<int>(repr::block_type::ekRAMP)) {
        size = 2;
      }
      std::shared_ptr<repr::cube_block> block_ptr(
          new repr::cube_block(rcppsw::math::vector2d(size, 1), ent_id));
      block_ptr->rloc(rcoord_vector);
      block_ptr->dloc(disc_loc);

      events::block_found_visitor found_block(block_ptr);
      found_block.visit(*(mdpo_perception()->dpo_store()));

      density.pheromone_set(pheromone_density);
      // caches
    } else {
      // TODO: Add caches to percieved_arena_map (for different controller)
    } /* if (state == 3) / else */
  }   /* if (state > 2) */
} /* integrate_recieved_packet */

rcppsw::math::vector2u communication_subsystem::get_most_valuable_cell(void) {
  rcppsw::math::vector2u cell_coords;

  // Get information regarding the location of the nest
  rcppsw::math::vector2d nest_loc = boost::get<rcppsw::math::vector2d>(
      block_sel_matrix()->find(controller::block_sel_matrix::kNestLoc)->second);
  int nest_x_coord = static_cast<int>(nest_loc.x());
  int nest_y_coord = static_cast<int>(nest_loc.y());

  // Initialize variables for keeping track of most valuable location
  int communicated_cell_value = 0;
  int current_cell_value = 0;

  int arena_x_coord = static_cast<int>(mdpo_perception()->map()->xdsize());
  int arena_y_coord = static_cast<int>(mdpo_perception()->map()->ydsize());
  int cell_x = 0;
  int cell_y = 0;

  for (int i = 0; i < arena_x_coord; ++i) {
    for (int j = 0; j < arena_y_coord; ++j) {
      // Current cell
      ds::cell2D current_cell =
          mdpo_perception()->map()->access<ds::occupancy_grid::kCell>(i, j);

      if (current_cell.state_has_block()) {
        // Current cell density
        rcppsw::swarm::pheromone_density& density =
            mdpo_perception()->map()->access<ds::occupancy_grid::kPheromone>(i,
                                                                             j);

        // Distance from the nest * number of blocks * pheromone density
        current_cell_value = static_cast<int>(
            (1 / ((((i - nest_x_coord) ^ 2) + ((j - nest_y_coord) ^ 2)) ^
                  (1 / 2))) *
            current_cell.block_count() * density.v());

        // Update variables
        if (current_cell_value > communicated_cell_value) {
          communicated_cell_value = current_cell_value;
          cell_x = i;
          cell_y = j;
        } /* update value and cell location */
      }   /* if cell has block */
    }     /* for(j..) */
  }       /* for(i..) */

  cell_coords.set(cell_x, cell_y);
  return cell_coords;
} /* get_most_valuable_cell */

std::vector<std::vector<uint8_t>> communication_subsystem::validate_messages(
    std::vector<rrhal::sensors::rab_wifi_sensor::rab_wifi_packet> readings) {
  std::vector<std::vector<uint8_t>> return_data;
  for (rrhal::sensors::rab_wifi_sensor::rab_wifi_packet packet : readings) {
    std::vector<uint8_t> data = packet.data;
    // Check if data isn't empty and if there is a value in the state position
    if (!data.empty() && static_cast<int>(data[2]) >= 1 &&
        static_cast<int>(data[0]) > 0 && static_cast<int>(data[1]) > 0) {
      return_data.push_back(data);
    }
  }
  return return_data;
}

NS_END(controller, fordyca);