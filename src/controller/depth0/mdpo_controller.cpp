/**
 * @file mdpo_controller.cpp
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
#include "fordyca/controller/depth0/mdpo_controller.hpp"
#include "fordyca/config/depth0/mdpo_controller_repository.hpp"
#include "fordyca/config/perception/perception_config.hpp"
#include "fordyca/controller/mdpo_perception_subsystem.hpp"
#include "fordyca/ds/dpo_semantic_map.hpp"
#include "fordyca/fsm/depth0/dpo_fsm.hpp"
#include "fordyca/representation/base_block.hpp"
#include "fordyca/representation/ramp_block.hpp"
#include "fordyca/representation/cube_block.hpp"
#include "fordyca/events/block_found.hpp"
#include "fordyca/fsm/expstrat/block_factory.hpp"
#include "fordyca/repr/base_block.hpp"
/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
mdpo_controller::mdpo_controller(void)
    : ER_CLIENT_INIT("fordyca.controller.depth0.mdpo"),
      m_communication_params() {}

mdpo_controller::~mdpo_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void mdpo_controller::ControlStep(void)
{
  ndc_pusht();
  ER_ASSERT(!(nullptr != block() && -1 == block()->robot_id()),
            "Carried block%d has robot id=%d",
            block()->id(),
            block()->robot_id());
  perception()->update(nullptr);

  if (m_communication_params.on)
  {
    communication_check();
  }

  fsm()->run();
  ndc_pop();
} /* ControlStep() */

void mdpo_controller::Init(ticpp::Element &node)
{
  /*
   * Note that we do not call \ref crw_controller::Init()--there
   * is nothing in there that we need.
   */
  base_controller::Init(node);

  ndc_push();
  ER_INFO("Initializing...");

  /* parse and validate parameters */
  config::depth0::mdpo_controller_repository param_repo;
  param_repo.parse_all(node);

  if (!param_repo.validate_all())
  {
    ER_FATAL_SENTINEL("Not all parameters were validated");
    std::exit(EXIT_FAILURE);
  }

  shared_init(param_repo);
  private_init(param_repo);

  auto *comm_params = param_repo.parse_results<params::communication_params>();
  m_communication_params = *comm_params;

  ER_INFO("Initialization finished");
  ndc_pop();
} /* Init() */

void mdpo_controller::communication_check(void)
{
  // Receive vector of all messages available
  std::vector<std::vector<uint8_t>> recieved_packet_data =
      saa_subsystem()->sensing()->recieve_message();

  // Calculate probabilities for sending and receiving a message
  float prob_receive = static_cast<float>(rand()) /
                       static_cast<float>(RAND_MAX);
  float prob_send = static_cast<float>(rand()) /
                    static_cast<float>(RAND_MAX);

  // Make sure there are available message to integrate and that the probability
  // is in an acceptable range.
  if (!recieved_packet_data.empty() && prob_receive >=
                                           (1 - m_communication_params.prob_receive))
  {
    hal::wifi_packet packet = hal::wifi_packet();

    // Iterate over every message and integrate its contents with the robot's
    // internal mapping
    for (std::vector<uint8_t> individual_message : recieved_packet_data)
    {
      packet.data = individual_message;
      integrate_recieved_packet(packet);
    }
  }

  // If prob_send is acceptable, then fill the packet's contents and begin
  // communication.
  if (prob_send >= (1 - m_communication_params.prob_send))
  {
    fill_packet();
  }
  else
  {
    // Only stop sending messages when the probabilty for sending falls below
    // an acceptable number.
    saa_subsystem()->actuation()->stop_sending_message();
  } /* if !recieved_packet_data.empty() */
} /* perform_communication */

void mdpo_controller::fill_packet(void)
{
  hal::wifi_packet packet = hal::wifi_packet();
  int x_coord;
  int y_coord;

  // Random Mode
  if (m_communication_params.mode.compare("random") == 0)
  {
    x_coord = static_cast<int>(rand()) % mdpo_perception()->map()->xdsize();
    y_coord = static_cast<int>(rand()) % mdpo_perception()->map()->ydsize();
    // Utility function
  }
  else if (m_communication_params.mode.compare("utility") == 0)
  {
    rcppsw::math::vector2u cell = get_most_valuable_cell();
    x_coord = cell.x();
    y_coord = cell.y();
  }
  else
  {
    // Fail safe coords
    x_coord = 2;
    y_coord = 2;
  }

  ds::cell2D cell = mdpo_perception()->map()->access<ds::occupancy_grid::kCell>(x_coord, y_coord);
  packet.data.push_back(static_cast<uint8_t>(x_coord)); // X Coord of cell
  packet.data.push_back(static_cast<uint8_t>(y_coord)); // Y Coord of cell

  // The state is what the cell contains (nothing, block, or cache)
  int state = 1;
  if (cell.state_is_empty())
  {
    state = 2;
  }
  else if (cell.state_has_block())
  {
    state = 3;
  }
  else if (cell.state_has_cache())
  {
    state = 4;
  }
  // Type of entity (block / cache) (will be 1 if the cell state is unknown)
  packet.data.push_back(static_cast<uint8_t>(state));

  auto entity = cell.entity();
  int id = 0;

  // Type is specific to blocks as there are ramp and cube blocks.
  int type = 1;
  if (entity)
  {
    id = entity->id();

    // Block
    if (state == 3)
    {
      // Ramp block
      if (cell.block()->type() == metrics::blocks::transport_metrics::kRamp)
      {
        type = 2;
        // Cube block
      }
      else
      {
        type = 3;
      } /* if block type */
    }   /* if state */
  }     /* if entity */

  // Entity ID (will be 0 if the cell is unknown)
  packet.data.push_back(static_cast<uint8_t>(id));

  // Type of block
  packet.data.push_back(static_cast<uint8_t>(type));

  rcppsw::swarm::pheromone_density &density = mdpo_perception()->map()->access<ds::occupancy_grid::kPheromone>(x_coord, y_coord);
  packet.data.push_back(static_cast<uint8_t>(density.last_result() * 10));

  saa_subsystem()->actuation()->start_sending_message(packet);
}

void mdpo_controller::integrate_recieved_packet(hal::wifi_packet packet)
{
  // Data extraction
  int x_coord = static_cast<int>(packet.data[0]);
  int y_coord = static_cast<int>(packet.data[1]);
  int state = static_cast<int>(packet.data[2]);
  int ent_id = static_cast<int>(packet.data[3]);

  rcppsw::math::vector2u disc_loc = rcppsw::math::vector2u(x_coord, y_coord);

  auto rcoord_vector = uvec2dvec(disc_loc,
                                 mdpo_perception()->map()->grid_resolution());

  // type of block (if it's not a block it will be -1)
  int type = static_cast<int>(packet.data[4]);

  double pheromone_density = static_cast<double>(packet.data[5]) / 10;

  if (state > 2)
  {
    rcppsw::swarm::pheromone_density &density = mdpo_perception()->map()->access<ds::occupancy_grid::kPheromone>(x_coord, y_coord);

    // If the recieved pheromone density is less than the known, don't
    // integrate the recieved information.
    if (pheromone_density <= 0.1 || density.last_result() >= pheromone_density)
    {
      return;
    }

    // blocks
    if (state == 3)
    {
      // ramp block
      if (type == 2)
      {
        std::shared_ptr<representation::ramp_block> block_ptr(new representation::ramp_block(rcppsw::math::vector2d(2, 1), ent_id));
        block_ptr->real_loc(rcoord_vector);
        block_ptr->discrete_loc(disc_loc);

        mdpo_perception()->map()->accept(*(
            new fordyca::events::block_found(block_ptr)));
        // cube block
      }
      else if (type == 3)
      {
        std::shared_ptr<representation::cube_block> block_ptr(new representation::cube_block(rcppsw::math::vector2d(1, 1), ent_id));
        block_ptr->real_loc(rcoord_vector);
        block_ptr->discrete_loc(disc_loc);

        mdpo_perception()->map()->accept(*(
            new fordyca::events::block_found(block_ptr)));
      } /* if type */

      // density.pheromone_set(pheromone_density);
      // caches
    }
    else
    {
      // TODO: Add caches to percieved_arena_map (for different controller)
    } /* if (state == 3) / else */
  }   /* if (state > 2) */
} /* integrate_recieved_packet */

rcppsw::math::vector2u mdpo_controller::get_most_valuable_cell(void)
{
  rcppsw::math::vector2u cell_coords;

  // Get information regarding the location of the nest
  rcppsw::math::vector2d nest_loc = boost::get<rcppsw::math::vector2d>(
      block_sel_matrix()->find("nest_loc")->second);
  int nest_x_coord = nest_loc.x();
  int nest_y_coord = nest_loc.y();

  // Initialize variables for keeping track of most valuable location
  int communicated_cell_value = 0;
  int current_cell_value = 0;

  int arena_x_coord = static_cast<int>(mdpo_perception()->map()->xdsize());
  int arena_y_coord = static_cast<int>(mdpo_perception()->map()->ydsize());
  int cell_x = 0;
  int cell_y = 0;

  for (int i = 0; i < arena_x_coord; ++i)
  {
    for (int j = 0; j < arena_y_coord; ++j)
    {
      // Current cell
      ds::cell2D current_cell = mdpo_perception()->map()->access<ds::occupancy_grid::kCell>(i, j);

      if (current_cell.state_has_block())
      {
        // Current cell density
        rcppsw::swarm::pheromone_density &density = mdpo_perception()->map()->access<ds::occupancy_grid::kPheromone>(i, j);

        // Distance from the nest * number of blocks * pheromone density
        current_cell_value = (1 / ((((i - nest_x_coord) ^ 2) +
                                    ((j - nest_y_coord) ^ 2)) ^
                                   (1 / 2))) *
                             current_cell.block_count() *
                             density.last_result();

        // Update variables
        if (current_cell_value > communicated_cell_value)
        {
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

void mdpo_controller::shared_init(
    const config::depth0::mdpo_controller_repository &param_repo)
{
  /* block selection matrix and DPO subsystem */
  dpo_controller::shared_init(param_repo);

  /* MDPO perception subsystem */
  config::perception::perception_config p =
      *param_repo.config_get<config::perception::perception_config>();
  p.occupancy_grid.upper.x(p.occupancy_grid.upper.x() + 1);
  p.occupancy_grid.upper.y(p.occupancy_grid.upper.y() + 1);

  dpo_controller::perception(
      rcppsw::make_unique<mdpo_perception_subsystem>(&p, GetId()));
} /* shared_init() */

void mdpo_controller::private_init(
    const config::depth0::mdpo_controller_repository &param_repo)
{
  auto *exp_config = param_repo.config_get<config::exploration_config>();
  fsm::expstrat::block_factory f;
  fsm::expstrat::base_expstrat::params p(nullptr,
                                         saa_subsystem(),
                                         perception()->dpo_store());
  dpo_controller::fsm(rcppsw::make_unique<fsm::depth0::dpo_fsm>(
      block_sel_matrix(),
      base_controller::saa_subsystem(),
      perception()->dpo_store(),
      f.create(exp_config->block_strategy, &p)));
} /* private_init() */

__rcsw_pure mdpo_perception_subsystem *mdpo_controller::mdpo_perception(void)
{
  return static_cast<mdpo_perception_subsystem *>(dpo_controller::perception());
} /* perception() */

__rcsw_pure const mdpo_perception_subsystem *mdpo_controller::mdpo_perception(
    void) const
{
  return static_cast<const mdpo_perception_subsystem *>(
      dpo_controller::perception());
} /* perception() */

using namespace argos; // NOLINT
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-variable-declarations"
#pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wglobal-constructors"
REGISTER_CONTROLLER(mdpo_controller, "mdpo_controller");
#pragma clang diagnostic pop

NS_END(depth0, controller, fordyca);
