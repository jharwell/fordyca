/**
 * @file arena_map-test.cpp
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
#define CATCH_CONFIG_PREFIX_ALL
#define CATCH_CONFIG_MAIN
#include <catch.hpp>
#include "fordyca/representation/arena_map.hpp"
#include "fordyca/params/params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
using namespace fordyca::representation;
using namespace fordyca;

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
struct grid_params params = {
  0.2, argos::CVector2(10, 5), argos::CVector2(0, 0),
  {25, 0.2, "random", true}
};
argos::CRange<argos::Real> nest_x(0.5, 1.5);
argos::CRange<argos::Real> nest_y(2.5, 3.5);

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/
void map_sanity_check(arena_map& map, int type) {
  /* Verify all cells that don't contain blocks are empty */
  for (size_t i = 0; i < params.upper.GetX()/params.resolution; ++i) {
    for (size_t j = 0; j < params.upper.GetY()/params.resolution; ++j) {
      cell2D& cell = map.access(i, j);
      if (!cell.state_has_block()) {
        if (0 == type) {
          CATCH_REQUIRE(cell.state_is_empty());
        } else {
          CATCH_REQUIRE(!cell.state_is_known());
        }
      }
    } /* for(j..) */
  } /* for(i..) */
}

void map_resolution_check(arena_map& map) {
  /*
   * Verify that every cell is pointing to the correct block (testing the
   * resolution-based indexing).
   */
  for (size_t i = 0; i < map.blocks().size(); ++i) {
    cell2D& cell = map.access(map.blocks()[i].discrete_loc().first,
                              map.blocks()[i].discrete_loc().second);
    CATCH_REQUIRE((cell.block() == &map.blocks()[i]));
  } /* for(i..) */
}

/*******************************************************************************
 * Test Cases
 ******************************************************************************/
CATCH_TEST_CASE("init-test", "[arena_map]") {
  argos::CRandom::CreateCategory("argos", 123);
  arena_map map(&params, nest_x, nest_y);
  CATCH_REQUIRE(map.n_blocks() == params.block.n_blocks);
  CATCH_REQUIRE(map.respawn_enabled() == params.block.respawn);

  /* verify all cells in a known state */
  map_sanity_check(map, 1);
}

CATCH_TEST_CASE("distribute-test", "[arena_map]") {
  arena_map map(&params, nest_x, nest_y);

  map.distribute_blocks(true);

  /* Verify all cells that actually contain blocks think they contain blocks */
  for (size_t i = 0; i < map.blocks().size(); ++i) {
    cell2D& cell = map.access(map.blocks()[i].discrete_loc().first,
                              map.blocks()[i].discrete_loc().second);
    CATCH_REQUIRE(cell.state_has_block());
  } /* for(i..) */

  map_sanity_check(map, 0);
  map_resolution_check(map);

  map.distribute_blocks(false);
  /* Verify all cells that actually contain blocks think they contain blocks */
  for (size_t i = 0; i < map.blocks().size(); ++i) {
    cell2D& cell = map.access(map.blocks()[i].discrete_loc().first,
                              map.blocks()[i].discrete_loc().second);
    CATCH_REQUIRE(cell.state_has_block());
  } /* for(i..) */

  map_sanity_check(map, 0);
  map_resolution_check(map);
}

CATCH_TEST_CASE("block-move-test", "[arena_map]") {
  arena_map map(&params, nest_x, nest_y);
  map.distribute_blocks(true);

  /*
   * Simulate a robot picking up, carrying, and then dropping a block, verifying
   * the arena map is in the current state all the way.
   */
  for (size_t i = 0; i < map.blocks().size(); ++i) {
    CATCH_REQUIRE(map.blocks()[i].id() != -1);
    cell2D& old_cell = map.access(map.blocks()[i].discrete_loc().first,
                              map.blocks()[i].discrete_loc().second);
    map.event_block_pickup(map.blocks()[i], 0);
    CATCH_REQUIRE(old_cell.state_is_empty());
    map_sanity_check(map, 0);

    CATCH_REQUIRE(map.blocks()[i].robot_index() != -1);
    map.event_block_nest_drop(map.blocks()[i]);
    cell2D& new_cell = map.access(map.blocks()[i].discrete_loc().first,
                                  map.blocks()[i].discrete_loc().second);
    CATCH_REQUIRE(new_cell.state_has_block());

    map_sanity_check(map, 0);
    map_resolution_check(map);
  } /* for(i..) */
}
