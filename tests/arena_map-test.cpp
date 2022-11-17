/**
 * @file arena_map-test.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#define CATCH_CONFIG_PREFIX_ALL
#define CATCH_CONFIG_MAIN
#include <catch.hpp>
#include "fordyca/representation/arena_map.hpp"
#include "fordyca/params/grid_params.hpp"
#include "fordyca/events/block_drop.hpp"
#include "fordyca/events/block_pickup.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
using namespace fordyca::representation;
using namespace fordyca;

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
struct params::grid_params gparams = {
  0.2, argos::CVector2(10, 5), argos::CVector2(0, 0),
  {25, 0.2, "random", true}
};
argos::CRange<double> nest_x(0.5, 1.5);
argos::CRange<double> nest_y(2.5, 3.5);

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/
void map_sanity_check(arena_map& map, int type) {
  /* Verify all cells that don't contain blocks are empty */
  for (size_t i = 0; i < gparams.upper.GetX()/gparams.resolution; ++i) {
    for (size_t j = 0; j < gparams.upper.GetY()/gparams.resolution; ++j) {
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
    if (map.blocks()[i].dloc().first != 100) {
      cell2D& cell = map.access(map.blocks()[i].dloc().first,
                                map.blocks()[i].dloc().second);
      printf("Verify cell at (%d, %d) contains block %d: %p/%p\n",
             cell.loc().first, cell.loc().second, i, cell.block(),
              &map.blocks()[i]);
      CATCH_REQUIRE((cell.block() == &map.blocks()[i]));
    }
  } /* for(i..) */
}

/*******************************************************************************
 * Test Cases
 ******************************************************************************/
CATCH_TEST_CASE("init-test", "[arena_map]") {
  argos::CRandom::CreateCategory("argos", 123);
  arena_map map(&gparams, nest_x, nest_y);
  CATCH_REQUIRE(map.n_blocks() == gparams.block.n_blocks);
  CATCH_REQUIRE(map.respawn_enabled() == gparams.block.respawn);

  /* verify all cells in a known state */
  map_sanity_check(map, 1);
}

CATCH_TEST_CASE("distribute-test", "[arena_map]") {
  argos::CRandom::CreateCategory("argos", 456);
  arena_map map(&gparams, nest_x, nest_y);

  map.distribute_blocks(true);

  /* Verify all cells that actually contain blocks think they contain blocks */
  for (size_t i = 0; i < map.blocks().size(); ++i) {
    cell2D& cell = map.access(map.blocks()[i].dloc().first,
                              map.blocks()[i].dloc().second);
    CATCH_REQUIRE(cell.state_has_block());
  } /* for(i..) */

  map_sanity_check(map, 0);
  map_resolution_check(map);

  map.distribute_blocks(false);

  /* Verify all cells that actually contain blocks think they contain blocks */
  for (size_t i = 0; i < map.blocks().size(); ++i) {
    cell2D& cell = map.access(map.blocks()[i].dloc().first,
                              map.blocks()[i].dloc().second);
    CATCH_REQUIRE(cell.state_has_block());
  } /* for(i..) */

  map_sanity_check(map, 0);
  map_resolution_check(map);
}

CATCH_TEST_CASE("block-move-test", "[arena_map]") {
  argos::CRandom::CreateCategory("argos", 789);
  arena_map map(&gparams, nest_x, nest_y);
  map.distribute_blocks(true);
  map_sanity_check(map, 0);
  map_resolution_check(map);

  /*
   * Simulate a robot picking up, carrying, and then dropping a block, verifying
   * the arena map is in the current state all the way.
   */
  for (size_t i = 0; i < map.blocks().size(); ++i) {
    printf("Test moving block %d\n", i);
    CATCH_REQUIRE(map.blocks()[i].id() != -1);
    cell2D& old_cell = map.access(map.blocks()[i].dloc().first,
                              map.blocks()[i].dloc().second);

    events::block_pickup op1(rcppsw::er::g_server, &map.blocks()[i], 0);
    map.accept(op1);
    CATCH_REQUIRE(old_cell.state_is_empty());
    CATCH_REQUIRE(map.blocks()[i].robot_index() != -1);
    events::block_drop op2(rcppsw::er::g_server, &map.blocks()[i]);
    map.accept(op2);
    cell2D& new_cell = map.access(map.blocks()[i].dloc().first,
                                  map.blocks()[i].dloc().second);
    CATCH_REQUIRE(new_cell.state_has_block());

    map_sanity_check(map, 0);
    map_resolution_check(map);
  } /* for(i..) */
}
