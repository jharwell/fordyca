/**
 * @file cell2D-test.cpp
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
#include "fordyca/representation/cell2D.hpp"
#include "fordyca/events/block_drop.hpp"
#include "fordyca/events/block_pickup.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
using namespace fordyca::representation;
using namespace fordyca::events;

/*******************************************************************************
 * Test Cases
 ******************************************************************************/
CATCH_TEST_CASE("init-test", "[cell2D]") {
  cell2D cell;
  CATCH_REQUIRE(!cell.state_is_known());
}

CATCH_TEST_CASE("transition-test", "[cell2D]") {
  cell2D cell;
  block b(0.2);
  block_drop(rcppsw::er::g_server, &b);
  cell.fsm().event_unknown();
  CATCH_REQUIRE(!cell.state_is_known());
  cell.fsm().event_empty();
  CATCH_REQUIRE(cell.state_is_empty());
  cell.fsm().event_block_drop();
  CATCH_REQUIRE(cell.state_has_block());
  cell.fsm().event_block_pickup();
  CATCH_REQUIRE(cell.state_is_empty());
  cell.fsm().event_unknown();
  CATCH_REQUIRE(!cell.state_is_known());
}

CATCH_TEST_CASE("cache-test", "[cell2D]") {
  cell2D cell;
  block b(0.2);
  block_drop(rcppsw::er::g_server, &b);

  cell.fsm().event_block_drop();
  cell.fsm().event_block_drop();
  CATCH_REQUIRE(cell.state_has_cache());
  cell.fsm().event_block_pickup();
  CATCH_REQUIRE(cell.state_has_block());
  cell.fsm().event_block_drop();
  CATCH_REQUIRE(cell.state_has_cache());
  cell.fsm().event_block_drop();
  CATCH_REQUIRE(cell.state_has_cache());
  cell.fsm().event_block_pickup();
  CATCH_REQUIRE(cell.state_has_cache());
  cell.fsm().event_block_pickup();
  CATCH_REQUIRE(cell.state_has_block());
  cell.fsm().event_block_pickup();
  CATCH_REQUIRE(cell.state_is_empty());
}
