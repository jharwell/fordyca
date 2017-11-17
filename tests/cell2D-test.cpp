/**
 * @file cell2D-test.cpp
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
