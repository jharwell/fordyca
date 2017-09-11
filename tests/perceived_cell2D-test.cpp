/**
 * @file perceived_cell2D-test.cpp
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
#include "fordyca/representation/perceived_cell2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
using namespace fordyca::representation;

/*******************************************************************************
 * Test Cases
 ******************************************************************************/
CATCH_TEST_CASE("init-test", "[perceived_cell2D]") {
  perceived_cell2D cell;
  CATCH_REQUIRE(!cell.state_is_known());
  CATCH_REQUIRE(cell.relevance() == 0.0);
}

CATCH_TEST_CASE("transition-test", "[perceived_cell2D]") {
  perceived_cell2D cell;
  cell.delta(0.1);
  cell.event_encounter(cell2D_fsm::ST_EMPTY);
  CATCH_REQUIRE(cell.state_is_known());
  CATCH_REQUIRE(cell.state_is_empty());
  CATCH_REQUIRE(std::fabs(cell.relevance() - 1.0) < perceived_cell2D::kEpsilon);

  for (size_t i = 0; i < 10; ++i) {
    cell.update_relevance();
    CATCH_REQUIRE(cell.relevance() - 1.0 - 0.1 * (i+1) < 0.00001);
  } /* for(i..) */

  CATCH_REQUIRE(!cell.state_is_known());

  cell.event_encounter(cell2D_fsm::ST_HAS_BLOCK);
  CATCH_REQUIRE(cell.state_is_known());
  CATCH_REQUIRE(cell.state_has_block());
  CATCH_REQUIRE(std::fabs(cell.relevance() - 1.0) < perceived_cell2D::kEpsilon);

  for (size_t i = 0; i < 10; ++i) {
    cell.update_relevance();
    CATCH_REQUIRE(cell.relevance() - 1.0 - 0.1 * (i+1) < 0.00001);
  } /* for(i..) */

  CATCH_REQUIRE(!cell.state_is_known());
}
