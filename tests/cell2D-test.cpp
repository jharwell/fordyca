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

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
using namespace fordyca::representation;

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
 * Test Cases
 ******************************************************************************/
CATCH_TEST_CASE("init-test", "[cell2D]") {
  cell2D cell;
  CATCH_REQUIRE(!cell.state_is_known());
}

CATCH_TEST_CASE("transition-test", "[cell2D]") {
  cell2D cell;
  cell.event_unknown();
  CATCH_REQUIRE(!cell.state_is_known());
  cell.event_empty();
  CATCH_REQUIRE(cell.state_is_empty());
  cell.event_has_block();
  CATCH_REQUIRE(cell.state_has_block());
  cell.event_empty();
CATCH_REQUIRE(cell.state_is_empty());
  cell.event_unknown();
CATCH_REQUIRE(!cell.state_is_known());
}
