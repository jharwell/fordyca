/**
 * @file line_of_sight-test.cpp
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
#include "fordyca/representation/line_of_sight.hpp"
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
 * Test Cases
 ******************************************************************************/
CATCH_TEST_CASE("init-test", "[line_of_sight]") {
  argos::CRandom::CreateCategory("argos", 123);
  arena_map map(&params, nest_x, nest_y);
  line_of_sight los(map.subgrid(1, 1, 0.2));
}

CATCH_TEST_CASE("resolution-test", "[line_of_sight]") {
  argos::CRandom::CreateCategory("argos", 123);
  arena_map map(&params, nest_x, nest_y);
  line_of_sight los(map.subgrid(1, 1, 0.4));
  CATCH_REQUIRE(los.size() == 16);

  line_of_sight los2(map.subgrid(0.2, 0.2, 0.2));
  CATCH_REQUIRE(los2.size() == 4);
}
