/**
 * @file line_of_sight-test.cpp
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
#include "fordyca/representation/line_of_sight.hpp"
#include "fordyca/representation/arena_map.hpp"
#include "fordyca/params/grid_params.hpp"

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
 * Test Cases
 ******************************************************************************/
CATCH_TEST_CASE("init-test", "[line_of_sight]") {
  argos::CRandom::CreateCategory("argos", 123);
  arena_map map(&gparams, nest_x, nest_y);
  line_of_sight los(map.subgrid(1, 1, 1), rcppsw::math::dcoord2(1, 1));
}

CATCH_TEST_CASE("resolution-test", "[line_of_sight]") {
  argos::CRandom::CreateCategory("argos", 123);
  arena_map map(&gparams, nest_x, nest_y);
  line_of_sight los(map.subgrid(5, 5, 1), rcppsw::math::dcoord2(1, 1));
  CATCH_REQUIRE(los.size() == 9);

  line_of_sight los2(map.subgrid(2, 2, 2), rcppsw::math::dcoord2(0, 0));
  CATCH_REQUIRE(los2.size() == 25);
}
