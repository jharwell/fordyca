/**
 * @file perceived_cell2D-test.cpp
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
#include "fordyca/representation/perceived_cell2D.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/events/cell_unknown.hpp"
#include "fordyca/events/block_found.hpp"
#include "fordyca/events/cache_found.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
using namespace fordyca::representation;
using namespace fordyca;

/*******************************************************************************
 * Test Cases
 ******************************************************************************/
CATCH_TEST_CASE("init-test", "[perceived_cell2D]") {
  perceived_cell2D cell(rcppsw::er::g_server);
  CATCH_REQUIRE(!cell.state_is_known());
  CATCH_REQUIRE(cell.density() == 0.0);
}

CATCH_TEST_CASE("transition-test", "[perceived_cell2D]") {
  perceived_cell2D cell(rcppsw::er::g_server);
  cell.rho(0.1);
  events::cell_empty op;
  cell.accept(op);
  CATCH_REQUIRE(cell.state_is_known());
  CATCH_REQUIRE(cell.state_is_empty());
  CATCH_REQUIRE(cell.density() < cell.epsilon());

  /*
   * Verify that pheromone decreases as expected according to the value of rho.
   */
  cell.add_pheromone(1.0);
  cell.update_density();
  for (size_t i = 0; i < 98; ++i) {
    double prev = cell.density();
    cell.update_density();
    CATCH_REQUIRE(cell.density() - 0.9 * prev < cell.epsilon());
  } /* for(i..) */
  CATCH_REQUIRE(!cell.state_is_known());

  /*
   * Verify that the same thing happens when a block is discovered.
   */
  block* b = new block(0.2);
  events::block_found op2(rcppsw::er::g_server, b);
  cell.accept(op2);
  CATCH_REQUIRE(cell.state_is_known());
  CATCH_REQUIRE(cell.state_has_block());
  CATCH_REQUIRE(cell.density() > 1.0);

  for (size_t i = 0; i < 98; ++i) {
    double prev = cell.density();
    cell.update_density();
    CATCH_REQUIRE(cell.density() - 0.9 * prev < cell.epsilon());
  } /* for(i..) */
  CATCH_REQUIRE(!cell.state_is_known());

  std::list<block*> list;
  list.push_back(b);
  list.push_back(b);
  cache* c = new cache(0.2, argos::CVector2(0.0, 0.0), list);
  events::cache_found op3(rcppsw::er::g_server, c);
  cell.accept(op2);
  cell.accept(op3);
  CATCH_REQUIRE(cell.state_is_known());
  CATCH_REQUIRE(cell.state_has_cache());
  CATCH_REQUIRE(cell.density() > 1.0);

  for (size_t i = 0; i < 98; ++i) {
    double prev = cell.density();
    cell.update_density();
    CATCH_REQUIRE(cell.density() - 0.9 * prev < cell.epsilon());
  } /* for(i..) */
  CATCH_REQUIRE(!cell.state_is_known());
  delete b;
}
