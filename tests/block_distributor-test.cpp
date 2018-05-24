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
#include <iostream>
#include <string>
#include "fordyca/support/block_distributor.hpp"
#include "fordyca/params/block_params.hpp"
#include "fordyca/representation/block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
using namespace fordyca::support;
using namespace fordyca::params;
using namespace fordyca::representation;
using namespace fordyca;
using namespace argos;
using namespace std;

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/

block_distributor * bdist_setup(block_params & bparams, double arena_left,
      double arena_right, double arena_bottom, double arena_top, double nest_left,
      double nest_right,  double nest_bottom,  double nest_top) {
  CRange<double> arena_y(arena_bottom, arena_top);
  CRange<double> arena_x(arena_left, arena_right);
  CRange<double> nest_y(nest_bottom, nest_top);
  CRange<double> nest_x(nest_left, nest_right);
  return new block_distributor(arena_y, arena_x, nest_y, nest_x, &bparams);
}

void print_vec_edges() {
  CRandom::CreateCategory("argos", 123);

  block_params bparams;
  bparams.dist_model = "random";
  block_distributor * bd = bdist_setup(bparams, 0, 10, 0, 10, 0, 2, 0, 2);
  block b(1);
  CVector2 vec, br, bl, tr, tl;
  double top, bottom, right, left;

  bd->distribute_block(b, &vec);
  br = bl = tr = tl = vec;
  top = bottom = vec.GetY();
  left = right = vec.GetX();
  for (int i = 0; i < 2000; i ++) {
    bd->distribute_block(b, &vec);
    if (vec.GetX() > right) right = vec.GetX();
    if (vec.GetX() < left) left = vec.GetX();
    if (vec.GetY() > top) top = vec.GetY();
    if (vec.GetY() < bottom) bottom = vec.GetY();
    if (vec.GetX() < bl.GetX() && vec.GetY() < bl.GetY()) bl = vec;
    if (vec.GetX() > br.GetX() && vec.GetY() < br.GetY()) br = vec;
    if (vec.GetX() > tr.GetX() && vec.GetY() > tr.GetY()) tr = vec;
    if (vec.GetX() < tl.GetX() && vec.GetY() > tl.GetY()) tl = vec;
  }
  cout << "\ncorners\ntop left vec: " << tl << endl;
  cout << "top right vec: " << tr << endl;
  cout << "bottom left vec: " << bl << endl;
  cout << "bottom right vec: " << br << endl;
  cout << "\nbounds\ntop: " << top << endl;
  cout << "bottom: " << bottom << endl;
  cout << "right: " << right << endl;
  cout << "left: " << left << endl;
  delete bd;
}

/*******************************************************************************
 * Test Cases
 ******************************************************************************/

CATCH_TEST_CASE("no-nest-random-inside-bounds", "[block_distributor]") {
  CRandom::CreateCategory("argos", 123);

  block_params bparams;
  bparams.dist_model = "random";
  block_distributor * bdist = bdist_setup(bparams, 0, 10, 0, 10, 0, 0, 0, 0);
  block b(1);
  CVector2 vec;
  for (int i = 0; i < 200; i++) {
    CATCH_REQUIRE(bdist->distribute_block(b, &vec));
    CATCH_REQUIRE(vec.GetX() >= 1);
    CATCH_REQUIRE(vec.GetX() <= 9);
    CATCH_REQUIRE(vec.GetY() >= 1);
    CATCH_REQUIRE(vec.GetY() <= 9);
  }
  delete bdist;
}

CATCH_TEST_CASE("bottom-left-corner-nest-random-inside-bounds", "[block_distributor]") {
  CRandom::CreateCategory("argos", 123);
  block_params bparams;
  bparams.dist_model = "random";
  block_distributor * bdist = bdist_setup(bparams, 0, 10, 0, 10, 0, 4, 0, 4);
  block b(1);
  CVector2 vec;
  for (int i = 0; i < 200; i++) {
    CATCH_REQUIRE(bdist->distribute_block(b, &vec));

    CATCH_REQUIRE(( (vec.GetX() >= 5 && vec.GetX() <= 9) ||
                    (vec.GetX() >= 1 && vec.GetY() >= 5 && vec.GetX() <= 9) ));

    CATCH_REQUIRE(( (vec.GetY() >= 5 && vec.GetY() <= 9) ||
                    (vec.GetY() >= 1 && vec.GetX() >= 5 && vec.GetY() <= 9) ));
  }
  delete bdist;
}

CATCH_TEST_CASE("upper-left-corner-nest-random-inside-bounds", "[block_distributor]") {
  CRandom::CreateCategory("argos", 123);
  block_params bparams;
  bparams.dist_model = "random";
  block_distributor * bdist = bdist_setup(bparams, 0, 10, 0, 10, 0, 4, 6, 10);
  block b(1);
  CVector2 vec;
  for (int i = 0; i < 200; i++) {
    CATCH_REQUIRE(bdist->distribute_block(b, &vec));

    CATCH_REQUIRE(( (vec.GetX() >= 5 && vec.GetX() <= 9) ||
                    (vec.GetX() >= 1 && vec.GetY() <= 5 && vec.GetX() <= 9) ));

    CATCH_REQUIRE(( (vec.GetY() <= 5 && vec.GetY() >= 1) ||
                    (vec.GetY() <= 9 && vec.GetX() >= 5 && vec.GetY() >= 1) ));
  }
  delete bdist;
}

CATCH_TEST_CASE("upper-right-corner-nest-random-inside-bounds", "[block_distributor]") {
  CRandom::CreateCategory("argos", 123);
  block_params bparams;
  bparams.dist_model = "random";
  block_distributor * bdist = bdist_setup(bparams, 0, 10, 0, 10, 6, 10, 6, 10);
  block b(1);
  CVector2 vec;
  for (int i = 0; i < 200; i++) {
    CATCH_REQUIRE(bdist->distribute_block(b, &vec));

    CATCH_REQUIRE(( (vec.GetX() <= 5 && vec.GetX() >= 1) ||
                    (vec.GetX() <= 9 && vec.GetY() <= 5 && vec.GetX() >= 1) ));

    CATCH_REQUIRE(( (vec.GetY() <= 5 && vec.GetY() >= 1) ||
                    (vec.GetY() <= 9 && vec.GetX() <= 5 && vec.GetY() >= 1) ));
  }
  delete bdist;
}

CATCH_TEST_CASE("bottom-right-corner-nest-random-inside-bounds", "[block_distributor]") {
  CRandom::CreateCategory("argos", 123);
  block_params bparams;
  bparams.dist_model = "random";
  block_distributor * bdist = bdist_setup(bparams, 0, 10, 0, 10, 6, 10, 0, 4);
  block b(1);
  CVector2 vec;
  for (int i = 0; i < 200; i++) {
    CATCH_REQUIRE(bdist->distribute_block(b, &vec));

    CATCH_REQUIRE(( (vec.GetX() <= 5 && vec.GetX() >= 1) ||
                    (vec.GetX() <= 9 && vec.GetY() >= 5 && vec.GetX() >= 1) ));

    CATCH_REQUIRE(( (vec.GetY() >= 5 && vec.GetY() <= 9) ||
                    (vec.GetY() >= 1 && vec.GetX() <= 5 && vec.GetY() <= 9) ));
  }
  delete bdist;
}

/*
CATCH_TEST_CASE("middle-nest-random-inside-bounds", "[block_distributor]") {
  CRandom::CreateCategory("argos", 123);
  block_params bparams;
  bparams.dist_model = "random";
  block_distributor * bdist = bdist_setup(bparams, 0, 10, 0, 10, 3, 7, 3, 7);
  block b(1);
  CVector2 vec;
  for (int i = 0; i < 200; i++) {
    CATCH_REQUIRE(bdist->distribute_block(b, &vec));

    CATCH_REQUIRE(( (vec.GetX() < 2 && vec.GetY() >= 1 && vec.GetY() <= 9) ||
                    (vec.GetX() > 8 && vec.GetY() >= 1 && vec.GetY() <= 9) ||
                    (vec.GetY() < 2 && vec.GetX() >= 1 && vec.GetX() <= 9) ||
                    (vec.GetY() > 8 && vec.GetX() >= 1 && vec.GetX() <= 9) ));
  }
  delete bdist;
}
*/

CATCH_TEST_CASE("no-nest-single-source", "[block_distributor]") {
  CRandom::CreateCategory("argos", 123);
  block_params bparams;
  bparams.dist_model = "single_source";
  block_distributor * bdist = bdist_setup(bparams, 0, 10, 0, 10, 0, 0, 0, 0);
  block b(1);
  CVector2 vec;
  for (int i = 0; i < 200; i++) {
    // the following requires need to be tweaked once I know exactly what the
    // intended behavior for single-source is
    CATCH_REQUIRE(bdist->distribute_block(b, &vec));
    CATCH_REQUIRE(vec.GetY() <= 9.5);
    CATCH_REQUIRE(vec.GetY() >= 0.5);
    CATCH_REQUIRE(vec.GetX() <= 9);
    CATCH_REQUIRE(vec.GetX() >= 8.25);
  }
  delete bdist;
}

CATCH_TEST_CASE("left-edge-nest-single-source", "[block_distributor]") {
  CRandom::CreateCategory("argos", 123);
  block_params bparams;
  bparams.dist_model = "single_source";
  block_distributor * bdist = bdist_setup(bparams, 0, 10, 0, 10, 0, 2, 0, 10);
  block b(1);
  CVector2 vec;
  for (int i = 0; i < 200; i++) {
    // the following requires need to be tweaked once I know exactly what the
    // intended behavior for single-source is
    CATCH_REQUIRE(bdist->distribute_block(b, &vec));
    CATCH_REQUIRE(vec.GetY() <= 9.5);
    CATCH_REQUIRE(vec.GetY() >= 0.5);
    CATCH_REQUIRE(vec.GetX() <= 9);
    CATCH_REQUIRE(vec.GetX() >= 8.25);
  }
  delete bdist;
}
