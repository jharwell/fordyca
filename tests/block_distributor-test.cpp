/**
 * @file block_distributor-test.cpp
 *
 * @copyright 2018 Aaron Koenigsberg, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
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
#include "fordyca/params/block_distribution_params.hpp"
#include "fordyca/representation/block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
using namespace fordyca;
using namespace argos;
using namespace fordyca::support;

/*******************************************************************************
 * Forward Decls
 ******************************************************************************/
void print_vec_edges(void);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
representation::extent_model g_arena = {"rectangle",
                                        "horizontal",
                                        {0, 10},
                                        {0, 10}};

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/
/* void print_vec_edges() { */
/*   CRandom::CreateCategory("argos", 123); */

/*   params::block_distribution_params bparams; */
/*   bparams.dist_type = "random"; */
/*   representation::extent_model  */
/*   block_distributor * bd = bdist_setup(bparams, g_arena, {0, 2, 0, 2}); */
/*   block b(1); */
/*   CVector2 vec, br, bl, tr, tl; */
/*   double top, bottom, right, left; */

/*   bd->distribute_block(b, &vec); */
/*   br = bl = tr = tl = vec; */
/*   top = bottom = vec.GetY(); */
/*   left = right = vec.GetX(); */
/*   for (int i = 0; i < 2000; i ++) { */
/*     bd->distribute_block(b, &vec); */
/*     if (vec.GetX() > right) right = vec.GetX(); */
/*     if (vec.GetX() < left) left = vec.GetX(); */
/*     if (vec.GetY() > top) top = vec.GetY(); */
/*     if (vec.GetY() < bottom) bottom = vec.GetY(); */
/*     if (vec.GetX() < bl.GetX() && vec.GetY() < bl.GetY()) bl = vec; */
/*     if (vec.GetX() > br.GetX() && vec.GetY() < br.GetY()) br = vec; */
/*     if (vec.GetX() > tr.GetX() && vec.GetY() > tr.GetY()) tr = vec; */
/*     if (vec.GetX() < tl.GetX() && vec.GetY() > tl.GetY()) tl = vec; */
/*   } */
/*   cout << "\ncorners\ntop left vec: " << tl << endl; */
/*   cout << "top right vec: " << tr << endl; */
/*   cout << "bottom left vec: " << bl << endl; */
/*   cout << "bottom right vec: " << br << endl; */
/*   cout << "\nbounds\ntop: " << top << endl; */
/*   cout << "bottom: " << bottom << endl; */
/*   cout << "right: " << right << endl; */
/*   cout << "left: " << left << endl; */
/*   delete bd; */
/* } */

/*******************************************************************************
 * Test Cases
 ******************************************************************************/

CATCH_TEST_CASE("no-nest-inside-bounds", "[random]") {
  CRandom::CreateCategory("argos", 123);

  representation::extent_model nest = {"rectangle",
                                       "horizontal",
                                       {0, 0},
                                       {0, 0}};
  params::block_distribution_params bparams;
  bparams.dist_type = "random";
  bparams.nest_model = nest;
  bparams.arena_model = g_arena;
  block_distributor * bdist = new block_distributor(&bparams);

  representation::block b(1);
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

CATCH_TEST_CASE("bottom-left-corner-nest-inside-bounds", "[random]") {
  CRandom::CreateCategory("argos", 123);

  representation::extent_model nest = {"rectangle",
                                       "horizontal",
                                       {0, 4},
                                       {0, 4}};
  params::block_distribution_params bparams;
  bparams.dist_type = "random";
  bparams.nest_model = nest;
  bparams.arena_model = g_arena;
  block_distributor * bdist = new block_distributor(&bparams);

  representation::block b(1);
  CVector2 vec;
  for (int i = 0; i < 200; i++) {
    CATCH_REQUIRE(bdist->distribute_block(b, &vec));

    if (vec.GetX() <= 5.0) {
      CATCH_REQUIRE(vec.GetY() >= 5);
    } else {
      CATCH_REQUIRE((vec.GetY() >= 1 && vec.GetY() <= 9));
    }
    if (vec.GetY() <= 5.0) {
      CATCH_REQUIRE((vec.GetX() >= 5 && vec.GetX() <= 9));
    } else {
      CATCH_REQUIRE((vec.GetX() >= 1 && vec.GetY() <= 9));
    }
  }
  delete bdist;
}

CATCH_TEST_CASE("upper-left-corner-nest-inside-bounds", "[random]") {
  CRandom::CreateCategory("argos", 123);
  representation::extent_model nest = {"rectangle",
                                       "horizontal",
                                       {0, 4},
                                       {6, 10}};
  params::block_distribution_params bparams;
  bparams.dist_type = "random";
  bparams.nest_model = nest;
  bparams.arena_model = g_arena;
  block_distributor * bdist = new block_distributor(&bparams);

  representation::block b(1);
  CVector2 vec;
  for (int i = 0; i < 200; i++) {
    CATCH_REQUIRE(bdist->distribute_block(b, &vec));

    if (vec.GetX() <= 5.0) {
      CATCH_REQUIRE(vec.GetY() <= 5);
    } else {
      CATCH_REQUIRE((vec.GetY() >= 1 && vec.GetY() <= 9));
    }
    if (vec.GetY() <= 5.0) {
      CATCH_REQUIRE((vec.GetX() >= 1 && vec.GetY() <= 9));
    } else {
      CATCH_REQUIRE((vec.GetX() >= 5 && vec.GetX() <= 9));
    }
  }
  delete bdist;
}

CATCH_TEST_CASE("upper-right-corner-nest-inside-bounds", "[random]") {
  CRandom::CreateCategory("argos", 123);
  representation::extent_model nest = {"rectangle",
                                       "horizontal",
                                       {6, 10},
                                       {6, 10}};
  params::block_distribution_params bparams;
  bparams.dist_type = "random";
  bparams.nest_model = nest;
  bparams.arena_model = g_arena;
  block_distributor * bdist = new block_distributor(&bparams);

  representation::block b(1);

  CVector2 vec;
  for (int i = 0; i < 200; i++) {
    CATCH_REQUIRE(bdist->distribute_block(b, &vec));
    if (vec.GetX() >= 5.0) {
      CATCH_REQUIRE((vec.GetY() <= 5 && vec.GetY() >= 1.0));
    } else {
      CATCH_REQUIRE((vec.GetY() >= 1 && vec.GetY() <= 9));
    }
    if (vec.GetY() >= 5.0) {
      CATCH_REQUIRE((vec.GetX() <= 5 && vec.GetX() >= 1.0));
    } else {
      CATCH_REQUIRE((vec.GetX() >= 1 && vec.GetY() <= 9));
    }
  } /* for(i..) */
  delete bdist;
}

CATCH_TEST_CASE("bottom-right-corner-nest-inside-bounds", "[random]") {
  CRandom::CreateCategory("argos", 123);
  representation::extent_model nest = {"rectangle",
                                       "horizontal",
                                       {6, 10},
                                       {0, 4}};
  params::block_distribution_params bparams;
  bparams.dist_type = "random";
  bparams.nest_model = nest;
  bparams.arena_model = g_arena;
  representation::block b(1);
  block_distributor * bdist = new block_distributor(&bparams);
  CVector2 vec;

  for (int i = 0; i < 200; i++) {
    CATCH_REQUIRE(bdist->distribute_block(b, &vec));
    if (vec.GetX() >= 5.0) {
      CATCH_REQUIRE(vec.GetY() >= 5);
    } else {
      CATCH_REQUIRE((vec.GetY() >= 1 && vec.GetY() <= 9));
      CATCH_REQUIRE((vec.GetX() >= 1));
    }
  } /* for(i..) */
  delete bdist;
}


/* Causes program to loop indefinitely */
CATCH_TEST_CASE("middle-nest-inside-bounds", "[random]") {
  CRandom::CreateCategory("argos", 123);
  argos::CRange<double> nest_x{4, 6};
  argos::CRange<double> nest_y{4, 6};
  representation::extent_model nest = {"rectangle",
                                       "horizontal",
                                       nest_x,
                                       nest_y};
  params::block_distribution_params bparams;
  bparams.dist_type = "random";
  bparams.nest_model = nest;
  bparams.arena_model = g_arena;
  block_distributor * bdist = new block_distributor(&bparams);

  representation::block b(1);
  CVector2 vec;
  for (int i = 0; i < 200; i++) {
    CATCH_REQUIRE(bdist->distribute_block(b, &vec));
    CATCH_REQUIRE(!nest_x.WithinMinBoundIncludedMaxBoundIncluded(vec.GetX()));
    CATCH_REQUIRE(!nest_y.WithinMinBoundIncludedMaxBoundIncluded(vec.GetY()));
  }
  delete bdist;
}

CATCH_TEST_CASE("no-nest", "[single_source]") {
  CRandom::CreateCategory("argos", 123);
  representation::extent_model nest = {"rectangle",
                                       "horizontal",
                                       {0, 0},
                                       {0, 0}};
  params::block_distribution_params bparams;
  bparams.dist_type = "single_source";
  bparams.nest_model = nest;
  bparams.arena_model = g_arena;

  block_distributor * bdist = new block_distributor(&bparams);
  representation::block b(1);
  CVector2 vec;
  for (int i = 0; i < 200; i++) {
    // the following requires need to be tweaked once I know exactly what the
    // intended behavior for single-source is
    CATCH_REQUIRE(bdist->distribute_block(b, &vec));
    CATCH_REQUIRE(vec.GetY() <= 9.0);
    CATCH_REQUIRE(vec.GetY() >= 1.0);
    CATCH_REQUIRE(vec.GetX() <= 9);
    CATCH_REQUIRE(vec.GetX() >= 5);
  }
  delete bdist;
}

CATCH_TEST_CASE("left-edge-nest", "[single_source]") {
  CRandom::CreateCategory("argos", 123);
  representation::extent_model nest = {"rectangle",
                                       "horizontal",
                                       {0, 2},
                                       {0, 10}};
  params::block_distribution_params bparams;
  bparams.nest_model = nest;
  bparams.arena_model = g_arena;
  bparams.dist_type = "single_source";

  block_distributor * bdist = new block_distributor(&bparams);
  representation::block b(1);
  CVector2 vec;

  for (int i = 0; i < 200; i++) {
    // the following requires need to be tweaked once I know exactly what the
    // intended behavior for single-source is
    CATCH_REQUIRE(bdist->distribute_block(b, &vec));
    CATCH_REQUIRE(vec.GetY() <= 9.0);
    CATCH_REQUIRE(vec.GetY() >= 1.0);
    CATCH_REQUIRE(vec.GetX() <= 9);
    CATCH_REQUIRE(vec.GetX() >= 5.0);
  }
  delete bdist;
}
