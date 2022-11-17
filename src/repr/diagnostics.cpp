/**
 * \file diagnostics.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/repr/diagnostics.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, repr, diagnostics);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
chactuators::diagnostic_actuator::map_type kColorMap = {
  { chactuators::diagnostics::ekEXPLORE, rutils::color::kMAGENTA },
  { chactuators::diagnostics::ekSUCCESS, rutils::color::kGREEN },
  { chactuators::diagnostics::ekTAXIS, rutils::color::kYELLOW },
  { chactuators::diagnostics::ekLEAVING_NEST, rutils::color::kGRAY50 },
  { chactuators::diagnostics::ekWAIT_FOR_SIGNAL, rutils::color::kWHITE },
  { chactuators::diagnostics::ekVECTOR_TO_GOAL, rutils::color::kBLUE },
  { chactuators::diagnostics::ekEXP_INTERFERENCE, rutils::color::kRED }
};

NS_END(diagnostics, repr, fordyca);
