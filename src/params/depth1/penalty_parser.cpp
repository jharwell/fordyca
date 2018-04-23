/**
 * @file penalty_parser.cpp
 *
 * @copyright 2018 John Harwell/Anthony Chen, All rights reserved.
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
 #include "fordyca/params/depth1/penalty_parser.hpp"
 #include "rcppsw/utils/line_parser.hpp"

 /*******************************************************************************
  * Namespaces
  ******************************************************************************/
 NS_START(fordyca, params);

 /*******************************************************************************
  * Member Functions
  ******************************************************************************/
 void penalty_parser::parse(argos::TConfigurationNode& node) {
   m_params = rcppsw::make_unique<struct penalty_params>();

   char* penstr;
   argos::GetNodeAttribute(node, "pen_func", penstr);
   if (penstr == "kSine") m_params->pen_func = kSine;
   else if(penstr == "kSquare") m_params->pen_func = kSquare;
   else if(penstr == "kStep") m_params->pen_func = kStep;
   else if(penstr == "kSaw") m_params->pen_func = kSaw;
   argos::GetNodeAttribute(node, "amp", m_params->amp);
   argos::GetNodeAttribute(node, "per", m_params->per);
   argos::GetNodeAttribute(node, "phase", m_params->phase);
   argos::GetNodeAttribute(node, "square", m_params->square);
   argos::GetNodeAttribute(node, "step", m_params->step);
   argos::GetNodeAttribute(node, "saw", m_params->saw);
 } /* parse() */

 void penalty_parser::show(std::ostream& stream) {
   stream << "====================\nCache params\n====================\n";
   stream << "pen_func=" << m_params->pen_func << std::endl;
   stream << "amp" << m_params->amp << std::endl;
   stream << "per=" << m_params->per << std::endl;
   stream << "phase=" << m_params->phase << std::endl;
   stream << "square=" << m_params->square << std::endl;
   stream << "step=" << m_params->step << std::endl;
   stream << "saw=" << m_params->saw << std::endl;
 } /* show() */

 __pure bool penalty_parser::validate(void) {
   if (m_params->amp <= 0.0) {
     return false;
   }
   if (m_params->per <= 0.0) {
     return false;
   }
   if (m_params->phase <= 0.0) {
     return false;
   }
   if (m_params->square <= 0.0) {
     return false;
   }
   if (m_params->step <= 0.0) {
     return false;
   }
   if (m_params->saw <= 0.0) {
     return false;
   }
   return true;
 } /* validate() */

 NS_END(params, fordyca);
