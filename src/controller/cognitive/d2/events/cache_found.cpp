/**
 * \file cache_found.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
#include "fordyca/controller/cognitive/d2/events/cache_found.hpp"

#include "fordyca/controller/cognitive/d2/birtd_dpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_mdpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_odpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_omdpo_controller.hpp"
#include "fordyca/subsystem/perception/dpo_perception_subsystem.hpp"
#include "fordyca/subsystem/perception/mdpo_perception_subsystem.hpp"
#include "fordyca/subsystem/perception/ds/dpo_semantic_map.hpp"
#include "fordyca/subsystem/perception/events/cache_found.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d2, events);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_found::cache_found(carepr::base_cache* cache)
    : ER_CLIENT_INIT("fordyca.controller.cognitive.d2.events.cache_found"),
      m_cache(cache) {}

/*******************************************************************************
 * Controllers
 ******************************************************************************/
void cache_found::visit(fccd2::birtd_mdpo_controller& c) {
  c.ndc_pusht();

  fspevents::cache_found found(m_cache);
  found.visit(*c.perception()->model<fspds::dpo_semantic_map>());

  c.ndc_pop();
} /* visit() */

void cache_found::visit(fccd2::birtd_dpo_controller& c) {
  c.ndc_pusht();

  fspevents::cache_found found(m_cache);
  found.visit(*c.perception()->model<fspds::dpo_store>());

  c.ndc_pop();
} /* visit() */

void cache_found::visit(fccd2::birtd_omdpo_controller& c) {
  c.ndc_pusht();

  fspevents::cache_found found(m_cache);
  found.visit(*c.perception()->model<fspds::dpo_semantic_map>());

  c.ndc_pop();
} /* visit() */

void cache_found::visit(fccd2::birtd_odpo_controller& c) {
  c.ndc_pusht();

  fspevents::cache_found found(m_cache);
  found.visit(*c.perception()->model<fspds::dpo_store>());

  c.ndc_pop();
} /* visit() */

NS_END(events, d2, cognitive, controller, fordyca);
