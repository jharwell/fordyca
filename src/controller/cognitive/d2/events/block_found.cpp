/**
 * \file block_found.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/cognitive/d2/events/block_found.hpp"

#include "fordyca/controller/cognitive/d2/birtd_dpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_mdpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_odpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_omdpo_controller.hpp"
#include "fordyca/subsystem/perception/dpo_perception_subsystem.hpp"
#include "fordyca/subsystem/perception/ds/dpo_semantic_map.hpp"
#include "fordyca/subsystem/perception/events/block_found.hpp"
#include "fordyca/subsystem/perception/mdpo_perception_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d2, events);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_found::block_found(crepr::sim_block3D* block)
    : ER_CLIENT_INIT("fordyca.controller.cognitive.d2.events.block_found"),
      m_block(block) {}

/*******************************************************************************
 * Depth2 Foraging
 ******************************************************************************/
void block_found::visit(controller::cognitive::d2::birtd_mdpo_controller& c) {
  c.ndc_uuid_push();

  fspevents::block_found found(m_block);
  found.visit(*c.perception()->model<fspds::dpo_semantic_map>());

  c.ndc_uuid_pop();
} /* visit() */

void block_found::visit(controller::cognitive::d2::birtd_dpo_controller& c) {
  c.ndc_uuid_push();

  fspevents::block_found found(m_block);
  found.visit(*c.perception()->model<fspds::dpo_store>());

  c.ndc_uuid_pop();
} /* visit() */

void block_found::visit(controller::cognitive::d2::birtd_omdpo_controller& c) {
  c.ndc_uuid_push();

  fspevents::block_found found(m_block);
  found.visit(*c.perception()->model<fspds::dpo_semantic_map>());

  c.ndc_uuid_pop();
} /* visit() */

void block_found::visit(controller::cognitive::d2::birtd_odpo_controller& c) {
  c.ndc_uuid_push();

  fspevents::block_found found(m_block);
  found.visit(*c.perception()->model<fspds::dpo_store>());

  c.ndc_uuid_pop();
} /* visit() */

NS_END(events, d2, cognitive, controller, fordyca);
