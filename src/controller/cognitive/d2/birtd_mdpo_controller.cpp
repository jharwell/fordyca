/**
 * \file birtd_mdpo_controller.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/cognitive/d2/birtd_mdpo_controller.hpp"

#include "cosm/arena/repr/base_cache.hpp"
#include "cosm/ds/cell2D.hpp"
#include "cosm/fsm/supervisor_fsm.hpp"
#include "cosm/repr/base_block3D.hpp"

#include "fordyca/controller/config/d2/controller_repository.hpp"
#include "fordyca/subsystem/perception/ds/dpo_semantic_map.hpp"
#include "fordyca/subsystem/perception/mdpo_perception_subsystem.hpp"
#include "fordyca/subsystem/perception/perception_subsystem_factory.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d2);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
birtd_mdpo_controller::birtd_mdpo_controller(void)
    : ER_CLIENT_INIT("fordyca.controller.cognitive.d2.birtd_mdpo") {}

birtd_mdpo_controller::~birtd_mdpo_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void birtd_mdpo_controller::init(ticpp::Element& node) {
  foraging_controller::init(node);
  ndc_uuid_push();
  ER_INFO("Initializing");

  config::d2::controller_repository config_repo;
  config_repo.parse_all(node);
  if (!config_repo.validate_all()) {
    ER_FATAL_SENTINEL("Not all parameters were validated");
    std::exit(EXIT_FAILURE);
  }

  shared_init(config_repo);

  ER_INFO("Initialization finished");
  ndc_uuid_pop();
} /* init() */

void birtd_mdpo_controller::shared_init(
    const config::d2::controller_repository& config_repo) {
  /* block/cache selection matrices, executive  */
  birtd_dpo_controller::shared_init(config_repo);

  /* MDPO perception subsystem */
  auto p = *config_repo.config_get<fspconfig::perception_config>();
  rmath::vector2d padding(p.mdpo.rlos.grid2D.resolution.v() * 5,
                          p.mdpo.rlos.grid2D.resolution.v() * 5);
  p.mdpo.rlos.grid2D.dims += padding;

  auto factory = fsperception::perception_subsystem_factory();
  perception(factory.create(p.type, &p));
} /* shared_init() */

NS_END(cognitive, d2, controller, fordyca);

using namespace fccd2; // NOLINT

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_MISSING_VAR_DECL()
RCPPSW_WARNING_DISABLE_MISSING_PROTOTYPE()
RCPPSW_WARNING_DISABLE_GLOBAL_CTOR()

REGISTER_CONTROLLER(birtd_mdpo_controller,
                    "birtd_mdpo_controller"); // NOLINT

RCPPSW_WARNING_DISABLE_POP()
