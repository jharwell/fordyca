/**
 * \file foraging_perception_subsystem.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <utility>

#include "rcppsw/metrics/base_metrics.hpp"

#include "cosm/subsystem/perception/mlos_perception_subsystem.hpp"

#include "fordyca/fordyca.hpp"
#include "fordyca/repr/forager_los.hpp"
#include "fordyca/subsystem/perception/known_objects_accessor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, subsystem, perception);

class oracular_info_receptor;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class foraging_perception_subsystem
 * \ingroup subsystem perception
 *
 * \brief Base class for robot perception common to all foraging controllers,
 * which is just the \ref dpo_store of objects.
 */
class foraging_perception_subsystem
    : public csperception::mlos_perception_subsystem<repr::forager_los>,
      public virtual rmetrics::base_metrics {
 public:
  explicit foraging_perception_subsystem(
      const csprlos::config::rlos_config* const config,
      std::unique_ptr<csperception::base_memory_model> model)
      : mlos_perception_subsystem(config, std::move(model)) {}

  ~foraging_perception_subsystem(void) override = default;

  /**
   * \brief Update the internal data structure/repr of the
   * environment/arena, after the LOS has been updated.
   *
   * \param receptor Handle to \ref oracular_info_receptor containing perfect
   * information about the environment. If non-NULL, bypasses LOS updating (why
   * use LOS when you have perception information?).
   */
  virtual void update(oracular_info_receptor* receptor) = 0;

  /**
   * \brief Get access to all objects known to the robot, independent of
   * perception model.
   */
  virtual const known_objects_accessor* known_objects(void) const = 0;

  /**
   * \brief Obtain a handle on the perception model, e.g., to access the
   * tracked objects.
   */
  template<typename TModelType>
  const TModelType* model(void) const {
    return static_cast<const TModelType*>(mlos_perception_subsystem::model());
  }
  template<typename TModelType>
  TModelType* model(void) {
    return static_cast<TModelType*>(mlos_perception_subsystem::model());
  }
};

NS_END(perception, subsystem, fordyca);

