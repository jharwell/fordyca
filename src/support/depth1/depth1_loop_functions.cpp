/**
 * @file depth1_loop_functions.cpp
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
#include <boost/range/adaptor/map.hpp>

/*
 * This is needed because without it boost instantiates static assertions that
 * verify that every possible handler<controller> instantiation is valid, which
 * includes checking for depth1 controllers being valid for new cache drop/cache
 * site drop events. These will not happen in reality (or shouldn't), and if
 * they do it's 100% OK to crash with an exception.
 */
#define BOOST_VARIANT_USE_RELAXED_GET_BY_DEFAULT
#include "fordyca/support/depth1/depth1_loop_functions.hpp"

#include "fordyca/controller/base_controller.hpp"
#include "fordyca/controller/depth1/gp_mdpo_controller.hpp"
#include "fordyca/controller/depth1/ogp_mdpo_controller.hpp"
#include "fordyca/ds/cell2D.hpp"
#include "fordyca/events/existing_cache_interactor.hpp"
#include "fordyca/metrics/blocks/transport_metrics_collector.hpp"
#include "fordyca/params/arena/arena_map_params.hpp"
#include "fordyca/params/oracle_params.hpp"
#include "fordyca/params/output_params.hpp"
#include "fordyca/params/visualization_params.hpp"
#include "fordyca/support/controller_task_extractor.hpp"
#include "fordyca/support/depth1/depth1_metrics_aggregator.hpp"
#include "fordyca/support/depth1/robot_arena_interactor.hpp"
#include "fordyca/support/depth1/static_cache_manager.hpp"
#include "fordyca/support/tasking_oracle.hpp"

#include "rcppsw/metrics/tasks/bi_tdgraph_metrics_collector.hpp"
#include "rcppsw/swarm/convergence/convergence_calculator.hpp"
#include "rcppsw/task_allocation/bi_tdgraph.hpp"
#include "rcppsw/task_allocation/bi_tdgraph_executive.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);
using ds::arena_grid;
namespace rmetrics = rcppsw::metrics;

/*******************************************************************************
 * Template Instantiations
 ******************************************************************************/
template void depth1_loop_functions::controller_configure(
    controller::depth1::gp_dpo_controller& controller);
template void depth1_loop_functions::controller_configure(
    controller::depth1::gp_mdpo_controller& controller);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
/**
 * @struct controller_configurer
 * @ingroup support depth1
 *
 * @brief Configure a depth1 controller during initialization.
 */
template <class T>
struct controller_configurer {
  using controller_type = T;

  controller_configurer(const params::loop_function_repository* const p,
                        tasking_oracle* const t,
                        depth1_metrics_aggregator* const agg)
      : mc_params(p), m_tasking_oracle(t), m_agg(agg) {}

  void operator()(controller_type* const c) const {
    /*
     * If NULL, then visualization has been disabled.
     */
    auto* vparams =
        mc_params->parse_results<struct params::visualization_params>();
    if (nullptr != vparams) {
      c->display_task(vparams->robot_task);
    }

    auto* oraclep = mc_params->parse_results<params::oracle_params>();
    if (oraclep->enabled) {
      auto oracular = dynamic_cast<controller::depth1::ogp_mdpo_controller*>(c);
      oracular->executive()->task_finish_notify(
          std::bind(&tasking_oracle::task_finish_cb,
                    m_tasking_oracle,
                    std::placeholders::_1));
      oracular->executive()->task_abort_notify(
          std::bind(&tasking_oracle::task_abort_cb,
                    m_tasking_oracle,
                    std::placeholders::_1));
      oracular->tasking_oracle(m_tasking_oracle);
    }
    c->executive()->task_finish_notify(
        std::bind(&depth1_metrics_aggregator::task_finish_or_abort_cb,
                  m_agg,
                  std::placeholders::_1));
    c->executive()->task_abort_notify(
        std::bind(&depth1_metrics_aggregator::task_finish_or_abort_cb,
                  m_agg,
                  std::placeholders::_1));
    c->executive()->task_alloc_notify(
        std::bind(&depth1_metrics_aggregator::task_alloc_cb,
                  m_agg,
                  std::placeholders::_1,
                  std::placeholders::_2));
  }

  const params::loop_function_repository* const mc_params;
  tasking_oracle* const m_tasking_oracle;
  depth1_metrics_aggregator* const m_agg;
};

/**
 * @struct controller_task_extractor_mapper
 * @ingroup support depth1
 *
 * @brief Map a controller instance to the task extraction action run during
 * convergence calculations.
 */
struct controller_task_extractor_mapper : public boost::static_visitor<int> {
  controller_task_extractor_mapper(const controller::base_controller* const c)
      : mc_controller(c) {}

  template <typename T>
  int operator()(const controller_task_extractor<T>& extractor) const {
    return extractor(
        dynamic_cast<const typename controller_task_extractor<T>::controller_type*>(
            mc_controller));
  }
  const controller::base_controller* const mc_controller;
};

/**
 * @struct controller_configurer_mapper
 * @ingroup support depth1
 *
 * @brief Map a particular controller instance to the configuration action run
 * during initialization.
 */
struct controller_configurer_mapper : public boost::static_visitor<void> {
  controller_configurer_mapper(controller::base_controller* const c)
      : mc_controller(c) {}

  using gp_dpo_configurer =
      controller_configurer<controller::depth1::gp_dpo_controller>;
  using gp_mdpo_configurer =
      controller_configurer<controller::depth1::gp_mdpo_controller>;

  void operator()(const gp_dpo_configurer& extractor) const {
    extractor(dynamic_cast<gp_dpo_configurer::controller_type*>(mc_controller));
  }
  void operator()(const gp_mdpo_configurer& extractor) const {
    extractor(dynamic_cast<gp_mdpo_configurer::controller_type*>(mc_controller));
  }
  controller::base_controller* const mc_controller;
};

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
depth1_loop_functions::depth1_loop_functions(void)
    : ER_CLIENT_INIT("fordyca.loop.depth1"),
      m_interactors(nullptr),
      m_tasking_oracle(nullptr),
      m_metrics_agg(nullptr),
      m_cache_manager(nullptr) {}

depth1_loop_functions::~depth1_loop_functions(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void depth1_loop_functions::Init(ticpp::Element& node) {
  ndc_push();
  ER_INFO("Initializing...");

  shared_init(node);
  private_init();

  ER_INFO("Initialization finished");
  ndc_pop();
} /* Init() */

void depth1_loop_functions::shared_init(ticpp::Element& node) {
  depth0_loop_functions::shared_init(node);
  /* initialize stat collecting */
  auto* arenap = params()->parse_results<params::arena::arena_map_params>();
  params::output_params output =
      *params()->parse_results<const struct params::output_params>();
  output.metrics.arena_grid = arenap->grid;
  m_metrics_agg = rcppsw::make_unique<depth1_metrics_aggregator>(&output.metrics,
                                                                 output_root());

  /* initialize oracles */
  oracle_init();
} /* shared_init() */

void depth1_loop_functions::private_init(void) {
  /* initialize cache handling and create initial cache */
  auto* cachep = params()->parse_results<params::caches::caches_params>();
  cache_handling_init(cachep);

  /*
   * Initialize convergence calculations to include task distribution (not
   * included by default).
   */
  conv_calculator()->task_dist_init(std::bind(
      &depth1_loop_functions::calc_robot_tasks, this, std::placeholders::_1));

  /* intitialize robot interactions with environment */
  m_interactors = rcppsw::make_unique<interactor_map>();
  m_interactors->emplace(typeid(controller::depth1::gp_dpo_controller),
                         gp_dpo_itype(arena_map(),
                                      m_metrics_agg.get(),
                                      floor(),
                                      tv_manager(),
                                      m_cache_manager.get()));
  m_interactors->emplace(typeid(controller::depth1::gp_mdpo_controller),
                         gp_mdpo_itype(arena_map(),
                                       m_metrics_agg.get(),
                                       floor(),
                                       tv_manager(),
                                       m_cache_manager.get()));

  /*
   * Configure robots by mapping the controller type into a templated configurer
   *  in order to be able to configure the current controller using if/else
   *  statements dependent on controller typenames.
   */
  rcppsw::ds::type_map<
      controller_configurer<controller::depth1::gp_dpo_controller>,
      controller_configurer<controller::depth1::gp_mdpo_controller>>
      map;
  map.emplace(typeid(controller::depth1::gp_dpo_controller),
              controller_configurer<controller::depth1::gp_dpo_controller>(
                  params(), m_tasking_oracle.get(), m_metrics_agg.get()));
  map.emplace(typeid(controller::depth1::gp_mdpo_controller),
              controller_configurer<controller::depth1::gp_mdpo_controller>(
                  params(), m_tasking_oracle.get(), m_metrics_agg.get()));

  for (auto& entity_pair : GetSpace().GetEntitiesByType("foot-bot")) {
    argos::CFootBotEntity& robot =
        *argos::any_cast<argos::CFootBotEntity*>(entity_pair.second);
    auto base = dynamic_cast<controller::base_controller*>(
        &robot.GetControllableEntity().GetController());
    boost::apply_visitor(controller_configurer_mapper(base),
                         map.at(base->type_index()));
  } /* for(entity_pair..) */
} /* private_init() */

void depth1_loop_functions::oracle_init(void) {
  auto* oraclep = params()->parse_results<params::oracle_params>();
  if (oraclep->enabled) {
    ER_INFO("Creating oracle");
    argos::CFootBotEntity& robot0 = *argos::any_cast<argos::CFootBotEntity*>(
        GetSpace().GetEntitiesByType("foot-bot").begin()->second);
    const auto& controller0 =
        dynamic_cast<controller::depth1::gp_mdpo_controller&>(
            robot0.GetControllableEntity().GetController());
    auto* bigraph =
        dynamic_cast<const ta::bi_tdgraph*>(controller0.executive()->graph());
    m_tasking_oracle = std::make_unique<support::tasking_oracle>(bigraph);
  }
} /* oracle_init() */

void depth1_loop_functions::pre_step_iter(argos::CFootBotEntity& robot) {
  auto controller = dynamic_cast<controller::depth1::gp_dpo_controller*>(
      &robot.GetControllableEntity().GetController());

  /* get stats from this robot before its state changes */
  m_metrics_agg->collect_from_controller(controller);
  controller->block_manip_collator()->reset();

  /* send the robot its view of the world: what it sees and where it is */
  loop_utils::set_robot_pos<decltype(*controller)>(robot);
  ER_ASSERT(std::fmod(controller->los_dim(), arena_map()->grid_resolution()) <=
                std::numeric_limits<double>::epsilon(),
            "LOS dimension (%f) not an even multiple of grid resolution (%f)",
            controller->los_dim(),
            arena_map()->grid_resolution());
  uint los_grid_size = controller->los_dim() / arena_map()->grid_resolution();
  loop_utils::set_robot_los<decltype(*controller)>(robot,
                                                   los_grid_size,
                                                   *arena_map());
  set_robot_tick<decltype(*controller)>(robot);

  /* update arena map metrics with robot position */
  auto coord =
      rmath::dvec2uvec(controller->position(), arena_map()->grid_resolution());
  arena_map()->access<arena_grid::kRobotOccupancy>(coord) = true;

  /*
   * The MAGIC of boost so that we can avoid a series of if()/else if() for each
   * of the types of controllers in depth0 for robot-arena interactions.
   *
   * We use the runtime type of the controller we have to index into a map
   * containing a boost::variant of robot_arena_interactor<T>'s. The variant we
   * access in the map will be one with the active type being
   * robot_arena_interactor<runtime-type-of-current-controller>.
   *
   * We then just use a simple visitor to perform all robot-arena interactions.
   */
  boost::apply_visitor(
      controller_interactor_mapper(controller, GetSpace().GetSimulationClock()),
      m_interactors->at(controller->type_index()));
} /* pre_step_iter() */

template <class ControllerType>
void depth1_loop_functions::controller_configure(ControllerType& controller) {
  /*
   * If NULL, then visualization has been disabled.
   */
  auto* vparams = params()->parse_results<struct params::visualization_params>();
  if (nullptr != vparams) {
    controller.display_task(vparams->robot_task);
  }

  auto* oraclep = params()->parse_results<params::oracle_params>();
  if (oraclep->enabled) {
    auto& oracular =
        dynamic_cast<controller::depth1::ogp_mdpo_controller&>(controller);
    oracular.executive()->task_finish_notify(
        std::bind(&tasking_oracle::task_finish_cb,
                  m_tasking_oracle.get(),
                  std::placeholders::_1));
    oracular.executive()->task_abort_notify(
        std::bind(&tasking_oracle::task_abort_cb,
                  m_tasking_oracle.get(),
                  std::placeholders::_1));
    oracular.tasking_oracle(m_tasking_oracle.get());
  }
  controller.executive()->task_finish_notify(
      std::bind(&depth1_metrics_aggregator::task_finish_or_abort_cb,
                m_metrics_agg.get(),
                std::placeholders::_1));
  controller.executive()->task_abort_notify(
      std::bind(&depth1_metrics_aggregator::task_finish_or_abort_cb,
                m_metrics_agg.get(),
                std::placeholders::_1));
  controller.executive()->task_alloc_notify(
      std::bind(&depth1_metrics_aggregator::task_alloc_cb,
                m_metrics_agg.get(),
                std::placeholders::_1,
                std::placeholders::_2));
} /* controller_configure() */

argos::CColor depth1_loop_functions::GetFloorColor(
    const argos::CVector2& plane_pos) {
  rmath::vector2d tmp(plane_pos.GetX(), plane_pos.GetY());
  if (arena_map()->nest().contains_point(tmp)) {
    return argos::CColor(arena_map()->nest().color().red(),
                         arena_map()->nest().color().green(),
                         arena_map()->nest().color().blue());
  }
  /*
   * Blocks are inside caches, so display the cache the point is inside FIRST,
   * so that you don't have blocks renderin inside of caches.
   */
  for (auto& cache : arena_map()->caches()) {
    if (cache->contains_point(tmp)) {
      return argos::CColor(cache->color().red(),
                           cache->color().green(),
                           cache->color().blue());
    }
  } /* for(&cache..) */

  for (auto& block : arena_map()->blocks()) {
    /*
     * Even though each block type has a unique color, the only distinction
     * that robots can make to determine if they are on a block or not is
     * between shades of black/white. So, all blocks must appear as black, even
     * when they are not actually (when blocks are picked up their correct color
     * is shown through visualization).
     */
    if (block->contains_point(tmp)) {
      return argos::CColor::BLACK;
    }
  } /* for(&block..) */

  return argos::CColor::WHITE;
} /* GetFloorColor() */

void depth1_loop_functions::PreStep() {
  ndc_push();
  base_loop_functions::PreStep();

  auto& collector = static_cast<metrics::blocks::transport_metrics_collector&>(
      *(*m_metrics_agg)["blocks::transport"]);
  arena_map()->redist_governor()->update(GetSpace().GetSimulationClock(),
                                         collector.cum_collected(),
                                         conv_calculator()->converged());

  /* Get metrics from caches */
  for (auto& c : arena_map()->caches()) {
    m_metrics_agg->collect_from_cache(c.get());
    c->reset_metrics();
  } /* for(&c..) */
  m_metrics_agg->collect_from_cache_manager(m_cache_manager.get());
  m_cache_manager->reset_metrics();

  for (auto& entity_pair : GetSpace().GetEntitiesByType("foot-bot")) {
    argos::CFootBotEntity& robot =
        *argos::any_cast<argos::CFootBotEntity*>(entity_pair.second);
    pre_step_iter(robot);
  } /* for(&entity..) */
  m_metrics_agg->collect_from_arena(arena_map());
  m_metrics_agg->collect_from_loop(this);
  pre_step_final();

  ndc_pop();
} /* PreStep() */

void depth1_loop_functions::Reset() {
  ndc_push();
  base_loop_functions::Reset();
  m_metrics_agg->reset_all();

  auto ret = m_cache_manager->create(arena_map()->blocks(),
                                     GetSpace().GetSimulationClock());
  if (ret.status) {
    arena_map()->caches_add(ret.caches);
    floor()->SetChanged();
  }
  ndc_pop();
}

std::vector<int> depth1_loop_functions::calc_robot_tasks(uint) const {
  std::vector<int> v;
  auto& robots =
      const_cast<depth1_loop_functions*>(this)->GetSpace().GetEntitiesByType(
          "foot-bot");

  /*
   * We map the controller type into a templated task extractor in order to be
   * able to extract the ID of the robot's current task without using if/else
   * statements dependent on the current controller types.
   */
  rcppsw::ds::type_map<
      controller_task_extractor<controller::depth1::gp_dpo_controller>,
      controller_task_extractor<controller::depth1::gp_mdpo_controller>>
      map;
  map.emplace(typeid(controller::depth1::gp_dpo_controller),
              controller_task_extractor<controller::depth1::gp_dpo_controller>());
  map.emplace(
      typeid(controller::depth1::gp_mdpo_controller),
      controller_task_extractor<controller::depth1::gp_mdpo_controller>());

  for (auto& entity_pair : robots) {
    auto* robot = argos::any_cast<argos::CFootBotEntity*>(entity_pair.second);
    auto base = dynamic_cast<controller::base_controller*>(
        &robot->GetControllableEntity().GetController());
    v.push_back(boost::apply_visitor(controller_task_extractor_mapper(base),
                                     map.at(base->type_index())));
  } /* for(&entity..) */
  return v;
} /* calc_robot_tasks() */

std::pair<uint, uint> depth1_loop_functions::d1_task_counts(void) const {
  /*
   * Homogeneous swarm, so we can just take the first robot to get a handle on
   * the task IDs for the tasks we are interested in. We also assume that all
   * controllers in depth1 are derived from the DPO controller.
   */
  argos::CFootBotEntity& robot = *argos::any_cast<argos::CFootBotEntity*>(
      (*const_cast<depth1_loop_functions*>(this)
            ->GetSpace()
            .GetEntitiesByType("foot-bot")
            .begin())
          .second);
  auto gp_dpo = dynamic_cast<controller::depth1::gp_dpo_controller*>(
      &robot.GetControllableEntity().GetController());
  ER_ASSERT(nullptr != gp_dpo, "Controller not derived from GP_DPO");
  auto& collector = static_cast<rmetrics::tasks::bi_tdgraph_metrics_collector&>(
      *(*m_metrics_agg)["tasks::distribution"]);

  /*
   * These are interval counts, which means they are likely much greater than
   * the current actual # of harvesters/collectors in the swarm, so we have to
   * correct for that.
   */
  uint n_harvesters = collector.int_task_counts()[gp_dpo->task_id(
      tasks::depth1::foraging_task::kHarvesterName)];
  n_harvesters =
      n_harvesters / std::max(collector.timestep() % collector.interval(), 1U);
  uint n_collectors = collector.int_task_counts()[gp_dpo->task_id(
      tasks::depth1::foraging_task::kCollectorName)];
  n_collectors =
      n_collectors / std::max(collector.timestep() % collector.interval(), 1U);
  return std::make_pair(n_harvesters, n_collectors);
} /* d1_task_counts() */

__rcsw_pure uint depth1_loop_functions::n_free_blocks(void) const {
  auto accum = [&](uint sum, const auto& b) {
    return sum + (-1 == b->robot_id());
  };

  return std::accumulate(
      arena_map()->blocks().begin(), arena_map()->blocks().end(), 0, accum);
} /* n_free_blocks() */

void depth1_loop_functions::pre_step_final(void) {
  /*
   * The cache is recreated with a probability that depends on the relative
   * ratio between the # foragers and the # collectors. If there are more
   * foragers than collectors, then the cache will be recreated very quickly. If
   * there are more collectors than foragers, then it will probably not be
   * recreated immediately. And if there are no foragers, there is no chance
   * that the cache could be recreated (trying to emulate depth2 behavior here).
   */
  auto pair = d1_task_counts();
  if (arena_map()->caches().empty()) {
    auto ret =
        m_cache_manager->create_conditional(arena_map()->blocks(),
                                            GetSpace().GetSimulationClock(),
                                            pair.first,
                                            pair.second);

    if (ret.status) {
      arena_map()->caches_add(ret.caches);
      __rcsw_unused ds::cell2D& cell = arena_map()->access<arena_grid::kCell>(
          arena_map()->caches()[0]->discrete_loc());
      ER_ASSERT(arena_map()->caches()[0]->n_blocks() == cell.block_count(),
                "Cache/cell disagree on # of blocks: cache=%zu/cell=%zu",
                arena_map()->caches()[0]->n_blocks(),
                cell.block_count());
      floor()->SetChanged();
    } else {
      ER_INFO(
          "Could not create static cache: n_harvesters=%u,n_collectors=%u,free "
          "blocks=%u",
          pair.first,
          pair.second,
          n_free_blocks());
    }
  }

  if (m_cache_manager->caches_depleted() > 0) {
    floor()->SetChanged();
  }

  m_metrics_agg->metrics_write_all(GetSpace().GetSimulationClock());
  m_metrics_agg->timestep_inc_all();
  m_metrics_agg->timestep_reset_all();
  m_metrics_agg->interval_reset_all();
} /* pre_step_final() */

void depth1_loop_functions::cache_handling_init(
    const struct params::caches::caches_params* cachep) {
  if (nullptr != cachep && cachep->static_.enable) {
    /*
     * Regardless of how many foragers/etc there are, always create an
     * initial cache.
     */
    rmath::vector2d cache_loc = rmath::vector2d(
        (arena_map()->xrsize() + arena_map()->nest().real_loc().x()) / 2.0,
        arena_map()->nest().real_loc().y());

    m_cache_manager = rcppsw::make_unique<static_cache_manager>(
        cachep, &arena_map()->decoratee(), cache_loc);

    /* return value ignored at this level (for now...) */
    auto ret = m_cache_manager->create(arena_map()->blocks(),
                                       GetSpace().GetSimulationClock());
    arena_map()->caches_add(ret.caches);
  }
} /* cache_handling_init() */

using namespace argos; // NOLINT
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wglobal-constructors"
#pragma clang diagnostic ignored "-Wmissing-variable-declarations"
REGISTER_LOOP_FUNCTIONS(depth1_loop_functions, "depth1_loop_functions");
#pragma clang diagnostic pop
NS_END(depth1, support, fordyca);
