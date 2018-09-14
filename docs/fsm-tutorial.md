# FSM Tutorial

This is a tutorial for how to add a new FSM to fordyca. It assumes that you have
already built the documentation for forydca and rcppsw and have some level of
familiarity with the FSM class hierarchy.

## Deriving Your FSM

All FSMs need to either derive from
`rcppsw::patterns::state_machine::simple_fsm` or
`fordyca::fsm::base_foraging_fsm`.

Choose `simple_fsm` if:

- This is an FSM that will not directly partake in foraging (i.e. the
  `vector_fsm` or `cell2D_fsm`).

- If you choose this FSM, you may also want to inherit from
  `rcppsw::task_allocation::taskable`, so that you can use this FSM as part of a
  task.

Choose `base_foraging_fsm` if:

- You want to reuse any of the states in `base_foraging_fsm` (and you should,
  rather than creating your own)

- You want to be able to send signals to a "parent" FSM (e.g. you have a state
  for doing X, but when X is done that state does not know what to do next, so
  throws a signal to its parent state to deal with; kind of like an exception,
  but more targeted).

- You want to be able to incorporate states from other FSMs in `fordyca::fsm`,
  and/or wrap FSMs from that namespace.

- If you are deriving from `base_foraging_fsm`, you will need to use the
  `HFSM_CONSTRUCT_STATE()` macro, and specify the parent state (i.e. the state
  which will handle any signals which child states cannot deal with). For most
  states, this should be `hfsm::top_state()`, which means that if that state
  ever returns something other than `HANDLED`, then something very bad has
  happened, and the whole simulation should be aborted.

## Creating States

- Before creating states, draw out your state diagram to verify that they are no
  sinks (i.e. graph cycles that control flow cannot escape from once entered).

- Use the provided HFSM_ or FSM_ macros when creating states. They hide
  complex template usage which detracts from readability and usability.

- Look at `fordyca::fsm::explore_for_goal_fsm` for examples of how to
  declare/define states. States almost always do not have data (and use the
  `__ND` macro), and very rarely need entry, exit, or guard functions.

- All states must be first declared in the .hpp file, and then defined in the
  .cpp file. There are macros for both of these. If you want to inherit a state
  from a parent hfsm, there is a special `INHERIT` macro you need to use instead
  of the `DECLARE` one, otherwise you will get a linking error.

- You transition between states using the `internal_event()` function from
  within the states themselves, which will cause execution flow to transition to
  the state you specify on the next timestep. On timesteps when you call
  `internal_event()`, make sure you return the `HANDLED` signal immediately
  after calling it, otherwise, weird things can happen!

- You send signals to an FSM (of any type), using `external_event()`, and you
  can also pass data to the FSM this way. Calling `external_event()` will cause
  the current state in the FSM to be sent the signal+data you specify; if that
  state is not waiting for said signal and just returns `HANDLED`, then it is
  silently lost, and things will likely break/not work properly later in the
  simulation.

- Your the order of states in your FSM state enum must match the order of states
  in your state map, otherwise state transitions will not work properly (all
  current FSMs should have a comment saying this, so you can look at any for
  examples).

## Visual Indications Of State Transitions

- There are a number of entry functions available in various FSMs that are used
  to have the robots change color when then enter a state for the first time,
  which can be very helpful in debugging. See `explore_for_goal_fsm` for an
  example of how to do this.
