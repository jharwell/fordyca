# New Features

- Initial implementation of depth 2 recursive partitioning controller and
  framework.

- Multi-source block distribution added (#291).
- Lots of additional metrics into ensemble swarm characteristics added.
- Incorporation of oracular controllers to give ceiling on possible
  performance.

# Bug Fixes

- Additional fixes to usage of static cache at scale
- Numerous bugfixes moving towards unstable depth2 implementation
- Differences in how ramp vs. cube blocks treated/handled fixed.
- Task interface time now calculated correctly.

# Improvements

- Switch to using log4cxx for debugging, rather than homegrown logging mechanism.
- Lights now owned by nest entities (#490).
- Multi-physics engine support verified.
- ARGoS dependencies reduced to minimum; rcppsw HAL is used whenever possible.

# Other Changes

- Rename stateless -> CRW (Correlated Random Walk) controller.
- Now under GPL 3.0.
