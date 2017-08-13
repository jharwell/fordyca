# FORDYCA (FOraging Robots use DYnamic CAches)

This is mainly just a collection of things I don't want to forget at the moment.

## Labelling Issues

- Critical is for things that are main features/super important, or are
  segmentation-fault level bugs.

- Major is for stuff that is supports main project features.

- Normal is for things that I will need eventually, but could be done now or at
  some point in the near-ish future.

- Minor is for things that would be nice to have (think enhancements), but that
  I can live with without too much headache at the moment.

- Feature is for pretty much everything that isn't "create an experimental
  scenario".

- Enhancements are pretty self-explanatory.

Also, branches should be named after their issue number, which is probably not the
same as their number within their category (i.e. the 4th bug is probably the
17th global issue).

## Running Experiments

For block distribution, the following block distribution methods are availabl:

- random - A random, uniform distribution of blocks in the arena.
- single_source - A small concentrated source of blocks opposite the nest in the
  arena.
- dual\_source - Dual sources on opposite sides of the nest in the arena.
- dual_source\_same\_side - Dual sources on the same side of the nest, separated
  by ~60 degrees.
