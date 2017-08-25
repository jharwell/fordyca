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

## Parameters

I have extended the base `.argos` file with a set of new parameters, documented
below. Unless otherwise stated, all parameters must be present in the XML file.

### Controller

The following XML tags are defined:

- grid: Parameters pertaining to a robots discretization of the continuous world
  into a grid.
- sensors: Parameters for robot sensors.
- actuators: Parameters for robot actuators.
- fsm: Parameters for state machine controlling a robot's actions.

#### `grid`
- `cell_decay_delta`: How fast the relevance of information about a particular
                      cell within a robot's 2D map of the world loses
                      relevance. Each timestep, the relevance value of the cell
                      is decremented by this value, starting the timestep after
                      the robot obtains information about the actual state of
                      the cell by driving over/near it.

- `resolution`: The size of the cells the arena is broken up (discretized)
                into. Should probably be the same as whatever the block size is,
                to make things easy.

#### `sensors`
- `diffusion`:

  - `go_straight_angle_range`: The angle range to the left/right of center (90
                               degrees on a unit circle) in which obstacles are
                               not ignored (outside of this range they are
                               ignored, assuming the robot will be able to drive
                               by them). Takes a pair like so: `-5:5` (for a 10
                               degree window).

  - `delta`: The longest distance away from the robot obstacles will be
             considered.

#### `actuators`
- `wheels`

  - `hard_turn_angle_threshold`: If actuators are told to change to a heading
                                 within a difference greater than the one
                                 specified by this parameter to the current
                                 heading, then a hard turn is executed (turn in
                                 place/opposite wheel speeds).

  - `soft_turn_angle_threshold`: If actuators are told to change to a heading
                                 within a difference greater than the one
                                 specified by this parameter to the current
                                 heading, but less than
                                 `hard_turn_angle_threshold`, then a soft turn
                                 is executed (keep moving forward and turn
                                 gradually).

  - `no_turn_angle_threshold`: If actuators are told to change to a heading
                               within a difference less than the one specified
                               by this parameter to the current heading, the
                               heading change is ignored.

#### `fsm`
- `unsuccessful_explore_dir_change`: If a robot is unsuccessful in finding what
                                     it is looking for in the # timesteps
                                     specified by this parameter, then it will
                                     randomly change direction.

### Loop Functions

The following XML tags are defined:

- logging: Parameters relative to logging simulation results.
- grid: Parameters relating to discretization of the arena.
- blocks: Parameters relating to blocks/block distribution
- nest: Parameters relating to the nest.
- visualization: Parameters for simulation visualizations, for help in
                 debugging.

#### `logging`
- `sim_stats`: The filename that foraging statistics will be written to.

#### `grid`
- `resolution`: The resolution that the arena will be represented at, in terms
                of the size of grid cells. Must be the same as the value passed
                to the robot controllers.

#### `blocks`
- `n_blocks`: # of blocks present in the arena.

- `dimension`: Since blocks are square, this the size of one side of the square.

- `dist_model`: The distribution model for the blocks. When blocks are
                distributed to a new location in the arena and made available
                for robots to pickup (either initially or after a block is
                deposited in the nest), they are placed in the arena in one of
                the following ways:

    - `random`: Placed in a random location in the arena.

    - `single_source` - Placed within a small arena opposite about 75% of the
                        way from the nest to the other side of the arena
                        (assumed horizontal, rectangular arena).

#### `nest`
- `size`: The size of the nest. Must be specified in a tuple like so:
  `0.5, 0.5`. Note the space--parsing does not work if it is omitted.

- `center`: Location for center of the nest (nest is a square).
            Must be specified in a tuple like so:
  `1.5, 1.5`. Note the space--parsing does not work if it is omitted.

#### `visualization`
- `robot_id`: Set to `true` or `false`. If true, robot id is displayed above
              each robot during simulation.

- `robot_los`: Set to `true` or `false`. If true, each robot's approximate line
               of sight is displayed as a red circle during simulation.

- `block_id`: Set to `true` or `false`. If true, each block's id displayed above
              it during simulation.

Explicitly setting visualization parameters is not required--they will be
treated as false if they are not.
