# FORDYCA (FOraging Robots use DYnamic CAches)

This is the main entry point for getting started on the project.

## Basic Setup

Before starting, make sure you have ARGoS installed, and can run the simple
foraging example given on the website.

These steps are for Linux, and while it may work on OSX, I have not tried it. It
definitely will not work on windows. You will need a recent version of the
following programs:

- cmake
- make
- g++
- cppcheck (optional; for additional static analysis)
- clang++ (optional; for additional syntax checking/static analysis )
- icpc (optional; for additional syntax checking)

1. After cloning this repo, you will also need to clone the following repos:

  - https://github.com/jharwell/rcppsw
  - https://github.com/jharwell/rcsw

  Before you can build anything, you will need to define some environment
  variables:

  - `rcsw` - Set to the path to wherever you cloned the `rcsw` repo.

  - `rcppsw` - Set to the path to wherever you cloned the `rcppsw` repo.

2. Verify you can build `rcsw`, `rcppsw`, and `fordyca` (in that order), by
   doing:

        cd /path/to/repo
        mkdir build && cd build
        cmake ..
        make

## Relevant Papers

## Style Guide

Generally speaking, I follow the "do as the standard library does" mantra for
this project. In particular:

- All source files have the `.cpp` extension, and all header files have the
  `.hpp` extension.

- All file, class, variable, and namespace names are `specified_like_this`, NOT
  `specifiedLikeThis` or `SpecifiedLikeThis`.

- Exactly one class definition per .cpp/.hpp file, unless there is a very good
  reason.

- The namespace hierarchy exactly corresponds to the directory hierarchy that
  the source/header files for classes can be found in.

- All classes have a doxygen brief, as do all non-getter/non-setter member
  functions. Tricky/nuanced issues with member variables should be documented,
  though in general the class name + member variable name + type should be
  enough documentation. If its not, chances are you are doing it wrong.

  This may seem like overkill, but I have learned over the years that `If it is
  hard to document, it is probably wrong, and if it is hard to test, it is
  almost assuredly wrong`. Forcing documenting all major parts of the code helps
  a lot with the first part.

- Code should pass the google C++ linter, ignoring the following items. For
  everything else, the linter warnings should be addressed.

  - Use of non-const references--I do this all the time.

  - Line length >= 80 ONLY for: member variable decls, function decls, inheritance
    lists, and function definitions. Sometimes the names of
    variables/namespaces/templates make this impossible to do without obfuscating
    the code.

## Development Guide

### Directory layout

- `src/` - All `.cpp` files live under here.

- `include/` - All `.hpp` files live under here.

- `tests` - All test code lives under here.

- `docs/` - All documentation besides this README lives under me.

- `exp/` - This is the directory where the ARGoS simulation input files live.

- `VERSION` - A file in the root root that holds the current/next versions of
  the code. Versions are numbered as `major.minor.patch+xxx`. `major`
  corresponds to releases/milestones in the code, and is only updated when
  `devel` is merged to `master`. `minor` corresponds to the addition of new
  features; each time a feature branch is merged into `devel`, the minor version
  should be incremented. `patch` corresponds to fixing bugs, so everytime a
  bugfix branch is merged into `devel`, this number should be incremented. The
  `+xxx` is for how many commits there have been since the last major release,
  and is calculated upon merging `devel` into `master`.

### Labelling Issues

All issues that are entered into github should have a `Priority`, a `Status`,
and a `Type` associated with them. Well usually. Sometimes it doesn't make
sense (Question for example) to have all three.

Priorities:

- `Critical` - Things that are main features/super important, or are
  segmentation-fault level bugs, as in "this must be fixed/addressed now before
  we can move forward".

- `Major` - Things that support/are main project features, but are not blocking
  other tasks.

- `Minor` - Things that would be nice to have (think enhancements), but that are
  not required at the moment, but will be needed at some point in the near-ish
  future.

- `Low` - Things that are not blocking any other tasks, can be implemented
  anytime without compromising the project in any way. A "wishlist" of things
  that would be nice to add, as it were.

Statuses:

- `Available` - The task is available to be worked on.

- `Blocked` - The task is blocked waiting for the completion of another task.

- `Completed` - The task has been completed. All tasks should be in this state
  before the issue is closed.

- `Future` - It is not possible to work on the task at the moment, because too
  much development needs to happen to make it accessible, or that it is
  something worth considering adding in the future, when the project is more
  mature.

- `In Progress` - The task is currently being worked on.

- `Review Needed` - The task has been completed, but needs to be reviewed (this
  should be tied to a pull request) before it can be moved to the completed
  state.

Types:

- `Bugfix` - This is a task to address a bug in the code.

- `Docs` - This is a task related to creating/updating documentation.

- `Enhancement` - This is a task that extends the functionality of an existing
  part of the code, but not so far that it is considered a new feature.

- `Feature` - This is a task that adds new functionality to the code.

- `Question` - This is not a task per-se, but a question whose resolution will
  lead to the creation of enhancements/features/refactors.

- `Refactor` - This is a task to refactor the code, not changing functionality
  but modifying the interface, changing data structures, etc. This should be
  accompanied by unit tests if applicable.

- `Task` - This is a task that relates to "chore" work for the project. Renaming
  files, moving things around, mucking about with the build process are all good
  examples of things that should get a `Task` label.

### Branches

All branches should have a corresponding issue on github, and the issue should
be named the same thing as the branch. This may seem pedantic, but when you have
hundreds or thousands of issues and branches, any little thing you can do to
increase the self-documenting nature of the development process is worth doing.

For more details, see [Git usage guidlines](docs/f17-git-usage.pdf). It's from
3081, and is focused on git usage from a course perspective, but there is still
a lot of good stuf in it.

### General Workflow

1. Find an issue on github to work on.

2. Mark said task as `Status: In Progress` so no one else starts working on it
   too, and assign it to yourself if it is not already assigned to you.

3. Branch off of the devel branch with a branch with the same name as the issue.

4. Work on the issue/task, committing as needed. You should push your changes
   regularly, so people can see that the issue is being actively worked on. Your
   commit messages don't have to be an essay, but they should all reference the
   issue # of the task so that in-progress commits show up in github, and
   describe what was done and why in reasonable detail. Don't do things like "in
   progress", or "misc updates", or if you do such things, rebase/collapse your
   history into a single detailed commit when you are done. Be sure you know
   what you are doing if you go this route...

5. Finish the task, updating the `VERSION` file appropriately if needed, and
   change status to `Status: Needs Review` and open a pull request.

6. Once the task has been reviewed and given the green light, merge it into
   devel, and marked the issue as `Status: Completed`. Don't close the issue.

7. Repeat as necessary.
