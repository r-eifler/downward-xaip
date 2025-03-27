# Goal Conflict Computation

This FD copy implements the algorithms to compute minimal insolvable goal subset 
(MUGS) 

1. Goal Lattice Search
2. Branch and Bound Search

as introduced in:

    @inproceedings{eifler:etal:aaai20,
        author       = {Rebecca Eifler and
                        Michael Cashmore and
                        J{\"{o}}rg Hoffmann and
                        Daniele Magazzeni and
                        Marcel Steinmetz},
        title        = {A New Approach to Plan-Space Explanation: Analyzing Plan-Property
                        Dependencies in Oversubscription Planning},
        booktitle    = {The Thirty-Fourth {AAAI} Conference on Artificial Intelligence, {AAAI}
                        2020, The Thirty-Second Innovative Applications of Artificial Intelligence
                        Conference, {IAAI} 2020, The Tenth {AAAI} Symposium on Educational
                        Advances in Artificial Intelligence, {EAAI} 2020, New York, NY, USA,
                        February 7-12, 2020},
        pages        = {9818--9826},
        publisher    = {{AAAI} Press},
        year         = {2020},
    }

    @inproceedings{eifler:etal:ijcai20,
        author       = {Rebecca Eifler and
                        Marcel Steinmetz and
                        {\'{A}}lvaro Torralba and
                        J{\"{o}}rg Hoffmann},
        title        = {Plan-Space Explanation via Plan-Property Dependencies: Faster Algorithms
                        {\&} More Powerful Properties},
        booktitle    = {Proceedings of the Twenty-Ninth International Joint Conference on
                        Artificial Intelligence, {IJCAI} 2020},
        pages        = {4091--4097},
        year         = {2020}
    }

## Dependencies

Please make sure to also clone the submodule under `src/translate/xaip` containing 
the translator extension.

### General 

For the basic variant (without LP solver and no support for temporal preferences)
in addition to the dependencies listed below for FD itself, the following
packages are required:

    apt-get install libeigen3-dev pkgconf libgrpc++-dev libboost-all-dev libgrpc++1 autoconf flex 

Build FD normally with:

    ./build.py


### LP Solver

For using [Potential Heuristics](https://ojs.aaai.org/index.php/ICAPS/article/view/13714)
an LP solver is required, either [SOPLEX](https://soplex.zib.de/) or 
[CPLEX](https://www.ibm.com/products/ilog-cplex-optimization-studio),
and the solver interface [OSI](https://github.com/coin-or/Osi) and the following library

    apt-get install bison 

For the installation of the LP solver and OSI please follow the corresponding installation
instructions.

FD expects the to find the libraries via the following environment variables:

    export DOWNWARD_COIN_ROOT=<path to osi installation/libray folder>
    export DOWNWARD_SOPLEX_ROOT=<path to soplex libraries>
    export DOWNWARD_CPLEX_ROOT=<path to cplex libraries>
    export DOWNWARD_CONCERT_ROOT=<path to cplex concert libraries>

Path should point to folders containing the `inlcude` and `lib` folders.

### Temporal Preferences

To support the finite LTL compilation two additional dependencies are required:

1. Command line tools of [Spot](https://spot.lre.epita.fr/index.html).
For installation instruction see [here](https://spot.lre.epita.fr/install.html).

1. [LTLfKit](https://bitbucket.org/acamacho/ltlfkit/src/master/) for the compilation 
from LTLf to automata.

The [FD translator extension](https://github.com/r-eifler/xaip-FD-translate) 
expects to find the LTLfKit under the environment 
variable:

    export LTL2HAO_PATH=<path to the top level folder of the ltlfkit>



## MUGS Computation

To compute the MUGS the following algorithms are supported.

### Goal Lattice Search

    ./fast-downward.py <domain file> <problem file> --heuristic 'hff=ff(cache_estimates=false, transform=adapt_costs(one), verbosity=silent)' --search '<search>(lazy_greedy([hff], preferred=[hff], bound=<n>, reopen_closed=true, verbosity=silent), heu=[hff], all_soft_goals=<true/false>)'

Parameters:

* `<search>` either `wgss` for starting with all soft goals and systematically 
removing goals or `sgss` for starting with the empty set and systematically  
adding goals (`sgss` works typically better)

* `bound`: the overall **strict** (`< bound`) cost bound of the OSP task

* `all_soft_goals`: indicates whether all goals defined in the problem file 
should be treated as soft goals. If set to `false`, then soft and hard goals 
need to be defined via an additional input file described below.

### Branch and Bound Search

    ./fast-downward.py <domain file> <problem file> --heuristic 'ngsh=ngs(<heuristic>)' --search 'gsastar(evals=[blind], eval=ngsh, bound=?, all_soft_goals=<true/false>)'

Parameters:
* `bound`: the overall **strict** (`< bound`) cost bound of the OSP task

* `all_soft_goals`: indicates whether all goals defined in the problem file 
should be treated as soft goals. If set to `false`, then soft and hard goals 
need to be defined via an additional input file described below.

For pruning the following heuristics can be used for `<heuristic>`:

Blind heuristic, no pruning.

    blind

Cartesian Abstraction Heuristics (*best overall performance*):

    cegar(subtasks=[goals()])

Different configurations for number of abstract states, transition or generation 
time are possible, the subtask option however is limited to `goals`.

h_max Heuristic:

    hmax(no_deadends=true)

Potential Heuristics (LP needs to be installed!)

    individual_goal_potentials()


### Temporal Preferences

Temporal preferences are defined via an additional json input file.
It is passed via

    ./fast-downward.py <domain file> <problem file> --translate-options --explanation-settings <preference file> --search-options ...
#### Dependencies

The support of temporal properties defined in LTLf relies on the library [LTLfKit](https://bitbucket.org/acamacho/ltlfkit).
For installation instructions we refer to [INSTALL](https://bitbucket.org/acamacho/ltlfkit/src/master/INSTALL.md.

To allow the planner to find the library the environment variable `LTL2HAO_PATH` must point (with an absolute path) 
to the folder containing the library, e.g. `LTL2HAO_PATH=/..../ltlfkit`.


#### Input Specification

Temporal preferences can be either defined using LTL or as Action Set preference.
The supported grammar is:

    {
        "plan_properties": [temporal_goal]
        "hard_goals": [goal_name]
        "soft_goals": [goal_name]
    }


    temporal_goal :=
    {
        "name": <unique_name>,
        "type": AS|LTL,
        "formula": <formula>,
        "actionSets": [action_set]
    }

    action_set :=
    {
        "name": <unique_name>,
        "actions": [action]
    }

    action := 
    {
        "name": <name>, 
        "params": [object|type]
    }

Examples are:

    {
        "name": "never_move_to_locationl2",
        "type": "AS",
        "formula": "! move_to_l2",
        "actionSets":[
            {
                "name": "move_to_l2",
                "actions":[
                    {"name": "move", "params": ["rover", "location", "l2", "level", "level", "level", "level", "level", "level"]}
                ]
            }
        ]
    }

    {
        "name": "do_uploaded_soil_sample0_before_uploaded_x_ray_image1",
        "type": "LTL",
        "formula": "U ! uploaded(x_ray_image1) uploaded(soil_sample0)"
    }

This also allows specifying hard and soft goals.
In this case the goals in the PDDL problem file are ignored. 
"Normal" goal facts (facts that have to be satisfied in the last state of the 
plan) can in this case be defined via

    {
        "name": <unique_name>,
        "type": "G",
        "formula": "at(p2,l0)",
    }

where `formula` contains the goal fact.

Predicates are defined with bracket enclosing the parameters, e.g. `at(p2,l0)`
rather than `(at p2 l0)` as in PDDL.

Formulas are defined in prefix notation, e.g. `&& a b`.
Action Set preferences only support disjunctive normal form.
LTL preferences support any formula including the logic operators `! && || -> <->` 
and the temporal operators `F G U Q X R`.


# Original FD README

<img src="misc/images/fast-downward.svg" width="800" alt="Fast Downward">

Fast Downward is a domain-independent classical planning system.

Copyright 2003-2022 Fast Downward contributors (see below).

For further information:
- Fast Downward website: <https://www.fast-downward.org>
- Report a bug or file an issue: <https://issues.fast-downward.org>
- Fast Downward mailing list: <https://groups.google.com/forum/#!forum/fast-downward>
- Fast Downward main repository: <https://github.com/aibasel/downward>


## Tested software versions

This version of Fast Downward has been tested with the following software versions:

| OS           | Python | C++ compiler                                                     | CMake |
| ------------ | ------ | ---------------------------------------------------------------- | ----- |
| Ubuntu 20.04 | 3.8    | GCC 9, GCC 10, Clang 10, Clang 11                                | 3.16  |
| Ubuntu 18.04 | 3.6    | GCC 7, Clang 6                                                   | 3.10  |
| macOS 10.15  | 3.6    | AppleClang 12                                                    | 3.19  |
| Windows 10   | 3.6    | Visual Studio Enterprise 2019 (MSVC 19.29) and 2022 (MSVC 19.31) | 3.22  |

We test LP support with CPLEX 12.9, SoPlex 3.1.1 and Osi 0.107.9.
On Ubuntu, we test both CPLEX and SoPlex. On Windows, we currently
only test CPLEX, and on macOS, we do not test LP solvers (yet).


## Contributors

The following list includes all people that actively contributed to
Fast Downward, i.e. all people that appear in some commits in Fast
Downward's history (see below for a history on how Fast Downward
emerged) or people that influenced the development of such commits.
Currently, this list is sorted by the last year the person has been
active, and in case of ties, by the earliest year the person started
contributing, and finally by last name.

- 2003-2022 Malte Helmert
- 2008-2016, 2018-2022 Gabriele Roeger
- 2010-2022 Jendrik Seipp
- 2010-2011, 2013-2022 Silvan Sievers
- 2012-2022 Florian Pommerening
- 2013, 2015-2022 Salomé Eriksson
- 2018-2022 Patrick Ferber
- 2021-2022 Clemens Büchner
- 2021-2022 Dominik Drexler
- 2022 Remo Christen
- 2015, 2021 Thomas Keller
- 2016-2020 Cedric Geissmann
- 2017-2020 Guillem Francès
- 2018-2020 Augusto B. Corrêa
- 2020 Rik de Graaff
- 2015-2019 Manuel Heusner
- 2017 Daniel Killenberger
- 2016 Yusra Alkhazraji
- 2016 Martin Wehrle
- 2014-2015 Patrick von Reth
- 2009-2014 Erez Karpas
- 2014 Robert P. Goldman
- 2010-2012 Andrew Coles
- 2010, 2012 Patrik Haslum
- 2003-2011 Silvia Richter
- 2009-2011 Emil Keyder
- 2010-2011 Moritz Gronbach
- 2010-2011 Manuela Ortlieb
- 2011 Vidal Alcázar Saiz
- 2011 Michael Katz
- 2011 Raz Nissim
- 2010 Moritz Goebelbecker
- 2007-2009 Matthias Westphal
- 2009 Christian Muise


## History

The current version of Fast Downward is the merger of three different
projects:

- the original version of Fast Downward developed by Malte Helmert
  and Silvia Richter
- LAMA, developed by Silvia Richter and Matthias Westphal based on
  the original Fast Downward
- FD-Tech, a modified version of Fast Downward developed by Erez
  Karpas and Michael Katz based on the original code

In addition to these three main sources, the codebase incorporates
code and features from numerous branches of the Fast Downward codebase
developed for various research papers. The main contributors to these
branches are Malte Helmert, Gabi Röger and Silvia Richter.


## License

The following directory is not part of Fast Downward as covered by
this license:

- ./src/search/ext

For the rest, the following license applies:

```
Fast Downward is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at
your option) any later version.

Fast Downward is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.
```
