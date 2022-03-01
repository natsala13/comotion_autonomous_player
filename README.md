# New autonomous player library
This is Nathan's autonomous player.
## Player strategy
In order to find the best path in every turn,
the player will first look for the best match between robots and goals (bonuses / circles)
for every match a heuristic will be computed taking in charge the path's length,
the end goal, and a pseudo assumption of all bonuses along the road.
using this heuristic the best match will be chosen.
having a match between robots and end goals, a path will be computed between every robot
to its goal using a greedy algorithm in order to add as many bonuses as possible
along the road without breaking the makespan limit.
Paths between points are computed using Dijkestra on a Prm map constructed on the
beginning of the game while every turn opponent robots' locations are removed and re-added
at the end of the turn. In addition, every turn, paths creating collisions between friendly
robots, are trimmed according to a collision if existing.

## Classes
* algorithms - contain usefull algos such as prm, greedy bonuses collector etc.
* autonomous_gmae - DEV a module helping the tests simulating a Comotion game
* heuristic - different heuristics estimating the value of a match
* players - different autonomous players - The chosen one pseudo_best_path_player
* plotter - DEV a helper module in order to plot graphs while debugging.
* utils - helper classes.
* tests - tests for the autonomous player's code plus simulation of games using config files.

## Issues
- [x] prm module
- [x] heuristic library
- [x] update prm graph in live according to robots new locations.
- [x] dijkestra partially up to depth t.
- [x] basic heuristic player.
- [x] remove opponent players from graph.
- [x] Smarter heuristic to move few robots simultaneously.
- [x] Finish at circles
- [x] Fasten heuristic calculation.
- [x] Heuristic runtime
- [x] Measure new runtime - 0.04 sec! (Due to distance matrix)
- [x] Add functionality for half a target.
- [x] What if there are more robots than bonuses.
- [x] I don't wanna change the sum over all valid states, neither take the minimum one, but the maximum heuristic one.
- [ ] PROFILING - Run time of each block etc.
- [ ] FEATURE - bridge test every turn around enemy robots.
- [ ] FEATURE, RUNTIME - maybe not using as many points as possible to avoid enormous runtime later.
- [x] HEURISTIC - Path that collects few bonuses at once.
- [ ] PROFILING - Distance matrix calculations takes a lot of time.
- [ ] CRITICAL - Time prunning.
- [ ] FEATURE - Parallel computing.
- [ ] CRITICAL - Robots of same team not colliding with one another.
- [ ] TESTING - Comparing different players, heuristic and params.
- [ ] REFACTOR - focused heuristic is not organised.
- [ ] FEATURE - allow robots not to move. 
- [ ] Reduce makespan to 1 to see that nothing is broken.
- [ ] TESTING -  Create a testing framework for multiple players / scenes / params.
- [ ] BUG - More robots than bonuses cause an exception.
- [ ] FEATURE - Allow upgrading some robot's paths and not only one. 

## Players
 - [x] Basic heuristic player
 - [x] Focused Heuristic player
 - [ ] Minmax player
 - [ ] Alpha Beta player
 - [ ] Monte Carlo player
 - [ ] Traveling Salesman heuristic


