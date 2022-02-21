# New autonomous player library

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


