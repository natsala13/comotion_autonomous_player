# New authonomous player library

## Issues
- [x] prm module
- [x] heuristic library
- [x] update prm graph in live according to robots new locations.
- [x] dijkestra partially up to depth t.
- [x] basic heuristic player.
- [x] remove oponent players from graph.
- [x] Smarter heuristic to move few robots simultaneously.
- [x] Finish at circles
- [x] Fasten heuristic calculation.
- [ ] Min max.
- [ ] Monte Carlo
- [ ] Profiling - Run time of each block etc..
- [ ] Prm - bridge test every turn around enemy robots.
- [ ] Prm - maybe not using as much points as possible to avoid enormous runtime later.
- [x] !!! Heuristic runtime
- [x] Measure new runtime - 0.04 sec! (Due to distance matrix)
- [ ] Add functionality for half a target.
- [ ] What if there are more robots than candies.
- [ ] Path that collects few candies at once.
- [ ] I don't wanna change the sum over all valid states, neither take the minimum one, but the maximum heuristic one.

## Questions
* Make an 8 d prm graph? Including all robots possible locations? NO
* How can I measure a path and not an end situation? (path eating few candies all at once?)
* Exact scene? is it possible to implement? NO.

* Why should all robots' paths be the same?
* Returning answer before time is up?
* Parallel computing
* FT Point Edge? To use Cigal algorithms
* running without gui?
* plotting 

## Extra Ideas
* Monte Carlo
* Min max
* Path evaluation instead of end nodes.
* Reinforcement learning
* DRRT - be carefull of tensor product 