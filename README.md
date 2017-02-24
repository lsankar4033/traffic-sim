##### What?
A simplified simulation of traffic along a one-way, single lane road.

##### Why?
On a road trip down from San Francisco to Sedona, my brother and I found ourselves in light traffic on a
number of long desert roads. We observed a periodicity in the starting and stopping we did because of
traffic. We attributed this periodicity the following related phenomena:
1. human error in maintaining a constant velocity
2. a tendency for drivers to speed up (to some max speed) when there is room in front of them and slow down
   (but never reverse) if there isn't

However, the question remained of what this periodicity *looked like* in the typical case. Is it a simple
harmonic oscillator? A sawtooth? A step function?

While a closed form solution might exist, this seemed like a very easy thing to examine with simulation.

And so this project was born.

##### How?
As with many simple physics simulations, I've opted for simplicity over precise realism. The key phenomena
that I aimed to preserve in the simulation are 1 and 2 mentioned above.

Of course this means that I may have missed out on some key factor in human driving that changes the
fundamental properties of the simulation, but I couldn't think of anything obvious in this regard. If you do,
feel free to let me know :).

Here are the simulation stipulations:
- each 'car' is a point mass on a 1-d line that at any time has a known displacement, velocity, and
  acceleration (no jerk or higher derivatives)
- the simulation is run as a series of time steps, each consisting of a sequence of steps:
  1. normally distributed 'jitter' in each car's velocity (simulating human error in maintaining velocity)
  2. set acceleration for each car based on closeness to next car
     - if next car is far enough away, accelerate to speed limit
     - if next car is close enough, decelerate to avoid hitting it
  3. independently set the new velocity + acceleration for each car independently
  4. resolve collisions by moving all cars involved in a collision to the point of the car earliest in the
     collision

Simulation results are analyzed in jupyter notebook(s) found in the `notebook/` folder.

##### TODO
- show key graph(s) in the README
- remove all the extraneous notebook code from git
- fit the displacement curves to actual harmonic oscillators and see how frequency depends on parameters
- make cars respond to cars behind them
- (optional) give cars finite size (so collisions don't happen only when points coincide)
