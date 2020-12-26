# Faster A* For Games
**Introduction**

The technique I used is mainly called **Goal Bounding**, it is an optimization technique for pathfinding algorithms. In theory, it can be applied to any search graph. After some digging I stumbled upon a paper written by **Steve Rabin and Nathan R. Sturtevant** where they explain the core fundamentals of how this works.
I'm going to take it a step further and implement it into a basic pathfinding program.

## Main Goal
**Concept**

coming soon

## Optimization
**Requirements**

coming soon

**Improvements**

coming soon

## Implementation
compilation algorithm
```
for each edge adjacent to a node
	precompute a bounding box that contains al optimally reachable goals starting from this edge
		(for this one can use a reversed dijkstra algorithm
```
runtime algorithm
```
if goal within bounding box
	do pathfinding
```
## References
Steve Rabin **and** Nathan R. Sturtevant, **Faster A * with Goal Bounding** [[PDF]
](http://www.gameaipro.com/GameAIPro3/GameAIPro3_Chapter22_Faster_A_Star_with_Goal_Bounding.pdf)

