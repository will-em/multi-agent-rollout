from __future__ import annotations

import heapq
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

Position = Tuple[int, int]


MOVES: Dict[int, Tuple[int, int]] = {
    0: (0, 0),
    1: (-1, 0),
    2: (1, 0),
    3: (0, -1),
    4: (0, 1),
}


def manhattan_distance(a: Position, b: Position) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def reconstruct_actions(came_from: Dict[Position, Tuple[Position, int]], goal: Position, start: Position) -> List[int]:
    if goal == start:
        return [0]
    actions: List[int] = []
    current = goal
    while current != start:
        previous, action = came_from[current]
        actions.append(action)
        current = previous
    actions.reverse()
    return actions


def astar_actions(
    start: Position,
    goal: Position,
    blocked: Iterable[Position],
    height: int,
    width: int,
) -> List[int]:
    blocked_set = set(blocked)
    blocked_set.discard(goal)
    frontier: List[Tuple[int, int, Position]] = [(manhattan_distance(start, goal), 0, start)]
    best_cost: Dict[Position, int] = {start: 0}
    came_from: Dict[Position, Tuple[Position, int]] = {}

    while frontier:
        _, cost_so_far, current = heapq.heappop(frontier)
        if current == goal:
            return reconstruct_actions(came_from, goal, start)

        for action, (dr, dc) in MOVES.items():
            if action == 0:
                continue
            nxt = (current[0] + dr, current[1] + dc)
            if not (0 <= nxt[0] < height and 0 <= nxt[1] < width):
                continue
            if nxt in blocked_set:
                continue
            new_cost = cost_so_far + 1
            if nxt not in best_cost or new_cost < best_cost[nxt]:
                best_cost[nxt] = new_cost
                came_from[nxt] = (current, action)
                priority = new_cost + manhattan_distance(nxt, goal)
                heapq.heappush(frontier, (priority, new_cost, nxt))

    return [0]


def path_length(
    start: Position,
    goal: Position,
    blocked: Iterable[Position],
    height: int,
    width: int,
) -> Optional[int]:
    actions = astar_actions(start, goal, blocked, height, width)
    if actions == [0] and start != goal:
        return None
    if start == goal:
        return 0
    return len(actions)
