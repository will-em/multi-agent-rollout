from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Dict, List, Sequence, Tuple

from python.pathfinding import MOVES, astar_actions, manhattan_distance, path_length

Position = Tuple[int, int]


class Cell(IntEnum):
    SPACE = 0
    WALL = 1
    BOX = 2


class Action(IntEnum):
    STAY = 0
    UP = 1
    DOWN = 2
    LEFT = 3
    RIGHT = 4


@dataclass
class AgentState:
    position: Position
    carrying: bool = False
    battery: int = 100
    max_battery: int = 100
    delivered_boxes: int = 0
    assigned_target: Position | None = None


@dataclass
class StepInfo:
    picked_agents: List[int] = field(default_factory=list)
    delivered_agents: List[int] = field(default_factory=list)
    out_of_battery: List[int] = field(default_factory=list)
    collision_agents: List[int] = field(default_factory=list)
    charging_stations: List[Position] = field(default_factory=list)
    batteries: List[int] = field(default_factory=list)
    target_types: List[str] = field(default_factory=list)


class WarehouseEnvironment:
    """Python warehouse environment with battery-constrained multi-agent dynamics."""

    STEP_COST = 1.0
    PICKUP_REWARD = -10000.0
    DROPOFF_REWARD = -10000.0
    RETURN_TO_START_REWARD = 0.0
    COLLISION_COST = 1e20
    LOW_BATTERY_PENALTY = 100.0
    DISCOUNT_FACTOR = 0.999

    def __init__(
        self,
        wall_offset: int = 10,
        box_offset: int = 5,
        n: int = 1,
        agent_count: int = 4,
        battery_capacity: int = 100,
        move_discharge: int = 1,
        idle_discharge: int = 0,
        charge_rate: int = 10,
        charging_stations: Sequence[Position] | None = None,
        low_battery_threshold: int = 15,
        charge_safety_margin: int = 3,
    ) -> None:
        self.wall_offset = wall_offset
        self.box_offset = box_offset
        self.n = n
        self.height = 35 + (4 * 3)
        self.width = 67 + (6 * 8)
        self.step_count = 0
        self.move_discharge = move_discharge
        self.idle_discharge = idle_discharge
        self.charge_rate = charge_rate
        self.low_battery_threshold = low_battery_threshold
        self.charge_safety_margin = charge_safety_margin

        self.matrix: List[List[int]] = [
            [Cell.SPACE for _ in range(self.width)] for _ in range(self.height)
        ]
        self.available_boxes: List[Position] = []
        self.boxes_left = 0
        self.agent_states: List[AgentState] = []
        self.start_positions: List[Position] = []

        self._populate_walls()
        self._populate_agents(agent_count=agent_count, battery_capacity=battery_capacity)
        self._populate_boxes()
        self.dropoff_points = self._build_dropoff_points()
        self.charging_stations = set(charging_stations or self._build_default_charging_stations())

    def clone(self) -> "WarehouseEnvironment":
        clone_env = WarehouseEnvironment(
            wall_offset=self.wall_offset,
            box_offset=self.box_offset,
            n=self.n,
            agent_count=0,
            battery_capacity=1,
            move_discharge=self.move_discharge,
            idle_discharge=self.idle_discharge,
            charge_rate=self.charge_rate,
            charging_stations=sorted(self.charging_stations),
            low_battery_threshold=self.low_battery_threshold,
            charge_safety_margin=self.charge_safety_margin,
        )
        clone_env.height = self.height
        clone_env.width = self.width
        clone_env.step_count = self.step_count
        clone_env.matrix = [row[:] for row in self.matrix]
        clone_env.available_boxes = list(self.available_boxes)
        clone_env.boxes_left = self.boxes_left
        clone_env.agent_states = [AgentState(**vars(agent)) for agent in self.agent_states]
        clone_env.start_positions = list(self.start_positions)
        clone_env.dropoff_points = list(self.dropoff_points)
        clone_env.charging_stations = set(self.charging_stations)
        return clone_env

    def _populate_walls(self) -> None:
        for i in range(self.height):
            fill_value = Cell.WALL if i in (0, self.height - 1) else Cell.SPACE
            for j in range(self.width):
                self.matrix[i][j] = int(fill_value)
            self.matrix[i][0] = int(Cell.WALL)
            self.matrix[i][self.width - 1] = int(Cell.WALL)

    def _populate_agents(self, agent_count: int, battery_capacity: int) -> None:
        placed_agents = 0
        column = 2
        while placed_agents < agent_count:
            for row in (1, 3, self.height - 4, self.height - 2):
                if placed_agents >= agent_count:
                    break
                pos = (row, column)
                self.agent_states.append(
                    AgentState(position=pos, battery=battery_capacity, max_battery=battery_capacity)
                )
                self.start_positions.append(pos)
                placed_agents += 1
            column += 2

    def _populate_boxes(self) -> None:
        for i in range(6, self.height - 2):
            if (i - 3) % 3 == 0:
                for j in range(4, self.width - 6):
                    if (j - 4) % 8 != 0:
                        pos = (i - 1, j + 1)
                        self.matrix[pos[0]][pos[1]] = int(Cell.BOX)
                        self.available_boxes.append(pos)
                        self.boxes_left += 1

    def _build_dropoff_points(self) -> List[Position]:
        points: List[Position] = []
        for i in range(4, self.height - 4):
            points.append((i, 1))
            points.append((i, self.width - 2))
        return points

    def _build_default_charging_stations(self) -> List[Position]:
        rows = (2, self.height // 2, self.height - 3)
        return [(row, 2) for row in rows] + [(row, self.width - 3) for row in rows]

    def get_blocked_cells(self) -> set[Position]:
        blocked = set()
        for r in range(self.height):
            for c in range(self.width):
                if self.matrix[r][c] in (Cell.WALL, Cell.BOX):
                    blocked.add((r, c))
        return blocked

    def is_done(self) -> bool:
        return self.boxes_left == 0

    def get_agent_batteries(self) -> List[int]:
        return [agent.battery for agent in self.agent_states]

    def get_agent_values(self) -> List[int]:
        return [1 if agent.carrying else -1 for agent in self.agent_states]

    def get_observation(self) -> Dict[str, object]:
        return {
            "step_count": self.step_count,
            "boxes_left": self.boxes_left,
            "agent_positions": [agent.position for agent in self.agent_states],
            "agent_batteries": self.get_agent_batteries(),
            "agent_carrying": [agent.carrying for agent in self.agent_states],
            "agent_targets": [agent.assigned_target for agent in self.agent_states],
            "charging_stations": sorted(self.charging_stations),
            "dropoff_points": list(self.dropoff_points),
        }

    def _discount(self) -> float:
        return self.DISCOUNT_FACTOR ** self.step_count

    def _is_charging_station(self, pos: Position) -> bool:
        return pos in self.charging_stations

    def _valid_destination(self, new_pos: Position, target: Position | None) -> bool:
        row, col = new_pos
        cell = self.matrix[row][col]
        if cell == Cell.WALL:
            return False
        if cell == Cell.BOX and target is not None and new_pos != target:
            return False
        if cell == Cell.BOX and target is None:
            return False
        return True

    def _next_position(self, pos: Position, action: int) -> Position:
        dr, dc = MOVES[action]
        return (pos[0] + dr, pos[1] + dc)

    def nearest_charging_station_distance(self, position: Position) -> int | None:
        blocked = self.get_blocked_cells()
        best: int | None = None
        for charger in self.charging_stations:
            distance = path_length(position, charger, blocked, self.height, self.width)
            if distance is not None and (best is None or distance < best):
                best = distance
        return best

    def nearest_charging_station(self, position: Position) -> Position:
        blocked = self.get_blocked_cells()
        best_station = min(
            self.charging_stations,
            key=lambda charger: path_length(position, charger, blocked, self.height, self.width) or 10**9,
        )
        return best_station

    def battery_feasible_actions(self, agent_idx: int, preferred_target: Position | None) -> List[int]:
        agent = self.agent_states[agent_idx]
        blocked = self.get_blocked_cells()
        feasible: List[int] = []
        for action in Action:
            discharge = self.idle_discharge if action == Action.STAY else self.move_discharge
            if agent.battery < discharge:
                continue
            nxt = self._next_position(agent.position, int(action))
            if not self._valid_destination(nxt, preferred_target):
                nxt = agent.position
            nearest_charge = self.nearest_charging_station_distance(nxt)
            if nearest_charge is None:
                continue
            post_move_battery = agent.battery - discharge
            if post_move_battery + self.charge_rate < nearest_charge + self.charge_safety_margin:
                if not self._is_charging_station(nxt):
                    continue
            feasible.append(int(action))
        return feasible or [0]

    def assign_initial_targets(self) -> List[Position]:
        boxes = list(self.available_boxes)
        targets: List[Position] = []
        for idx, agent in enumerate(self.agent_states):
            target = boxes[idx]
            targets.append(target)
            agent.assigned_target = target
        for target in targets:
            self.available_boxes.remove(target)
        return targets

    def assign_box_target(self, agent_idx: int) -> Position | None:
        if not self.available_boxes:
            return None
        agent = self.agent_states[agent_idx]
        blocked = self.get_blocked_cells()
        target = min(
            self.available_boxes,
            key=lambda box_pos: path_length(agent.position, box_pos, blocked, self.height, self.width) or 10**9,
        )
        self.available_boxes.remove(target)
        self.agent_states[agent_idx].assigned_target = target
        return target

    def update_targets(self, before_values: Sequence[int], targets: List[Position | None]) -> List[bool]:
        after_values = self.get_agent_values()
        updated = [False] * len(self.agent_states)
        for idx, agent in enumerate(self.agent_states):
            if after_values[idx] > before_values[idx]:
                target = min(self.dropoff_points, key=lambda point: manhattan_distance(agent.position, point))
                targets[idx] = target
                agent.assigned_target = target
                updated[idx] = True
            elif after_values[idx] < before_values[idx]:
                target = self.assign_box_target(idx)
                targets[idx] = target
                updated[idx] = True
        return updated

    def step(self, actions: Sequence[int], targets: Sequence[Position | None]) -> Tuple[float, StepInfo]:
        if len(actions) != len(self.agent_states) or len(targets) != len(self.agent_states):
            raise ValueError("actions and targets must match the number of agents")

        new_positions: List[Position] = []
        cost = 0.0
        occupancy: Dict[Position, List[int]] = {}
        out_of_battery: List[int] = []
        collision_agents: set[int] = set()

        for agent_idx, agent in enumerate(self.agent_states):
            action = Action(actions[agent_idx])
            required_energy = self.idle_discharge if action == Action.STAY else self.move_discharge
            if agent.battery < required_energy:
                out_of_battery.append(agent_idx)
                action = Action.STAY
                required_energy = self.idle_discharge

            preferred_target = targets[agent_idx]
            candidate = self._next_position(agent.position, int(action))
            if not self._valid_destination(candidate, preferred_target):
                candidate = agent.position
            new_positions.append(candidate)
            occupancy.setdefault(candidate, []).append(agent_idx)

        for position, agents in occupancy.items():
            if len(agents) > 1:
                cost += self.COLLISION_COST * self._discount()
                collision_agents.update(agents)

        for idx, old_agent in enumerate(self.agent_states):
            if new_positions[idx] == old_agent.position:
                continue
            for jdx, other_agent in enumerate(self.agent_states):
                if idx == jdx:
                    continue
                if new_positions[idx] == other_agent.position and new_positions[jdx] == old_agent.position:
                    cost += self.COLLISION_COST * self._discount()
                    collision_agents.update({idx, jdx})

        delivered_agents: List[int] = []
        picked_agents: List[int] = []
        target_types: List[str] = []

        for idx, agent in enumerate(self.agent_states):
            action = Action(actions[idx])
            moved = new_positions[idx] != agent.position
            discharge = self.idle_discharge if action == Action.STAY else self.move_discharge
            if idx in out_of_battery:
                discharge = 0

            if moved:
                cost += self.STEP_COST

            agent.battery = max(0, agent.battery - discharge)
            agent.position = new_positions[idx]

            target = targets[idx]
            if target in self.charging_stations:
                target_types.append("charge")
            elif target in self.dropoff_points:
                target_types.append("dropoff")
            elif target is not None:
                target_types.append("box")
            else:
                target_types.append("none")

            if target is not None and agent.position == target and moved:
                if self.matrix[agent.position[0]][agent.position[1]] == Cell.BOX and not agent.carrying:
                    agent.carrying = True
                    self.matrix[agent.position[0]][agent.position[1]] = int(Cell.SPACE)
                    cost += self.PICKUP_REWARD * self._discount()
                    picked_agents.append(idx)
                elif agent.carrying and agent.position in self.dropoff_points:
                    agent.carrying = False
                    agent.delivered_boxes += 1
                    self.boxes_left -= 1
                    cost += self.DROPOFF_REWARD * self._discount()
                    delivered_agents.append(idx)
                elif agent.position == self.start_positions[idx]:
                    cost += self.RETURN_TO_START_REWARD * self._discount()

            nearest_charge = self.nearest_charging_station_distance(agent.position)
            if nearest_charge is not None and agent.battery < nearest_charge + self.charge_safety_margin:
                cost += self.LOW_BATTERY_PENALTY * self._discount()

            if self._is_charging_station(agent.position):
                agent.battery = min(agent.max_battery, agent.battery + self.charge_rate)

        self.step_count += 1
        info = StepInfo(
            picked_agents=picked_agents,
            delivered_agents=delivered_agents,
            out_of_battery=out_of_battery,
            collision_agents=sorted(collision_agents),
            charging_stations=sorted(self.charging_stations),
            batteries=self.get_agent_batteries(),
            target_types=target_types,
        )
        return cost, info

    def shortest_actions_to_target(self, agent_idx: int, target: Position) -> List[int]:
        agent = self.agent_states[agent_idx]
        blocked = self.get_blocked_cells()
        return astar_actions(agent.position, target, blocked, self.height, self.width)

    def render_ascii(self, agents_as_numbers: bool = True) -> str:
        agent_positions = {agent.position: idx for idx, agent in enumerate(self.agent_states)}
        rows: List[str] = []
        for r in range(self.height):
            tokens: List[str] = []
            for c in range(self.width):
                pos = (r, c)
                if pos in agent_positions:
                    idx = agent_positions[pos]
                    agent = self.agent_states[idx]
                    suffix = "*" if agent.carrying else ""
                    tokens.append(f"A{idx}{suffix}" if agents_as_numbers else f"A{suffix}")
                elif pos in self.charging_stations:
                    tokens.append("C")
                elif pos in self.dropoff_points:
                    tokens.append("D")
                elif self.matrix[r][c] == Cell.WALL:
                    tokens.append("#")
                elif self.matrix[r][c] == Cell.BOX:
                    tokens.append("B")
                else:
                    tokens.append(".")
            rows.append(" ".join(tokens))
        return "\n".join(rows)


__all__ = [
    "Action",
    "AgentState",
    "Cell",
    "StepInfo",
    "WarehouseEnvironment",
]
