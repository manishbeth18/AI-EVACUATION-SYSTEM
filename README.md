# 🔥 Smart Emergency Evacuation System

A real-time AI-based evacuation simulation using a Risk-Aware A* Algorithm with dynamic fire and heat propagation.

This project demonstrates how classical pathfinding algorithms can be extended with risk modeling and dynamic replanning to simulate intelligent emergency evacuation in hazardous environments.

---

## 🚀 Features

- A* Pathfinding Algorithm
- Risk-Aware Navigation
- Dynamic Fire Spreading
- Heat / Smoke Propagation
- Real-Time Path Replanning
- Interactive Grid-Based Environment
- Visual Simulation using Pygame
- User-Controlled Scenario Creation

---

## 🧠 Core Concept

Traditional pathfinding focuses only on the shortest path.

This project improves decision-making by incorporating environmental danger into the path cost function.

Formula used:

f(n) = g(n) + h(n) + λ * Risk(n)

Where:
- g(n) → Distance travelled from start
- h(n) → Manhattan heuristic distance to exit
- Risk(n) → Hazard penalty (fire / heat proximity)
- λ → Risk weight factor

This allows the agent to choose safer paths instead of only shortest paths.

---

## 🔥 Hazard Simulation

The system includes a probabilistic hazard model:

- Fire spreads dynamically over time
- Heat/smoke propagates faster than fire
- Nearby danger increases traversal cost
- Path updates continuously as the environment changes

---

## 🎮 Controls

| Key | Function |
|------|----------|
| W | Wall Mode |
| S | Set Start Position |
| E | Set Exit |
| F | Place Fire |
| C | Clear Entire Grid |
| R | Reset Path Only |
| SPACE | Pause / Resume Fire Spread |
| Left Click | Place Object |
| Right Click | Erase Cell |

---

## 🛠️ Technologies Used

- Python
- Pygame
- A* Search Algorithm
- Graph-Based Pathfinding
- Cellular Automaton Fire Simulation

---
