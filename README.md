#  Smart Emergency Evacuation System

This project simulates emergency evacuation using a risk-aware A* algorithm in a dynamic environment.

##  Features
- Grid-based simulation
- A* pathfinding
- Risk-aware decision making
- Dynamic fire spread
- Heat/smoke zones
- Real-time path replanning

##  Algorithm
We use a modified A* algorithm:

f(n) = g(n) + h(n) + α * Risk(n)

- g(n): cost from start
- h(n): heuristic (Manhattan distance)
- Risk(n): hazard penalty

##  Controls
- W → Walls
- S → Start
- E → End
- F → Fire
- Left click → Place
- Right click → Erase

##  Run
```bash
pip install -r requirements.txt
python main.py
