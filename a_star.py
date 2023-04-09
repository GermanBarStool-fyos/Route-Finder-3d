import json
from support import AStarNode

print("""
Assumptions are as follows:
1. Diagonal moves allowed, not just orthogonal
2. All moves cost the same - upward movement costs the same as down/accross/diagonal/orthogonal/etc.
3. Diagonal movement across not-fly zone is allowed
""")

#  Load zones
with open('zones.json') as f:
    zones = json.load(f)

# Initialize usable
astar = AStarNode((0, 0, 0), (20, 20, 20), zones)

#  Start path-finding
print('Starting path-finding')
astar.open_all_neighbor_node(astar.current_node)
while True:
    if astar.current_node == astar.finish:
        break
    if astar.check_for_open():
        astar.open_all_neighbor_node(astar.current_node)
        astar.current_node = astar.get_lowest_cost()
    else:
        raise ValueError('no solution')

# Collect path and display on graph
astar.get_path()
print(f'Path is: {astar.path}')
astar.show_graph()


