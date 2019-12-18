# Paolo Takagi-Atilano, Nov 15th

from collections import deque

# search node for search
class SearchNode:
    def __init__(self, state, parent=None):
        self.state = state
        self.parent = parent

        if self.parent is None:
            self.depth = 0
        else:
            self.depth = self.parent.depth + 1


# backchaining for solution pathway
def backchain(destination_node):
    state_list = deque()
    state_list.appendleft(destination_node.state)
    prev = destination_node
    while prev.parent is not None:
        prev = prev.parent
        state_list.appendleft(prev.state)
    return state_list


# actual bfs search algorithm
def bfs_search(start, goal, nodes, neighbors):
    print(neighbors)
    print("START: ", start)
    print("GOAL: ", goal)
    queue = deque()
    visited = set()
    root = SearchNode(start)
    queue.append(root)
    visited.add(tuple(root.state))

    while queue:
        curr = queue.pop()

        for state in neighbors[tuple(curr.state)]:
            if state == tuple(goal):
                return backchain(SearchNode(state, curr))
            else:
                if state not in visited:
                    child = SearchNode(state, curr)
                    queue.append(child)
                    visited.add(tuple(state))
    return None


# returns successors of given node
def get_successors(node, neighbors):
    return neighbors[node]