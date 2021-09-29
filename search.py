"""
Map Search
"""

import comp140_module7 as maps

class Queue:
    """
    A simple implementation of a FIFO queue.
    """

    def __init__(self):
        """
        Initialize the queue.
        """
        self._queue = []

    def __len__(self):
        """
        Returns: an integer representing the number of items in the queue.
        """
        return len(self._queue)

    def __str__(self):
        """
        Returns: a string representation of the queue.
        """
        return str(self._queue)

    def push(self, item):
        """
        Add item to the queue.

        input:
            - item: any data type that's valid in a list
        """
        self._queue.insert(0, item)

    def pop(self):
        """
        Remove the least recently added item.

        Assumes that there is at least one element in the queue.  It
        is an error if there is not.  You do not need to check for
        this condition.

        Returns: the least recently added item.
        """
        return self._queue.pop()

    def clear(self):
        """
        Remove all items from the queue.
        """
        self._queue = []
        
    
class Stack:
    """
    A simple implementation of a LIFO stack.
    """

    def __init__(self):
        """
        Initialize the stack.
        """
        self._stack = []

    def __len__(self):
        """
        Returns: an integer representing the number of items in the stack.
        """
        return len(self._stack)

    def __str__(self):
        """
        Returns: a string representation of the stack.
        """
        return str(self._stack)

    def push(self, item):
        """
        Add item to the stack.

        input:
            - item: any data type that's valid in a list
        """
        self._stack.append(item)

    def pop(self):
        """
        Remove the most recently added item.

        Assumes that there is at least one element in the stack.  It
        is an error if there is not.  You do not need to check for
        this condition.

        Returns: the most recently added item.
        """
        return self._stack.pop()

    def clear(self):
        """
        Remove all items from the stack.
        """
        self._stack = []


def bfs_dfs(graph, rac_class, start_node, end_node):
    """
    Performs a breadth-first search or a depth-first search on graph
    starting at the start_node.  The rac_class should either be a
    Queue class or a Stack class to select BFS or DFS.

    Completes when end_node is found or entire graph has been
    searched.

    inputs:
        - graph: a directed Graph object representing a street map
        - rac_class: a restricted access container (Queue or Stack) class to
          use for the search
        - start_node: a node in graph representing the start
        - end_node: a node in graph representing the end

    Returns: a dictionary associating each visited node with its parent
    node.
    """
    #initialize the distance and parent dictionaries. 
    rac = rac_class()
    dist = {}
    parent = {}
    nodes = graph.nodes()
    #set distances of nodes to infinity and parents of nodes to be null
    for node in nodes:
        dist[node] = float("inf")
        parent[node] = None
    dist[start_node] = 0
    #push start node first because need while loop to run
    rac.push(start_node)
    while len(rac) > 0:
        node = rac.pop()  
        neighbors = graph.get_neighbors(node)
        for nbr in neighbors:
            #if infinity means unfound, therefore set distance. 
            if dist[nbr] == float("inf"):
                dist[nbr] = dist[node] + 1
                parent[nbr] = node  
                rac.push(nbr)
                #return the parent if reach end node
            if nbr == end_node:
                return parent
    #return if loop through all nodes. 
    return parent
    
def dfs(graph, start_node, end_node, parent):
    """
    Performs a recursive depth-first search on graph starting at the
    start_node.
    
    Completes when end_node is found or entire graph has been
    searched.
    
    inputs:
        - graph: a directed Graph object representing a street map
        - start_node: a node in graph representing the start
        - end_node: a node in graph representing the end
        - parent: a dictionary that initially has one entry associating
                  the original start_node with None
    
    Returns: the modified parent dictionary which maps each visited node
    to its parent node
    """
    #base case 1 is when start node is equal to end node 
    if start_node == end_node:
        return parent
    start_neighbors = graph.get_neighbors(start_node)
    has_no_parents = True
    new_neighbors = []
    #loop to see if node has parents or not 
    for node in start_neighbors:
        if node not in parent:
            has_no_parents = False
            new_neighbors.append(node)
    #if no parents, then you have nothing left to search.
    if has_no_parents:
        return parent
    #recursive to loop through the graph. 
    for nbr in new_neighbors:
        parent[nbr] = start_node
        dfs(graph, nbr, end_node, parent)
    return parent

def astar(graph, start_node, end_node,
          edge_distance, straight_line_distance):
    """
    Performs an A* search on graph starting at start_node.

    Completes when end_node is found or entire graph has been
    searched.

    inputs:
        - graph: a directed Graph object representing a street map
        - start_node: a node in graph representing the start
        - end_node: a node in graph representing the end
        - edge_distance: a function which takes two nodes and a graph
                         and returns the actual distance between two
                         neighboring nodes
        - straight_line_distance: a function which takes two nodes and
                         a graph and returns the straight line distance 
                         between two nodes

    Returns: a dictionary associating each visited node with its parent
    node.
    """
    open_set = []
    closed_set = []
    dist = {}
    parents = {}
    open_set.append(start_node)
    parents[start_node] = None
    if start_node == end_node:
        return parents
    #dist in form of [actual cost, heuristic cost]
    dist[start_node] = [0, straight_line_distance(start_node, end_node, graph)]
    while len(open_set) > 0:
        #find lowest cost in open_set
        lowest_cost_node = open_set[0]
        lowest_cost = dist[lowest_cost_node][0] + dist[lowest_cost_node][1]
        if len(open_set) != 1:
            for nodes in open_set[1:]:
                cost = dist[nodes][0] + dist[nodes][1]
                if cost < lowest_cost:
                    lowest_cost = cost
                    lowest_cost_node = nodes
        #find neighbors of deleted node
        neighbors = graph.get_neighbors(lowest_cost_node)
        good_neighbor = []
        for nodes in neighbors:
            if nodes not in closed_set:
                good_neighbor.append(nodes)
        #give distances to nodes
        for nbr in good_neighbor:
            open_set.append(nbr)
            #check if distance is in, if it is, check if the distance is greater.
            if nbr in dist:
                if dist[nbr][0] > (edge_distance(lowest_cost_node, nbr, graph)
                                   + dist[lowest_cost_node][0]):
                    dist[nbr][0] = (edge_distance(lowest_cost_node, nbr, graph)
                                   + dist[lowest_cost_node][0])
                    parents[nbr] = lowest_cost_node
            else:
                dist[nbr] = [edge_distance(lowest_cost_node, nbr, graph)
                             + dist[lowest_cost_node][0], 
                             straight_line_distance(nbr, end_node, graph)]
                parents[nbr] = lowest_cost_node
        open_set.remove(lowest_cost_node)
        closed_set.append(lowest_cost_node)
        if lowest_cost_node == end_node:
            return parents
    return parents

# You can replace functions/classes you have not yet implemented with
# None in the call to "maps.start" below and the other elements will
# work.

maps.start(bfs_dfs, Queue, Stack, dfs, astar)
