import Map


class Node:
    def __init__(self, position):
        self.parent = None
        self.children = []
        self.position = position
        self.weight = 0
        self.heuristic = 0
        self.total_cost = 0

    def find_valid_neighbors(self, map_obj):
        """
        Find valid neighboring nodes in the map.

        Args:
            map_obj (Map_Obj): The map object.

        Returns:
            list: List of valid neighboring nodes.
        """
        neighbors = [
            [self.position[0] - 1, self.position[1]],
            [self.position[0] + 1, self.position[1]],
            [self.position[0], self.position[1] - 1],
            [self.position[0], self.position[1] + 1],
        ]
        valid_neighbors = []
        for neighbor in neighbors:
            try:
                cell_value = map_obj.get_cell_value(neighbor)
                if cell_value >= 0:
                    valid_neighbors.append(Node(neighbor))
            except IndexError:
                continue
        return valid_neighbors


class AStarSearch:
    def __init__(self, map_obj):
        self.map_obj = map_obj
        self.closed = []
        self.opened = []

    def euclidean_distance(self, position, goal_position):
        """
        Calculate the Euclidean distance between two positions.

        Args:
            position (list): The current position.
            goal_position (list): The goal position.

        Returns:
            int: The Euclidean distance.
        """
        return abs(position[0] - goal_position[0]) + abs(position[1] - goal_position[1])

    def find_path(self):
        """
        Find the shortest path using A* search algorithm.

        Returns:
            tuple: A tuple containing the final node and a status message.
        """
        start_node = Node(self.map_obj.get_start_pos())
        self.opened.append(start_node)
        current_node = start_node

        while current_node.position != self.map_obj.get_goal_pos():
            if not self.opened:
                return None, "Failed"

            current_node = self.opened.pop()
            self.closed.append(current_node)

            if current_node.position == self.map_obj.get_goal_pos():
                return current_node, "Success"

            neighbors = current_node.find_valid_neighbors(self.map_obj)
            for neighbor in neighbors:
                new_neighbor = self.get_or_create_node(neighbor)
                if new_neighbor not in self.opened and new_neighbor not in self.closed:
                    self.attach_and_evaluate(new_neighbor, current_node)
                    self.opened.append(new_neighbor)
                    self.opened.sort(key=lambda node: node.total_cost, reverse=True)
                elif current_node.weight + self.map_obj.get_cell_value(new_neighbor.position) < new_neighbor.weight:
                    self.attach_and_evaluate(new_neighbor, current_node)
                    if new_neighbor in self.closed:
                        self.propagate_path_improvements(new_neighbor)

    def get_or_create_node(self, node):
        """
        Get an existing node from opened or closed lists, or create a new one.

        Args:
            node (Node): The node to find or create.

        Returns:
            Node: The existing or new node.
        """
        for existing_node in self.opened + self.closed:
            if existing_node.position == node.position:
                return existing_node
        return node

    def attach_and_evaluate(self, child, parent):
        """
        Attach a child node to its parent and evaluate its attributes.

        Args:
            child (Node): The child node.
            parent (Node): The parent node.
        """
        child.parent = parent
        child.weight = parent.weight + self.map_obj.get_cell_value(child.position)
        child.heuristic = self.euclidean_distance(child.position, self.map_obj.get_goal_pos())
        child.total_cost = child.weight + child.heuristic

    def propagate_path_improvements(self, node):
        """
        Propagate path improvements for a node and its children.

        Args:
            node (Node): The node to propagate improvements from.
        """
        for child_node in node.children:
            if node.weight + self.map_obj.get_cell_value(child_node.position) < child_node.weight:
                child_node.parent = node
                self.attach_and_evaluate(child_node, node)
                self.propagate_path_improvements(child_node)

    def backtrack_and_visualize(self, final_node):
        """
        Backtrack from the final node to visualize the path on the map.

        Args:
            final_node (Node): The final node.

        Returns:
            str: The map with the path.
        """
        current_node = final_node
        while current_node.parent is not None:
            self.map_obj.set_cell_value(current_node.position, 5)
            current_node = current_node.parent
        return self.map_obj.show_map()

    def find_shortest_path(self):
        """
        Find and visualize the shortest path.

        Returns:
            str: The map with the path, or a failure message.
        """
        final_node, status = self.find_path()
        if status == "Success":
            return self.backtrack_and_visualize(final_node)
        else:
            print("Failed")


if __name__ == "__main__":
    map_task = Map.Map_Obj(task=4)
    astar = AStarSearch(map_task)
    astar.find_shortest_path()
