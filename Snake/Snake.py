import turtle
import Misc


# This is a linked queue
# Where the back end will push and the front end will pop
class Node(turtle.Turtle):
    """
    Node for the snake
    """
    # Establishes the data of the node
    # An array of custom coordinates
    snake_coordinates: []
    # An address to the next node
    next_node: None
    # The node's turtle
    node_turtle: turtle.Turtle()

    def __init__(self, coordinates, next_node):
        """
        Constructor for the node
        :param coordinates: Coordinates of the node
        :param next_node: Address to the next node in the queue
        """
        # Goes to the turtle's superclass to construct one unique to the node
        super().__init__("turtle")
        # Sets the node's coordinates to the parameter
        self.snake_coordinates = coordinates
        # Sets the node's next address to the parameter
        self.next_node = next_node
        # Sets the node's turtle to the one constructed
        self.node_turtle = super()

    def get_data(self):
        """
        Gives the coordinates of the node
        :return: coordinates
        """
        # Return the node's coordinate data
        return self.snake_coordinates

    def set_data(self, new_coordinates):
        """
        Sets the coordinates of the node
        :param new_coordinates: The new coordinates
        :return: None
        """
        # Sets the newCoordinates parameter to the coordinates data
        self.snake_coordinates = new_coordinates

    def get_next_node(self):
        """
        Returns the node's address to the next node in the queue
        :return: Address of next node
        """
        # Return the address of the next node
        return self.next_node

    def set_next_node(self, next_node):
        """
        Sets the destination address of the node
        :param next_node: Address
        :return: None
        """
        # Sets the nextNode parameter to the address data
        self.next_node = next_node

    def place_block(self):
        """
        Places a new block at the end of the back block
        :return: None
        """
        # Uses the hash map to determine the screen's coordinates of the custom coordinates
        map_coordinates = Misc.map_coordinates(self.snake_coordinates[0], self.snake_coordinates[1])
        # Clears the turtle's drawings
        self.node_turtle.reset()
        # Stops the turtle from drawing when moving
        self.node_turtle.penup()
        # Sets the turtle to green as the head
        self.node_turtle.color("green")
        # Makes the turtle a square
        self.node_turtle.shape("square")
        # Sets the turtle to be 17 x 17
        self.node_turtle.shapesize(0.85, 0.85)
        # Sends the turtle to the new coordinates
        self.node_turtle.goto(map_coordinates[0], map_coordinates[1])
        # Updates the screen
        Misc.screen.update()


class Snake:
    """
    Linked Queue of the snake
    """
    # Sets the node variables and the length of the snake
    first_node: Node
    last_node: Node
    length = 0

    def start_snake(self, first_entry, middle_entry, last_entry):
        """
        Places each snake block in the specified coordinates
        :param first_entry:  for the back block
        :param middle_entry: Coordinates for the middle block
        :param last_entry: Coordinates for the front block
        :return: None
        """
        # Constructs the front node
        last_node = Node(last_entry, None)
        # Places the front node
        last_node.place_block()
        # Constructs the middle node
        middle_node = Node(middle_entry, last_node)
        # Places the middle node
        middle_node.place_block()
        # Sets the color of the middle block as white
        middle_node.color("white")
        # Constructs the back node
        first_node = Node(first_entry, middle_node)
        # Places the back node
        first_node.place_block()
        # Sets the color of the last block as white
        first_node.color("white")
        # Sets the length to 3
        self.length = 3
        # Sets the corresponding nodes to the global variables
        self.first_node = first_node
        self.last_node = last_node

    def add_block(self):
        """
        Adds a new block to the snake
        :return: None
        """
        # Establishes a new node and sets it to have the same data as the first node, and to point at the first node
        new_node = Node(self.first_node.get_data(), self.first_node)
        # Sets the new node as the first node
        self.first_node = new_node
        # Increments the length
        self.length += 1

    def move_block(self, direction):
        """
        Moves the back block of the snake in front of the first block
        :param direction: Direction that the block goes in front of the first block
        :return: None
        """
        new_coordinates = []
        # Gets the coordinates for the front block
        last_coordinates = self.last_node.get_data()
        # Changes the coordinates of the back block according to the direction parameter
        if direction == "up":
            new_coordinates = [last_coordinates[0], last_coordinates[1] + 1]
        elif direction == "down":
            new_coordinates = [last_coordinates[0], last_coordinates[1] - 1]
        elif direction == "left":
            new_coordinates = [last_coordinates[0] - 1, last_coordinates[1]]
        elif direction == "right":
            new_coordinates = [last_coordinates[0] + 1, last_coordinates[1]]
        # Points the front block to the back block
        self.last_node.set_next_node(self.first_node)
        # Sets the head to white
        self.last_node.node_turtle.color("white")
        # Sets the 2nd to last block as firstNode
        self.first_node = self.first_node.get_next_node()
        # Sets the last block as the front block
        self.last_node = self.last_node.get_next_node()
        # Erases the block's pointer because it is now in the front
        self.last_node.set_next_node(None)
        # Sets the block's coordinates accordingly
        self.last_node.set_data(new_coordinates)
        # Places the block at its new coordinates
        self.last_node.place_block()

    def is_collided(self, size, location):
        """
        Determines if the snake hits itself or a wall
        :return: Returns a boolean
        """
        # Sets the node being looked at
        inner_node = self.first_node
        # Looks through all the nodes and compares them to the front one to see if they are in the same coordinates
        for _ in range(size):
            # If the front node is in the same coordinates as any other node
            if location == inner_node.get_data():
                return True
            else:
                # Look at the next node
                inner_node = inner_node.get_next_node()
        # If the front node hits the border of the screen
        if location[0] > 39 or location[0] < 0 or location[1] > 23 or location[1] < 0:
            return True
        return False
