import turtle
import random
import Snake
import functools

# Establishes screen width and height
screen_width = 840
screen_height = 520
# Establishes screen and turtle variables
screen = turtle.Screen()
setup = turtle.Turtle()
apple = turtle.Turtle()
# Writes the apple's custom coordinates
apple_position = [0, 0]
# Establishes the snake class
snake = Snake.Snake()
# Establishes the direction that the snake is going
direction = "right"
# Boolean which says whether the snake is in an allowed position
game_on = True


def screen_setup():
    """
    Sets up the initial screen for Snake
    :return: None
    """
    # Sets the width and height of the screen
    screen.setup(screen_width, screen_height)
    # Sets the background color of the screen
    screen.bgcolor("black")
    # Sets the title
    screen.title("Snake")
    # Sets the turtle's speed to automatic
    screen.tracer(0)
    # Creates a border around the screen's edge
    border()
    # Sets up the coordinates of the first 3 snake blocks
    start_snake()
    # Sets up the apple turtle
    setup_apple()
    # Randomly places the apple in a grid spot
    place_apple()


def screen_movement():
    """
    Starts the movement code for the snake
    :return: None
    """
    # Changes the direction variable based on what key is pressed
    direction_block()
    # Moves the snake and determines if the snake collides with anything
    move_block(direction)


def border():
    """
    Sets up the screen with a white outer border
    :return: None
    """
    # Stops the pen from drawing when moving
    setup.penup()
    # Hides turtle
    setup.ht()
    # Goes to the top right corner of the outer border
    setup.goto(screen_width / 2, screen_height / 2)
    # Sets color to be white inside the border
    setup.color("white")
    # Begins the filling of the border
    setup.begin_fill()
    # Goes around the screen with the turtle
    make_square(screen_width, screen_height)
    # Fills in the border with white
    setup.end_fill()
    # Sets the inside of the screen to be black
    setup.color("black")
    # Goes to the inner corner of the border
    setup.goto((screen_width / 2) - 18, (screen_height / 2) - 19)
    # Starts filling the inside screen with black
    setup.begin_fill()
    # Goes around the screen for the inner border's side
    make_square(screen_width - 40, screen_height - 40)
    # Fills the inside screen with black
    setup.end_fill()


def make_square(width, height):
    """
    Goes in a square from where the turtle starts
    :param width: Width of the square
    :param height: Height of the square
    :return: None
    """
    # Loops the two sides of the square starting in the top right corner
    for _ in range(2):
        # Turn right
        setup.right(90)
        # Go along the vertical side of the screen
        setup.fd(height)
        # Turn right
        setup.right(90)
        # Go along the horizontal side of the screen
        setup.fd(width)


def setup_apple():
    """
    Sets the apple's turtle up
    :return: None
    """
    # Sets the apple's color to red
    apple.color("red")
    # Stops drawing when it is moving
    apple.penup()
    # Sets the apple to a square
    apple.shape("square")
    # Sets the apple to a 17 x 17 square
    apple.shapesize(0.85, 0.85)


def place_apple():
    """
    Places the apple randomly in one of the empty grids
    :return: None
    """
    # Determines which column the apple will appear in
    x = random.randint(0, 39)
    # Determines which row the apple will appear in
    y = random.randint(0, 23)
    # Applies the current apple's location to the applePosition variable
    apple_position[0] = x
    apple_position[1] = y
    # Goes through a hashmap to change the x value to it's corresponding coordinate
    coordinates = map_coordinates(x, y)
    # If the apple's coordinates is in an unoccupied spot
    if not snake.is_collided(snake.length, apple_position):
        # Go to that spot
        apple.goto(coordinates[0], coordinates[1])
    else:
        # Find a new spot
        place_apple()


def map_coordinates(x, y):
    """
    Hashmap which changes from the grid coordinates to the screen coordinates
    :param x: Column which apple is appearing
    :param y: Row which apple is appearing
    :return: A coordinate array with x at pos(0) and y at pos(1)
    """
    # Changes x grid coordinate to x screen coordinate
    x = (-19 * 20) + (x * 20) - 8.5
    # Changes y grid coordinate to y screen coordinate
    y = (-11 * 20) + (y * 20) - 8.5
    # Puts x and y into an array
    coordinates = [x, y]
    # Returns the array
    return coordinates


def start_snake():
    """
    Creates the initial 3-block snake
    :return: None
    """
    # The coordinates for each block
    first_entry = [17, 11]
    middle_entry = [18, 11]
    last_entry = [19, 11]
    # Goes into the Snake class to create each block
    snake.start_snake(first_entry, middle_entry, last_entry)


def move_block(inner_direction):
    """
    Moves the snake in the specified direction
    :param inner_direction:  which the snake goes
    :return: None
    """
    # Moves the back block in front of the first block in the specified direction
    snake.move_block(inner_direction)
    # Checks if snake is hitting anything
    snake_collision()
    # If the snake is not collided
    if game_on:
        # Call the moveBlock method in 125 ms
        screen.ontimer(functools.partial(move_block, direction), 125)
    else:
        # Fail the game
        fail_game()


def direction_block():
    """
    Sets keyboard inputs to move the snake
    :return: None
    """
    # Calls directionChanger with the specified direction when the specified key is pressed
    screen.onkeypress(functools.partial(direction_changer, "left"), "a")
    screen.onkeypress(functools.partial(direction_changer, "right"), "d")
    screen.onkeypress(functools.partial(direction_changer, "up"), "w")
    screen.onkeypress(functools.partial(direction_changer, "down"), "s")


def direction_changer(inner_direction):
    """
    Changes the global direction variable
    :param inner_direction: The direction which it is being switched to
    :return: None
    """
    global direction
    # Changes the direction according to the input
    if direction == "up" or direction == "down":
        if inner_direction == "left":
            direction = "left"
        elif inner_direction == "right":
            direction = "right"
    elif direction == "left" or direction == "right":
        if inner_direction == "up":
            direction = "up"
        elif inner_direction == "down":
            direction = "down"


def snake_collision():
    """
    Checks if snake is hitting the apple, or is hitting itself/wall
    :return: None
    """
    global game_on
    # If the snake eats the apple
    if snake.last_node.get_data() == apple_position:
        # Add a snake block
        snake.add_block()
        # Move the apple
        place_apple()
    # If the snake hits itself/wall
    if snake.is_collided(snake.length - 1, snake.last_node.get_data()):
        # Fail the game
        game_on = False


def fail_game():
    """
    Stops the game and says GameOver
    :return: None
    """
    # Sets up a turtle for GameOver
    writer = turtle.Turtle()
    # Sets the word's color to be red
    writer.color("red")
    # Puts the words on top of the screen
    writer.goto(0, 100)
    # Writes GameOver
    writer.write("GameOver", font=('Arial', 100, 'normal'), align="center")


def play_game():
    """
    Starts the main-game loop and updates the screen
    :return: None
    """
    # Updates the screen
    screen.update()
    # Listens for keyboard inputs
    screen.listen()
    # Starts the main-game loop
    screen.mainloop()
