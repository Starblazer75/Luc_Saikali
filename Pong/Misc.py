import turtle
import functools

# Initializes the screen
screen = turtle.Screen()
# Initializes the paddles
left_paddle = turtle.Turtle()
right_paddle = turtle.Turtle()
# Initializes the ball
main_ball = turtle.Turtle()
# Initializes the score writer
score_writer = turtle.Turtle()
# Initializes the scores
left_score = 0
right_score = 0
# Initializes the beginning speeds
x_speed = 1.5
y_speed = 1.5
# Value which determines if the game is playing
game_on = True


def screen_setup():
    """
    Sets up the screen
    :return: None
    """
    # Background color
    screen.bgcolor("Black")
    # Title
    screen.title("Pong")
    # Sets speed of all turtles to automatic
    screen.tracer(0)
    # Sets screen size; 800 wide, 600 tall
    screen.setup(800, 600)
    # Creates both paddles
    paddle_setup(left_paddle, "left")
    paddle_setup(right_paddle, "right")
    # Creates the ball
    ball_setup(main_ball)
    # Initializes scoreboard
    score_change(left_score, right_score)
    # Starts the ball moving
    ball_movement()


def paddle_setup(paddle, side):
    """
    Sets up both paddles
    :param paddle: The designated paddle to set up
    :param side: The side that it goes to
    :return: None
    """
    # Stops drawing when moving
    paddle.penup()
    # Sets up shape and color
    paddle.shape("square")
    paddle.color("white")
    # Stretches the paddle to be 100 tall and 20 wide
    paddle.shapesize(5, 1)
    # If side parameter is left, go to the left side
    if side == "left":
        paddle.goto(-350, 0)
    # If side parameter is right, go to the right side
    elif side == "right":
        paddle.goto(350, 0)


def ball_setup(ball):
    """
    Sets up the ball
    :param ball: Turtle of the ball
    :return: None
    """
    # Stops drawing when the ball is moving
    ball.penup()
    # Creates the shape and color of the ball
    # 20 x 20 pixel ball
    ball.color("white")
    ball.shape("square")


def score_change(sub_left_score, sub_right_score):
    """
    Changes the score
    :param sub_left_score: The score which goes on the left side
    :param sub_right_score: The score which goes on the right side
    :return: None
    """
    # Resets the current score
    score_writer.reset()
    # Hides the turtle drawer
    score_writer.hideturtle()
    # Brings the score up to the top and sets the color
    score_writer.sety(225)
    # Makes the score white
    score_writer.color("white")
    # Writes the score centered with a 10 space gap
    score_writer.write(str(sub_left_score) + "          " +
                       str(sub_right_score), align="center",
                       font=('Arial', 40, 'normal'))


def paddle_movement(paddle, direction):
    """
    Moves the paddles
    :param paddle: Designated paddle to move
    :param direction: Direction that it needs to move
    :return: None
    """
    # Gets the y-coordinate of the paddle
    y = paddle.ycor()
    # If direction is up
    if direction == "up":
        # Move the paddle up
        y += 40
    # If the direction is down
    elif direction == "down":
        # Move the paddle down
        y -= 40
    # If the paddle is within the screen's borders
    if 250 > y > -250:
        # Move the paddle
        paddle.sety(y)
    # Update the screen
    screen.update()


def keyboard_inputs():
    """
    Code for the keyboard inputs
    :return: None
    """
    # Calls paddle_movement with the parameters if the key is pressed
    screen.onkeypress(functools.partial(paddle_movement, left_paddle, "up"), "w")
    screen.onkeypress(functools.partial(paddle_movement, left_paddle, "down"), "s")
    screen.onkeypress(functools.partial(paddle_movement, right_paddle, "up"), "Up")
    screen.onkeypress(functools.partial(paddle_movement, right_paddle, "down"), "Down")


def ball_movement():
    """
    Code that moves the ball
    :return: None
    """
    # Determines when it bounces off either horizontal wall of the screen
    ball_height()
    # Determines when the ball hits either vertical wall to score points
    ball_scoring()
    # Determines when the ball collides with a paddle
    ball_collision()
    # Code that moves the ball
    ball_moving()
    # Updates the screen anytime the ball moves
    screen.update()
    # Every 10 milliseconds, this class will be called
    if game_on:
        screen.ontimer(ball_movement, 10)
    else:
        end_game()


def ball_height():
    """
    Determines when the ball should bounce off of the ceiling or floor
    :return: None
    """
    # Establishes x and y as the variables initialized at the beginning
    global x_speed
    global y_speed
    # If the ball hits the top side or the bottom side, reverse it's vertical direction
    if main_ball.ycor() > 290:
        main_ball.sety(290)
        y_speed *= -1
    if main_ball.ycor() < -290:
        main_ball.sety(-290)
        y_speed *= -1


def ball_scoring():
    """
    Determines if someone scores and if someone wins
    :return: None
    """
    # Establishes score_1, score_2, and x_speed as the variables initialized at the beginning
    global left_score
    global right_score
    global x_speed
    global game_on
    # If the ball hits either vertical wall, change the score and write the new one on the screen
    # and reset the speed going in the opposite direction of wall hit
    if main_ball.xcor() > 390:
        left_score += 1
        score_change(left_score, right_score)
        main_ball.goto(0, 0)
        x_speed = -1.5
    if main_ball.xcor() < -390:
        right_score += 1
        score_change(left_score, right_score)
        main_ball.goto(0, 0)
        x_speed = 1.5
    # If either player's score reaches 7, that player wins
    if left_score == 7 or right_score == 7:
        game_on = False


def end_game():
    score_writer.goto(0, 100)
    if left_score == 7:
        score_writer.write("Player 1 Wins!", align="center", font=('Arial', 70, 'normal'))
    elif right_score == 7:
        score_writer.write("Player 2 Wins!", align="center", font=('Arial', 70, 'normal'))


def ball_collision():
    """
    Determines if a ball collides with a paddle
    :return: None
    """
    # Establishes x_speed and y_speed as the variables initialized at the beginning
    global x_speed
    global y_speed
    # If the ball is sharing space with the paddle, go the opposite direction
    # Since the ball and paddle's coordinates are at their center of mass,
    # I add/subtract their coordinates accordingly
    # and increase the ball's speed by 25%
    if main_ball.xcor() - 10 <= left_paddle.xcor() + 10 and left_paddle.ycor() - 50 <= main_ball.ycor() - 10 and main_ball.ycor() + 10 <= left_paddle.ycor() + 50:
        main_ball.setx(left_paddle.xcor() + 30)
        x_speed *= -1.5
    if main_ball.xcor() + 10 >= right_paddle.xcor() - 10 and right_paddle.ycor() - 50 <= main_ball.ycor() - 10 and main_ball.ycor() + 10 <= right_paddle.ycor() + 50:
        main_ball.setx(right_paddle.xcor() - 30)
        x_speed *= -1.5


def ball_moving():
    """
    Code that moves the ball
    :return: None
    """
    # Establishes x_speed and y_speed as the variables initialized at the beginning
    global x_speed
    global y_speed
    # Moves the ball according to the speed
    main_ball.setx(main_ball.xcor() + x_speed)
    main_ball.sety(main_ball.ycor() + y_speed)


def play_game():
    """
    Main game loop
    :return: None
    """
    # Scans keyboard for inputs
    screen.listen()
    # Updates the game's screen
    screen.update()
    # Causes the program to continuously loop until screen is closed
    screen.mainloop()
