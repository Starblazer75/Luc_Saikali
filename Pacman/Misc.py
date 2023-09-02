import turtle
import Map
import functools
from Movers import Movers

screen_width = 570
screen_height = 660
screen = turtle.Screen()
drawer = turtle.Turtle()
pac_man = Movers("yellow", [0, 0])
direction = "right"


# 17 x 19
def screen_setup():
    screen.setup(screen_width, screen_height)
    screen.bgcolor("black")
    screen.title("PacMan")
    screen.tracer(0)
    border()
    # make_grid()
    Map.make_map()
    movement()


def initialize_ghost():
    blue_ghost = Movers("blue", [8, 10])
    pink_ghost = Movers("pink", [7, 10])
    green_ghost = Movers("green", [9, 10])
    red_ghost = Movers("red", [8, 12])


def border():
    drawer.color("white")
    drawer.ht()
    drawer.penup()
    drawer.speed(9)
    drawer.goto(screen_width / 2, screen_height / 2)
    drawer.begin_fill()
    make_square(screen_width, screen_height)
    drawer.end_fill()
    drawer.color("black")
    drawer.goto((screen_width / 2) - 30, (screen_height / 2) - 30)
    drawer.begin_fill()
    make_square(screen_width - 60, screen_height - 60)
    drawer.end_fill()
    drawer.goto(screen_width / 2, (screen_height / 2) - (10 * 30))
    drawer.begin_fill()
    make_square(30, 29)
    drawer.end_fill()
    drawer.goto(-1 * (screen_width / 2) + 30, (screen_height / 2) - (10 * 30))
    drawer.begin_fill()
    make_square(30, 29)
    drawer.end_fill()


def make_square(width, height):
    drawer.setheading(270)
    for _ in range(2):
        drawer.fd(height)
        drawer.right(90)
        drawer.fd(width)
        drawer.right(90)


def make_grid():
    drawer.goto((screen_width / 2) - 30, (screen_height / 2) - 25)
    drawer.pendown()
    drawer.color("white")
    make_vertical()
    make_horizontal()


def make_vertical():
    for _ in range(9):
        drawer.setheading(270)
        drawer.fd(screen_height - 50)
        drawer.right(90)
        drawer.fd(30)
        drawer.right(90)
        drawer.fd(screen_height - 50)
        drawer.left(90)
        drawer.fd(30)


def make_horizontal():
    drawer.setheading(270)
    drawer.fd(5)
    drawer.left(90)
    drawer.fd(25)
    for _ in range(11):
        drawer.setheading(0)
        drawer.fd(screen_width - 50)
        drawer.right(90)
        drawer.fd(30)
        drawer.right(90)
        drawer.fd(screen_width - 50)
        drawer.left(90)
        drawer.fd(30)


def movement():
    screen.onkeypress(functools.partial(pac_man.direction_changer, "up"), "w")
    screen.onkeypress(functools.partial(pac_man.direction_changer, "down"), "s")
    screen.onkeypress(functools.partial(pac_man.direction_changer, "left"), "a")
    screen.onkeypress(functools.partial(pac_man.direction_changer, "right"), "d")
    auto_movement()


def auto_movement():
    pac_man.move()
    screen.ontimer(auto_movement, 200)


def play_game():
    screen.listen()
    screen.update()
    screen.mainloop()
