import turtle
import Map
import Misc


class Movers(turtle.Turtle):
    node_turtle: turtle.Turtle
    color: ""
    object_coordinates: []
    direction: ""

    def __init__(self, color, coordinates):
        super().__init__("turtle")
        screen = turtle.Screen()
        screen.tracer(0)
        self.direction = "right"
        self.node_turtle = super()
        self.node_turtle.penup()
        self.node_turtle.shape("square")
        self.node_turtle.shapesize(1.5, 1.5)
        self.node_turtle.color(color)
        self.color = color
        self.object_coordinates = coordinates
        coordinates = Map.map_coordinates(coordinates[0], coordinates[1])
        self.node_turtle.goto(coordinates[0], coordinates[1])

    def move(self):
        if self.direction == "left" and self.object_coordinates == [0, 10]:
            self.object_coordinates[0] = 16
            coordinates = Map.map_coordinates(self.object_coordinates[0], self.object_coordinates[1])
            self.node_turtle.goto(coordinates[0], coordinates[1])
        elif self.direction == "right" and self.object_coordinates == [16, 10]:
            self.object_coordinates[0] = 0
            coordinates = Map.map_coordinates(self.object_coordinates[0], self.object_coordinates[1])
            self.node_turtle.goto(coordinates[0], coordinates[1])
        elif self.direction == "up" and Map.collision_array[self.object_coordinates[0]][self.object_coordinates[1] + 1] == 1:
            if self.object_coordinates[1] + 1 <= 19:
                self.object_coordinates[1] += 1
                coordinates = Map.map_coordinates(self.object_coordinates[0], self.object_coordinates[1])
                self.node_turtle.goto(coordinates[0], coordinates[1])
        elif self.direction == "down" and Map.collision_array[self.object_coordinates[0]][self.object_coordinates[1] - 1] == 1:
            if self.object_coordinates[1] - 1 >= 0:
                self.object_coordinates[1] -= 1
                coordinates = Map.map_coordinates(self.object_coordinates[0], self.object_coordinates[1])
                self.node_turtle.goto(coordinates[0], coordinates[1])
        elif self.direction == "left" and Map.collision_array[self.object_coordinates[0] - 1][self.object_coordinates[1]] == 1:
            if self.object_coordinates[0] - 1 >= 0:
                self.object_coordinates[0] -= 1
                coordinates = Map.map_coordinates(self.object_coordinates[0], self.object_coordinates[1])
                self.node_turtle.goto(coordinates[0], coordinates[1])
        elif self.direction == "right" and Map.collision_array[self.object_coordinates[0] + 1][self.object_coordinates[1]] == 1:
            if self.object_coordinates[0] + 1 <= 16:
                self.object_coordinates[0] += 1
                coordinates = Map.map_coordinates(self.object_coordinates[0], self.object_coordinates[1])
                self.node_turtle.goto(coordinates[0], coordinates[1])
        Misc.screen.update()

    def direction_changer(self, new_direction):
        coordinates = [0, 0]
        coordinates[0] = self.object_coordinates[0]
        coordinates[1] = self.object_coordinates[1]
        if new_direction == "up":
            coordinates[1] += 1
        elif new_direction == "down":
            coordinates[1] -= 1
        elif new_direction == "left":
            coordinates[0] -= 1
        elif new_direction == "right":
            coordinates[0] += 1
        if Map.collision_array[coordinates[0]][coordinates[1]] == 1:
            self.direction = new_direction
