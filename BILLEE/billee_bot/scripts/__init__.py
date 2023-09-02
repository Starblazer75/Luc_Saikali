import pygame 

pygame.init()

window_width = 800
window_height = 600
game_display = pygame.display.set_mode((window_width, window_height))
pygame.display.set_caption("Xbox Controller Test")


pygame.joystick.init()

joystick_count = pygame.joystick.get_count()
if joystick_count > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

while True:
    for event in pygame.event.get():
        if event.type == pygame.JOYAXISMOTION:
            # Handle axis motion
            axis = event.axis
            value = event.value
            # Process the axis value as needed

        elif event.type == pygame.JOYBUTTONDOWN:
            # Handle button press
            button = event.button
            print(button)
            # Process the button press as needed

        elif event.type == pygame.JOYBUTTONUP:
            # Handle button release
            button = event.button
            # Process the button release as needed

    # Add any additional game logic or rendering here

    pygame.display.update()

pygame.joystick.quit()
pygame.quit()



