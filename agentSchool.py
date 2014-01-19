import pygame
import experience as exp


# Define some colors
black = ( 0, 0, 0)
white = ( 255, 255, 255)
green = ( 0, 255, 0)
red = ( 255, 0, 0)

predatorSize = [50, 50, 10]

#Create a new experience
exp = exp.Experience(1, 10., 0., 2., 300., 300., -300., -300)

pygame.init()

# Set the height and width of the aquarium
size = [600, 600]
screen = pygame.display.set_mode(size)
pygame.display.set_caption("Agent School")
#Loop until the user clicks the close button.
done = False
# Used to manage how fast the screen updates
clock = pygame.time.Clock()

test = exp.getXVec()

# -------- Main Program Loop -----------
while done == False:
	for event in pygame.event.get(): # User did something
		if event.type == pygame.QUIT: # If user clicked close
			done = True # Flag that we are done so we exit this loop
	# Set the screen background
	screen.fill(black)
	# Draw the rectangle
	pygame.draw.ellipse(screen, green, [exp.getXP() + 300, exp.getYP() + 300, predatorSize[0], predatorSize[1]], predatorSize[2])
	#pygame.draw.ellipse(screen, red, [exp.getXVec()[0] + 300, exp.getYVec()[0] + 300, 30, 30], 10)
	exp.update(1.)
	# Limit to 20 frames per second
	clock.tick(50)
	# Go ahead and update the screen with what we've drawn.
	pygame.display.flip()
	# Be IDLE friendly. If you forget this line, the program will 'hang'
	# on exit.
pygame.quit()
