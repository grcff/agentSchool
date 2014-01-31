import numpy as np
import pygame
import experience as exp


# Define some colors
black = ( 0, 0, 0)
white = ( 255, 255, 255)
green = ( 0, 255, 0)
red = ( 255, 0, 0)

agentNumber = 50
#TODO put predatorSize as a class parameter 
predatorSize = [50, 50, 10]
boxSize = [500., 500., -500., -500]

#Create a new experience
exp = exp.Experience(agentNumber, 10., 0., 5., boxSize[0], boxSize[1], boxSize[2], boxSize[3])

pygame.init()

# Set the height and width of the aquarium
size = [1050, 1050]
screen = pygame.display.set_mode(size)
pygame.display.set_caption("Agent School")
#Loop until the user clicks the close button.
done = False
# Used to manage how fast the screen updates
clock = pygame.time.Clock()

# -------- Main Program Loop -----------
while done == False:
	for event in pygame.event.get(): # User did something
		if event.type == pygame.QUIT: # If user clicked close
			done = True # Flag that we are done so we exit this loop
	# Set the screen background
	screen.fill(black)
	# Draw the rectangle
	pygame.draw.ellipse(screen, green, [exp.getXP() - boxSize[2], exp.getYP() - boxSize[3], predatorSize[0], predatorSize[1]], predatorSize[2])
	
	for i in range(agentNumber):	
		pygame.draw.ellipse(screen, red, [exp.getX(i) - boxSize[2], exp.getY(i) - boxSize[3], 30, 30], 10)
		pygame.draw.line(screen, white, [exp.getX(i) - boxSize[2] + 30/2, exp.getY(i) - boxSize[3] + 30/2], [exp.getX(i) - boxSize[2] + 20*np.cos(exp.getYaw(i)) + 30/2, exp.getY(i) - boxSize[3] + 20*np.sin(exp.getYaw(i)) + 30/2], 5)
		print 'X'		
		print exp.getX(i) - boxSize[2]
		print 'Y'		
		print exp.getY(i) - boxSize[2]

	exp.update(1.)
	# Limit to 20 frames per second
	clock.tick(50)
	# Go ahead and update the screen with what we've drawn.
	pygame.display.flip()
	# Be IDLE friendly. If you forget this line, the program will 'hang'
	# on exit.
pygame.quit()
