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
boxSize = [1000., 800]

#Create a new experience
exp = exp.Experience(agentNumber, 10., 10., 5., boxSize[0], boxSize[1], 0., 0.)

pygame.init()

# Set the height and width of the aquarium
size = [1050, 850]
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
	pygame.draw.ellipse(screen, green, [exp.getXP(), exp.getYP(), predatorSize[0], predatorSize[1]], predatorSize[2])
		
	try:
		for i in range(agentNumber):	
			pygame.draw.ellipse(screen, red, [exp.getX(i), exp.getY(i), 16, 16], 5)
			pygame.draw.line(screen, white, [exp.getX(i) + 8, exp.getY(i) + 8], [exp.getX(i) + 10*np.cos(exp.getYaw(i)) + 8, exp.getY(i) + 10*np.sin(exp.getYaw(i)) + 8], 3)
			"""			
			for j in range(exp.getNN(i)):
				pygame.draw.line(screen, green, [exp.getX(i) - boxSize[2] + 30/2 + 5, exp.getY(i) - boxSize[3] + 30/2], [exp.getXN(i, j) - boxSize[2] + 30/2, exp.getYN(i, j) - boxSize[3] + 30/2], 1)
				pygame.draw.line(screen, green, [exp.getX(i) - boxSize[2] + 30/2 - 5, exp.getY(i) - boxSize[3] + 30/2], [exp.getXN(i, j) - boxSize[2] + 30/2, exp.getYN(i, j) - boxSize[3] + 30/2], 1)
			"""			
	
	except TypeError:
		print exp.getX(i) - boxSize[2]
		print exp.getY(i) - boxSize[3]

	exp.update(1.)
	# Limit to 20 frames per second
	clock.tick(50)
	# Go ahead and update the screen with what we've drawn.
	pygame.display.flip()
	# Be IDLE friendly. If you forget this line, the program will 'hang'
	# on exit.
pygame.quit()
