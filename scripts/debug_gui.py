#!/usr/bin/python
import rospy
from tycho.msg import WheelStatus


import sys, pygame
pygame.init()

size = width, height = 800, 600
speed = [2, 2]
black = 0, 0, 0
screen = pygame.display.set_mode(size)

background = pygame.image.load("/home/pi/Desktop/Link to tycho/resources/TychoHUD_Background.png")

# Order: FL, FR, BL, BR
wheel_base = pygame.image.load("/home/pi/Desktop/Link to tycho/resources/TychoHUD_WheelFLBR.png")
wheel_images = [
	pygame.transform.flip(wheel_base,True,True),
	pygame.transform.flip(wheel_base,False,True),
	pygame.transform.flip(wheel_base,True,False),
	wheel_base
]
wheel_locations = [(439,102), (629,102), (439,496), (629,496)]


def drawImageCenteredRotated(screen, image, x, y, rot):
	if rot != 0:
		imr = pygame.transform.rotate(image, rot)
	else:
		imr = image
	#
	rect = imr.get_rect()
	dx = x - rect.right/2
	dy = y - rect.bottom/2
	screen.blit(imr, rect.move(dx,dy))
#

def drawScreen(screen, angles ):
	black = 0,0,0
	screen.fill( black )
	for i,im in enumerate(wheel_images):
		drawImageCenteredRotated(screen, im, wheel_locations[i][0], wheel_locations[i][1], angles[i] )
	screen.blit(background, background.get_rect())
	pygame.display.flip()
#

def displayMessageContents(m):
	#print(m.front_left_angle, m.front_right_angle, m.back_left_angle, m.back_right_angle)
	#drawScreen(screen, (m.front_left_angle, m.front_right_angle, m.back_left_angle, m.back_right_angle) )

	drawScreen(screen, (m.steering_angle, 0, 0, 0) )
#


rospy.Subscriber("tycho/wheel_status", WheelStatus, displayMessageContents,queue_size=1)
rospy.init_node('DebugGUI')
drawScreen(screen, (0, 0, 0, 0) )
r = 0
while True:
	for event in pygame.event.get():
		if event.type == pygame.QUIT: sys.exit()
	
	# starts the node
	#rospy.spin()
#
