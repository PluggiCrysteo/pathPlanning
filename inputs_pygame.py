import os, pygame,numpy, time, argparse, socket,string,sys




################## SETTING UP CONST
black = (0,0,0)
white = (255,255,255)
red = (100,0,0)
blue = (0,0,140)
purple = (120,0,120)

################# GETTING PARAM
parser = argparse.ArgumentParser(description='Display path calculated by another process')
parser.add_argument('-b','--bitmapPath', help='Specify the path of the bitmap to load and display.', type=str)
parser.add_argument('-s','--scale', help='Set the scale that will be used to display the map. The more pixels your map has, the smaller scale should be. Default is 50.', type=int)
args = parser.parse_args()
 
if args.scale != None :
	scale = args.scale
else:
	scale = 50

if args.bitmapPath != None :
	bitmapPath = args.bitmapPath
else:
	bitmapPath = "bitmap"
	
bitmap = open(bitmapPath)
height = sum(1 for line in bitmap)
width = os.path.getsize(bitmapPath)/height -1

########### INIT

nodeMap = [[0 for x in range(height)] for x in range(width)] 

screen = pygame.display.set_mode((scale*width,scale*height))
screen.fill((255,255,255))



###################### REDRAW BACKGROUND WHEN WE RECLICK
def drawBackground(nodeMap,screen, bitmap):
	bitmap.seek(0)
	for j in range(0, height):
		for i in range(0, width):
			value = bitmap.read(1)
			
			if value == 'X' :
				nodeMap[i][j] = pygame.draw.rect(screen,black,(i*scale,scale*j,scale,scale),0)
			elif value == 'S' :
				nodeMap[i][j] = pygame.draw.rect(screen,red,(i*scale,j*scale,scale,scale),0)
			else :
				realval = ord(value)-ord('0')
				nodeMap[i][j] = pygame.draw.rect(screen,(0,200-10*realval,0),(i*scale,j*scale,scale,scale),0)
		bitmap.read(1)
	pygame.display.update()

################# FUNCTION THAT PARSE AND DRAW THE RECEIVED DATA
def parseAndUseData(data, nodeMap, screen,scale) :
	substring = data
	while 1 :
		try:
			indexStartX = substring.index('(')+1
			indexEndX = substring.index(',')
			x = string.atoi(substring[indexStartX:indexEndX])
			
			indexStartY = substring.index(',')+1
			indexEndY = substring.index(')')
			y = string.atoi(substring[indexStartY:indexEndY])
			
			substring = substring[indexEndY+1:len(substring)]
			nodeMap[x][y] = pygame.draw.rect(screen,blue,(x*scale,y*scale,scale,scale),0)
			pygame.display.update()		

		except :
			break

	return;

################# SETUP TCP SERVER
localhost = '127.0.0.1'
port = 1234
buffer_size = 1024
socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
socket.bind((localhost,port))
socket.listen(0)
	
def requestPath(startX,startY,goalX,goalY):
	os.system("./pathplanner " + str(startX) + " "+ str(startY) + " "+ str(goalX) + " "+ str(goalY))
	try:
		conn, addr = socket.accept()
		data = conn.recv(buffer_size)
		parseAndUseData(data, nodeMap, screen,scale)
	except:
		conn.close()
		socket.close()
		return
		
###########################		
startX = None
startY = None
goalX = None
goalY = None
drawBackground(nodeMap,screen, bitmap)
while 1:
	for event in pygame.event.get():
		if event.type == pygame.QUIT :
			pygame.quit()
			sys.exit()
		elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE :
			break
		elif event.type == pygame.MOUSEBUTTONDOWN :
			if startX == None :
				drawBackground(nodeMap,screen, bitmap)
				startX = pygame.mouse.get_pos()[0]/scale
				startY = pygame.mouse.get_pos()[1]/scale
				nodeMap[startX][startY] = pygame.draw.rect(screen,purple,(startX*scale,startY*scale,scale,scale),0)
				pygame.display.update()	
			else :
				goalX = pygame.mouse.get_pos()[0]/scale
				goalY = pygame.mouse.get_pos()[1]/scale
				requestPath(startX,startY,goalX,goalY)
				startX = None
				startY = None
				goalX = None
				goalY = None

			
	
# TODO :
# GUI (menu with esc)
# include scaling somehow (could be done solely in python)



