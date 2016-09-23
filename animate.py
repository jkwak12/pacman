'''for main function placement, place this function at the end of your main function calls, 
		BUT before any print statements. This function will clear your terminal. 		'''
def animate_path(maze, dots, path):
	frameNumber = 0
	start = path[0]
	goal = path[len(path) - 1]
	pacmanStart = getStart(maze)
	writePacman = 1
	folder = 'animation'
	if not os.path.isdir(folder):	#just some checking for existence of frames folder
		os.mkdir(folder)
	for current in path:
		tempPath = path[0:(frameNumber + 1)] 
		frameString = 'animation/frame' + str(frameNumber) + '.txt'
		f = open(frameString, 'w+')
		for i in range(len(maze)):
			for j in range(len(maze[0])):
				if (i,j) == tempPath[len(tempPath) - 1] and writePacman:
					f.write('P')
					writePacman = 0
				elif (i,j) in tempPath and not (i,j) == start and not (i,j) == goal:
					f.write(' ')
				elif (i,j) == pacmanStart:
					f.write(' ')
				else:
					f.write(Maze[i][j])
			f.write('\n')
		#pdb.set_trace()
		frameNumber += 1
		writePacman = 1
		f.close()
	frame = 0
	delay = (5/len(dots))  		#the delay between frames. The more dots, the shorter the delay
	while frame < frameNumber:
		os.system('cls' if os.name == 'nt' else 'clear')		#clear screen
		f = open('animation/frame' + str(frame) + '.txt')
		print(f.read())
		f.close()
		time.sleep(delay)	  
		frame += 1
	shutil.rmtree(folder) #delete frame files
	return
