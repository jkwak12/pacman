import pdb 

'''Just drag this method into your main file somewhere'''


#assume the list 'dots' is already in the order they were eaten in
#also added back in the printing of dots along the path since a TA in office hours
#		said that they wanted the visible path alongside the order of dots eaten
def write_dots_eaten(maze, dots, path):
	f = open('solution.txt','w+')
	start = path[0]
	goal = path[len(path) - 1]
	for i in range(len(maze)):
		for j in range(len(maze[0])):
			if (i,j) in path and not (i,j) == start and not (i,j) == goal:
				f.write('.')
			else:
				f.write(Maze[i][j])
		f.write('\n')	
	'''done writing maze state with path of dots'''

	'''now including the order of initial dots eaten'''	
	f.seek(0,0)	#move file cursor to beginning
	contents = f.read()
	count = 48		#ascii value for '0'
	for dot in dots:
		x = dot[1]	#column value 
		y = dot[0]	#row value
		index = y*(len(maze[0]) + 1) + x
		contents = contents[:index] + chr(count) + contents[index+1:]	#some crazy offset
		count += 1
		if(count == 58): 	#is this ascii value '10'?
			count = 97		#ascii value for 'a'
		elif(count == 123):	#is this value greater than alphabet length?
			count = 65		#ascii value for 'A'
	f.seek(0,0)
	f.write(contents)
	f.close()
