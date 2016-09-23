import pdb 

'''Just drag this method into your main file somewhere'''


#assume the list 'dots' is already in the order they were eaten in
def write_dots_eaten(maze, dots):
	f = open('solution.txt','w+')
	for i in range(len(maze)):
		for j in range(len(maze[0])):
			f.write(maze[i][j])
		f.write('\n')
	'''done writing initial maze state'''	
	f.seek(0,0)	#move file cursor to beginning
	contents = f.read()
	count = 48		#ascii value for '0'
	for dot in dots:
		x = dot[1]	#column value 
		y = dot[0]	#row value
		index = y*(len(maze[0]) + 1) + x	#some crazy offset, just take for granted ;)
		contents = contents[:index] + chr(count) + contents[index+1:]	
		count += 1
		if(count == 58): 	#is this ascii value '10'?
			count = 97		#ascii value for 'a'
		elif(count == 123):	#is this value greater than alphabet length?
			count = 65		#ascii value for 'A'
	f.seek(0,0)
	f.write(contents)		
	f.close()
