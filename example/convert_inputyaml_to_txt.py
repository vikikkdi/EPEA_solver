if __name__ == '__main__':
	f = open('input.yaml').read().split('\n')
	i = 0
	while f[i] != 'map:':
		i+=1
	
	start = []
	goal = []
	obstacles = []

	agents = f[1:i]
	map_strcuture = f[i+1:]

	n_agents = len(agents)//3

	for i in agents:
		if 'start:' in i:
			t = i.split('[')
			j = t[1].split(',')
			j[1] = j[1][:-1]
			j[0] = int(j[0].strip()) + 1
			j[1] = int(j[1].strip()) + 1
			start.append(tuple(j))
		elif 'goal:' in i:
			t = i.split('[')
			j = t[1].split(',')
			j[1] = j[1][:-1]
			j[0] = int(j[0].strip()) + 1
			j[1] = int(j[1].strip()) + 1
			goal.append(tuple(j))
	
	t = map_strcuture[0].split('[')
	j = t[1].split(',')
	j[1] = j[1][:-1]
	j[0] = int(j[0].strip())
	j[1] = int(j[1].strip())
	
	x, y = j[0], j[1]

	map_strcuture = map_strcuture[2:]

	for i in map_strcuture:
		t = i.split('[')
		j = t[1].split(',')
		j[1] = j[1][:-1]
		j[0] = int(j[0].strip()) + 1
		j[1] = int(j[1].strip()) + 1
		obstacles.append(tuple(j))

	a = [['.']*(y+2) for i in range(x+2)]
	for i in range(y+2):
		a[0][i] = '@'
		a[x+1][i] = '@'

	for i in range(x+2):
		a[i][0] = '@'
		a[i][y+1] = '@'

	for i in obstacles:
		m = i[0]
		n = i[1]
		a[m][n] = "@"

	for i in range(x+2):
		for j in range(y+2):
			if j != y+1:
				a[i][j] = a[i][j]+' '
	
	with open('input.txt', 'w') as f:
		f.write(str(x+2)+' '+str(y+2)+'\n')

		for i in range(x+2):
			for j in range(y+2):
				f.write(a[i][j])
			f.write('\n')
		f.write(str(n_agents)+'\n')

		for i in range(n_agents):
			f.write(str(start[i][0])+' '+str(start[i][1])+' '+str(goal[i][0])+' '+str(goal[i][1])+'\n')