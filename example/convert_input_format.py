import sys

def yaml_to_txt(filename):
	f = open(filename).read().split('\n')
	i = 0
	while f[i] != 'map:':
		i+=1
	
	start = []
	goal = []
	obstacles = []

	agents = f[1:i]
	map_str = f[i+1:]

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
	
	t = map_str[0].split('[')
	j = t[1].split(',')
	j[1] = j[1][:-1]
	j[0] = int(j[0].strip())
	j[1] = int(j[1].strip())
	
	x, y = j[0], j[1]

	map_str = map_str[2:]

	for i in map_str:
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

	out_file = filename.split('.')
	
	with open(out_file[0]+'.txt', 'w') as f:
		f.write(str(x+2)+' '+str(y+2)+'\n')

		for i in range(x+2):
			for j in range(y+2):
				f.write(a[i][j])
			f.write('\n')
		f.write(str(n_agents)+'\n')

		for i in range(n_agents):
			f.write(str(start[i][0])+' '+str(start[i][1])+' '+str(goal[i][0])+' '+str(goal[i][1])+'\n')

def txt_to_yaml(filename):
	f = open(filename).read().split('\n')

	t = f[0].split()
	x, y = int(t[0]), int(t[1])

	map_str = f[1:x+1]
	num_agents = int(f[x+1])
	agents = f[x+2:-1]
	starts = []
	goals = []
	obstacles = []

	for i in agents:
		t = i.split()
		starts.append([int(t[0]), int(t[1])])
		goals.append([int(t[2]), int(t[3])])

	for i in range(len(map_str)):
		t = map_str[i].split()
		for j in range(len(t)):
			if t[j] == '@':
				obstacles.append([i, j])

	out_file = filename.split('.')

	with open(out_file[0]+'.yaml', 'w') as f:
		f.write('agents:\n')
		for i in range(num_agents):
			f.write('-   goal: ['+str(goals[i][0])+', '+str(goals[i][1])+']\n')
			f.write('    name: agent'+str(i)+'\n')
			f.write('    start: ['+str(starts[i][0])+', '+str(starts[i][1])+']\n')

		f.write('map:\n')
		f.write('    dimensions: ['+str(x)+', '+str(y)+']\n')
		f.write('    obstacles:\n')

		for i in range(len(obstacles)):
			if i == len(obstacles)-1:
				f.write('    - ['+str(obstacles[i][0])+', '+str(obstacles[i][1])+']')
			else:
				f.write('    - ['+str(obstacles[i][0])+', '+str(obstacles[i][1])+']\n')


if __name__ == '__main__':
	if len(sys.argv) < 2:
		print("Give input file as second argument")
		sys.exit(0)

	c = sys.argv[1].split('.')
	if c[1] == 'yaml':
		yaml_to_txt(sys.argv[1])
	elif c[1] == 'txt':
		txt_to_yaml(sys.argv[1])
	