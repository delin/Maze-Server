#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  game_server.py
#
#  Copyright 2012 Unknown <delin@eridan.la>
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.


import json, socket, numpy, random, math, time, operator, threading, sys, select
from heapq import heappush, heappop # for priority queue


map_width = 20
map_height = 70
min_lenth = 18
timelimit = 500
sleep_time = 0.15

class bcolors:
	BG = '\033[30m'
	EXIT = '\033[1;33m'
	BLOCK = '\033[94m'
	BOT = '\033[31m'
	NULL = '\033[1;30m'
	SEE = '\033[38;5;22m'
	ENDC = BG
	END = '\033[0m'

class node:
	xPos = 0 # x position
	yPos = 0 # y position
	distance = 0 # total distance already travelled to reach the node
	priority = 0 # priority = distance + remaining distance estimate

	def __init__(self, xPos, yPos, distance, priority):
		self.xPos = xPos
		self.yPos = yPos
		self.distance = distance
		self.priority = priority

	def __lt__(self, other): # comparison method for priority queue
		return self.priority < other.priority

	def updatePriority(self, xDest, yDest):
		self.priority = self.distance + self.estimate(xDest, yDest) * 10 # A*

	# give higher priority to going straight instead of diagonally
	def nextMove(self, dirs, d): # d: direction to move
		if dirs == 8 and d % 2 != 0:
			self.distance += 14
		else:
			self.distance += 10

	# Estimation function for the remaining distance to the goal.
	def estimate(self, xDest, yDest):
		xd = xDest - self.xPos
		yd = yDest - self.yPos
		# Euclidian Distance
		d = math.sqrt(xd * xd + yd * yd)
		# Manhattan distance
		#~ d = abs(xd) + abs(yd)
		# Chebyshev distance
		#~ d = max(abs(xd), abs(yd))
		return(d)

class path_find:
	def __init__ (self):
		self.dirs = 4 # number of possible directions to move on the map
		if self.dirs == 4:
			self.dx = [1, 0, -1, 0]
			self.dy = [0, 1, 0, -1]
		elif self.dirs == 8:
			self.dx = [1, 1, 0, -1, -1, -1, 0, 1]
			self.dy = [0, 1, 1, 1, 0, -1, -1, -1]

	# A-star algorithm.
	# The path returned will be a string of digits of directions.
	def path_find(self, grid, xA, yA, xB, yB):
		if xA == xB and yA == yB:
			return ''

		self.n = map_height # horizontal size of the map
		self.m = map_width # vertical size of the map
		self.the_map = grid

		closed_nodes_map = [] # map of closed (tried-out) nodes
		open_nodes_map = [] # map of open (not-yet-tried) nodes
		dir_map = [] # map of dirs
		row = [0] * self.n
		route = []

		for i in range(self.m): # create 2d arrays
			closed_nodes_map.append(list(row))
			open_nodes_map.append(list(row))
			dir_map.append(list(row))

		pq = [[], []] # priority queues of open (not-yet-tried) nodes
		pqi = 0 # priority queue index
		# create the start node and push into list of open nodes
		n0 = node(xA, yA, 0, 0)
		n0.updatePriority(xB, yB)
		heappush(pq[pqi], n0)
		open_nodes_map[yA][xA] = n0.priority # mark it on the open nodes map

		# A* search
		while len(pq[pqi]) > 0:
			# get the current node w/ the highest priority
			# from the list of open nodes
			n1 = pq[pqi][0] # top node
			n0 = node(n1.xPos, n1.yPos, n1.distance, n1.priority)
			x = n0.xPos
			y = n0.yPos
			heappop(pq[pqi]) # remove the node from the open list
			open_nodes_map[y][x] = 0
			closed_nodes_map[y][x] = 1 # mark it on the closed nodes map

			# quit searching when the goal is reached
			#~ if n0.estimate(xB, yB) == 0:
			if x == xB and y == yB:
				# generate the path from finish to start
				# by following the dirs
				path = ''
				while not (x == xA and y == yA):
					j = dir_map[y][x]
					c = str((j + self.dirs / 2) % self.dirs)
					path = c + path
					x += self.dx[j]
					y += self.dy[j]

				return path

			# generate moves (child nodes) in all possible dirs
			for i in range(self.dirs):
				xdx = x + self.dx[i]
				ydy = y + self.dy[i]

				if not (xdx < 0 or xdx > self.n - 1 or ydy < 0 or ydy > self.m - 1
						or self.the_map[ydy][xdx] == 1 or closed_nodes_map[ydy][xdx] == 1):
					# generate a child node
					m0 = node(xdx, ydy, n0.distance, n0.priority)
					m0.nextMove(self.dirs, i)
					m0.updatePriority(xB, yB)
					# if it is not in the open list then add into that
					if open_nodes_map[ydy][xdx] == 0:
						open_nodes_map[ydy][xdx] = m0.priority
						heappush(pq[pqi], m0)
						# mark its parent node direction
						dir_map[ydy][xdx] = (i + self.dirs / 2) % self.dirs
					elif open_nodes_map[ydy][xdx] > m0.priority:
						# update the priority
						open_nodes_map[ydy][xdx] = m0.priority
						# update the parent direction
						dir_map[ydy][xdx] = (i + self.dirs / 2) % self.dirs
						# replace the node
						# by emptying one pq to the other one
						# except the node to be replaced will be ignored
						# and the new node will be pushed in instead
						while not (pq[pqi][0].xPos == xdx and pq[pqi][0].yPos == ydy):
							heappush(pq[1 - pqi], pq[pqi][0])
							heappop(pq[pqi])
						heappop(pq[pqi]) # remove the target node
						# empty the larger size priority queue to the smaller one
						if len(pq[pqi]) > len(pq[1 - pqi]):
							pqi = 1 - pqi
						while len(pq[pqi]) > 0:
							heappush(pq[1 - pqi], pq[pqi][0])
							heappop(pq[pqi])
						pqi = 1 - pqi
						heappush(pq[pqi], m0) # add the better node instead
		return '' # if no route found

class server_handler:
	def __init__ (self):
		self.host = ''
		self.port = 12388
		self.backlog = 16
		self.size = 1024
		self.server = None
		self.threads = []

	def open_socket (self):
		try:
			self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
			self.server.bind((self.host, self.port))
			self.server.listen(self.backlog)
		except socket.error as message:
			if self.server:
				self.server.close()
			print("Could not open socket: " + message)
			sys.exit(1)

	def run (self):
		self.open_socket()
		input = [self.server, sys.stdin]

		print("Server started.")
		running = True
		while running:
			inputready, outputready, exceptready = select.select(input, [], [])

			for s in inputready:
				if s == self.server:
					c = client_handler(self.server.accept())			# handle the server socket
					c.start()
					self.threads.append(c)
					print("Accept new connection")
				elif s == sys.stdin:
					junk = sys.stdin.readline()							# handle standard input
					running = False

		# close all threads
		self.server.close()
		for c in self.threads:
			c.join()

class client_handler(threading.Thread):
	def __init__ (self, (client, address)):
		threading.Thread.__init__(self)
		self.client = client
		self.address = address
		self.size = 1024

	def run (self):
		self.engine()
		self.client.close()

	def msg_send (self, data):
		if len(data) <= 0:
			return False

		return self.client.send(json.dumps(data) + '\n')

	def msg_recv (self):
		buf = ''
		msg = ''

		while buf != '\n':
			msg = msg + buf
			buf = self.client.recv(1)
			if len(buf) <= 0:
				return ''

		#~ print "<--", msg
		return json.loads(msg)

	def engine (self):
		# Auth
		data = {"status": "ok", "message": "ready for login"}
		res = self.msg_send(data)
		if res <= 0:
			return False

		data = self.msg_recv()
		if len(data) <= 0:
			return False
		if 'message_type' in data and data['message_type'] == 'login':
			print 'login ok'
		else:
			return False

		data = {"status": "ok", "message": "logged in"}
		res = self.msg_send(data)
		if res <= 0:
			return False

		data = {"width": map_width, "height": map_height, "timelimit": timelimit}
		res = self.msg_send(data)
		if res <= 0:
			return False

		# Make World
		grid = self.gen_grid()

		route = []
		pfind = path_find()

		ok = True
		while ok:
			# bot
			bot_x = random.randint(0, map_width - 1)
			bot_y = random.randint(0, map_height - 1)

			# exit
			exit_x = random.randint(0, map_width - 1)
			exit_y = random.randint(0, map_height - 1)

			route = pfind.path_find(grid, bot_y, bot_x, exit_y, exit_x)
			xd = bot_x - exit_x
			yd = bot_y - exit_y
			dst = math.sqrt(xd * xd + yd * yd)
			if len(route) > 0 and dst >= min_lenth:
				grid[bot_x][bot_y] = 10
				grid[exit_x][exit_y] = 11

				ok = False

		print("lenth of pass: ", len(route))
		print(bcolors.BG)
		for jx in xrange(map_width):
			for jy in xrange(map_height):
				if grid[jx][jy] == 11:
					print bcolors.EXIT +"$" + bcolors.ENDC,
				elif grid[jx][jy] == 10:
					print bcolors.BOT + "☃" + bcolors.ENDC,
				elif grid[jx][jy] == 1:
					print bcolors.BLOCK + "☰" + bcolors.ENDC,
				elif grid[jx][jy] == 0:
					print bcolors.SEE + " " + bcolors.ENDC,
				elif grid[jx][jy] >= 100:
					print bcolors.SEE + "¤" + bcolors.ENDC,
				else:
					print bcolors.NULL + "?" + bcolors.ENDC,

			print ''
		print(bcolors.END)

		s = {'d': [0, 1], 'r': [1, 0], 'u': [0, -1], 'l': [-1, 0]}
		bot_position = [bot_x, bot_y]
		exits = [[exit_x, exit_y]]

		turn_no = 0

		# Main game loop
		for t in xrange(timelimit + 1):
			time.sleep(sleep_time)

			see_empty = []
			see_bricks = []
			see_exits = []

			x0 = bot_position[0] - 4
			y0 = bot_position[1] - 4

			for bx in xrange(9):
				for by in xrange(9):
					x = x0 + bx
					y = y0 + by

					if x >= 0 and x < map_width and y >= 0 and y < map_height:
						if abs(x - bot_position[0]) + abs(y - bot_position[1]) < 6:
							if grid[x][y] == 1:
								see_bricks.append([x, y])
							elif grid[x][y] == 0:
								see_empty.append([x, y])
							elif grid[x][y] == 11:
								see_exits.append([x, y])

			data = {"turn_no": turn_no, "bot_position": bot_position, "bricks": see_bricks, "exits": see_exits,"empty": see_empty}
			res = self.msg_send(data)
			if res <= 0:
				return False

			data = self.msg_recv()
			if len(data) <= 0:
				return False

			if 'command' in data and data['command'] in s:
				to = data['command']
				to_x = bot_position[0] + s[to][0]
				to_y = bot_position[1] + s[to][1]
				#~ print to_x, ":", to_y, "|", grid[to_x][to_y]

				if grid[to_x][to_y] == 0:
					grid[to_x][to_y] = 10
					bot_position[0] = to_x
					bot_position[1] = to_y
				if grid[to_x][to_y] == 10:
					grid[to_x][to_y] = 0
					bot_position[0] = to_x
					bot_position[1] = to_y
				elif grid[to_x][to_y] == 11:
					bot_position[0] = to_x
					bot_position[1] = to_y

					data = {"turn_no": turn_no, "bot_position": bot_position, "bricks": see_bricks, "exits": see_exits,"empty": see_empty}
					res = self.msg_send(data)
					if res <= 0:
						return False

					print "Exit finded!!!!!1111"
					return True
			else:
				return False

			turn_no += 1

		print("timelimit")

		return True

	def gen_grid (self):
		grid = numpy.zeros([map_width, map_height], int)

		grid_fill = map_height * map_width / 2

		for i in xrange(grid_fill):
			x = random.randint(0, map_width - 1)
			y = random.randint(0, map_height - 1)
			grid[x][y] = 1

		return grid

def main():
	srv = server_handler()
	srv.run()

	return 0

if __name__ == '__main__':
	main()

