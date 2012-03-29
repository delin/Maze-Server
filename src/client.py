#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  main.py
#
#  Copyright 2012 Delin <delin@eridan.la>
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

import json, socket, numpy, random, math, time, operator, curses
from heapq import heappush, heappop # for priority queue
from os import system

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
	def __init__ (self, grid, width, height):
		self.dirs = 4 # number of possible directions to move on the map

		if self.dirs == 4:
			self.dx = [1, 0, -1, 0]
			self.dy = [0, 1, 0, -1]
		elif self.dirs == 8:
			self.dx = [1, 1, 0, -1, -1, -1, 0, 1]
			self.dy = [0, 1, 1, 1, 0, -1, -1, -1]

		self.n = width # horizontal size of the map
		self.m = height # vertical size of the map
		self.the_map = grid

	# A-star algorithm.
	# The path returned will be a string of digits of directions.
	def path_find(self, xA, yA, xB, yB):
		if xA == xB and yA == yB:
			return ''

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

				#~ print "xdx: ", xdx, "\nydy: ", ydy
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

class clserver:
	def __init__(self, sock = None):
		if sock is None:
			self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		else:
			self.sock = sock

	def srv_connect(self, host, port):
		self.sock.connect((host, port))

	def msg_send(self, msg):
		sent = self.sock.send(msg)
		#~ print "-->", msg

	def msg_recv(self):
		buf = ''
		msg = ''

		while buf != '\n':
			msg = msg + buf
			buf = self.sock.recv(1)

		#~ print "<--", msg
		return json.loads(msg)

	def srv_disconnect(self):
		self.sock.close()

class bcolors:
	BG = '\033[30m'
	EXIT = '\033[1;33m'
	BLOCK = '\033[94m'
	BOT = '\033[31m'
	NULL = '\033[1;30m'
	SEE = '\033[38;5;22m'
	ENDC = BG
	END = '\033[0m'

class clproto:
	def __init__(self):
		self.srv = clserver()
		#~ self.srv.srv_connect("cloudcontest.ru", 8000)
		self.srv.srv_connect("192.168.1.20", 8000)

	def proto_close(self):
		self.srv.srv_disconnect()
		print "\n\n\nDisconnected"

	def auth(self):
		msg = self.srv.msg_recv()

		if msg['status'] == 'ok':
			print "Connected."

		self.srv.msg_send('{"message_type": "login", "user_token": "94479c347353cb21aa932fbd"}\n')

		msg = self.srv.msg_recv()
		if msg['status'] == 'ok':
			print "[sys] Auth Success."
		else:
			print "\n\n\nError:", msg['message']
			return False

		msg = self.srv.msg_recv()

		self.world_x = msg['width']
		self.world_y = msg['height']
		self.world = numpy.zeros([msg['width'], msg['height']], int)
		self.timelimit = msg['timelimit']

		for x in xrange(self.world_x):
			for y in xrange(self.world_y):
				self.world[x][y] = -1

		print "Map size: ", msg['width'], "x", msg['height']
		print "Timelimit: ", self.timelimit

		return True

	def engine(self):
		self.ex = -1
		self.ey = -1
		to_x = 0
		to_y = 0
		q = 0
		to_st = 0

		my_path = []

		screen = curses.initscr()

		curses.start_color()
		curses.use_default_colors()

		color_BLACK = 0
		color_WHITE = 0
		color_CIAN = 1
		color_YELLOW = 2
		color_BLUE = 3
		color_GRAY = 4
		color_RED = 5

		curses.init_pair(color_BLACK, 0, -1)
		curses.init_pair(color_WHITE, -1, -1)
		curses.init_pair(color_CIAN, 6, -1)
		curses.init_pair(color_YELLOW, 3, -1)
		curses.init_pair(color_RED, 1, -1)
		curses.init_pair(color_BLUE, 4, -1)
		curses.init_pair(color_GRAY, 7, -1)

		for i in xrange(self.timelimit + 1):
			msg = self.srv.msg_recv()

			if 'status' in msg and msg['status'] == 'error':
				print "Error:", msg['message']
				return False

			turn_time = time.time()

			for j in xrange(len(msg['bricks'])):
				x = msg['bricks'][j][0]
				y = msg['bricks'][j][1]
				self.world[x][y] = 1

			for j in xrange(len(msg['empty'])):
				empty_x = msg['empty'][j][0]
				empty_y = msg['empty'][j][1]
				if self.world[empty_x][empty_y] == -1:
					self.world[empty_x][empty_y] = 0

			self.bot_x = msg['bot_position'][0]
			self.bot_y = msg['bot_position'][1]

			route = []
			pfind = path_find(self.world, self.world_y, self.world_x)

			tr_a = []
			tmp = []
			if len(my_path) >= 5:
				if my_path[len(my_path) - 1] == my_path[len(my_path) - 3] \
					and my_path[-1] == my_path[len(my_path) - 5]:
						print "tracebeck: ", my_path

						#~ if len(path_list) > 2:
						if len(msg['empty'][0]) > 0:
							tr_a.append(msg['empty'][random.randint(0, len(msg['empty']) - 1)])
						#~ elif len(msg['empty'][0]) > 1:
							#~ tr_a.append(msg['empty'][1])
							#~ tr_a.append(msg['empty'][0])
						#~ else:
							#~ tr_a.append([0, self.world_y])
							#~ tr_a.append([0, 0])

			if to_st == 100:
				route = pfind.path_find(self.bot_y, self.bot_x, self.ey, self.ex)
			elif len(msg['exits']) > 0:
				print "Exit detected!"

				self.ex = msg['exits'][0][0]
				self.ey = msg['exits'][0][1]

				if self.ex == self.bot_x and self.ey == self.bot_y:
					print "Finished!"

				to_x = self.ex
				to_y = self.ey
				to_st = 100

				route = pfind.path_find(self.bot_y, self.bot_x, to_y, to_x)
			else:
				if to_x == self.bot_x and to_y == self.bot_y and len(tr_a) > 0:
					tr_a.pop()

				if len(tr_a) > 0:
					to_x = tr_a[-1][0]
					to_y = tr_a[-1][1]
				else:
					path_list = []
					path_list.append([0, 0, self.world_y * self.world_x])

					for ix in xrange(self.world_x):
						for iy in xrange(self.world_y):
							if self.world[ix][iy] == -1:
								xd = ix - self.bot_x
								yd = iy - self.bot_y
								dst = math.sqrt(xd * xd + yd * yd)

								if dst < path_list[-1][2]:
									path_list.append([ix, iy, dst])
								elif dst == path_list[-1][2]:
									if path_list[-1][0] * path_list[-1][1] < ix * iy:
										path_list.append([ix, iy, dst])
					if len(path_list) > 0:
						#~ print len(path_list)

						route = pfind.path_find(self.bot_y, self.bot_x, path_list[-1][1], path_list[-1][0])
						while len(route) == 0 and len(path_list) > 0:
							route = pfind.path_find(self.bot_y, self.bot_x, path_list[-1][1], path_list[-1][0])
							path_list.pop()

				if len(route) == 0:
					route = pfind.path_find(self.bot_y, self.bot_x, to_y, to_x)

				if len(route) == 0 and len(tr_a) > 0:
					tr_a[-1][0] = random.randint(0, self.world_x - 1)
					tr_a[-1][1] = random.randint(0, self.world_y - 1)
					to_x = tr_a[-1][0]
					to_y = tr_a[-1][1]

					route = pfind.path_find(self.bot_y, self.bot_x, to_y, to_x)

			s = [[1, 0, 'd'], [0, 1, 'r'], [-1, 0, 'u'], [0, -1, 'l']]

			if len(route) > 0:
				to = s[int(route[0])][2]
			else:
				to = s[random.randint(0, 3)][2]

			my_path.append([self.bot_y, self.bot_x])
			#~ print "Turn #", msg['turn_no']
			#~ print "Move to", to

			self.srv.msg_send('{"command": "' + to + '"}\n')

			if len(route) > 0:
				jx = self.bot_y
				jy = self.bot_x

				for j in range(len(route)):
					k = int(route[j])
					jx += s[k][0]
					jy += s[k][1]
					if self.world[jy][jx] == 0:
						self.world[jy][jx] = 10
					elif self.world[jy][jx] == 1:
						self.world[jy][jx] = 11
					elif self.world[jy][jx] == -1:
						self.world[jy][jx] = 12

			screen.clear()
			screen.addstr(1, 1, 'Generated in: ' + str(time.time() - turn_time), curses.color_pair(color_WHITE))
			screen.addstr(2, 1, 'Turn:         ' + str(msg['turn_no']), curses.color_pair(color_WHITE))
			screen.addstr(3, 1, 'Move to:      ' + to, curses.color_pair(color_WHITE))


			for jx in xrange(self.world_x):
				for jy in xrange(self.world_y):
					if jx == self.ex and jy == self.ey:
						screen.addstr(jx + 5, jy + 5, "$", curses.color_pair(color_YELLOW))
					elif jx == self.bot_x and jy == self.bot_y:
						screen.addstr(jx + 5, jy + 5, "@", curses.color_pair(color_RED))
					elif self.world[jx][jy] == 1:
						screen.addstr(jx + 5, jy + 5, "#", curses.color_pair(color_BLUE))
					elif self.world[jx][jy] == 0:
						screen.addstr(jx + 5, jy + 5, " ", curses.color_pair(color_BLACK))
					elif self.world[jx][jy] >= 10:
						screen.addstr(jx + 5, jy + 5, ".", curses.color_pair(color_CIAN))
					else:
						screen.addstr(jx + 5, jy + 5, "?", curses.color_pair(color_GRAY))

				screen.addstr(jx + 5, 0, str(jx) + ":", curses.color_pair(color_WHITE))

			screen.refresh()

			#~ todisp = bcolors.BG
			#~ for jx in xrange(self.world_x):
				#~ for jy in xrange(self.world_y):
					#~ if jx == self.ex and jy == self.ey:
						#~ todisp += bcolors.EXIT + "$" + bcolors.ENDC
					#~ elif jx == self.bot_x and jy == self.bot_y:
						#~ todisp += bcolors.BOT + "@" + bcolors.ENDC
					#~ elif self.world[jx][jy] == 1:
						#~ todisp += bcolors.BLOCK + "#" + bcolors.ENDC
					#~ elif self.world[jx][jy] == 0:
						#~ todisp += bcolors.SEE + " " + bcolors.ENDC
					#~ elif self.world[jx][jy] >= 10:
						#~ todisp += bcolors.SEE + "¤" + bcolors.ENDC
					#~ else:
						#~ todisp += bcolors.NULL + "?" + bcolors.ENDC
					#~ todisp += ' '
				#~ todisp += '\n'
			#~ todisp = todisp + bcolors.END
#~
			#~ print todisp

			if self.bot_x == self.ex and self.bot_y == self.ey:
				curses.nocbreak(); screen.keypad(0); curses.echo()
				print "\n\n\nFINISHED in", msg['turn_no'], "^_^"
				return 0

			for jx in xrange(self.world_y):
				for jy in xrange(self.world_x):
					if self.world[jy][jx] == 10:
						self.world[jy][jx] = 0
					elif self.world[jy][jx] == 11:
						self.world[jy][jx] = 1
					elif self.world[jy][jx] == 12:
						self.world[jy][jx] = -1

		curses.nocbreak(); screen.keypad(0); curses.echo()
		print "\n\n\n;-("

def main():
	print "Starting.."

	srv = clproto()
	round_time = time.time()
	if srv.auth() == True:
		srv.engine()

	print '\n\n\nRuntime:', time.time() - round_time

	srv.proto_close()

	return 0

if __name__ == '__main__':
	main()
