import Queue
import math
import copy

class state:
	x= 0
	y= 0
	k = [0,0]
	def __init__(self,x=0,y=0):
		self.x = x
		self.y = y

	def __eq__(self,other):
		return ((self.x ==other.x) and self.y == other.y)
	def __ne__(self,other):
		return ((self.x != other.x) or (self.y != other.y))
	def __gt__(self,other):
		if (self.k[0]-0.00001 > other.k[0]): return True
		elif (self.k[0] > other.k[0]-0.00001): return False
		return self.k[1] > other.k[1]

	def __le__(self,other):
		if (self.k[0] < other.k[0]): return True
		elif (self.k[0] > other.k[0]): return False
		return self.k[1] < other.k[1] + 0.00001

	def __lt__(self,other):
		if (self.k[0] + 0.00001 < other.k[0]): return True
		elif (self.k[0] - 0.00001 > other.k[0] ): return False
		return self.k[1] < other.k[1]

	def __hash__(self):
		return self.x + self.y*34245

	def __repr__(self):
		return "x:" + str(self.x) + "y:" + str(self.y)

class cellInfo():
	g = None
	rhs = None
	cost = None

class ipoint():
	x,y = None,None

def eightCondist(a,b):
	temp = None
	minim = abs(a.x-b.x)
	maxim = abs(a.y-b.y)
	if minim > maxim:
		minim, maxim = maxim, minim
	return (math.sqrt(2) - 1) * minim + maxim
		


class Dstar:
	openList = Queue.PriorityQueue()
	cellHash = {}
	openHash = {}
	s_start, s_goal, s_last = state(), state() ,state()
	maxSteps = 80000
	C1 = 1
	k_m = 0
	tmp = None
	path = []

	def __init__(self,sx,sy,gx,gy):
		self.openList = Queue.PriorityQueue()
		self.cellHash = {}
		self.openHash = {}

		self.s_start.x = sx
		self.s_start.y = sy
		self.s_goal.x = gx
		self.s_goal.y = gy

		tmp = cellInfo()
		tmp.g = tmp.rhs = 0
		tmp.cost = self.C1



	def close(self,x,y):
		if (math.isinf(x) and math.isinf(y)): return True
		return (abs(x-y < 0.00001))


	def makeNewCell(self,u):
		exists = self.cellHash.get(u,None)
		if exists: return
		cinf = cellInfo()
		cinf.g = cinf.rhs = self.heuristic(u, self.s_goal)
		cinf.cost = self.C1
		self.cellHash[u] = cinf

	def heuristic(self,a,b):
		return eightCondist(a, b) * self.C1

	def occupied(self,u):
		cur = self.cellHash.get(u,None)
		if cur == None : return False
		return cur.cost < 0

	def getG(self,u):
		cur = self.cellHash.get(u,None)
		if cur == None:
			return self.heuristic(u, self.s_goal)
		return self.cellHash[u].g

	def getRHS(self,u):
		cur = self.cellHash.get(u,None)
		if cur == None:
			return self.heuristic(u, self.s_goal)
		return self.cellHash[u].rhs


	def setG(self,u,g):
		self.makeNewCell(u)
		self.cellHash[u].g = g

	def setRHS(self, u ,rhs):
		self.makeNewCell(u)
		self.cellHash[u].rhs = rhs


	def calculateKey(self,u):
		val = min(self.getRHS(u),self.getG(u))
		u.k[0] = val + self.heuristic(u, self.s_start) + self.k_m
		u.k[1] = val
		return u

	def cost(self,a,b):
		xd = abs(a.x-b.x)
		xy = abs(a.y-b.y)
		scale = 1

		if xd+xy > 1: scale = math.sqrt(2)

		val = self.cellHash.get(a,None)
		if val == None: return scale*self.C1
		return scale * val.cost

	def updateCell(self,x,y,val):
		u = state()
		u.x = x
		u.y = y

		if ((u == self.s_start) or (u == self.s_goal)): return

		self.makeNewCell(u)
		self.cellHash[u].cost = val
		self.updateVertex(u)

	def getSucc(self,u):
		s = []
		u.k[0], u.k[1] = -1, -1
		if self.occupied(u): return

		for x in range(-1,2):
			for y in range(-1,2):
				print x,y
				if not(x == 0 and y == 0):
					s.append(state(u.x+x,u.y + y))
		return s.reverse()

	def updateVertex(self,u):
		s = []

		if (u != self.s_goal):
			succL = self.getSucc(u)
			tmp = float("Inf")
			tmp2 = None

			for i in succL:
				tmp2 = self.getG(i) + self.cost(u, i)
				if tmp2 < tmp: tmp = tmp2

			if not(self.close(self.getRHS(u),tmp)):
				self.setRHS(u, tmp)
		if not(self.close(self.getG(u), self.getRHS(u))):
			self.insert(u)

	def keyHashCode(self,u):
		return u.k[0] + 1193*u.k[1]

	def isValid(self,u):
		cur = self.openHash.get(u)
		if cur == None: return False
		if not(self.close(self.keyHashCode(u), cur)): return False
		return True

	def insert(self,u):
		u = self.calculateKey(u)
		cur = self.openHash.get(u,None)
		csum = self.keyHashCode(u)

		if (cur != None and self.close(csum,cur)):
			return

		self.openHash[u] = csum
		self.openList.put(u)

	def remove(self,u):
		cur = self.openHash.get(u)
		if cur == None: return
		del self.openHash[u]

	def trueDist(a,b):
		x = a.x - b.x
		y = a.y - b.y
		return math.sqrt(x*x + y*y)

	def getPred(self,u):
		s = []

		u.k[0], u.k[1] = -1,-1

		for x in range(-1,2):
			for y in range(-1,2):
				print x,y
				if not(x == 0 and y == 0):
					nu = state(u.x+x,u.y + y)
					if not self.occupied(nu): s.append(nu)
		return s.reverse()

	def updateStart(self,x,y):
		self.s_start.x = x
		self.s_start.y = y

		self.k_m += self.heuristic(self.s_last, self.s_goal)

		self.s_start = self.calculateKey(self.s_start)

		self.s_last = self.s_start

	def updateGoal(self,x,y):
		toAdd = []

		kk = []

		for cell,info in self.cellHash.iteritems():
			if not self.close(info.cost, self.C1):
				tp = [ipoint(),None]
				tp[0].x = cell.x
				tp[0].y = cell.y
				tp[1] = info.cost
				toAdd.append(tp)

		self.cellHash = {}
		self.openHash = {}

		self.openList = Queue.PriorityQueue()

		self.k_m = 0

		self.s_goal.x = x
		self.s_goal.y = y

		tmp = cellInfo()
		tmp.g = tmp.rhs = 0
		tmp.cost = self.C1

		self.cellHash[s_goal] = tmp
		self.s_start = self.calculateKey(self.s_start)

		for item in toAdd:
			updateCell(item[0].x,item[0].y, item[1])


	def computeShortestPath(self):
		s = []

		if self.openList.empty(): return 1



		








x = Dstar(0, 10, 40, 40)
y = state()
y.x , y.y = 1,2
x.makeNewCell(y)
m = x.getSucc(y)