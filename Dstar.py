import Queue
import math
import copy
import cv2
import numpy as np
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

	def __ge__(self,other):		
		return self.__gt__(other) or self.__eq__(other)

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
		return "x:" + str(self.x) + "y:" + str(self.y) + " k= [" + str(self.k[0])+" , " + str(self.k[1]) + "]"

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
	return ((math.sqrt(2) - 1) * minim) + maxim
		


class Dstar:
	minx,miny,maxx,maxy=0,0,200,200
	openList = Queue.PriorityQueue()
	cellHash = {}
	openHash = {}
	s_start, s_goal, s_last = state(), state() ,state()
	maxSteps = 80000
	C1 = 1
	k_m = 0
	path = []

	def __init__(self,sx,sy,gx,gy):
		self.openList = Queue.PriorityQueue()
		self.cellHash = {}
		self.openHash = {}

		self.k_m = 0
		self.path = []
		self.s_start.x = sx
		self.s_start.y = sy
		self.s_goal.x = gx
		self.s_goal.y = gy

		tmp = cellInfo()
		tmp.g = tmp.rhs = 0
		tmp.cost = self.C1

		self.cellHash[self.s_goal] = tmp

		tmp = cellInfo()
		tmp.g = tmp.rhs = self.heuristic(self.s_start,self.s_goal)
		tmp.cost = self.C1
		self.cellHash[self.s_start] = tmp
		print tmp.g,tmp.rhs
		self.s_start = self.calculateKey(self.s_start)
		self.s_last = self.s_start



	def close(self,x,y):
		if (math.isinf(x) and math.isinf(y)): return True
		return (abs(x-y) < 0.00001)

	def makeNewCell(self,u):
		exists = self.cellHash.get(u,None)
		if exists: return
		self.maxx = max([self.maxx,u.x]) 
		self.minx = min([self.minx,u.x]) 
		self.maxy = max([self.maxy,u.y]) 
		self.miny = min([self.miny,u.y]) 
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
		return cur.g

	def getRHS(self,u):
		if (u == self.s_goal): return 0;

		cur = self.cellHash.get(u,None)
		if cur == None:
			return self.heuristic(u, self.s_goal)
		return cur.rhs

	def setG(self,u,g):
		self.makeNewCell(u)
		self.cellHash[u].g = g

	def setRHS(self, u ,rhs):
		self.makeNewCell(u)
		self.cellHash[u].rhs = rhs

	
	def computeShortestPath(self):
		s = []

		if self.openList.empty(): return 1

		k = 0
		s_start =self.calculateKey(self.s_start)
		# if not(self.openList.queue[0] < s_start): 
		# 	print "self.getRHS(s_start) != self.getG(s_start)" , self.getRHS(s_start) , self.getG(s_start)
		# 	print "STATR", s_start, self.openList.queue, len(self.openList.queue)
						
		while( ((not self.openList.empty()) and (self.openList.queue[0] < s_start)) or
		 ( (self.getRHS(s_start) != self.getG(s_start)))
		  ):
			k = k + 1
			print "QUEEE",len(self.openList.queue)
			if k > self.maxSteps:
				print "Reached max steps"
				return -1

			u = state()

			test = (self.getRHS(self.s_start) != self.getG(self.s_start))

			while True:
				if self.openList.empty():
					# print "Run OUT"
					return 1
				u = self.openList.get()
				if self.isValid(u):
					if (not(u < s_start) and (not test)): 
						print  "not u < start", u ,s_start, test
						return 2
					break
				

			del self.openHash[u]
			

			k_old =  u;

			if (k_old < self.calculateKey(u)):
				self.insert(u)
			elif (self.getG(u) > self.getRHS(u)):
				self.setG(u,self.getRHS(u))
				s = self.getPred(u)
				for i in s:
					self.updateVertex(i)

			else:
				self.setG(u,float("Inf"))
				s = self.getPred(u)
				for i in s:
					self.updateVertex(i)
				self.updateVertex(u)
		print "STEPS", k
		return 0

	def updateVertex(self,u,flag=None):

		if (u != self.s_goal):
			succL = self.getSucc(u)
			tmp = float("Inf")
			tmp2 = None

			for i in succL:
				tmp2 = self.getG(i) + self.cost(u, i)
				if tmp2 < tmp: tmp = tmp2

			if not(self.close(self.getRHS(u),tmp)):
				# print "SET RHS = " , u, tmp
				self.setRHS(u, tmp)

		if not(self.close(self.getG(u), self.getRHS(u))):
			# print "inserted"
			self.insert(u)
		# else:
			# print "NOT INSERTED"


	def insert(self,u):
		u = self.calculateKey(u)
		cur = self.openHash.get(u,None)
		csum = self.keyHashCode(u)

		if (cur != None and self.close(csum,cur)):
			return

		self.openHash[u] = csum
		self.openList.put(u)


	def calculateKey(self,u):
		val = min(self.getRHS(u),self.getG(u))
		u.k = [val + self.heuristic(u, self.s_start) + self.k_m,val]
		# print u,self.heuristic(u, self.s_start)
		return u

	def cost(self,a,b):
		xd = abs(a.x-b.x)
		xy = abs(a.y-b.y)
		scale = 1

		if xd+xy > 1: scale = math.sqrt(2)

		val = self.cellHash.get(a,None)
		if val == None: return scale*self.C1
		# print "this SHOULD BE NEGATIVE",val.cost, val,a
		return scale * val.cost

	def updateCell(self,x,y,val,flag = None):
		u = state()
		u.x = x
		u.y = y

		if ((u == self.s_start) or (u == self.s_goal)): return

		self.makeNewCell(u)
		self.cellHash[u].cost = val
		self.updateVertex(u,flag = flag)

	def getSucc(self,u,r = 1):
		s = []
		u.k[0], u.k[1] = -1, -1
		if self.occupied(u): return []

		for x in range(-r,1+r):
			for y in range(-r,1+r):
				if not(x == 0 and y == 0):
					s.append(state(u.x+x,u.y + y))
		return s

	
	def keyHashCode(self,u):
		return u.k[0] + 1193*u.k[1]

	def isValid(self,u):
		cur = self.openHash.get(u)
		if cur == None: return False
		if not(self.close(self.keyHashCode(u), cur)):
			# print "THERE BUT NOOT VALID", self.keyHashCode(u),cur
			return False
		return True

	
	def remove(self,u):
		cur = self.openHash.get(u)
		if cur == None: return
		del self.openHash[u]

	def trueDist(self,a,b):
		x = a.x - b.x
		y = a.y - b.y
		return math.sqrt(x*x + y*y)

	def getPred(self,u):
		s = []

		u.k[0], u.k[1] = -1,-1

		for x in range(-1,2):
			for y in range(-1,2):
				if not(x == 0 and y == 0):
					nu = state(u.x+x,u.y + y)
					if not self.occupied(nu): s.append(nu)
		return s

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
		#originally cost = c1 also in init
		tmp.cost = self.trueDist(self.s_start,self.s_goal)

		self.cellHash[self.s_goal] = tmp
		self.s_start = self.calculateKey(self.s_start)

		for item in toAdd:
			self.updateCell(item[0].x,item[0].y, item[1])


	
	def replan(self):
		self.path = []
		path = []
		print self.s_start
		res = self.computeShortestPath()
		print "RES", res

		if res < 0:
			print "NO PATH1"
			return False


		cur_state= self.s_start

		if (math.isinf(self.getG(self.s_start))):
			print "NO PATH2"
			return False

		while cur_state != self.s_goal:
			path.append(cur_state)
			# s_set = set()
			# self.visualize(path)
			# print cur_state
			
			s = self.getSucc(cur_state)
			if len(s) == 0:
				print "NO PATH3"
				return False

			cmin = float("Inf")
			tmin = None

			smin = state()
			for i,place in enumerate(s):
				val = self.cost(cur_state,place)
				val2 = self.trueDist(place,self.s_goal) + self.trueDist(self.s_start,place)

				val += self.getG(place)
				# print "p",i,place
				# self.visualize(point=place,points=list(s_set))
				# self.visualize(point=place,waitKey=1)


				if self.close(val,cmin):
					if tmin > val2:
						tmin = val2
						cmin = val
						smin = place
				elif val < cmin:
						tmin = val2
						cmin = val
						smin = place
			cur_state = smin
		path.append(self.s_goal)
		self.path = path
		return True

	def visualize(self,path=None,points = None,point = None,waitKey=1):
		frame = np.zeros((self.maxx+ 30,self.maxy+30,3),np.uint8)
		if point:
				frame[point.x,point.y] = (255,255,255)
		if points:
			for point in points:
				frame[point.x,point.y] = (255,0,255)

		for node,x in self.openHash.items():
			frame[node.x,node.y] = (128,0,128)

		for node,info in self.cellHash.iteritems():
			if info.cost == 1:
				frame[node.x,node.y] = (0,255,0)
			if info.cost < 0:
				r = 255 * info.cost * -1
				frame[node.x,node.y] = (0,0,r)
			else:
				frame[node.x,node.y] = (255,0,0)

		# for x in self.openList.queue:
		# 	frame[x.x,x.y] = (128,0,128)
		

		for x in self.path:
			frame[x.x,x.y] = (0,255,0)
		frame[self.s_start.x,self.s_start.y] = (255,0,0)
		cv2.imshow("path",frame)
		if (path or point) and waitKey:
			cv2.waitKey(waitKey)

#usage ()
xx = Dstar(40,50,140,90)
# for x in range(xx.maxx):
# 	for y in range(xx.maxy):
		# xx.updateCell(x,y,0)
cv2.namedWindow('path',cv2.WINDOW_NORMAL)
xx.visualize()

drawing = False # true if mouse is pressed
mode = True # if True, draw rectangle. Press 'm' to toggle to curve
ix,iy = -1,-1
autoreplan = False
change = False
# mouse callback function
def draw_circle(event,x,y,flags,param):
	global ix,iy,drawing,mode,change
	if event == cv2.EVENT_LBUTTONDOWN:
		drawing = True
		ix,iy = x,y
	elif event == cv2.EVENT_MOUSEMOVE:
		if drawing:
			xx.updateCell(y,x,-1)
			change = True
		# if drawing == True:
		# ,0,255),-1)
	elif event == cv2.EVENT_LBUTTONUP:
		change = True
		drawing = False
		xx.updateCell(y,x,-1)

# for y in range(60):
# 	xx.updateCell(50,y,-10)
# 	xx.updateCell(51,y,-10)
# 	xx.updateCell(y,2*y+3,-1)
# 	xx.updateCell(y,2*y+1,-1)
# 	xx.updateCell(y,2*y+2,-1)

cv2.setMouseCallback('path',draw_circle)

while(1):
	k = cv2.waitKey(1) & 0xFF
	if k == ord('m'):
		mode = not mode
	elif k == ord("a"):
		autoreplan = not autoreplan
	elif k == ord("r"):
		xx.replan()
		xx.visualize()
	elif k == 27:
		break
	if autoreplan and change: 
		xx.replan()
		xx.visualize()
		change = False

	if change:
		xx.visualize()
		change = False




# xx.updatseGoal(100,100)

