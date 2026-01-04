from Track import *
from cs1graphics import *
import time

class TrackVisualization:
	def __init__(self, track, size):
		self._track = track
		self._delay = .01
		
		minX = 1e6
		maxX = -1e6
		minY = 1e6
		maxY = -1e6
		
		buffer = 2
		
		for piece in track._components:
			if piece[0] == 'line':
				minX = min(minX, piece[1][0])
				maxX = max(maxX, piece[1][0])
				minX = min(minX, piece[2][0])
				maxX = max(maxX, piece[2][0])
				minY = min(minY, piece[1][1])
				maxY = max(maxY, piece[1][1])
				minY = min(minY, piece[2][1])
				maxY = max(maxY, piece[2][1])
			
			if piece[0] == 'arc':
				if  piece[4] > piece[3]:
					deg0 = int(180*piece[3]/pi)
					deg1 = int(180*piece[4]/pi)
					for a in range(deg0, deg1+1):
						a = pi/180*a
						minX = min(minX, piece[1][0]+piece[2]*cos(a))
						maxX = max(maxX, piece[1][0]+piece[2]*cos(a))
						minY = min(minY, piece[1][1]+piece[2]*sin(a))
						maxY = max(maxY, piece[1][1]+piece[2]*sin(a))
				else:
					deg0 = int(180*piece[3]/pi)
					deg1 = int(180*piece[4]/pi)
					for a in range(deg0, deg1-1, -1):
						a = pi/180*a
						minX = min(minX, piece[1][0]+piece[2]*cos(a))
						maxX = max(maxX, piece[1][0]+piece[2]*cos(a))
						minY = min(minY, piece[1][1]+piece[2]*sin(a))
						maxY = max(maxY, piece[1][1]+piece[2]*sin(a))
			
		minX -= buffer
		minY -= buffer
		maxX += buffer
		maxY += buffer
		
		if maxX - minX > maxY - minY:
			self._width = size
			self._height = size*(maxY-minY)/(maxX-minX)
			self._canvas = Canvas(self._width, self._height)
		else:
			self._width = size*(maxX-minX)/(maxY-minY)
			self._height = size
			self._canvas = Canvas(self._width, self._height)
		
		self._canvas.setTitle(track._name)
		self._canvas.setBackgroundColor('light green')
		
		self._minX = minX
		self._minY = minY
		self._maxX = maxX
		self._maxY = maxY
		
		width = self._newX(2) - self._newX(0)
		
		p = Path()
		p.setBorderColor('red')
		p.setBorderWidth(1.1*width)
		for piece in track._components:
			if piece[0] == 'line':
				p.addPoint(Point(self._newX(piece[1][0]),self._newY(piece[1][1])))
				p.addPoint(Point(self._newX(piece[2][0]),self._newY(piece[2][1])))
			else:
				if  piece[4] > piece[3]:
					deg0 = int(180*piece[3]/pi)
					deg1 = int(180*piece[4]/pi)
					for a in range(deg0, deg1+1):
						a = pi/180*a
						p.addPoint(Point(self._newX(piece[1][0]+piece[2]*cos(a)), self._newY(piece[1][1]+piece[2]*sin(a))))
				else:
					deg0 = int(180*piece[3]/pi)
					deg1 = int(180*piece[4]/pi)
					for a in range(deg0, deg1-1, -1):
						a = pi/180*a
						p.addPoint(Point(self._newX(piece[1][0]+piece[2]*cos(a)), self._newY(piece[1][1]+piece[2]*sin(a))))
		self._canvas.add(p)
		
		p = p.clone()
		p.setBorderColor('light grey')
		p.setBorderWidth(width)
		self._canvas.add(p)
		
		p = p.clone()
		p.setBorderColor('white')
		p.setBorderWidth(.05*width)
		self._canvas.add(p)
		
		# Start-finish line
		p = Path( Point(self._newX(track._startFinish[0][0]), self._newY(track._startFinish[0][1])), Point(self._newX(track._startFinish[1][0]), self._newY(track._startFinish[1][1])) )
		
		p.setBorderColor('yellow')
		p.setBorderWidth(.05*width)
		self._canvas.add(p)
		
		# Car
		self._rays = []
		for i in range(5):
			p = Path([Point(0,0), Point(10,10)])
			p.setBorderColor('violet')
			p.setBorderWidth(.05*width)
			self._rays.append(p)
			self._canvas.add(p)
			
		self._car = Circle(.15*width)
		self._car.setFillColor('blue')
		self._canvas.add(self._car)
			
		# Score
		self._reward = Text('0', .75*width, Point(.85*self._width, .1*self._height))
		self._canvas.add(self._reward)
		
		self._action = Text('', .5*width, Point(.15*self._width, .1*self._height))
		self._canvas.add(self._action)
		
	def setDelay(self, delay):
		self._delay = delay
		
	def _newX(self, x):
		return (x-self._minX)*self._width/(self._maxX-self._minX)
		
	def _newY(self, y):
		return (y-self._minY)*self._height/(self._maxY-self._minY)

	def update(self, car, observations, totalReward, action):
		self._car.moveTo(self._newX(car._x), self._newY(car._y))

		for i, r in enumerate(self._rays):
			r.setPoint( Point(self._newX(car._x), self._newY(car._y)), 0 )
			if observations[i][1]:
				r.setPoint( Point(self._newX(observations[i][1][0]), self._newY(observations[i][1][1])), 1 )
			else:
				r.setPoint( Point(self._newX(car._x), self._newY(car._y)), 1 )
			
		self._reward.setMessage(f'{totalReward:.2f}')
		self._action.setMessage(f'{action[0]}\n{action[1]}')
		
		time.sleep(self._delay)
		


'''
s1 = ('line', (0,0), (10,0))
s2 = ('arc', (10,5), 5, -pi/2, pi/2)
s3 = ('line', (10,10), (0,10))
s4 = ('arc', (0,5), 5, pi/2, 3*pi/2)
t = Track([s1,s2,s3,s4])
v = TrackVisualization(t, 1000)


s1 = ('arc', (0,0), 1.5, -pi/2, pi/2)
s2 = ('arc', (0,3), 1.5, -pi/2, -3*pi/2)
s3 = ('arc', (0,0), 4.5, pi/2, -pi/2)
s4 = ('arc', (0,3), 7.5, -pi/2, -3*pi/2)
s5 = ('arc', (0,0), 10.5, pi/2, -pi/2)
s6 = ('arc', (0,-9), -1.5, pi/2, -pi/2)
s7 = ('arc', (0,0), 7.5, -pi/2, pi/2)
s8 = ('arc', (0,3), 4.5, pi/2, 3*pi/2)
sf = ( (0,.5), (0,2.5) )
sf2 = ( (-.1,.5), (-.1,2.5) )

t = Track([s1,s2,s3,s4,s5,s6,s7,s8], sf)
v = TrackVisualization(t, 1000)
'''
