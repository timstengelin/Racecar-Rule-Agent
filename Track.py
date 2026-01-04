from math import sin, cos, sqrt, atan2, pi
from Geometry import *
import sys

class Track:
	"""Stores a track for racecar simulations."""
	
	def __init__(self, components, startFinishLine, name):
		"""Create a new track instance.
		
		Parameters
		----------
		components: list of lines and circular arcs that build the track (optional
		startFinishLine:  endpoints of the start finish line
		
		piences of the components are tuples of one of two types:
			('line', start, end)
			('arc, center, radius, startAngle, endAngle)
		where all coordinates are tuples and angles in radians.  For circular
		arcs ensure the the endAngle is larger than the start.  
		The track will have width 2 units centered on the curve.
		"""
		
		self._components = list()
		self._startFinish = startFinishLine
		self._length = 0.0
		self._componentLength = []
		self._name = name
		
		courseStart = None
		courseEnd = None
		if components:
			for piece in components:
				start, end = Track.endPoints(piece)
				if not courseStart:
					courseStart = start
				courseEnd = end
				self._components.append(piece)
				
				l = Track.length(piece)
				self._componentLength.append(l)
				self._length += l
				
	@staticmethod
	def endPoints(component):
		"""Find the endpoints of component of the track."""
		if component[0] == 'line':
			return component[1], component[2]
		else:
			c_x = component[1][0]
			c_y = component[1][1]
			radius = component[2]
			start_angle = component[3]
			end_angle = component[4]
			
			start = (c_x + radius*cos(start_angle), c_y + radius*sin(start_angle))
			end = (c_x + radius*cos(end_angle), c_y + radius*sin(end_angle))
			
			return start, end
			
	@staticmethod
	def length(component):
		"""Calculate the length of one of the track components."""
		if component[0] == 'line':
			(x0, y0) = component[1]
			(x1, y1) = component[2]
			return sqrt( (x0-x1)**2 + (y0-y1)**2 )
		else:
			r = component[2]
			start_angle = component[3]
			end_angle = component[4]
			
			return r*abs(end_angle-start_angle)
			
	def closestPoint(self, px, py):
		nearest_d = float('inf')
		
		for piece in self._components:
			if piece[0] == 'line':
				d, p, l = metaToSegment((px,py), piece[1], piece[2])
			else:
				d, p, l = metaToArc((px,py), piece[1], piece[2], piece[3], piece[4])
				
			if d < nearest_d:
				nearest_d = d
				nearest_p = p
				
		return nearest_p
		
	def distance(self, point):
		nearest_d = float('inf')
		
		for piece in self._components:
			if piece[0] == 'line':
				d = distanceToSegment(point, piece[1], piece[2])
			else:
				d = distanceToArc(point, piece[1], piece[2], piece[3], piece[4])
				
			if d < nearest_d:
				nearest_d = d
				
		return nearest_d
		
	def distanceTravelled(self, x, y): 
		closest_d, closest_p, closest_l, closest_i = metaToAll((x,y), self._components)
		
		l = 0
		for i in range(closest_i):
			l += self._componentLength[i]
			
		return l + closest_l

	def getLidar(self, point, direction):
		distances = []
		for _, a in enumerate([direction-45/180*pi, direction-10/180*pi, direction, direction+10/180*pi, direction+45/180*pi]):
			while a > 2*pi: a -= 2*pi
			while a < -2*pi: a += 2*pi
			nearest_d = float('inf')
			nearest_p = point
			for i, piece in enumerate(self._components):
				if piece[0] == 'line':
					d, p = distanceOnRayToSegmentBoundary(point, a, piece[1], piece[2])
				else:
					d, p = distantanceOnRayToArcBoundary(point, a, piece[1], piece[2], piece[3], piece[4])
					
				if d < nearest_d:
					nearest_d = d
					nearest_p = p
					
			# TODO: Commented out to surpress console ouput during learning
			#if nearest_d == float('inf'):
				#print('ERROR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')

			distances.append((nearest_d,nearest_p))
			
		return tuple(distances)
				
					
def buildTrack(num):
	if num == 1:
		s1 = ('arc', (0,0), 10, pi/2, pi)
		s2 = ('arc', (0,0), 10, pi, 3*pi/2)
		s3 = ('arc', (0,0), 10, -pi/2, 0)
		s4 = ('arc', (0,0), 10, 0, pi/2)
		sf = ( (0,11), (0,9) )
		return Track([s1,s2,s3,s4], sf, 'Large Circle')
	elif num == 2:
		s1 = ('arc', (0,0), 10, pi/2, 0)
		s2 = ('arc', (0,0), 10, 0, -pi/2)
		s3 = ('arc', (0,0), 10, -pi/2, -pi)
		s4 = ('arc', (0,0), 10, pi, pi/2)
		sf = ( (0,9), (0,11) )
		return Track([s1,s2,s3,s4], sf, 'Large Circle CCW')
	elif num == 3:
		s1 = ('arc', (0,0), 5, pi/2, pi)
		s2 = ('arc', (0,0), 5, pi, 3*pi/2)
		s3 = ('arc', (0,0), 5, -pi/2, 0)
		s4 = ('arc', (0,0), 5, 0, pi/2)
		sf = ( (0,6), (0,4) )
		return Track([s1,s2,s3,s4], sf, 'Medium Circle')
	elif num == 4:
		s1 = ('arc', (0,0), 5, pi/2, 0)
		s2 = ('arc', (0,0), 5, 0, -pi/2)
		s3 = ('arc', (0,0), 5, -pi/2, -pi)
		s4 = ('arc', (0,0), 5, pi, pi/2)
		sf = ( (0,4), (0,6) )
		return Track([s1,s2,s3,s4], sf, 'Medium Circle CCW')
	elif num == 5:
		s1 = ('arc', (0,0), 2, pi/2, pi)
		s2 = ('arc', (0,0), 2, pi, 3*pi/2)
		s3 = ('arc', (0,0), 2, -pi/2, 0)
		s4 = ('arc', (0,0), 2, 0, pi/2)
		sf = ( (0,3), (0,1) )
		return Track([s1,s2,s3,s4], sf, 'Small Circle')
	elif num == 6:
		s1 = ('arc', (0,0), 2, pi/2, 0)
		s2 = ('arc', (0,0), 2, 0, -pi/2)
		s3 = ('arc', (0,0), 2, -pi/2, -pi)
		s4 = ('arc', (0,0), 2, pi, pi/2)
		sf = ( (0,1), (0,3) )
		return Track([s1,s2,s3,s4], sf, 'Small Circle CCW')
	elif num == 7:
		s1 = ('arc', (10,5), 5, -pi/2, 0)
		s2 = ('arc', (10,5), 5, 0, pi/2)
		s3 = ('line', (10,10), (0,10))
		s4 = ('arc', (0,5), 5, pi/2, pi)
		s5 = ('arc', (0,5), 5, pi, 3*pi/2)
		s6 = ('line', (0,0), (10,0))
		sf = ( (10,-1), (10,1) )
		return Track([s1,s2,s3,s4,s5,s6], sf, 'Oval')
	elif num == 8:
		s1 = ('arc', (0,0), 1.5, -pi/2, 0)
		s2 = ('arc', (0,0), 1.5, 0, pi/2)
		s3 = ('arc', (0,3), 1.5, -pi/2, -pi)
		s4 = ('arc', (0,3), 1.5, -pi, -3*pi/2)
		s5 = ('arc', (0,0), 4.5, pi/2, 0)
		s6 = ('arc', (0,0), 4.5, 0, -pi/2)
		s7 = ('arc', (0,3), 7.5, -pi/2, -pi)
		s8 = ('arc', (0,3), 7.5, -pi, -3*pi/2)
		s9 = ('arc', (0,0), 10.5, pi/2, 0)
		s10 = ('arc', (0,0), 10.5, 0, -pi/2)
		s11 = ('arc', (0,-9), 1.5, -pi/2, -pi)
		s12 = ('arc', (0,-9), 1.5, -pi, -3*pi/2)
		s13 = ('arc', (0,0), 7.5, -pi/2, 0)
		s14 = ('arc', (0,0), 7.5, 0, pi/2)
		s15 = ('arc', (0,3), 4.5, pi/2, pi)
		s16 = ('arc', (0,3), 4.5, pi, 3*pi/2)
		sf = ( (0,-2.5), (0,-.5) )
		return Track([s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15,s16], sf, 'Spiral')
	else:
		print('Invalid track')
		sys.exit(0)
