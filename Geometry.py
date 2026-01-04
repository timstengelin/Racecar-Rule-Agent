from math import *

def metaToSegment(point, start, end):
	'''Return distance to segment, closest point on segment, length driven on segment'''
	
	(px, py) = point
	(x0, y0) = start
	(x1, y1) = end
				
	segment_length_squared = (x1 - x0)**2 + (y1 - y0)**2

	t = max(0, min(1, ((px - x0) * (x1 - x0) + (py - y0) * (y1 - y0)) / segment_length_squared))

	closest_x = x0 + t * (x1 - y0)
	closest_y = y0 + t * (y1 - y0)
	
	return sqrt((px-closest_x)**2 + (py-closest_y)**2), (closest_x, closest_y), sqrt((px-x0)**2 + (py-y0)**2)

def distanceToSegment(point, start, end):
	d, p, l = metaToSegment(point, start, end)
	return d
	
def lengthOnSegment(point, start, end):
	d, p, l = metaToSegment(point, start, end)
	return l
	
def metaToArc(point, center, radius, startAngle, endAngle):
	(c_x, c_y) = center
	(px, py) = point
	
	distance_to_center = sqrt((px - c_x)**2 + (py - c_y)**2)

	# Angle from center to point
	angle = atan2(py - c_y, px - c_x)
				
	interior = False
	for a in [angle, angle+2*pi, angle-2*pi]:
		if startAngle <= a <= endAngle or startAngle >= a >= endAngle:
			closest_x = c_x + radius*cos(a)
			closest_y = c_y + radius*sin(a)
			l = radius*abs(a-startAngle)
			d = sqrt((px-closest_x)**2 + (py-closest_y)**2)
			return d, (closest_x, closest_y), l
			
	start_x = c_x + radius * cos(startAngle)
	start_y = c_y + radius * sin(startAngle)
	end_x = c_x + radius * cos(endAngle)
	end_y = c_y + radius * sin(endAngle)

	distance_to_start = sqrt((px - start_x)**2 + (py - start_y)**2)
	distance_to_end = sqrt((px - end_x)**2 + (py - end_y)**2)

	if distance_to_start < distance_to_end:
		closest_x = start_x
		closest_y = start_y
		l = 0
		d = distance_to_start
	else:
		closest_x = end_x
		closest_y = end_y
		l = radius*(endAngle-startAngle)
		d = distance_to_end
		
	return d, (closest_x, closest_y), l

def metaToAll(point, components):
	closest_d = float('inf')
	for i, piece in enumerate(components):
		if piece[0] == 'line':
			d, p, l = metaToSegment(point, piece[1], piece[2])
		else:
			d, p, l = metaToArc(point, piece[1], piece[2], piece[3], piece[4])
			
		if d < closest_d:
			closest_i = i
			closest_d = d
			closest_p = p
			closest_l = l
			
	return closest_d, closest_p, closest_l, closest_i

def distanceToArc(point, center, radius, startAngle, endAngle):
	d, p, l = metaToArc(point, center, radius, startAngle, endAngle)
	return d

def lengthOnArc(point, center, radius, startAngle, endAngle):
	d, p, l = metaToArc(point, center, radius, startAngle, endAngle)
	return l
	
def parallelSegments(start, end):
	vector = (end[0]-start[0], end[1]-start[1])
	
	l = sqrt( vector[0]**2 + vector[1]**2 )
	vector = ( vector[0]/l, vector[1]/l )
	
	return [ ((start[0]+vector[1], start[1]-vector[0]), (end[0]+vector[1], end[1]-vector[0])),
		((start[0]-vector[1], start[1]+vector[0]), (end[0]-vector[1], end[1]+vector[0])) ]
		
def distanceOnRayToSegmentBoundary(point, direction, start, end):
	closest_t = float('inf')
	closest_p = None
	for (s,e) in parallelSegments(start, end):
		v1 = (s[0]-e[0], s[1]-e[1])
		v2 = (s[0]-point[0], s[1]-point[1])
		
		denom = v1[0]*sin(direction) - v1[1]*cos(direction)
		
		if denom != 0:
			t = (v1[0]*v2[1] - v1[1]*v2[0]) / denom
			u = (v2[0]*sin(direction) - v2[1]*cos(direction)) / denom
			
			if t >= 0 and 0 <= u <= 1:
				p = (point[0] + t*cos(direction), point[1] + t*sin(direction))
			
				if t < closest_t:
					closest_t = t
					closest_p = p
		
	return closest_t, closest_p

def distantanceOnRayToArcBoundary(point, direction, center, radius, startAngle, endAngle):
	closest_t = float('inf')
	closest_p = None
	for r in [radius-1,radius+1]:
		a = 1
		b = 2*(cos(direction)*(point[0]-center[0]) + sin(direction)*(point[1]-center[1]))
		c = (point[0]-center[0])**2 + (point[1]-center[1])**2 - r**2
		
		d = b*b - 4*a*c
		
		ts = []
		if a != 0:
			if d > 0:
				ts.append( (-b+sqrt(d))/(2*a) )
				ts.append( (-b-sqrt(d))/(2*a) )
			elif d == 0:
				ts.append( -b/(2*a) )
		else:
			ts.append(r)
		
		for t in ts:
			if t > 0 and t < closest_t:
				p =  (point[0] + t*cos(direction), point[1] + t*sin(direction))
				angle = atan2(p[1] - center[1], p[0] - center[0])
				for offset in [-4*pi,-2*pi,0,2*pi,4*pi]:
					if startAngle <= angle + offset <= endAngle or startAngle >= angle + offset >= endAngle:
						closest_t = t
						closest_p = p
						
	return closest_t, closest_p 
