class Entity:

	#Enemy, Guard, or Waypoint
	category = None
	#World coordinates of entity
	position = [0,0]


	def __init__(self, category, position):
		self.category = category
		self.position = position