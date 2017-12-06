class Entity:

	category = None
	position = [0,0]


	def __init__(self, category, position):
		self.category = category
		self.position = position