def translate(angle: float) -> int:
	"""
	Converts an angle in degrees to the corresponding input
	for the duty_u16 method of the servo class

	See https://docs.micropython.org/en/latest/library/machine.PWM.html for more
	details on the duty_u16 method
	"""

	MIN = 1638 # 0 degrees
	MAX = 8192 # 180 degrees
	DEG = (MAX - MIN) / 180 # value per degree

	# clamp angle to be between 0 and 180
	angle = max(0, min(180, angle))

	return int(angle * DEG + MIN)