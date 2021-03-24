import math

lam = 0

def convert_coordinates(x,y,z,l):

	#if(x==0):
	#	if(y>0):
	#		phi_1 = -math.pi/2
	#	else:
	#		phi_1 = math.pi/2
	#else:
	#	if(y==0):
	#		if(x>0):
	#			phi_1 = 0
	#		else:
	#			phi_1 = math.pi
	if(x!=0 and y!=0):
		phi_1 = math.atan(y/x)
		print(phi_1)

	lam = l * (1-(math.fabs(math.cos(phi_1)*math.sin(phi_1))))
	print(lam)
	r = math.sqrt(x*x+y*y)	

	phi_3 = math.acos((r*r+z*z-135*135-147*147)/(2*135*147))
	
	phi_2 = math.atan(z/r) - math.atan((147*math.sin(phi_3))/(135+147*math.cos(phi_3)))

	phi_1_prime = lam+phi_1

	x_prime = math.cos(phi_1_prime)*(math.cos(phi_2)*135 + math.cos(phi_2+phi_3)*147)
	y_prime = math.sin(phi_1_prime)*(math.cos(phi_2)*135 + math.cos(phi_2+phi_3)*147)
	print(" phi_1_prime = %s\n phi_2 = %s\n phi_3 = %s\n x_prime = %s\n y_prime = %s\n z_prime = %s\n" % (phi_1_prime,phi_2,phi_3,x_prime, y_prime, z))
	return x_prime, y_prime, z
	

#convert_coordinates(200,0,100)
