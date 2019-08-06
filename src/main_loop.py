# - *- coding: utf- 8 - *-

import RPi.GPIO as io
import time
import math

io.setmode(io.BOARD)

hz = 50
dt = 1/hz
kr = 48
enc_res = 0.01636246
num_samples = 100
special_words = ['BackSpace', 'Tab', 'Enter', 'Cap', 'Shift2', 'Ctrl1', 
	'WIN1', 'Alt1', 'Alt2', 'WIN2', 'MClick', 'Ctrl2', 'Shift1', '\\']
L1=0.115 # m
L2=0.064 # m
a1=0.018 # m

# mm units converted at output of keypose
off = 56 # mm
a0=50+off # mm 
zz=24 # mm
zs=float (19) #mm
yy=18 #mm
k=float (0.048)
keydic={'Ctrl1':[0,-(a0+20),12],
		'WIN1':[0,-(a0+20),12+zz],
		'Alt1':[0,-(a0+20),12+2*zz],
		' ':[0,-(a0+20),130],
		'Alt2':[0,-(a0+20),130+12+2*zz],
		'WIN2':[0,-(a0+20),142+3*zz],
		'MClick':[0,-(a0+20),142+4*zz],
		'Ctrl2':[0,-(a0+20),142+5*zz],
		'Shift1':[float (k*yy),-(a0+20+yy),22],
		'z':[float (k*yy),-(a0+20+yy),53],
		'Z':[float (k*yy),-(a0+20+yy),53],
		'x':[float (k*yy),-(a0+20+yy),53+zs],
		'X':[float (k*yy),-(a0+20+yy),53+zs],
		'c':[float (k*yy),-(a0+20+yy),53+2*zs],
		'C':[float (k*yy),-(a0+20+yy),53+2*zs],
		'v':[float (k*yy),-(a0+20+yy),53+3*zs],
		'V':[float (k*yy),-(a0+20+yy),53+3*zs],
		'b':[float (k*yy),-(a0+20+yy),53+4*zs],
		'B':[float (k*yy),-(a0+20+yy),53+4*zs],
		'n':[float (k*yy),-(a0+20+yy),53+5*zs],
		'N':[float (k*yy),-(a0+20+yy),53+5*zs],
		'm':[float (k*yy),-(a0+20+yy),53+6*zs],
		'M':[float (k*yy),-(a0+20+yy),53+6*zs],
		',':[float (k*yy),-(a0+20+yy),53+7*zs],
		'.':[float (k*yy),-(a0+20+yy),53+8*zs],
		'/':[float (k*yy),-(a0+20+yy),53+9*zs],
		'Shift2':[float (k*yy),-(a0+20+yy),22+10*zs+43],
		'Cap':[float (2*k*yy),-(a0+20+2*yy),17],
		'a':[float (2*k*yy),-(a0+20+2*yy),43],
		'A':[float (2*k*yy),-(a0+20+2*yy),43],
		's':[float (2*k*yy),-(a0+20+2*yy),43+zs],
		'S':[float (2*k*yy),-(a0+20+2*yy),43+zs],
		'd':[float (2*k*yy),-(a0+20+2*yy),43+2*zs],
		'D':[float (2*k*yy),-(a0+20+2*yy),43+2*zs],
		'f':[float (2*k*yy),-(a0+20+2*yy),43+3*zs],
		'F':[float (2*k*yy),-(a0+20+2*yy),43+3*zs],
		'g':[float (2*k*yy),-(a0+20+2*yy),43+4*zs],
		'G':[float (2*k*yy),-(a0+20+2*yy),43+4*zs],
		'h':[float (2*k*yy),-(a0+20+2*yy),43+5*zs],
		'H':[float (2*k*yy),-(a0+20+2*yy),43+5*zs],
		'j':[float (2*k*yy),-(a0+20+2*yy),43+6*zs],
		'J':[float (2*k*yy),-(a0+20+2*yy),43+6*zs],
		'k':[float (2*k*yy),-(a0+20+2*yy),43+7*zs],
		'K':[float (2*k*yy),-(a0+20+2*yy),43+7*zs],
		'l':[float (2*k*yy),-(a0+20+2*yy),43+8*zs],
		'L':[float (2*k*yy),-(a0+20+2*yy),43+8*zs],
		';':[float (2*k*yy),-(a0+20+2*yy),43+9*zs],
		'\\':[float (2*k*yy),-(a0+20+2*yy),43+10*zs],
		'Enter':[float (2*k*yy),-(a0+20+2*yy),33+20+11*zs],
		'Tab':[float (3*k*yy),-(a0+20+3*yy),15],
		'q':[float (3*k*yy),-(a0+20+3*yy),39],
		'Q':[float (3*k*yy),-(a0+20+3*yy),39],
		'w':[float (3*k*yy),-(a0+20+3*yy),zs+39],
		'W':[float (3*k*yy),-(a0+20+3*yy),zs+39],
		'e':[float (3*k*yy),-(a0+20+3*yy),2*zs+39],
		'E':[float (3*k*yy),-(a0+20+3*yy),2*zs+39],
		'r':[float (3*k*yy),-(a0+20+3*yy),3*zs+39],
		'R':[float (3*k*yy),-(a0+20+3*yy),3*zs+39],
		't':[float (3*k*yy),-(a0+20+3*yy),4*zs+39],
		'T':[float (3*k*yy),-(a0+20+3*yy),4*zs+39],
		'y':[float (3*k*yy),-(a0+20+3*yy),5*zs+39],
		'Y':[float (3*k*yy),-(a0+20+3*yy),5*zs+39],
		'u':[float (3*k*yy),-(a0+20+3*yy),6*zs+39],
		'U':[float (3*k*yy),-(a0+20+3*yy),6*zs+39],
		'i':[float (3*k*yy),-(a0+20+3*yy),7*zs+39],
		'I':[float (3*k*yy),-(a0+20+3*yy),7*zs+39],
		'o':[float (3*k*yy),-(a0+20+3*yy),8*zs+39],
		'O':[float (3*k*yy),-(a0+20+3*yy),8*zs+39],
		'p':[float (3*k*yy),-(a0+20+3*yy),9*zs+39],
		'P':[float (3*k*yy),-(a0+20+3*yy),9*zs+39],
		'[':[float (3*k*yy),-(a0+20+3*yy),10*zs+39],
		']':[float (3*k*yy),-(a0+20+3*yy),11*zs+39],
		'\\':[float (3*k*yy),-(a0+20+3*yy),12*zs+30+14],
		'`':[float (4*k*yy),-(a0+20+4*yy),float (zs/2)],
		'1':[float (4*k*yy),-(a0+20+4*yy),float (zs/2+zs)],
		'2':[float (4*k*yy),-(a0+20+4*yy),float (zs/2+2*zs)],
		'3':[float (4*k*yy),-(a0+20+4*yy),float (zs/2+3*zs)],
		'4':[float (4*k*yy),-(a0+20+4*yy),float (zs/2+4*zs)],
		'5':[float (4*k*yy),-(a0+20+4*yy),float (zs/2+5*zs)],
		'6':[float (4*k*yy),-(a0+20+4*yy),float (zs/2+6*zs)],
		'7':[float (4*k*yy),-(a0+20+4*yy),float (zs/2+7*zs)],
		'8':[float (4*k*yy),-(a0+20+4*yy),float (zs/2+8*zs)],
		'9':[float (4*k*yy),-(a0+20+4*yy),float (zs/2+9*zs)],
		'0':[float (4*k*yy),-(a0+20+4*yy),float (zs/2+10*zs)],
		'-':[float (4*k*yy),-(a0+20+4*yy),float (zs/2+11*zs)],
		'=':[float (4*k*yy),-(a0+20+4*yy),float (zs/2+12*zs)],
		'BackSpace':[float (4*k*yy),-(a0+20+4*yy),13*zs+19]
		}
	
# set parameters of robot (SI UNITS)
L1,L2=0.115,0.064
len_link1=0.07
len_link2=0.04#distances of centers of mass from joint axes
m_link1=0.005
m_link2=0.003
m_motor=0.06
k=0.048
R=3.6
V=5
r_pulley=0.0181102/2 #unit meters
K_p,K_d=0.25,0.125

# motor 1
m1_in1_pin = 12
m1_in2_pin = 16
m1_en_pin = 18
chan_list = [m1_en_pin, m1_in1_pin, m1_in2_pin]
io.setup(chan_list, io.OUT)
p1 = io.PWM(m1_in1_pin, hz)
p2 = io.PWM(m1_in2_pin, hz)

# motor 2
m2_in1_pin = 22
m2_in2_pin = 32
m2_en_pin = 36
chan_list = [m2_en_pin, m2_in1_pin, m2_in2_pin]
io.setup(chan_list, io.OUT)
p3 = io.PWM(m2_in1_pin, hz)
p4 = io.PWM(m2_in2_pin, hz)

# motor 3
m3_in1_pin = 38
m3_in2_pin = 40
m3_en_pin = 37
chan_list = [m3_en_pin, m3_in1_pin, m3_in2_pin]
io.setup(chan_list, io.OUT)
p5 = io.PWM(m3_in1_pin, hz)
p6 = io.PWM(m3_in2_pin, hz)

# sensor 1
en1_pin = 35
io.setup(en1_pin, io.IN, pull_up_down=io.PUD_UP)

# sensor 2
en2_pin = 33
io.setup(en2_pin, io.IN, pull_up_down=io.PUD_UP)

# encoder 1
encoder1_sensors = [en1_pin, en2_pin]
A1_old = 0
encoder1_count = 0
A1_t1 = time.time()
vel1 = 0
vel1_vec = []

# sensor 3
en3_pin = 31
io.setup(en3_pin, io.IN, pull_up_down=io.PUD_UP)

# sensor 4
en4_pin = 29
io.setup(en4_pin, io.IN, pull_up_down=io.PUD_UP)

# encoder 2
encoder2_sensors = [en3_pin, en4_pin]
A2_old = 0
encoder2_count = 0
A2_t1 = time.time()
vel2 = 0
vel2_vec = []

# sensor 5
en5_pin = 15
io.setup(en5_pin, io.IN, pull_up_down=io.PUD_UP)

# sensor 6
en6_pin = 13
io.setup(en6_pin, io.IN, pull_up_down=io.PUD_UP)

# encoder 3
encoder3_sensors = [en5_pin, en6_pin]
A3_old = 0
encoder3_count = 0
A3_t1 = time.time()
vel3 = 0
vel3_vec = []

def clockwise(duty, pwm1, pwm2, en_pin):
	io.output(en_pin, io.HIGH)
	pwm1.start(duty)
	time.sleep(duty/100*dt)
	pwm2.start(100-duty)
	
def counter_clockwise(duty, pwm1, pwm2, en_pin):
	io.output(en_pin, io.HIGH)
	pwm2.start(duty)
	time.sleep(duty/100*dt)
	pwm1.start(100-duty)	

def countstorad(count):
	# returns the joints space angle in radians
	rad = 2*math.pi*count/8/kr
	return rad	

def radtocount(rad):
	count = rad*kr*8/(2*math.pi)
	return count
	
def initializeEncoders():
	global encoder2_count, encoder3_count
	encoder2_count = 0
	encoder3_count = -math.pi/2
	
def resetEncoders():
	global encoder1_count, encoder2_count, encoder3_count
	encoder1_count = 0
	encoder2_count = 0
	encoder3_count = radtocount(-math.pi/2)

def encoder1Callback(channel):
	# this function is called when an encoder reading is detected
	global A1_old, encoder1_count, A1_t1, vel1, vel1_vec
	A1_t2 = time.time()
	if io.input(channel):
		A = 1
	else:
		A = 0
	if io.input(encoder1_sensors[1]):
		B = 1
	else:
		B = 0
	if A != A1_old:
		if A != B:
			encoder1_count += 1
			vel1_vec.insert(0,enc_res/(A1_t2 - A1_t1))
		else:
			encoder1_count -= 1
			vel1_vec.insert(0,-enc_res/(A1_t2 - A1_t1))
	if len(vel1_vec) > num_samples:
		vel1_vec.pop()
	vel1 = sum(vel1_vec)/len(vel1_vec)
	A1_old = A
	A1_t1 = A1_t2
io.add_event_detect(en1_pin, io.BOTH, callback=encoder1Callback)
	
def encoder2Callback(channel):
	# this function is called when an encoder reading is detected
	global A2_old, encoder2_count, A2_t1, vel2, vel2_vec
	A2_t2 = time.time()
	if io.input(channel):
		A = 1
	else:
		A = 0
	if io.input(encoder2_sensors[1]):
		B = 1
	else:
		B = 0
	if A != A2_old:
		if A != B:
			encoder2_count -= 1
			vel2_vec.insert(0,-enc_res/(A2_t2 - A2_t1))
		else:
			encoder2_count += 1
			vel2_vec.insert(0,enc_res/(A2_t2 - A2_t1))
	if len(vel2_vec) > num_samples:
		vel2_vec.pop()
	vel2 = sum(vel2_vec)/len(vel2_vec)
	A2_old = A
	A2_t1 = A2_t2
io.add_event_detect(en3_pin, io.BOTH, callback=encoder2Callback)

def encoder3Callback(channel):
	# this function is called when an encoder reading is detected
	global A3_old, encoder3_count, A3_t1, vel3, vel3_vec
	A3_t2 = time.time()
	if io.input(channel):
		A = 1
	else:
		A = 0
	if io.input(encoder3_sensors[1]):
		B = 1
	else:
		B = 0
	if A != A3_old:
		if A != B:
			encoder3_count -= 1
			vel3_vec.insert(0,-enc_res/(A3_t2 - A3_t1))
		else:
			encoder3_count += 1
			vel3_vec.insert(0,enc_res/(A3_t2 - A3_t1))
	if len(vel3_vec) > num_samples:
		vel3_vec.pop()
	vel3 = sum(vel3_vec)/len(vel3_vec)
	A3_old = A
	A3_t1 = A3_t2
io.add_event_detect(en5_pin, io.BOTH, callback=encoder3Callback)

def invskinem(pose=[0,-.1,0]):
    	d1=pose[2]/(r_pulley)
	c3=float(((pose[0]-a1)**2+pose[1]**2-L1**2-L2**2)/(2*L1*L2))
	try:
		s3=-math.sqrt(1-c3**2)
	except:
		print('Whoops!')
		return
	th3=math.atan2(s3,c3)
	k=((pose[0]-a1)**2+L1**2+pose[1]**2-L2**2)/(2*L1)
	th2=math.atan2(math.sqrt((pose[0]-a1)**2+pose[1]**2-k**2),k)+math.atan2(pose[1],pose[0]-a1)
	return [d1,th2,th3]
	 
def keypose(read):
	if read in keydic:
		output=keydic.get(read)
		output = [float(output[0])/1000, float(output[1])/1000, float(output[2])/1000]
		return output
	else:
		print('Whoops! No keys found!')
	return
	
def control1(pos_d):
	try:
		# initialize the encoders
		##################################################
		#This is for motor1 control
		##################################################
		tolerance=0.005
		pos_error1=100
		f = open('data','a')
		f.write('New Data Theta 1 \n')
		print("Controlling motor 1")
		while abs(pos_error1) >=tolerance:
			pos_error1=pos_d[0]-countstorad(encoder1_count)
			duty_cycle_1=100
			if pos_error1>0:
				clockwise(duty_cycle_1, p1, p2, m1_en_pin)
			elif pos_error1<0:
				clockwise(100-duty_cycle_1,p1,p2,m1_en_pin)
			row = str(countstorad(encoder1_count))+'\t'+str(vel1)+'\n'
			f.write(row)
		p1.stop()
		p2.stop()
		time.sleep(2)
		##################################################
		#This is for motor2 and motor3 control
		##################################################
		print("Controlling Motors 2 and 3")
		f.write('New Data Theta 2, Theta 3 \n')
		position_error=[100,100]
		while max(abs(position_error[0]),abs(position_error[1])) > tolerance:
			# get current position
			pos_current=[countstorad(encoder2_count),countstorad(encoder3_count)]
			angular_velocity=[vel2,vel3]
			row = str(countstorad(encoder2_count))+'\t'+str(vel2)+'\t'+str(countstorad(encoder3_count))+'\t'+str(vel3)+'\n'
			f.write(row)
			# estimate g(q)
			g_q=[(m_link1*len_link1+m_motor*L1+m_link2*L1)*math.cos(pos_current[0])+\
			m_link2*len_link2*math.cos(pos_current[0]+pos_current[1]),\
			m_link2*len_link2*math.cos(pos_current[0]+pos_current[1])]
			# calculate position error
			position_error=[pos_d[1]-pos_current[0],pos_d[2]-pos_current[1]]
			# u = PD control with gravity compensation
			u=[g_q[0]+K_p*position_error[0]-K_d*angular_velocity[0],\
			g_q[1]+K_p*position_error[1]-K_d*angular_velocity[1]]
			for i in range(2):
				if u[i]>=0.08:
					u[i]=0.08
				elif u[i]<=-0.08:
					u[i]=-0.08
			
			# duty = function(u)
			V_d=[R*u[0]/k+k*angular_velocity[0],R*u[1]/k+k*angular_velocity[1]]
			duty=[V_d[0]/V*100,V_d[1]/V*100]
			# move the motors according to duty
			#motor1 duty cycle ##############################
			if duty[0]>0:
				if duty[0]>=100:
					duty[0]=100
				elif duty[0]<=70:
					duty[0]=50
				clockwise(duty[0], p3, p4, m2_en_pin)
			else:
				if duty[0]<=-100:
					duty[0]=0
				elif duty[0] > -100 and duty[0] <= -70:
					duty[0]=100+duty[0]
				elif duty[0]>-70:
					duty[0]=50
				clockwise(duty[0],p3,p4,m2_en_pin)
			###################################################
			#motor2 duty cycle ################################
			if duty[1]>0:
				if duty[1]>=100:
					duty[1]=100
				elif duty[1]<=70:
					duty[1]=50
				clockwise(duty[1], p5, p6, m3_en_pin)
			else:
				if duty[1]<=-100:
					duty[1]=0
				elif duty[1] > -100 and duty[1] <= -70:
					duty[1]=100+duty[1]
				elif duty[1]>-70:
					duty[1]=50
				clockwise(duty[1],p5,p6,m3_en_pin)
			####################################################

def control2(pos_d):
    try:
		# initialize the encoders
		tolerance=0.005
		pos_error1=100
		##################################################
		#This is for motor2 and motor3 control
		##################################################
		print("Controlling Motors 2 and 3")
		position_error=[100,100]
		f.open('data','a')
		f.write('New Data Theta 2 Theta 3 \n')
		while max(abs(position_error[0]),abs(position_error[1])) > tolerance:
			# get current position
			pos_current=[countstorad(encoder2_count),countstorad(encoder3_count)]
			angular_velocity=[vel2,vel3]
			row = str(countstorad(encoder2_count))+'\t'+str(vel2)+'\t'+str(countstorad(encoder3_count))+'\t'+str(vel3)+'\n'
			f.write(row)
			# estimate g(q)
			g_q=[(m_link1*len_link1+m_motor*L1+m_link2*L1)*math.cos(pos_current[0])+\
			m_link2*len_link2*math.cos(pos_current[0]+pos_current[1]),\
			m_link2*len_link2*math.cos(pos_current[0]+pos_current[1])]
			# calculate position error
			position_error=[pos_d[1]-pos_current[0],pos_d[2]-pos_current[1]]
			# u = PD control with gravity compensation
			u=[g_q[0]+K_p*position_error[0]-K_d*angular_velocity[0],\
			g_q[1]+K_p*position_error[1]-K_d*angular_velocity[1]]
			for i in range(2):
				if u[i]>=0.08:
					u[i]=0.08
				elif u[i]<=-0.08:
					u[i]=-0.08
			
			# duty = function(u)
			V_d=[R*u[0]/k+k*angular_velocity[0],R*u[1]/k+k*angular_velocity[1]]
			duty=[V_d[0]/V*100,V_d[1]/V*100]
			# move the motors according to duty
			#motor1 duty cycle ##############################
			if duty[0]>0:
				if duty[0]>=100:
					duty[0]=100
				elif duty[0]<=70:
					duty[0]=50
				clockwise(duty[0], p3, p4, m2_en_pin)
			else:
				if duty[0]<=-100:
					duty[0]=0
				elif duty[0] > -100 and duty[0] <= -70:
					duty[0]=100+duty[0]
				elif duty[0]>-70:
					duty[0]=50
				clockwise(duty[0],p3,p4,m2_en_pin)
			###################################################
			#motor2 duty cycle ################################
			if duty[1]>0:
				if duty[1]>=100:
					duty[1]=100
				elif duty[1]<=70:
					duty[1]=50
				clockwise(duty[1], p5, p6, m3_en_pin)
			else:
				if duty[1]<=-100:
					duty[1]=0
				elif duty[1] > -100 and duty[1] <= -70:
					duty[1]=100+duty[1]
				elif duty[1]>-70:
					duty[1]=50
				clockwise(duty[1],p5,p6,m3_en_pin)
			####################################################
		##################################################
		#This is for motor1 control
		##################################################
		print("Controlling motor 1")
		while abs(pos_error1) >=tolerance:
			row = str(countstorad(encoder1_count))+'\t'str(vel1)
			f.write(row)
			pos_error1=pos_d[0]-countstorad(encoder1_count)
			duty_cycle_1=100
			if pos_error1>0:
				clockwise(duty_cycle_1, p1, p2, m1_en_pin)
			elif pos_error1<0:
				clockwise(100-duty_cycle_1,p1,p2,m1_en_pin)
		p1.stop()
		p2.stop()
		p3.stop()
		p4.stop()
		p5.stop()
		p6.stop()
	except KeyboardInterrupt:
		p1.stop()
		p2.stop()
		p3.stop()
		p4.stop()
		p5.stop()
		p6.stop()
		io.cleanup()
		
def correctEncoders(desired):
	global encoder1_count, encoder2_count, encoder3_count
	pos_correct = [radtocount(desired[0]), radtocount(desired[1]), radtocount(desired[2])]
	encoder1_count = pos_correct[0]
	encoder2_count = pos_correct[1]
	encoder3_count = pos_correct[2]

def taskcontrol(command_list):
    initializeEncoders()
	n = len(command_list)
	for i in xrange(0,n):
	# for each key in string_desired
		# get position
		current_pos = [countstorad(encoder1_count), countstorad(encoder2_count), countstorad(encoder3_count)]
		# get nearest home position (theta 1 is arbitrary)
		nearest_home = [current_pos[0], 0, -math.pi/2]
		print(current_pos)
		if (current_pos[1] != nearest_home[1]) or (current_pos[2] != nearest_home[2]):
			print("Going to nearest home")
			control2(nearest_home)
			correctEncoders(nearest_home)
			print("I'm Home!")
			time.sleep(1)
		cart_pos_d = keypose(command_list[i])
		# getpose(key_desired)
		joint_pos_d = invskinem(cart_pos_d)
		# inverse kinematics to find joint space position
		print("Move to "+command_list[i])
		control1(joint_pos_d)
		correctEncoders(joint_pos_d)
		print("Motion Complete!")
		time.sleep(1)
		# control(pose_desired)
	# end for loop
	abs_home = [0, 0, -math.pi/2]
	# return to global home position
	print("Going to Absolute Home Position")
	control2(abs_home)
	correctEncoders(abs_home)
	
# This will run when executing the python file, causing the robot to type 'hello'
command_list = ['h','e','l','l','o']
taskcontrol(command_list)