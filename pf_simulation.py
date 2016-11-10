# A simple particle filter from Sebastian thrun's Mobile Robot MOOC
# Robot has x,y, theta location
# Sensing is distance of robot from 4 landmarks (with sensor noise)
# movement is arbitrary with sensor noise


from math import *
import random
import math
import turtle as tur

'''
    Assumption in order to make this PF work:
    1. Suppose Landmarks are sets of [X,Y]
        with obstacles centered at [X,Y]
        Therefore, each landmark tuple denotes a different obstacle
    2. Orientation indicates servo(90)
    3. Every Instance of GoPiGo is always instantiated with servo fixed at servo(90)
'''
landmarks = [[50.0, 25.0], [120.0, 25.0], [140.0, 100.0], [160.0, 60.0]]
world_size = [200.0, 150.0]  # [x,y]


class robot:
    '''
        Custome Structure
          Bot{
              X,
              Y,
              theta (orientation),
              noise_set{
                  forward
                  turn
                  sense
                  ?servo noise -- this is part of the GoPiGo hardward limitation, yet adding this noise could greatly undermine the effort to locate the robot [test later]
              }
          }
    '''

    def __init__(self):
        self.x = random.random() * world_size[0]
        self.y = random.random() * world_size[1]
        self.orientation = random.random() * 2.0 * math.pi
        self.forward_noise = 0.0
        self.turn_noise = 0.0
        self.sense_noise = 0.0
        self.collision_dist = 10
        self.forward_cnt = 10
        # self.servo_noise = 0.0
    '''
        Place the GoPiGo particles in simulation
    '''

    def set(self, new_x, new_y, new_orientation):
        if new_x < 0 or new_x >= world_size[0]:
            raise ValueError, 'X coordinate out of bound'
        if new_y < 0 or new_y >= world_size[1]:
            raise ValueError, 'Y coordinate out of bound'
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError, 'Orientation must be in [0..2pi]'
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)

    def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.forward_noise = float(new_f_noise)
        self.turn_noise = float(new_t_noise)
        self.sense_noise = float(new_s_noise)
    '''
        Overwrite this part to replicate the ultrasound sensor
        Method:
            Suppose we have cart at orientation theta. We will base all our calculations on the preimses that
            the world is orthogonal and (0,0) is the (0,0).

            The orientation, in the meantime, is relative the world Y-axis.

            for each given robot particle p[i], we will have p[i].X p[i].Y p[i].orientation
            In the meantime, we decided to sample the environment every 20 degrees.

            First we can tell the function f(i) at orientation is : y = tan(orientation) x - tan(orientation) * p[i].X + p[i].Y

            Then, we can get that 20 degree equals to 20 * pi / 180 = pi / 9

            thus sample degree 10 30 50 70 90 110 130 150 170
            (we don't sample degree 0 and 180 as the servo jitters a lot at these location)

            In actual robot, the commands would be:
            	myscan = []
            	servo(10)
                for i in range (0,9):
                	servo(10+20*i)
                    myscan.append(us_dist(15))
                return myscan

            Then in particles, we need to calculate the angles of turning. Which can be deduced simply based on orientation

            orientation - pi / 9 * i

            The reason we use this here is that,
            	if orientation < reducted angle, then y = tan(orientation - pi/9*j) (X - p[i].X) + p[i].Y
            	if orientation >= reducted angle, then y = tan(orientation - pi/9*j) (X - p[i].X) + p[i].Y

            In particles, the commands would be:
            	servo_10_ori = orientation - pi * 4 / 9
                Z = []
                for i in range (0,9):
                	dist = get_intersect(servo_10_ori + pi * i / 9, p[i].X, p[i].Y)
                    dist += random.gauss(0.0, self.sense_noise)
            		Z.append(dist)
                return Z

            In the get_intersect we project lines from given location and orientation and
            intersect with the world boundaries as well as the objects (landmarks) i:
            (landmark[i][0]-5.7,landmark[i][1]-5.7)
            (landmark[i][0]-5.7,landmark[i][1]+5.7)
            (landmark[i][0]+5.7,landmark[i][1]+5.7)
            (landmark[i][0]+5.7,landmark[i][1]-5.7)

            Thus we can translate the objects (landmarks) i into 4 lines:
            X = landmark[i][0]-5.7
            X = landmark[i][0]+5.7
            Y = landmark[i][1]-5.7
            Y = landmark[i][1]+5.7

            Since the ultrasonic sensor, suppose it works fine and yields no weird readings

            In particles the program would be:
            	for i in range(0,landmakrs[i]):
                	intersect with 4 lines
                    find the min distance
                    if distance < min
                    	min = distance
              	intersect with world boundaries
                	if distance < min
                    	min = distance
                return distance

            check_valid calculates the distance if the given intersect is valid
            otherwise it returns -1 as a sign of failure
    '''

    def check_domain(self,ix,iy,rad):
        if rad > 2*math.pi:
            rad -= 2*math.pi
        elif rad < 0:
            rad += 2*math.pi
        if rad >= 0 and rad < math.pi / 2:
            if ix-self.x >= 0 and iy-self.y >= 0:
                return 1
        if rad >= math.pi / 2 and rad < math.pi:
            if ix-self.x < 0 and iy-self.y > 0:
                return 1
        if rad >= math.pi and rad < math.pi * 3 / 2:
            if ix-self.x < 0 and iy-self.y < 0:
                return 1
        if rad >= math.pi * 3 / 2 and rad < math.pi * 2:
            if ix-self.x > 0 and iy-self.y < 0:
                return 1
        return -1

    def check_valid(self, lid, ix, iy, id,rad):
        if id == 1:
            if iy < landmarks[lid][1] - 5.7 or iy > landmarks[lid][1] + 5.7:
                return -1
        elif id == 0:
            if ix < landmarks[lid][0] - 5.7 or ix > landmarks[lid][0] + 5.7:
                return -1
        elif id == 2:
            if ix < 0 or ix >= world_size[0]:
                return -1
        elif id == 3:
            if iy < 0 or iy >= world_size[1]:
                return -1
        if self.check_domain(ix,iy,rad) == -1:
            return -1
        distance = sqrt(pow(self.x - ix, 2) + pow(self.y - iy, 2))
        return distance

    def get_intersects(self,k,rad):
        min_distance = 10000
        tmp_dist = 0
        for i in range(0, len(landmarks)):
            # line: y = k * (x - self.x) + self.y
            y = k * (landmarks[i][0] - 5.7 - self.x) + self.y
            tmp_dist = self.check_valid(i, landmarks[i][0] - 5.7, y, 1,rad)
            if tmp_dist < min_distance and tmp_dist != -1:
                min_distance = tmp_dist
            y = k * (landmarks[i][0] + 5.7 - self.x) + self.y
            tmp_dist = self.check_valid(i, landmarks[i][0] - 5.7, y, 1,rad)
            if tmp_dist < min_distance and tmp_dist != -1:
                min_distance = tmp_dist
            x = (landmarks[i][1] - 5.7 - self.y) / k + self.x
            tmp_dist = self.check_valid(i, x, landmarks[i][1] - 5.7, 0,rad)
            if tmp_dist < min_distance and tmp_dist != -1:
                min_distance = tmp_dist
            x = (landmarks[i][1] + 5.7 - self.y) / k + self.x
            tmp_dist = self.check_valid(i, x, landmarks[i][1] + 5.7, 0,rad)
            if tmp_dist < min_distance and tmp_dist != -1:
                min_distance = tmp_dist

        y = k * (0 - self.x) + self.y
        tmp_dist = self.check_valid(i, 0, y, 3,rad)
        if tmp_dist < min_distance and tmp_dist != -1:
            min_distance = tmp_dist
        y = k * (world_size[0] - self.x) + self.y
        tmp_dist = self.check_valid(i, world_size[0], y, 3,rad)
        if tmp_dist < min_distance and tmp_dist != -1:
            min_distance = tmp_dist
        x = (0 - self.y) / k + self.x
        tmp_dist = self.check_valid(i, x, 0, 2,rad)
        if tmp_dist < min_distance and tmp_dist != -1:
            min_distance = tmp_dist
        x = (world_size[1] - self.y) / k + self.x
        tmp_dist = self.check_valid(i, x, world_size[1], 2,rad)
        if tmp_dist < min_distance and tmp_dist != -1:
            min_distance = tmp_dist

        return min_distance

    def sense(self):
        servo_10_ori = self.orientation - pi * 4 / 9
        Z = []
        for i in range(0, 9):
            rad = servo_10_ori + math.pi * i / 9
            k = tan(rad)
            #print rad
            dist = self.get_intersects(k,rad)
            #print dist
            dist += random.gauss(0.0, self.sense_noise)
            Z.append(dist)
        return Z
    '''
        Overwrite this function

        check_obstacle is called to detect obstacles infront of the robot
        it takes in sensor readings and kept turning until there are no obstacle in front.
        It then returns the furn and forward specs to instruct the particles to move.
        if the Z[4] aka servo(90) has no obstacle in front, the turn would be 0
    '''

    def check_obstacle(self,oz):
        # check whether there is obstacle in the movement window
        turn = 0
        res = []
        while oz[4] <= self.collision_dist + sqrt(2 * 5.7 * 5.7):
            #print 'spin..' + str(oz[4])
            turn = random.random() * 2 * pi
            orientation = self.orientation + float(turn) + random.gauss(0.0, self.turn_noise)
            orientation %= 2 * pi
            self.orientation = orientation
            oz = self.sense()
        res.append(turn)
        res.append(self.forward_cnt)
        return res

    def check_bump(self,weight):

        for i in range(0,len(landmarks)):
            if landmarks[i][0] - 5.7 <= self.x and self.x <= landmarks[i][0] + 5.7:
                if landmarks[i][1] - 5.7 <= self.y and landmarks[i][1] + 5.7 >= self.y:
                    return 0

        return weight

    def move(self, turn, forward):
        if forward < 0:
            raise ValueError, 'Robot cant move backwards'
        # turn, and add randomness to the turning command
        self.orientation = self.orientation + float(turn) + random.gauss(0.0, self.turn_noise)
        self.orientation %= 2 * pi

        '''
            original randome turn at each move is deprecated due to optometry errors
        '''

        # move, and add randomness to the motion command
        dist = float(forward) + random.gauss(0.0, self.forward_noise)
        x = self.x + (cos(self.orientation) * dist)
        y = self.y + (sin(self.orientation) * dist)
        x %= world_size[0]  # cyclic truncate
        y %= world_size[1]
        # set particle
        res = robot()
        res.set(x, y, self.orientation)
        res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)
        return res
    '''
        calculates probability x (baseline measure) under the distribution of mu (particle measure)
    '''

    def Gaussian(self, mu, sigma, x):
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))
    '''
        measurement prob takes in sensor baseline Z (which is the measure in the original code)
        as well as the particle's sensor scan particle_measure (which is calculated based on self.x and self.y in original code)
        it then passes these arguments to Gaussian and calculate the probability of such measurement.
    '''

    def measurement_prob(self, pmeasure, Z):
        # calculates how likely a measurement should be
        prob = 1.0

        for i in range(0, 9):
            #dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            prob *= self.Gaussian(pmeasure[i], self.sense_noise, Z[i])
            #print prob
        return prob

    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))


def eval(r, p):
    sum = 0.0
    for i in range(len(p)):  # calculate mean error
        dx = (p[i].x - r.x + (world_size[0] / 2.0)) % world_size[0] - (world_size[0] / 2.0)
        dy = (p[i].y - r.y + (world_size[1] / 2.0)) % world_size[1] - (world_size[1] / 2.0)
        err = sqrt(dx * dx + dy * dy)
        sum += err
    return sum / float(len(p))

def drawbot(ori,x,y,scale):
    tur.color('red')
    tur.shape("turtle")
    tur.penup()
    tur.setpos(-scale/world_size[0]+x*scale, -scale/world_size[1]+y*scale)
    tur.seth(ori * 180 / math.pi)
    tur.stamp()

def drawpart(ori,x,y,weight,scale):
    if weight > 0.001:
        tur.color('blue')
    else:
        tur.color('#00FFFF')
    tur.penup()
    tur.setposition(-scale/world_size[0]+x*scale, -scale/world_size[1]+y*scale)
    tur.seth(ori * 180 / math.pi)
    tur.stamp()

def draw_rec(dis,scale):
    tur.forward(dis * scale)
    tur.left(90)
    tur.forward(dis * scale)
    tur.left(90)
    tur.forward(dis * scale)
    tur.left(90)
    tur.forward(dis * scale)
    tur.left(90)

def world(scale):
    tur.penup()
    tur.setposition(-scale/world_size[0], -scale/world_size[1])
    tur.pendown()
    tur.forward(world_size[0] * scale)
    tur.left(90)
    tur.forward(world_size[1] * scale)
    tur.left(90)
    tur.forward(world_size[0] * scale)
    tur.left(90)
    tur.forward(world_size[1] * scale)

    for i in range(len(landmarks)):
        tur.penup()
        tur.setposition(-scale/world_size[0]+(landmarks[i][0]-5.7)*scale,-scale/world_size[0]+(landmarks[i][1]-5.7)*scale)
        tur.pendown()
        draw_rec(11.4,scale)
# --------

N = 2000
T = 10

tur.mode("standard")
tur.color('red')
tur.speed(0)

scale = 3

world(scale)

my_robot = robot()

p = []
for i in range(N):
    r = robot()
    r.set_noise(1, 1, 3.0)  # provided noise.
    if i % 10 == 0:
        drawpart(r.orientation,r.x,r.y,1,scale)
    p.append(r)
drawbot(my_robot.orientation,my_robot.x,my_robot.y,scale)
print my_robot.orientation
#(my_robot.x,my_robot.y)

print 'Mean error at start', eval(my_robot, p)
# show particle's initial locations
print p
for t in range(T):
    #    print p
    '''
        for each iteration - i.e. each time the robot moves
        we need to:

        1. take a initial sensor measurement
        2. Check if there are obstacles in front of the robot
            [checkObstacles takes in the initial measurement and keeps spinning until there are no obstacles in front]
            [it then returns the (turn) and (forward) to instruct the particles to turn]
        3. The robot then do another sensor measurement and use it as the baseline to for all particles
    '''
    init_sense = my_robot.sense()
    print init_sense
    print my_robot
    movement_spec = my_robot.check_obstacle(init_sense)

    #Z = my_robot.sense()
    print movement_spec[0]
    my_robot = my_robot.move(movement_spec[0], movement_spec[1])
    Z = my_robot.sense()

    p2 = []

    for i in range(N):
        p2.append(p[i].move(movement_spec[0], movement_spec[1]))
    p = p2
    pscnt = 0
    w = []
    for i in range(N):
        partical_measure = p[i].sense()
        if pscnt < 10 and i % 100 == 0:
            print partical_measure
            pscnt = pscnt + 1
        weight = p[i].measurement_prob(partical_measure, Z)
        if p[i].check_bump(weight) == -1:
            weight = 0
        #print weight
        w.append(weight)
    p3 = []

    # this is importance sampling code
    tur.clearstamps()
    #world()
    index = int(random.random() * N)
    beta = 0.0
    mw = max(w)
    for i in range(N):
        beta += random.random() * 3 * mw
        while beta > w[index]:
            beta -= w[index]
            index = (index + 1) % N
        p3.append(p[index])
        #print str(p[index].x) + " " + str(p[index].y) + " " + str(w[index])
    p = p3
    drawbot(my_robot.orientation,my_robot.x, my_robot.y, scale)
    print my_robot
    tur.shape('arrow')
    cnt = 0
    for i in range(0,N):
        if i % 20 == 0:
            if cnt == 0:
                print p[i]
                cnt += 1
            drawpart(p[i].orientation,p[i].x, p[i].y, w[i],scale)
    #drawbot(my_robot.x, my_robot.y)
    print 'Mean error', eval(my_robot, p)
    print 'done'

print ' '
if eval(my_robot, p) > 0.0:
    for i in range(N / 100):
        print 'Final particle #', i * 100, p[i * 100]
        drawpart(p[i].orientation,p[i].x, p[i].y, w[i],scale)
    drawbot(my_robot.orientation,my_robot.x,my_robot.y,scale)
    print ' '
    print 'Actual Robot Location', my_robot