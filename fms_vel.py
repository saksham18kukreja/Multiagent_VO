# make a class of robots, with functions such as updating their state
# state variables include: position, velocity, direction.

import numpy as np 
import matplotlib.pyplot as plt
import os

SIM_LOOP = 350

class robot:
    def __init__(self,position, velocity, direction, radius, path_length):
        self.position = position
        self.velocity = velocity
        self.direction = direction
        self.radius = radius
        self.path_length = path_length
        self.priority = True


    def update_state(self,timestep=0.01):
        self.position[0] = self.position[0] + self.velocity[0]*timestep
        self.position[1] = self.position[1] + self.velocity[1]*timestep


class fms_main:
    def __init__(self):
        #arguments: start position, goal position, starting yaw of the vehicle, safety radius, priority (highest-first)
        self.a = robot([0,10],[1,0],0.0,1.5,1)
        self.b = robot([10,20],[0,-1],-1.57,1.5,2)
        self.c = robot([10,0],[0,1],1.57,1.2,0)
        # self.d = robot([20,0],[-1,1],2.35,1.5,0)
        # self.e = robot([0,0],[1,1],0.50,1.5,5)

        self.robots = []
        self.time = 1
        self.dt = 0.2
        self.robots.append(self.a)
        self.robots.append(self.b)
        self.robots.append(self.c)
        # self.robots.append(self.d)
        # self.robots.append(self.e)
        self.vmax = 8


    # calculates euclidean distance between two robots
    def calculate_distance(self,a,b):
        return np.sqrt((a.position[0] - b.position[0])**2 + (a.position[1] - b.position[1])**2)

    # calculates manhattan distance between two robots
    def calcuate_manhattan_distance(self,a,b):
        return np.abs(a.position[0] - b.position[0]) + np.abs(a.position[1] - b.position[1])


    # calculates common elements in two lists
    def find_common_velocities(self,list_a,list_b):
        if (not list_a):
            return list_b

        else:
            list_c = []
            for a in list_a:
                for b in list_b:
                    if ((a[0] == b[0]) and (a[1] == b[1])):
                        list_c.append([a[0],a[1]])


            return list_c


    # define a priority zone based on prediction of collision or just the distance between two robots
    def calculate_priority_zone(self):
        in_priority_zone = []

        for i in range(len(self.robots)):   
            for j in range(len(self.robots)):    
                if (i==j):
                    continue
                if (self.calculate_distance(self.robots[i],self.robots[j])<5):
                    in_priority_zone.append([i,j])

        return in_priority_zone


    # calculates the priority of the vehicles at a junction using the path_length
    def calculate_priority(self,robots_in_priority_zone):

        for robo in self.robots:
                robo.priority = True


        if (robots_in_priority_zone):

            for i in range(len(robots_in_priority_zone)):
                if ((self.robots[robots_in_priority_zone[i][0]].path_length > self.robots[robots_in_priority_zone[i][1]].path_length)):
                    self.robots[robots_in_priority_zone[i][1]].priority = False
                else:
                    self.robots[robots_in_priority_zone[i][0]].priority = False
                    


    # main velocity obstacle loop
    def compute_velocity(self):
        cmd_vel_high = []
        cmd_vel_low = []
        in_priority_zone = []          
        
        for i in range(len(self.robots)):

            suitable_velocity = []
            if (i == 1):
                self.vmax = 12
            else:
                self.vmax = 10

            for j in range(len(self.robots)):
                
                if(i==j):
                    continue

                suitable_velocity_obs = []
                collision = False
                
                for v in np.arange(0.0,self.vmax,0.2):

                    for t in np.arange(0,self.time, self.dt):

                        vx = v * np.cos(self.robots[i].direction)
                        vy = v * np.sin(self.robots[i].direction)

                        new_position = [self.robots[i].position[0] + (vx - self.robots[j].velocity[0])*t, self.robots[i].position[1] + (vy - self.robots[j].velocity[1])*t]
                        new_velocity = [vx,vy]
                        new_pos = robot(new_position,new_velocity,0.45,1,5)

                        if (self.calculate_distance(new_pos,self.robots[j]) > (self.robots[i].radius+self.robots[j].radius)):
                            suitable_velocity_obs.append([vx,vy])  

                        # elif (self.calcuate_manhattan_distance(self.robots[i],self.robots[j])<10):
                        #     collision = True

                        else:
                            collision = True

                if(collision):
                    in_priority_zone.append([i,j])

                suitable_velocity = self.find_common_velocities(suitable_velocity, suitable_velocity_obs)

            if (suitable_velocity):
                cmd_vel_high.append(suitable_velocity[-1])
                cmd_vel_low.append(suitable_velocity[0])

            else:
                vx = self.vmax * np.cos(self.robots[i].direction)
                vy = self.vmax * np.sin(self.robots[i].direction)
                cmd_vel_high.append([0,0])
                cmd_vel_low.append([0,0])


        self.calculate_priority(in_priority_zone) 

        print(in_priority_zone)            
        # print("priority of robot 0 is :",self.robots[0].priority)
        # print("priority of robot 1 is :",self.robots[1].priority)
        # print("priority of robot 2 is :",self.robots[2].priority)
        return cmd_vel_low, cmd_vel_high  
        


    def main(self):
        
        for j in range(SIM_LOOP):
            
            velocities_low, velocities_high = self.compute_velocity()

            for i in range(len(self.robots)):
                if (self.robots[i].priority == True):
                    self.robots[i].velocity = velocities_high[i]

                else:
                    self.robots[i].velocity = velocities_low[i]

            vel1 = round(np.linalg.norm(self.robots[0].velocity))
            vel2 = round(np.linalg.norm(self.robots[1].velocity))
            vel3 = round(np.linalg.norm(self.robots[2].velocity))
            
            print("speed of a is ",vel1)
            print("speed of b is ",vel2)
            print("speed of c is ",vel3)
            # print("speed of d is ",self.robots[3].velocity)
            # print("speed of e is ",self.robots[4].velocity)
            print("#########################")
            

            plt.cla()

            plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])

            self.a.update_state()
            self.b.update_state()
            self.c.update_state()
            # self.d.update_state()
            # self.e.update_state()

            new_pos = robot([10,10],[0,0],0.0,0,0)
            dist = self.calculate_distance(self.robots[1],new_pos)
            # dist_2 = self.calculate_distance(self.robots[2],new_pos)
            if (dist<0.2):
                self.robots[1].direction = 0

            # if (dist_2<0.2):
            #     self.robots[2].direction = 0


            plt.plot(self.a.position[0],self.a.position[1],'o',color='red')
            plt.plot(self.b.position[0],self.b.position[1],'o',color='blue')
            plt.plot(self.c.position[0],self.c.position[1],'o',color='black')
            # plt.plot(self.d.position[0],self.d.position[1],'o',color='grey')
            # plt.plot(self.e.position[0],self.e.position[1],'o',color='green')
            
            plt.text(0.05, 0.95, f"Velocity of robot a: {vel1}\n"
                    f"Velocity of robot b: {vel2}\n"
                    f"Velocity of robot c: {vel3}", transform=plt.gca().transAxes,
                    fontsize=10, verticalalignment='top', bbox=dict(boxstyle='round', alpha=0.5))
            
            plt.xlim(-5, 25)
            plt.ylim(-5, 25)
            plt.grid(True)
            plt.pause(0.000000001)            
            # plt.close()

if __name__ == '__main__':
    fms = fms_main()
    fms.main()