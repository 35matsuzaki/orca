#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os
import sys
import seaborn as sns
from matplotlib.animation import FuncAnimation
import math

from robots import *

sns.set_style("whitegrid")

class ClearPath():
    def __init__(self,):
        self.fig= plt.figure()
        self.ax_xy = self.fig.add_subplot(121)
        self.ax_vxvy = self.fig.add_subplot(122)
        self.area = [-3, 3]
        self.robot1 = SimpleBot(id = 0, odom = np.array((-2,0, 0.0, 0.0)))
        self.robot2 = SimpleBot(id = 1, odom = np.array((1.7,-1.2, 0.0, 0.0)), color="blue")
        self.robot3 = SimpleBot(id = 2, odom = np.array((1.1,1.2, 0.0, 0.0)), color="green")
        self.robot4 = SimpleBot(id = 3, odom = np.array((1.3,0, 0.0, 0.0)), color="yellow")
        #self.robot4 = SimpleBot(id = 3, odom = np.array((1.0,3.0, 0.0, 0.0)), color="yellow")
        self.robots = [self.robot1, self.robot2, self.robot3, self.robot4] # index must be the same as robot*.id
        self.goals = np.array(((2,0), (-2, -1.2), (-2, 1.2), (-2, 0)))
        self.norm = 10.0
        self.tau = 3.0
        self.reciprocal_param = 1.0
        self.vel_pref_list = np.empty((len(self.robots),2))
        self.vel_resolution = 0.05

    def print_robot(self):
        for robot in self.robots:
            print robot.color
    def calc_tangent_point(self, radius, robot, oth_robot, point = np.zeros(2), tau = 1.0):
        point = point - (oth_robot.odom[0:2] - robot.odom[0:2])/tau #move center of robot
        radius = radius/tau
        x_a = (2.0*point[0]*radius**2 + np.sqrt(4.0*point[0]**2*radius**4-4.0*(radius**4-point[1]**2*radius**2)*(point[0]**2+point[1]**2)))/(2.0*(point[0]**2+point[1]**2))
        x_b = (2.0*point[0]*radius**2 - np.sqrt(4.0*point[0]**2*radius**4-4.0*(radius**4-point[1]**2*radius**2)*(point[0]**2+point[1]**2)))/(2.0*(point[0]**2+point[1]**2))

        y_a = (radius**2-point[0]*x_a)/point[1]
        y_b = (radius**2-point[0]*x_b)/point[1]


        return np.array(([x_a, y_a], [x_b, y_b])) + (oth_robot.odom[0:2] - robot.odom[0:2])/tau

    def calc_inter_section(self, vo):
        inter_sections = np.empty(vo.shape[0]*(vo.shape[0]-1)/2)
        for i in range(vo.shape[0]):
            vertices = vo[i]
            #line 1
            a1 = (vertices[0][1] - vertices[1][1])/(vertices[0][0] - vertices[1][0])
            b1 = -1.0 * vertices[1][0] * a1 + vertices[1][1]
            #line 2
            a2 = (vertices[0][1] - vertices[2][1])/(vertices[0][0] - vertices[2][0])
            b2 = -1.0 * vertices[2][0] * a1 + vertices[2][1]
            for j in range(vo.shape[0]):
                if j <= i:
                    continue # except for the duplication
                oth_vertices = vo[i]
                #line 1
                oth_a1 = (oth_vertices[0][1] - oth_vertices[1][1])/(oth_vertices[0][0] - oth_vertices[1][0])
                oth_b1 = -1.0 * oth_vertices[1][0] * a1 + oth_vertices[1][1]
                #line 2
                oth_a2 = (oth_vertices[0][1] - oth_vertices[2][1])/(oth_vertices[0][0] - oth_vertices[2][0])
                oth_b2 = -1.0 * oth_vertices[2][0] * a1 + oth_vertices[2][1]

    def calc_vel_pref(self, plot=True):
        self.vel_pref_list = np.empty((len(self.robots),2))

        for indx in range(len(self.robots)):
            robot = self.robots[indx]
            goal = self.goals[indx]

            p = goal - robot.odom[0:2]
            direction = math.atan2(p[1], p[0]) #atan2(y,x)
            vel_pref = np.array((robot.max_vel*math.cos(direction), robot.max_vel*math.sin(direction)))

            self.vel_pref_list[indx] = vel_pref

    def calc_linear_function(self, point1, point2):
        # y = ax + b -> return a, b
        a = (point1[1] - point2[1])/(point1[0] - point2[0])
        b = point2[1] - point2[0]*a
        return [a, b]
    def where_relative_vel(self, relative_vel, tangent_points, tau_tangent_points, tau_circle):
        # return: info for where relative_vel is
        # tau_circle = {'origin': (x,y), 'radius' : radius}
        is_inside_tau_circle = False
        origin = tau_circle['origin']
        radius = tau_circle['radius']
        if (origin[0] - relative_vel[0])**2 + (origin[1] - relative_vel[1])**2 < radius**2:
            is_inside_tau_circle = True


        is_inside_tanget_point_lines = False
        line1 = self.calc_linear_function([0.0, 0.0], tangent_points[0])
        line2 = self.calc_linear_function([0.0, 0.0], tangent_points[1])

        is_relative_vel_upper_line1 = relative_vel[1] > line1[0]*relative_vel[0] + line1[1]
        is_circle_origin_upper_line1 = origin[1] > line1[0]*origin[0] + line1[1]
        is_relative_vel_upper_line2 = relative_vel[1] > line2[0]*relative_vel[0] + line2[1]
        is_circle_origin_upper_line2 = origin[1] > line2[0]*origin[0] + line2[1]

        if (is_relative_vel_upper_line1 == is_circle_origin_upper_line1) and (is_relative_vel_upper_line2 == is_circle_origin_upper_line2):
            is_inside_tanget_point_lines = True


        is_inside_tau_tanget_point_lines = False
        tau_line1 = self.calc_linear_function(origin, tau_tangent_points[0])
        tau_line2 = self.calc_linear_function(origin, tau_tangent_points[1])
        is_relative_vel_upper_tau_line1 = relative_vel[1] > tau_line1[0]*relative_vel[0] + tau_line1[1]
        is_origin_upper_tau_line1 = 0.0 > tau_line1[0]*0.0 + tau_line1[1]
        is_relative_vel_upper_tau_line2 = relative_vel[1] > tau_line2[0]*relative_vel[0] + tau_line2[1]
        is_origin_upper_tau_line2 = 0.0 > tau_line2[0]*0.0 + tau_line2[1]
        if (is_relative_vel_upper_tau_line1 == is_origin_upper_tau_line1) and (is_relative_vel_upper_tau_line2 == is_origin_upper_tau_line2):
            is_inside_tau_tanget_point_lines = True

        return is_inside_tanget_point_lines, is_inside_tau_tanget_point_lines, is_inside_tau_circle


    def calc_u_vec(self, robot, oth_robot, plot_u_vec=True):
        #point1 : v^opt_A - v^opt_B = v^pref_A - v^current_B
        #point2 : tau_circle_origin
        #point3 : tangent_point
        robot_vel_opt = self.vel_pref_list[robot.id] #v^opt_A := v^pref_A
        #oth_robot_vel_opt = self.vel_pref_list[oth_robot.id] #v^opt_B :=v^pref_B
        oth_robot_vel_opt = oth_robot.odom[2:4] #v^opt_B :=v^current_B

        tangent_points = self.calc_tangent_point(robot.radius+oth_robot.radius, robot, oth_robot)
        tau_tangent_points = tangent_points/self.tau
        tau_circle = {'origin': (oth_robot.odom[0:2] - robot.odom[0:2])/self.tau, 'radius':(robot.radius + oth_robot.radius)/self.tau}

        point1 = robot_vel_opt - oth_robot_vel_opt

        is_inside_tanget_point_lines, is_inside_tau_tanget_point_lines, is_inside_tau_circle = self. where_relative_vel(point1, tangent_points, tau_tangent_points, tau_circle)
        vertices = (np.zeros(2),
        self.norm*tangent_points[0],
        self.norm*tangent_points[1])


        candidate = np.empty((3,2))
        point2 = (oth_robot.odom[0:2] - robot.odom[0:2])/self.tau
        tau_radius = (robot.radius + oth_robot.radius)/self.tau

        p2p1 = point1 - point2
        theta = math.atan2(p2p1[1], p2p1[0]) #atan2(y,x)
        norm_p2p1 = np.sum(p2p1*p2p1)

        for i, point3 in enumerate(tangent_points):
            a = point3[1]/point3[0]
            candidate[i] = np.array(((a*point1[1]+point1[0])/(a**2+1), a*(a*point1[1]+point1[0])/(a**2+1)))

        if tau_radius < norm_p2p1 : # if point2 is out of circle
            candidate[2] = np.array((100.0, 100.0))
        else:
            candidate[2] = point2 + np.array((tau_radius*math.cos(theta), tau_radius*math.sin(theta)))


        self.ax_vxvy.add_patch(plt.Polygon(vertices, facecolor=oth_robot.color, alpha=.6))
        point = oth_robot.odom[0:2] - robot.odom[0:2]
        self.ax_vxvy.scatter(point[0], point[1],color=oth_robot.color)
        self.ax_vxvy.text(point[0],point[1], "p%s - p%s" %(oth_robot.id, robot.id))
        self.ax_vxvy.scatter(point[0]/self.tau, point[1]/self.tau,color=oth_robot.color)
        self.ax_vxvy.text(point[0]/self.tau, point[1]/self.tau, "(p%s - p%s)/tau" %(oth_robot.id, robot.id))
        self.ax_vxvy.add_patch(plt.Circle(point/self.tau, (robot.radius + oth_robot.radius)/self.tau, facecolor=oth_robot.color, alpha=.3, linestyle = "--"))
        self.ax_vxvy.add_patch(plt.Circle(point, robot.radius + oth_robot.radius, facecolor=oth_robot.color, alpha=.3, linestyle = "--"))
        self.ax_vxvy.scatter(*zip(*tangent_points), marker = "o", color = oth_robot.color)
        self.ax_vxvy.scatter(*zip(*tau_tangent_points), marker = "o", color = oth_robot.color)

        u_point = np.zeros(2)
        if not is_inside_tanget_point_lines or (is_inside_tau_tanget_point_lines and not is_inside_tau_circle):
            #self.ax_vxvy.scatter(point1[0], point1[1], s = 500, color=oth_robot.color, marker="^")
            u_point = point1
        elif (is_inside_tau_circle and is_inside_tau_tanget_point_lines):
            #self.ax_vxvy.scatter(point1[0], point1[1], s = 500, color=oth_robot.color, marker="*")
            u_point = candidate[2]
        else: #elif is_inside_tanget_point_lines:
            #self.ax_vxvy.scatter(point1[0], point1[1], s = 500, color=oth_robot.color, marker="p")
            norm_p3p1_0 = np.linalg.norm(point1-candidate[0])
            norm_p3p1_1 = np.linalg.norm(point1-candidate[1])
            if norm_p3p1_0 < norm_p3p1_1:
                u_point = candidate[0]
            else:
                u_point = candidate[1]

        self.ax_vxvy.text(point1[0], point1[1], "v%s - v%s" %(oth_robot.id, robot.id))
        self.ax_vxvy.plot((point1[0],candidate[0][0]), (point1[1],candidate[0][1]), marker ="o", color ="red")
        self.ax_vxvy.plot((point1[0],candidate[1][0]), (point1[1],candidate[1][1]), marker ="o", color ="red")
        self.ax_vxvy.plot((point1[0],candidate[2][0]), (point1[1],candidate[2][1]), marker ="o", color ="blue")

        self.ax_vxvy.text(u_point[0], u_point[1], "u_vec" )
        self.ax_vxvy.plot(u_point[0], u_point[1], marker ="o", color ="yellow")

        debug_norm = np.linalg.norm(u_point - point1)
        self.ax_vxvy.add_patch(plt.Circle(point1, debug_norm, facecolor="yellow", alpha=.3))

        return u_point-point1 #return u_vec refer to 'eq5'

    def calc_orca(self, plot_orca=True):
        robot = self.robots[0]
#        for robot in self.robots:
        orca = np.empty((len(self.robots)-1,3)) # (robot_num, (a,b,flag)) #a,b := y = ax+b, flag:= lower(1) or upper(0) or nothing(-1)
        idx = 0
        for oth_robot in self.robots:
            if robot.id == oth_robot.id: #ignore myself
                continue

            robot_vel_opt = self.vel_pref_list[robot.id] #v^opt_A := v^pref_A
            self.ax_vxvy.arrow(x=0,y=0,dx=robot_vel_opt[0],dy=robot_vel_opt[1],width=0.01,head_width=0.07,head_length=0.1,length_includes_head=True,color=robot.color)

            u_vec = self.calc_u_vec(robot, oth_robot)
            if np.linalg.norm(u_vec) < 1e-4:
                orca[idx] = np.array((0.0, 0.0, -1))
                idx += 1
                continue

            n_vec = np.copy(u_vec) # HACK:

            orca_vec = robot_vel_opt + self.reciprocal_param * u_vec # reciprocal_param := 0.5 refer to eq6
            self.ax_vxvy.arrow(x=robot_vel_opt[0],y=robot_vel_opt[1],dx=self.reciprocal_param * u_vec[0]+0.000001,dy=self.reciprocal_param * u_vec[1]+0.000001,width=0.01,head_width=0.07,head_length=0.1,length_includes_head=True,color=oth_robot.color)
            #self.ax_vxvy.arrow(x=0,y=0,dx=orca_vec[0],dy=orca_vec[1],width=0.01,head_width=0.07,head_length=0.1,length_includes_head=True,color="k")

            orca_points = np.array(((self.area[0],-(self.area[0]-orca_vec[0])*n_vec[0]/n_vec[1]+orca_vec[1]),(self.area[1], -(self.area[1]-orca_vec[0])*n_vec[0]/n_vec[1]+orca_vec[1])))
            self.ax_vxvy.plot(*zip(*orca_points), color = oth_robot.color)

            orca_boundary = self.calc_linear_function(orca_points[0], orca_points[1])
            lower = robot_vel_opt[1] > orca_boundary[0] * robot_vel_opt[0] + orca_boundary[1]
            self.ax_vxvy.text(0.1, 0.9-oth_robot.id*0.05, '%s lower: %s' %(oth_robot.id, lower), color = oth_robot.color, transform=self.ax_vxvy.transAxes)

            orca[idx] = np.array((orca_boundary[0], orca_boundary[1], int(lower)))
            idx +=1

        return orca

    def calc_vel_new(self, ):
        robot = self.robots[0]
        orca = self.calc_orca()
        #orca:(robot_num, (a,b,flag)) #a,b := y = ax+b, flag:= lower(1) or upper(0) or nothing(-1)
        #robot_vel_new = self.vel_pref_list[robot.id]
        robot_vel_new = np.zeros(2)
        min_norm = 100.0
        for vx in np.arange(-robot.max_vel, robot.max_vel+self.vel_resolution, self.vel_resolution):
            for vy in np.arange(-robot.max_vel, robot.max_vel+self.vel_resolution, self.vel_resolution):
                if vx**2 + vy**2 > robot.max_vel**2:
                    continue
                is_inside_orca = True
                for orca_boundary in orca:
                    if int(orca_boundary[2]) == -1:
                        continue
                    lower = int(orca_boundary[2])
                    if lower:
                        is_inside_orca = vy < orca_boundary[0] * vx + orca_boundary[1]
                    else:
                        is_inside_orca = vy > orca_boundary[0] * vx + orca_boundary[1]
                    if not is_inside_orca:
                        break
                if not is_inside_orca:
                    continue

                norm = np.linalg.norm(self.vel_pref_list[robot.id] - np.array((vx, vy)))
                if norm < min_norm:
                    min_norm = norm
                    robot_vel_new = np.array((vx,vy))
                #self.ax_vxvy.plot(vx, vy, "r.")
        self.ax_vxvy.plot(robot_vel_new[0], robot_vel_new[1], "bo")
        return robot_vel_new



    def run(self, data):
        self.ax_xy.cla()
        self.ax_vxvy.cla()
        self.ax_xy.set_aspect('equal', adjustable='box')
        self.ax_vxvy.set_aspect('equal', adjustable='box')
        self.ax_xy.set_ylim(self.area[0], self.area[1])
        self.ax_xy.set_xlim(self.area[0], self.area[1])
        self.ax_vxvy.set_ylim(self.area[0], self.area[1])
        self.ax_vxvy.set_xlim(self.area[0], self.area[1])
        #im = ax_xy.plot(rand)
        self.calc_vel_pref()
        robot_vel_new = self.calc_vel_new()
        self.robot1.move(cmd = robot_vel_new, dt=0.1)
        self.robot2.move(cmd = self.vel_pref_list[1], dt=0.1)
        self.robot3.move(cmd = self.vel_pref_list[2], dt=0.1)
        self.robot4.move(cmd = self.vel_pref_list[3], dt=0.1)
        self.robot1.plot(self.ax_xy)
        self.robot2.plot(self.ax_xy)
        self.robot3.plot(self.ax_xy)
        self.robot4.plot(self.ax_xy)


if __name__ == "__main__":
    cp = ClearPath()
    ani = animation.FuncAnimation(cp.fig, cp.run, interval=10)
    plt.show()
