#!/usr/bin/env python
# This wrapper formulates the IK problem as reaching a point by a fingertip, with constraints on the kinematics of the finger.

from math import sqrt,fabs
import numpy as np
import pyOpt
import copy

class OptimalController_collision:
    def __init__(self,robot,cost_fns,constraint_fns,bounds,opt_method,opt_options={}):
        self.robot=robot    
        self.cost_fns=cost_fns
        self.constraint_fns=constraint_fns
        
        self.method=opt_method
        self.opt_options=opt_options
        self.bounds=bounds
        
    def regenerate_control(self,x_n,x0,x_d,u):
        self.x0=x0
        self.x_d=x_d
        m=self.robot.m
        T=self.robot.T
        opt_low_bounds=[]
        opt_up_bounds=[]

        # Adding bounds to variables
        q0=self.cost_fns.get_q(x_n,u)
        ## Perform optimization 
        # Create optimization problem
        opt_prob=pyOpt.Optimization('TP37 Constrained Problem',self.objfunc)
        opt_prob.addObj('f')


        # Add variables
        opt_prob.addVarGroup('q',len(q0),type='c',value=q0,upper=self.bounds[1],lower=self.bounds[0])
        
        # In equality Constraints:
        temp = T
        constraint = np.zeros(temp)
        constraint[:] = -0.01
        opt_prob.addConGroup('g',temp, type='i',upper=constraint)
        # opt_prob.addConGroup('g',self.constraint_fns.length_*T,type='i',
        #                      lower=self.constraint_fns.bounds[0],upper=self.constraint_fns.bounds[1])

        #TODO: Add equality constraints
        
        method=getattr(pyOpt,self.method)
        optimizer=method(options=self.opt_options)

        [fstr,xstr,inform]=optimizer(opt_prob,sens_type=self.grad_obj_func)
        #print inform
        print 'Final cost: '+str(fstr)
        q_star=xstr
        # Get x,u from q:
        x,u=self.constraint_fns.get_x_u_from_q(q_star)
      
        return  u,x
        
    # Creating optimization function to use with pyOpt
    def objfunc(self,q):
        f=self.cost_fns.objectiveFunction(q)

        g = list(f[1])
        print('g_con', g)
        fail=0
        return f[0],g,fail

    def grad_obj_func(self,q,f,g):
        [g_obj,g_constraint] = self.cost_fns.objectiveGradient(q)
        g_obj=np.array([g_obj])
        # print('g', g_obj)
        g_constraint = np.array(g_constraint)


        temp_T = self.robot.T
        temp_con = []
        for i in range(temp_T):
            return_list = np.zeros(self.robot.T*7)
            return_list[i*7:(i+1)*7] = g_constraint[i*7:(i+1)*7]
            temp_con.append(return_list)
        temp_con = np.array(temp_con)
        # print('temp_con', temp_con)
        fail=0

        return g_obj,temp_con,fail
        
    def learn_controller(self,x0,x_desired,u,dyn_model):

        T=self.robot.T
        t=self.robot.delta_t

        x_n=dyn_model.predict_T(x0,u,T)

        #print x_n
        x_d=x_desired
        u,x=self.regenerate_control(x_n,x0,x_d,u)


        #x=[]
        return u,x
