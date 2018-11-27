
#import numpy as np
#import pyOpt_wrapper_manipulator as pyOpt_wrapper
#import pyOpt_wrapper_grasping as pyOpt_wrapper
import pyOpt_wrapper
import pyOpt_wrapper_collision
#import  manipulator_opt_functions as opt_fn
#import  manipulator_opt_functions_time_parsing as opt_fn
class TrajOpt:
    '''
    This class contains trajectory optimization methods, currently has ILQR,
    SNOPT,SLSQP
    Initialization:
    x0= initial state
    x_des= desired state
    u= input to the system(shooting method)
    robot= robot model from robot_models.py
    control='ILQR','SLSQP','SNOPT'
    '''
    def __init__(self,robot,cost_fns,constraint_fns,variable_bounds,opt_method,opt_options={}):
        self.robot=robot
        self.opt_options=opt_options
        self.cost_fns=cost_fns
        self.constraint_fns=constraint_fns
        self.opt_method=opt_method
        print opt_method
        self.variable_bounds=variable_bounds
    def optimizer_init(self):
        if(self.opt_method=='SLSQP'):
            optimizer=pyOpt_wrapper.OptimalController(self.robot,self.cost_fns,self.constraint_fns,self.variable_bounds,self.opt_method,self.opt_options)
            return optimizer

        elif(self.opt_method=='SNOPT'):
            optimizer = pyOpt_wrapper_collision.OptimalController_collision(self.robot, self.cost_fns, self.constraint_fns,
                                                        self.variable_bounds, self.opt_method, self.opt_options)

            return optimizer
        
    def optimize(self,optimizer,x0,x_des,u,dyn_model):
        u_opt,x_opt=optimizer.learn_controller(x0,x_des,u,dyn_model)
        return u_opt,x_opt
        
