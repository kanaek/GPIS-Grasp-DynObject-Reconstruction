#!/usr/bin/env python
# This function wraps the cost function with pyOpt compatible objective function
import numpy as np
class CostFns_info:
    def __init__(self,immediate_cost,final_cost,grad,grad_final,robot_model,x_d,u0):
        self.eps=1e-15
        self.delta_t=robot_model.delta_t
        self.cost=immediate_cost
        self.final_cost=final_cost
        self.gradient_u=grad
        self.gradient_u_final=grad_final
        
        self.robot=robot_model
        self.x_d=x_d
        self.u0=u0
        self.n=len(x_d[0])
        self.m=robot_model.m
       
    def multi_cost(self,data):
        return self.cost(data[0:self.n],data[self.n:self.n+self.m],self.robot)

    def test_cost(self,data):
        return data

    def objectiveFunction(self,q):
        '''
        q - decision variable vector of the form (U[0], X[1], U[1],..., U[H-1], X[H])
        H - time horizon
        n - system state dimension
        m - control dimension
        x0 - inital state
        x_d - desired trajectory
        dyn_model - model simulating system dynamics of the form x[k+1] = dyn_model.predict(x[k], u[k])
        '''

        x,u=self.get_x_u_from_q(q)

        cost=0.0
        x_d=self.x_d
        u_arr=u[1:-1]

        x_d_arr=x_d[:-1]
        t=np.array([range(self.robot.T-1)]).T

        if(self.robot.T>1):
            data_arr=np.concatenate((x_d_arr,u_arr),axis=1)

            data_arr=list(np.concatenate((data_arr,t),axis=1))
            self.multi_cost(data_arr[0])

            #cost_arr=map(self.multi_cost,data_arr)
            cost_arr=[self.multi_cost(k) for k in data_arr]

            # cost for shortest joint trajectory:
            cost=sum(cost_arr)
            #print cost
            cost+=self.final_cost(x_d[-1],u[-1],self.robot)
        else:
            # This case works when you want to optimize for only one time-step
            cost+=self.final_cost(x_d,u[-1],self.robot)
            #print cost

        #print cost
        return cost


    def multi_gradient(self,data):
        return self.gradient_u(data[0:self.n],data[self.n:self.n+self.m],self.robot)
        #return self.gradient_u(data[0:self.n],data[self.n:self.n+self.m],self.robot,data[2self.n+self.m:])
        
    def objectiveGradient(self,q):
        ## TODO: Make this generic for any robot model 
        #  print 'Computing Objective Gradient'
        
        #m=self.robot.m
        x, u = self.get_x_u_from_q(q)
        
        
        #k=len(q)-2*step
        x_d=self.x_d
        
        
        u_arr=u[1:-1]

        x_d_arr=x_d[:-1]
        t=np.array([range(self.robot.T-1)]).T
        # Compute gradient for inputs:
        if(self.robot.T>1):
            data_arr=np.concatenate((x_d_arr,u_arr),axis=1)
            data_arr=list(np.concatenate((data_arr,t),axis=1))
            #sc_pool=TPool()       
            gradient_arr=[self.multi_gradient(data) for data in data_arr]
            #gradient_ar=rp[self.multi_gradient(k) for k in data_arr]
            gradient=np.ravel(gradient_arr)
            gradient=np.append(gradient,self.gradient_u_final(x_d[-1],u[-1],self.robot))
        else:
            gradient=self.gradient_final(x_d[-1],u[-1],self.robot)

        # Compute gradient for states:
        #gradient_x=np.ravel([np.zeros(self.robot.n) for i in range(self.robot.T)])
        gradient=list(gradient)#+list(gradient_x)
        return gradient
    
    def get_x_u_from_q(self,q):
        m=self.robot.m
        n=self.robot.n
        u=np.array([[0.0 for k in range(m)] for i in range(self.robot.T+1)])
        x=np.array([[] for i in range(self.robot.T)])
        x_start_offset=m*self.robot.T
        u[0,:]=self.u0

        for k in range(self.robot.T):
            u_start=k*m
            u_stop=u_start+m
            x_start=k*n+x_start_offset
            x_stop=x_start+n
            u[k+1,:]=q[u_start:u_stop]
            #x[k,:]=q[x_start:x_stop]
        
        return x,u        

    def get_q(self,x,u):
        # q=[u[1],u[2],.. u[T], x[0],x[1]..x[T]]
        q=list(u[1:].ravel())#+list(x.ravel())
        return q
