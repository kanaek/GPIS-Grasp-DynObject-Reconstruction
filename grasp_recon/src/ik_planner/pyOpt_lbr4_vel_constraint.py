# The constraints are: FK for the fingertips and velocity limits for the joints
import numpy as np
import matplotlib.pylab as plt
class ConstraintFns:
    def __init__(self,robot_model,u0,vel_limit,DYN_constraint=False):
        self.robot=robot_model
        self.DYN=DYN_constraint
        self.u0=u0
        # Constraint length per time step:
        self.length_=robot_model.m#+1#robot_model.n
        # Bounds for constraints:
        joint_vel_ratio=np.ravel([1.0,1.0,1.0,1.0,1.0,1.0,1.0]*robot_model.T)
        lower_limit_velocity=joint_vel_ratio*(-1.0*vel_limit)
        upper_limit_velocity=joint_vel_ratio*(vel_limit)
        #fk_constraint_low=np.ones(robot_model.T*1)*0.0
        #fk_constraint_up=np.ones(robot_model.T*1)*0.00001
        self.bounds=[lower_limit_velocity,upper_limit_velocity] 
        #self.bounds=[lower_limit_velocity,upper_limit_velocity] 
        #self.fk_dimensions=[1.0,1.0,1.0,1.0,1.0,1.0]
    def constraint(self,q):
        G=list(self.velocity_constraint(q))#+list(self.fk_constraint(q))
        return G

    #no use
    def constraint_gradient(self,q):
        J=self.velocity_gradient(q)#+self.fk_gradient(q)
        #J=self.fk_gradient(q)
        '''
        fig = plt.figure()
        ax = fig.add_subplot(1,1,1)
        ax.set_aspect('equal')
        plt.imshow(J, interpolation='none', cmap=plt.cm.ocean)
        plt.xlabel('q(variables)')
        plt.ylabel('constraints')
        plt.xticks(np.arange(0,44, 2.0))
        plt.yticks(np.arange(0,len(J[:,0]), 2.0))
                
        plt.colorbar()
        plt.show()
        '''
        return J
    '''
    def fk_constraint(self,q):
        # Compute the difference between the current end-effector pose and the joint angles
        x,u=self.get_x_u_from_q(q)
        G=np.zeros(self.robot.T)
        finger=3
        for k in range(self.robot.T):
            G[k]=np.sum(((x[k]-self.robot.end_effector_pose_array(u[k+1]))*self.fk_dimensions)**2)
        #print G
        return G

    def fk_gradient(self,q):
        # Use Jacobian to compute the gradient
        x,u= self.get_x_u_from_q(q)
        J=np.zeros(((self.robot.T)*(self.length_),q.size))

        # Getting x position from q
        x_start_offset=self.robot.m*self.robot.T

        # Compute the offset to the fk constraints from the list of constraints:
        # TODO: compute at initialization
        fk_constraint_start=self.robot.m*self.robot.T
        fk_constraint_length=1
    
        
        for i in range(self.robot.T):

            # Computing gradient for timestep i:
            const_=2.0*(x[i]-self.robot.end_effector_pose_array(u[i+1]))*self.fk_dimensions
            # Differentiating constraint w.r.t x which is 6 dimensional pose
            j_dx=const_

            # Differntiating constraint w.r.t u (FK becomes the velocity Jacobian):
            j_du=np.array(-1.0*np.matrix(const_)*self.robot.jacobian_full(u[i+1])).ravel()
            
            # Adding to the gradient matrix:
            u_start=i*self.robot.m
            u_stop=u_start+self.robot.m
            x_start=i*self.robot.n+x_start_offset
            x_stop=x_start+self.robot.n


            J[fk_constraint_start+i*fk_constraint_length,u_start:u_stop]+=j_du#*1000.0
            #print j_dx
            J[fk_constraint_start+i*fk_constraint_length,x_start:x_stop]+=j_dx#*1000.0


        return J    
    '''
        
    def velocity_constraint(self,q):
        x,u= self.get_x_u_from_q(q)
        G=np.zeros(self.robot.m*self.robot.T)
        for k in xrange(self.robot.T):
            G[k*self.robot.m:(k+1)*self.robot.m]=np.ravel(u[k+1]-u[k])#/self.delta_t
        
        return G

    def velocity_gradient(self,q):
        x,u= self.get_x_u_from_q(q)
        J=np.zeros(((self.robot.T)*(self.length_),q.size))
        m=self.robot.m
        velocity_grad=np.zeros((m,m*2))

        for k in range(m):        
            velocity_grad[k,k]=-1.0#/(self.delta_t)
            velocity_grad[k,k+m]=1.0#/(self.delta_t)

        J[0:m,0:m]=np.eye(m)*(1.0)
        for k in range(1,self.robot.T):
            row_start = (k)*m
            row_stop = row_start+m

            col_start =(m)*(k-1)
            col_stop = col_start+2*m
            # Each Jacobian will have gradients of the dynamics model
            J[row_start:row_stop,col_start:col_stop]+=velocity_grad
           
        return J    

    def get_x_u_from_q(self,q):
        m=self.robot.m
        n=self.robot.n
        u=np.array([[0.0 for k in range(m)] for i in range(self.robot.T+1)])
        x=np.array([[] for i in range(self.robot.T)])
        u[0,:]=self.u0

        for k in range(self.robot.T):
            u_start=k*m
            u_stop=u_start+m
            
            u[k+1,:]=q[u_start:u_stop]
            
        return x,u        

    def get_q(self,x,u):
        # q=[u[1],u[2],.. u[T], x[0],x[1]..x[T]]
        q=list(u[1:].ravel())#+list(x.ravel())
        return q
