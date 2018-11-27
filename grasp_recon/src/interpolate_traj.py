# This file reads a trajectory file and interpolates the joint angles to make the trajectory slow and fit a larger time span.
import numpy as np

def interpolate():
    file=open('traj','r')
    pos_array=np.loadtxt(file)
    t=10
    T=100
    dt=10
    new_traj=np.zeros((T,16))
    
    for i in xrange(len(pos_array)-1):       
        print i
        print str(i*dt)+":"+str(i*dt+dt)
        for k in range(len(pos_array[i])):

            new_traj[i*dt:i*dt+dt,k]=np.linspace(pos_array[i,k],pos_array[i+1,k],dt)

    f=open("interp_test","w")
    for i in range(0,T):
        for k in range(16):
            f.write(str(new_traj[i,k]))
            f.write(' ')
        f.write('\n')
    
    f.close()

def write_pre_plan(pos_array,f_name):
    #t=len(pos_array)-1
    #T=10
    #dt=T/t
    #new_traj=np.zeros((T,16))
    '''
    for i in range(len(pos_array)-1):
        for k in range(len(pos_array[i])):
            new_traj[i*dt:i*dt+dt,k]=np.linspace(pos_array[i,k],pos_array[i+1,k],dt)
    '''
    f=open(f_name,"w")
    for i in range(0,len(pos_array)):
        for k in range(16):
            f.write(str(pos_array[i,k]))
            f.write(' ')
        f.write('\n')
    
    f.close()
    
def interpolate_joints(pos_array,f_name,write_to_file=True):
    t=len(pos_array)-1
    T=100
    dt=T/t
    new_traj=np.zeros((T,16))
    for k in range(len(pos_array[0])):
        new_traj[0:dt,k]=np.linspace(pos_array[0,k],pos_array[1,k],dt)

    for i in range(1,len(pos_array)-1):
        print str(i)+' '+str(i+1)
        for k in range(len(pos_array[i])):
            new_traj[i*dt-1:i*dt+dt,k]=np.linspace(pos_array[i,k],pos_array[i+1,k],dt+1)

    if(write_to_file):
        f=open(f_name,"w")
        for i in range(0,T):
            for k in range(16):
                f.write(str(new_traj[i,k]))
                f.write(' ')
            f.write('\n')
    
        f.close()
    else:
        return new_traj

def interpolate_traj(pos_array,T):
    t=len(pos_array)-1
    dt=T/t
    new_traj=np.zeros((T,len(pos_array[0])))

    for i in range(len(pos_array)-1):
        for k in range(len(pos_array[i])):
            new_traj[i*dt:i*dt+dt,k]=np.linspace(pos_array[i,k],pos_array[i+1,k],dt)
    return new_traj

def deinterpolate_pre_plan(interpolated_plan,T,init_joints):
    pre_plan=np.zeros((11,16))
    #pre_plan[0]=init_joints
    for i in range(len(interpolated_plan)):
        if(i%10==0):
            #print i
            pre_plan[(i/10)]=interpolated_plan[i]
            #pre_plan=pre_plan[0:T]
    pre_plan[-1]=interpolated_plan[-1]
    return pre_plan
