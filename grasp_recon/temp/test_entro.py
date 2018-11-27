import GPflow
import numpy as np
from matplotlib import pyplot as plt
#plt.style.use('ggplot')
#matplotlib inline
N = 20
X = np.random.rand(N,1)
Y = np.sin(12*X) + 0.66*np.cos(25*X) + np.random.randn(N,1)*0.1 + 3
plt.plot(X, Y, 'kx', mew=2)
k = GPflow.kernels.Matern52(1, lengthscales=0.3)
meanf = GPflow.mean_functions.Linear(1,0)
m = GPflow.gpr.GPR(X, Y, k,meanf)
m.likelihood.variance = 0.01
xx = np.linspace(-0.1, 1.1, 30)[:,None]
mean, var = m.predict_y(xx)
var2=m.predict_f_full_cov(xx)
def plot(m):
    #xx = np.linspace(-0.1, 1.1, 30)[:,None]
    mean, var = m.predict_y(xx)
    mean2 = m.predict_f(xx)
    var2=m.predict_f_full_cov(xx)
    #var = m.build_predict(xx)
    #mean2 = m.predict_f(xx)
    '''
    print xx
    print mean
    print var
    print var2
    print var2[1]
    '''
    print mean,var
    print 'latent function mean and variance'
    print mean2
    return var2[1]
    #print mean2
    #print mean2[0]

    #plt.show()


var = plot(m)
temp_var = np.zeros((var.shape[0],var.shape[1]),dtype=np.float)
for i in range(var.shape[0]):
    for j in range(var.shape[1]):
        temp_var[i][j] = var[i][j][0]

xx_t=np.transpose(xx)
print xx_t*temp_var*xx
print 'determinant'
print np.linalg.det(xx_t*temp_var*xx)
'''
#print var.shape
#print var.shape[0]
#print temp_var
det_min = np.linalg.det(temp_var)
print det_min

large = np.ones((3,3),float)
large[1][2] = 19
large[2][2] = 20
print large
a = np.array([[1, 2], [3, 4]])
det_min = np.linalg.det(large)
print det_min
'''