#!/usr/bin/env python
'''
import rospy
from tabletop_obj_segmentation.srv import SegmentGraspObject
from pcl_mesh_tools.srv import update_env
rospy.init_node("grasp_client")
rospy.wait_for_service('object_segmenter')
obj_pose_srv=rospy.ServiceProxy('object_segmenter',SegmentGraspObject)
obj_resp=obj_pose_srv(False)
pose=obj_resp.obj.pose

rospy.wait_for_service('collision_checker/update_env_cloud')
update_env_cloud = rospy.ServiceProxy('collision_checker/update_env_cloud', update_env)
res = update_env_cloud()
'''
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from mpl_toolkits.mplot3d import Axes3D
import matplotlib as mpl
from pylab import *

fig = plt.figure(figsize=(6, 2))
ax1 = fig.add_axes([0.05, 0.80, 0.9, 0.15])
colors = [(1, 0, 0), (0.5, 0.5, 0), (0, 1, 0)]  # R -> G -> B
n_bins = 200  # Discretizes the interpolation into bins
cmap_name = 'my_list'
cmap = LinearSegmentedColormap.from_list(
        cmap_name, colors, N=n_bins)
norm = mpl.colors.Normalize(vmin=0.2, vmax=0.3)
cb1 = mpl.colorbar.ColorbarBase(ax1, cmap=cmap,
                                norm=norm,
                                orientation='horizontal')
cb1.set_label('GPIS coviance')
plt.show()
fig.savefig('foo.png')
'''
colors = [(1, 0, 0), (0.5, 0.5, 0), (0, 1, 0)]  # R -> G -> B
n_bins = 200  # Discretizes the interpolation into bins
cmap_name = 'my_list'
cmx = LinearSegmentedColormap.from_list(
        cmap_name, colors, N=n_bins)


def randrange(n, vmin, vmax):
    return (vmax-vmin)*np.random.rand(n) + vmin

fig = plt.figure(figsize=(8,6))

ax = fig.add_subplot(111,projection='3d')
n = 10

xs = randrange(n, 0, 20)
ys = randrange(n, 0, 20)
zs = randrange(n, 0, 20)
the_fourth_dimension = randrange(n,0,20)
#print the_fourth_dimension

test1 =[[0.047, 0.0778, 0.577],[0.047, 0.077, 0.8]]
cov=[0.2,0.23]
test1_np = np.array(test1)
test1_x = test1_np[:,0]
print test1_x
test1_y = test1_np[:,1]
test1_z = test1_np[:,2]
#colors2 = cmx(the_fourth_dimension/max(the_fourth_dimension))
colors2 = cmx((np.array(cov)-min(cov))/(max(cov)-min(cov)))
print colors2
colmap = cm.ScalarMappable(cmap=cmx)
#colmap.set_array(the_fourth_dimension)
colmap.set_array(np.array(cov))

#yg = ax.scatter(test1_x, test1_y, test1_z,s=100,c= colors2,marker='o')
cb = fig.colorbar(colmap,orientation='horizontal')

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
plt.show()


# --- Colormaps from a list ---

colors = [(1, 0, 0), (0.5, 0.5, 0), (0, 1, 0)]  # R -> G -> B
n_bins = 200  # Discretizes the interpolation into bins
cmap_name = 'my_list'
#fig, axs = plt.subplots(2, 1, figsize=(6, 9))
#fig.subplots_adjust(left=0.02, bottom=0.06, right=0.95, top=0.94, wspace=0.05)
fig = plt.figure()
axs = fig.add_subplot(111,projection='3d')
#for n_bin, ax in zip(n_bins, axs):
    # Create the colormap
cm = LinearSegmentedColormap.from_list(
        cmap_name, colors, N=n_bins)
# Fewer bins will result in "coarser" colomap interpolation
im = axs.imshow(z, interpolation='nearest', origin='upper', cmap=cm)
axs.set_title("N bins: %s" % n_bins)
fig.colorbar(im, ax=axs,orientation='horizontal')


# --- Custom colormaps ---

cdict1 = {'red':   ((0.0, 0.0, 0.0),
                   (0.5, 0.0, 0.1),
                   (1.0, 1.0, 1.0)),

         'green': ((0.0, 0.0, 0.0),
                   (1.0, 0.0, 0.0)),

         'blue':  ((0.0, 0.0, 1.0),
                   (0.5, 0.1, 0.0),
                   (1.0, 0.0, 0.0))
        }

cdict2 = {'red':   ((0.0, 0.0, 0.0),
                   (0.5, 0.0, 1.0),
                   (1.0, 0.1, 1.0)),

         'green': ((0.0, 0.0, 0.0),
                   (1.0, 0.0, 0.0)),

         'blue':  ((0.0, 0.0, 0.1),
                   (0.5, 1.0, 0.0),
                   (1.0, 0.0, 0.0))
        }

cdict3 = {'red':  ((0.0, 0.0, 0.0),
                   (0.25, 0.0, 0.0),
                   (0.5, 0.8, 1.0),
                   (0.75, 1.0, 1.0),
                   (1.0, 0.4, 1.0)),

         'green': ((0.0, 0.0, 0.0),
                   (0.25, 0.0, 0.0),
                   (0.5, 0.9, 0.9),
                   (0.75, 0.0, 0.0),
                   (1.0, 0.0, 0.0)),

         'blue':  ((0.0, 0.0, 0.4),
                   (0.25, 1.0, 1.0),
                   (0.5, 1.0, 0.8),
                   (0.75, 0.0, 0.0),
                   (1.0, 0.0, 0.0))
        }

# Make a modified version of cdict3 with some transparency
# in the middle of the range.
cdict4 = cdict3.copy()
cdict4['alpha'] = ((0.0, 1.0, 1.0),
                #   (0.25,1.0, 1.0),
                   (0.5, 0.3, 0.3),
                #   (0.75,1.0, 1.0),
                   (1.0, 1.0, 1.0))


# Now we will use this example to illustrate 3 ways of
# handling custom colormaps.
# First, the most direct and explicit:

blue_red1 = LinearSegmentedColormap('BlueRed1', cdict1)

# Second, create the map explicitly and register it.
# Like the first method, this method works with any kind
# of Colormap, not just
# a LinearSegmentedColormap:

blue_red2 = LinearSegmentedColormap('BlueRed2', cdict2)
plt.register_cmap(cmap=blue_red2)

# Third, for LinearSegmentedColormap only,
# leave everything to register_cmap:

plt.register_cmap(name='BlueRed3', data=cdict3)  # optional lut kwarg
plt.register_cmap(name='BlueRedAlpha', data=cdict4)

# Make the figure:

fig, axs = plt.subplots(2, 2, figsize=(6, 9))
fig.subplots_adjust(left=0.02, bottom=0.06, right=0.95, top=0.94, wspace=0.05)

# Make 4 subplots:

im1 = axs[0, 0].imshow(Z, interpolation='nearest', cmap=blue_red1)
fig.colorbar(im1, ax=axs[0, 0])

cmap = plt.get_cmap('BlueRed2')
im2 = axs[1, 0].imshow(Z, interpolation='nearest', cmap=cmap)
fig.colorbar(im2, ax=axs[1, 0])

# Now we will set the third cmap as the default.  One would
# not normally do this in the middle of a script like this;
# it is done here just to illustrate the method.

plt.rcParams['image.cmap'] = 'BlueRed3'

im3 = axs[0, 1].imshow(Z, interpolation='nearest')
fig.colorbar(im3, ax=axs[0, 1])
axs[0, 1].set_title("Alpha = 1")

# Or as yet another variation, we can replace the rcParams
# specification *before* the imshow with the following *after*
# imshow.
# This sets the new default *and* sets the colormap of the last
# image-like item plotted via pyplot, if any.
#

# Draw a line with low zorder so it will be behind the image.
axs[1, 1].plot([0, 10*np.pi], [0, 20*np.pi], color='c', lw=20, zorder=-1)

im4 = axs[1, 1].imshow(Z, interpolation='nearest')
fig.colorbar(im4, ax=axs[1, 1])

# Here it is: changing the colormap for the current image and its
# colorbar after they have been plotted.
im4.set_cmap('BlueRedAlpha')
axs[1, 1].set_title("Varying alpha")
#

fig.suptitle('Custom Blue-Red colormaps', fontsize=16)

plt.show()
'''
