#!/usr/bin/env python
""" 
Python script to plot output vectors resulting from the PIV analysis
Carl J. Legleiter 
6/23/2023
9/11/2023 - Revisit for new ROSPIV branch
File info: ~\TRiVIA\ROSPIV\rosPyShowVectors.py
But for ROS node production also need to copy to: C:\catkin_ws\src\streamflowpiv\scripts\rosPyShowVectors.py

**** PRELIMINARIES ****
To make this into a ROS node, copy this file to: C:\catkin_ws\src\streamflowpiv\scripts\rosPyShowVectors.py
Go to a separate bash terminal and ...
    cd /c/catkin_ws/src/streamflowpiv/scripts
    chmod +x rosPyShowVectors.py
Open the file C:\catkin_ws\src\streamflowpiv\CMakeLists.txt and add the following at the end:
    # 6/23/2023 (CJL): TACKED THIS ON TO MAKE USE OF A PYTHON SCRIPT
    catkin_install_python(PROGRAMS scripts/rosPyShowVectors.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )    
In a ROS terminal build the plotting node like this ...
    cd C:\catkin_ws
    catkin_make
    devel\setup.bat    
To run the node from the ROS terminal, use this syntax
    cd C:\catkin_ws
    devel\setup.bat
    rosrun streamflowpiv rosPyShowVectors.py
"""

import numpy as np
import matplotlib.pyplot as plt
# Note that before we could import rospy, we had to point to the folder with  the ROS-specific site-packages folder like so:
import sys
sys.path.append('C:\\opt\\ros\\noetic\\x64\\Lib\\site-packages')
import rospy
# The PIV output (velocity vector coordinates and components) is published to the ROS topic named /pivOut (or /pivFilt if we also call the post-processing node rosPostProc) as a 3D array with 4 layers (x, y, u, and v) stored in ROS messages of type std_msgs/Float32MultiArray
from std_msgs.msg import Float32MultiArray


# Callback function to get 3D array with PIV output and produce a plot of the velocity vectors
def getData(data):
    # Read in the actual data sent to this callback by the subscriber
    inVec       =   data.data
    print("Received initial PIV output")
    # We have to reshape the data vector into a 3D array, so query ROS parameter server to get the array dimensions
    height      =   int(rospy.get_param("/numY"))
    width       =   int(rospy.get_param("/numX"))
    # We know the number of layers in the 3D array is 4 (x, y, u, and v)
    layers      =   4
    # Now reorganize the data vector into a 3D array, but be careful. Here's a critical bit of information from this very helpful website:
        # https://numpy.org/doc/stable/user/numpy-for-matlab-users.html
    # "Note that the scan order used by reshape in NumPy defaults to the ‘C’ order, whereas MATLAB uses the Fortran order. If you are simply converting to a linear sequence and back this doesn’t matter. But if you are converting reshapes from MATLAB code which relies on the scan order, then this MATLAB code: z = reshape(x,3,4); should become z = x.reshape(3,4,order='F').copy() in NumPy."
    # So we have to tack on a couple of extra bits to our reshape command
    inArray    =  np.array(inVec).reshape(height,width,layers,order='F').copy()    
    
    # Now we can extract the x and y coordinates and u and v components of the vectors
    xGrid       =   inArray[:,:,0]
    yGrid       =   inArray[:,:,1]
    uGrid       =   inArray[:,:,2]
    vGrid       =   inArray[:,:,3]
    magGrid     =   np.sqrt(uGrid**2 + vGrid**2)

    # Get the pixel size and frame interval from the parameter server and use them to scale the velocity vectors
    pixSize     =   rospy.get_param("/pixSize")
    frameInt    =   rospy.get_param("/frameInterval")
    uScale      =   uGrid * pixSize / frameInt
    vScale      =   vGrid * pixSize / frameInt

    # Display the velocity vectors in a quiver plot
    # See this site for an example: https://matplotlib.org/stable/gallery/images_contours_and_fields/quiver_simple_demo.html
    fig, ax     =   plt.subplots()
    cmap        =   plt.get_cmap('viridis')
    ax.set_title('PIV-derived velocity field (m/s)')
    # Old plotting code:
        # # Plot the velocity magnitude as an image   
        # im = ax.imshow(magGrid,origin='lower',extent=[np.nanmin(xGrid),np.nanmax(xGrid),np.nanmin(yGrid),np.nanmax(yGrid)],vmin=0,vmax=np.nanmean(magGrid))
        # # Add a colorbar
        # cbar = ax.figure.colorbar(im, ax=ax)
        # # Now overlay velocity vectors on the image
        # q = ax.quiver(xGrid,yGrid,uGrid,vGrid,magGrid,angles='xy')
        # # Aborted attempt to add a quiver key (scale bar)
        # # qk= ax.quiverkey(q, 0.9, 0.9, np.nanmean(magGrid), 'Mean velocity magnitude: ' + str(np.nanmean(uGrid)) + ' m/s', labelpos='E', coordinates='figure')
        # # qk= ax.quiverkey(q, 0.9, 0.9, 1, 'Velocity magnitude: 1 m/s', labelpos='E', coordinates='figure')
        # plt.show()
    
    # New plotting code from John Vicino:
    # Get a masked version of the magnitude grid         
    magScale_masked =   np.ma.masked_where(np.isnan(magGrid), magGrid)
    # Plot the masked magnitude grid with the pcolormesh plotting routine
    c = ax.pcolormesh(xGrid, yGrid, magScale_masked, cmap=cmap, shading='auto')
    # Set the color limits for the plot based on the 98th percentile of the magnitude grid
    clim        =   (0, np.nanpercentile(magGrid, 98))
    c.set_clim(clim)
    # Format the plot
    ax.axis('tight')
    ax.set_aspect('equal', 'box')
    # Add a colorbar
    plt.colorbar(c, ax=ax)
    # Plot all the vectors
    ax.quiver(xGrid, yGrid, uScale, vScale, color='k', alpha=1.0)    
    # Uncomment the below lines if you want to use subsampled arrows
    # subsample_factor = 3  # Adjust this value to control the density of arrows
    # subsampled_xPiv = xPiv[::subsample_factor]
    # subsampled_yPiv = yPiv[::subsample_factor]
    # subsampled_uScale = uScale[::subsample_factor]
    # subsampled_vScale = vScale[::subsample_factor]
    # ax.quiver(subsampled_xPiv, subsampled_yPiv, subsampled_uScale, subsampled_vScale, scale=50, color='k',  alpha=0.5)
    # Finally, actually show the plot
    plt.show()

# Define the "main" function to set up the node and subscribers
if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('rosPyShowVectors')
    # Set up subscribers to get the grids of vector coordinates and components
    # print("Ready to plot velocity field, waiting for initial PIV output ...")
    # rospy.Subscriber("pivOut", Float32MultiArray, getData)
    print("Ready to plot filtered velocity field, waiting for post-processed PIV output ...")
    rospy.Subscriber("pivFilt", Float32MultiArray, getData)
    # Spin keeps python from exiting until the node is stopped
    rospy.spin()