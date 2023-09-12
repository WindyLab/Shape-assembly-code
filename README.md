This file is the source code of our paper “Mean-shift exploration in shape 
assembly of robot swarms”. It is coded by Matlab (R2020a). 


This file contains 16 m files and 1 folder.
• MainLoop.m is the main program, which is used to set simulation parameters,
  start/stop simulation and animation.
• Fcn_NegotPositState.m and Fcn_NegotOrientState are used to negotiate 
  position and orientation of the target formation.
• Fcn_CalEnteringCmd, Fcn_CalExplorationCmd, and Fcn_CalInteractionCmd are 
  used to calculate 





Human-swarm interface is an easy-to-use Matlab (R2020a) implementation for 
designing desired shape, which allows the user to either load a predesigned 
image or manually draw one in the workspace. 

The interface provides users with the follow function settings.
• Grid scale of the workspace. There are three options for user selection, 
namely ’sparse’, ’normal’, and ’dense’, corresponding to the grid size 
20 × 20, 60 × 60, and 100 × 100.
• Gray level used for gray transformation. The user can input the gray level 
h to be set in the edit box, and then click the ’setting’ button to complete 
the setting.
• Size of mouse brush. There are also three options for user selection, that 
is, ’small’, ’medium’, and ’big’. The option ’small’ means that the brush 
size is 1 grid, and the other two correspond to 5 and 10 grids, respectively.
• Color of mouse brush. The user can choose to draw black or white in the 
workspace. Recall that the desired shape is represented by the black grids 
while the white grids are invalid background. 

Note that the ﬁrst two settings are must be set by the user and the last two 
are required only for manual drawing. 
When all parameters are set, you can click the "Load" button to generate a 
gray grid.

Please send all bug reports to authors. 

The authors may be contacted via:

Shiyu Zhao
Associate Professor
Director of the Intelligent Unmanned Systems Laboratory
School of Engineering, Westlake University
Email: zhaoshiyu@westlake.edu.cn

and

Guibin Sun
Postdoctoral Fellow
School of Automation Science and Electrical Engineering
Beihang University
Email: sunguibinx@buaa.edu.cn

@Copyright: If using this procedure for publications, please cite the article 
"Mean-shift selfless exploration in shape assembly of robot swarms".


