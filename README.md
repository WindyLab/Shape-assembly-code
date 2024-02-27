<div align="center">
  <h1>Mean-shift exploration in shape assembly of robot swarms</h1>
<p align="center">
  <a href="https://www.nature.com/articles/s41467-023-39251-5">
    <img src="https://img.shields.io/badge/Paper-blue?logo=googledocs&logoColor=white&labelColor=grey&color=blue"></a>
  <a href="https://www.bilibili.com/video/BV1Pk4y1H7A3/?spm_id_from=333.999.0.0&vd_source=288648f5b920459d12ebbcfd2da00a19">
    <img src="https://img.shields.io/badge/Video-blue?logo=bilibili&logoColor=white&labelColor=grey&color=blue"></a>
  <a href="https://www.youtube.com/watch?v=inoifg2tcJM&feature=youtu.be">
    <img src="https://img.shields.io/badge/Video-blue?logo=youtube&logoColor=white&labelColor=grey&color=blue"></a>
  <a href="https://opensource.org/licenses/MIT">
    <img src="https://img.shields.io/badge/License-MIT-yellow.svg"></a>
</p>

![image](https://github.com/WestlakeIntelligentRobotics/Shape-assembly-code/assets/125523389/257a4227-ac3e-4f8e-8f2a-49e666366dde)

</div>

This file is the source code of our paper "Mean-shift exploration in shape assembly of robot swarms". It is coded by Matlab (R2020a). 


This file contains 16 m files and 1 folder.

- MainLoop.m is the main program, which is used to set simulation parameters, start/stop simulation and animation.
- Fcn_NegotPositState.m and Fcn_NegotOrientState are used to negotiate position and orientation of the target formation.
- Fcn_CalEnteringCmd, Fcn_CalExplorationCmd, and Fcn_CalInteractionCmd are used to calculate shape-entering, shape-exploring, and interaction velocity commands, respectively.
- Fcn_GetNeighborSet is used to calculate neighboring set, and Fcn_GetPerformMetric is used to calculate performance metrics.
- Lib_InitialFunctions, Lib_ShapeFunctions, Lib_UpdateFunctions, Lib_AnimatFunctions, Lib_DrawingFunctions, and Lib_RecordFunctions are used for simulation initialization, shape processing, robot motion update, animated simulation, curve drawing, and data record.
- The folder "ShapeImage" contains 4 different shapes, including snowflake, starfish, letter "R", letter "O", and letter "B".

Note that simulation parameters may need to be readjusted depending on the swarm size and shape type.

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

## Citing

If you find our work useful, please consider citing:

```BibTeX
Sun, G., Zhou, R., Ma, Z. et al. Mean-shift exploration in shape assembly of robot swarms. Nat Commun 14, 3476 (2023). https://doi.org/10.1038/s41467-023-39251-5
```



