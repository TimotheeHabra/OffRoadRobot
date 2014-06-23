Processing contact by Simbody library
=====================================


 General information 
---------------------------
Here is a template for coupling open-source [Simbody library](https://simtk.org/home/simbody/) for contact detecting  and processing with Robotran functions. 
Simbody provides a set of functions for contact detecting and processing. 

There are two algorithms for contact processing: 

1. Hertz contact for objects, which shape can be well approximated by paraboloid -- spheres, ellipsoids; (Not coupled yet with Robotran)

2. Elastic foundation model for mesh-to-mesh contact – for primitives like boxes, spheres and so on. Meshes can be created by means of Simbody functions. 
Also you can load meshes from WaveFront OBJ files. 

Notice that both methods include surface-to-plane contacts. 

The way of coupling Simbody and Robotran is the following: for each body that can contact with other bodies or ground 
(i.e. a body that is fixed in inertial frame) we create a “shadow” body in Simbody world. Then we
apply contact surfaces to it and define mechanical properties of contact – such as coefficients of friction, damping and stiffness (TODO! a link). 
During simulation Robotran S-sensors provide coordinates and velocities for each "shadow" 
body, and Simbody function computes all contacts and returns values of contact forces and torques brought to the origins of the bodies' coordinate frames. 
These values are used in the “user_ExtForces” function of Robotran.

Installation of Simbody 
-------------------------------

Simbody is an open-source, object-oriented C++ API, and its source codes can be download from [here](https://simtk.org/project/xml/downloads.xml?group_id=47).
We used Simbody 3.3.1 Release on Windows and Linux OS. Note that you should build 64-bit version of this library to be compatible with the whole project.
Instructions of building Simbody can be found on the Simbody official site: 

* [Mac and Linux](https://simtk.org/docman/view.php/47/1583/HowToBuildSimbodyFromSource_MacLinux.pdf)

* [Windows](https://simtk.org/docman/view.php/47/1582/HowToBuildSimbodyFromSource_Windows.pdf) 
(to make 64-bit version it is necessary to specify a generator “Visual Studio 11 Win64” for the Simbody project in Cmake).
Don't forget to build Release versions of the library (not only Debug)!


Definition of contact properties
-----------------------------------
### To define S- and F-sensors ###

Add S- and F-sensors to the bodies that you want to have contact with ground or each other. 
Sensors should be placed in the origin of body coordinate frame. 
To get IDs of the sensors run a script in Matlab getindices.m (TODO! in template there should be this script):

`prjname = 'OffRoadRobot';      % project name`

`[mbs_data, mbs_info] = mbs_load(prjname,'default');`

`id_S_FR_Sensor = mbs_get_S_sensor_id(mbs_info,'FR_Sensor'); % do not forget to change name to that you specified in MBSysPad`

`fprintf('// S sensors\n');`

`fprintf('#define S_FR_Sensor %d\n', id_S_FR_Sensor);`

`%% F-Sensors`

`id_F_FR_Sensor = mbs_get_F_sensor_id(mbs_info,'FR_Leg_Force');`

`fprintf('// F sensors\n');`

`fprintf('#define F_FR_Sensor %d\n', id_F_FR_Sensor);`


### Fill the contact properties ###

First, define the header file "SimbodyBodiesDefinitions.h":

* Define a number of contact bodies (except the ground that does not move during simulation. If in your project you need a moving ground, then you should add it like others bodies)

`#define NB_CONTACT_BODIES 4`

* Add IDs for sensors - just copy the output that appeared in Matlab workspace in step 1 and put it here

`// S sensors`

`#define S_FR_Sensor 1`

`...`

`// F sensors`

`#define F_FR_Sensor 1`

* Specify arrays for sensors IDs. S- and F- sensors of different bodies should go in the same order in these arrays. 
So the algorithm considers that first members of these arrays are attached to the first body in Simbody world, 
second members to the second body and so on.

`#define S_SENSORS_ARRAY {S_FR_Sensor, S_FL_Sensor, S_RL_Sensor,S_RR_Sensor}`

`#define F_SENSORS_ARRAY {F_FR_Sensor, F_FL_Sensor, F_RL_Sensor,F_RR_Sensor}`

Second, specify coefficients of contact model and meshes in the file "ContactPropertiesDefinitions.cpp".
There you have two functions to fill. The first one 'fill_ground_contact_properties' defines properties of ground (TODO! to add a flag to specify if ground exists in the model),
the second one 'fill_bodies_contact_properties' is used for bodies (you should fill the properties in the order you used for sensors in the previous file). 

You need to fill the following fields:

1. Mechanical parameters of the contact (link to the description of this parameters is in the "Comments" paragraph below):

 `BodyContProp->ud = 0.9;   // dynamic   dry friction coefficient`

 `BodyContProp->us = 1.1;   // static;  it is required ud < us`

 `BodyContProp->uv = 0;     // viscous (force/velocity)`

 `BodyContProp->c = 1e3;   // dissipation (1/v)`

 `BodyContProp->k = 5e3;    // stiffness (pascals)`

 `BodyContProp->thickness = 0.01;`
	
2. Type of geometry. For ground '0' here means OneHalfSpace  z+; '1' means a mesh defined by a file. 
(TODO! may be here -1 will be to specify if there is no ground; 
for bodies we can specify different primitives such as sphere, box, etc.)

 `BodyContProp->Geometry = 1; // 0 means OneHalfSpace  z+; 1 means a mesh`

3. Transformation of geometry - you should use the same numbers as in MBSysPad for vrml meshes. 
These fields are only applicable for mesh ground only (TODO! to add possibility to have inclined or  shifted ground)

 `BodyContProp->ScaleFactor = 0.5; // scaling factor. Applicable only for mesh`

 `BodyContProp->Transform[0] = -2.5; // translation in X, (TODO! to change name of the field from transform to translate? )`

 `BodyContProp->Transform[1] = -2; // ... Y,`

 `BodyContProp->Transform[2] = -0.05; // and Z directions`

 `BodyContProp->Rotation[0] = 90; // [degrees] Rotation about X,`

 `BodyContProp->Rotation[1] = 0; // Y,`

 `BodyContProp->Rotation[2] = 0; // Z axis.`
	
4. Name of a file describing a mesh

 `sprintf(BodyContProp->FileName, "ground_mine.obj");  // file should be in the folder \Standalone\src\project\simbody` (TODO! Where?)


Switching on flags in CMake
---------------------------------------

You should switch on following flags in CMake to have Simbody in your project:

* __FLAG_CXX_PROJECT__           : ON: cpp files and cc files (among which the main file) are compiled with a C++ compiler - OFF: no C++ used.
* __FLAG_SIMBODY__               : ON: Simbody used to handle the contacts - OFF: no Simbody feature.
* __FLAG_SIMBODYVIZ__            : ON: only for Windows. Add an OpenGL vizualization of Simbody world - it is useful to check if all meshes are attached to bodies and ground clearly - OFF: no Simbody feature.

and you should switch  off
* __FLAG_GROUND_CONTACT_MODEL__  : ON: Manual ground Contact Model (GCM) used - OFF: no the case (only used for the CoMan model).

(TOCHECK! Only for Windows - what for Linux?)
Also check the version of Simbody libraries in __SIMBODY_LIBRARIES_COM___, __-MATH__ and __-SIMB__ fields. By default it should be RELEASE version:
* C:/Program Files/Simbody/lib/SimTKcommon.lib
* C:/Program Files/Simbody/lib/SimTKmath.lib
* C:/Program Files/Simbody/lib/SimTKsimbody.lib
For __DEBUG__ versions there exists a postfix "_d" at the end of the file-name (like SimTKcommon_d.lib). DEBUG versions are very slow.

Comments
------------------------------

The physical meaninigs of the mechanical contact properties are described
[here](https://simtk.org/api_docs/simbody/api_docs33/Simbody/html/classSimTK_1_1ContactMaterial.html#details).

Exact formulae for normal and tangent forces are described 
[here](https://simtk.org/api_docs/simbody/api_docs33/Simbody/html/classSimTK_1_1ElasticFoundationForce.html#details).

To convert meshes to WaveFront *.obj (or *.vrml) format from Collada *.dae files we use [MeshLab](http://meshlab.sourceforge.net/).
Also it is useful for repairing of meshes: finding holes, removing unreferenced vetrices and duplicated vertices or faces.
(Filters->Cleaning and Repairing). Also Simbody requires that a mesh was a closed manifold - that means that vertices of the mesh's triangles 
were listed in the counter-clockwise order if we look at it from the outside of the surface. 
(In case of this problem look at the 'Select non-manifold vertices' command.)
Also we do not advise to use polygonal mesh for a contact - although Simbody has a function to convert polygonal mesh to a trianular one, 
it didn't work properly on some of our examples.

