# TUTORIAL

## How to set up the generalized barycentric warp for the anatomical meshes

This workflow is a bit annoying (and slow!) but fortunately has to only be done once. 

First: Call the mvc_proprocessing target. It has no arguments but needs to be run from the volumetric_animation directory (ProjectFileDir, if on CLion).
This will precompute, which points are contained in which prisms. Data describing this is stored under "data/mvc"

Second: Call "solve_prism_mvcs.py". This uses scipy solver to compute generalized barycentric coordinates for the prism collisions

Third: Start the viewer. All data should be present and the barycentric_warp should work. The methods responsible for this are in "bary_warp.h", 
"barycentric_warp.cpp/h" contain wrapper code. 

Ideally do not touch this unless you really have to, it's not particularly pretty. 