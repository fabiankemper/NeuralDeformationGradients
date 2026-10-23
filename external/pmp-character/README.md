# PMP Character
This is an extension for the [pmp-library](https://github.com/pmp-library/pmp-library) to support character rendering.
This includes
- Linear Blend Skinning and Dual Quaternion Skinning
- Texture Loading
- Loading from FBX, BIM and GLTF
- Arbitrary skeleton format with joints 
- Scene Graph Viewing of Scenes consisting of characters, pointsets and meshes
## Authors and acknowledgment
Written by Friedemann Runte. (@peasman) with help of @swenninger

## License
For open source projects, say how it is licensed.

## Project status
- Currently Working:
  - Loading a GLTF File
    - Texture
    - Blendshapes
    - Joints + Skinning Weights
    - Meshes
    - Scenerendering

  
## Scene Graph Rendering
The `SceneGraphViewer` class is a viewer, capable of rendering a diverse set of objects in a scene graph. 
To make this work, every possible element of a scene graph is of type `SceneNode` and has a list of children.
Children are declared over `std::shared_ptr` to make memory management easier.

When using the `SceneGraphViewer` you can add new Elements to the Scene by creating a `SceneNode` with
all its children and add it via `add_node_to_root_node` to the Viewer. This automatically creates renderers 
for every child in the node hierarchy. The only thing you have to take care of 
is the initialization via `std::make_shared<SceneNode>` or else the viewer cannot 
differentiate between multiple subclasses of `SceneNode` for rendering. 

# The character class
The `Character` class is used to display a Character consisting of one or multiple 
skinned meshes and a skeleton structure. 

## Creating Characters
When loading a `Character` there are 2 ways to do this. 

### FBX
You can load a character from an fbx file using the bundled [ufbx](https://github.com/ufbx/ufbx) loader. 
For this you need to use the `FBX_file` class, which handles all loading operations.
After creating a `FBX_file` Object, you can use the `read` method to load a character into a predefined variable.
Writing fbx files is not supported, export to gltf instead.

### GLTF
You can load a Character from a `.glb` or `.gltf` file in a similar way to fbx. You can also export to gltf quite easily. 
When importing a Character from `.glb` or `.gltf` you once again create a `gltf_file` object. 
Then you create a `std::shared_ptr<Character>` to the Character to be stored in. 
This could also just be a pointer and is subject to change. 
You then call `gltf_file.load(character, path);` to load the character. 

For exporting you dont need a gltf_file object. You can simply use `gltf_file::write(character, path)`.








