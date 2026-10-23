# Neural Deformation Gradients

Official implementation of the paper [Skeletal-Driven Animation of Anatomical Humans via Neural Deformation Gradients](https://doi.org/10.1111/cgf.70388) (Computer Graphics Forum, Eurographics 2026).

## Installation

### Clone the repository

```bash
git clone https://github.com/fabiankemper/NeuralDeformationGradients.git
cd NeuralDeformationGradients
```

## Building Project

### Dependencies

Compatible with Linux, MacOS and Windows. Minimal supported GCC version is 12.

**External dependencies**

- Libzip
- ONNX Runtime
- OpenMP (optional, but recommended)

```bash
# install common dependencies on
apt install libzip-dev zipcmp ziptool zipmerge zipcmp libcmp libcmp-dev
# macos
brew install libomp
```

For installation of ONNX runtime follow the instructions in the [documentation](https://onnxruntime.ai/docs/install/) and download the [release tarball](https://github.com/microsoft/onnxruntime/releases) which matches your system.

For installation of the [FBXSDK](https://aps.autodesk.com/developer/overview/fbx-sdk) follow the Autodesk installation guides. You might adjust the CMAKE Variable `FBX_EXT_DIR` to point to your installation dir.


### Getting the models

First, prepare the virtual environment which is needed for downloading the huggingface model files.

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Then simply use the downloader script, which places the modes in their correct locations.

```bash
python download_models.py
```

### Building

```bash
cmake -S . -B build -G Ninja
cmake --build build --target character_viewer
```

#### MacOS

```bash
cmake -S . -B build -G Ninja -DCMAKE_C_COMPILER=clang \
  -DCMAKE_CXX_COMPILER=clang++ \
  -DOpenMP_CXX_FLAGS="-Xpreprocessor -fopenmp" \
  -DOpenMP_omp_LIBRARY=/opt/homebrew/opt/libomp/lib/libomp.dylib \
  -DCMAKE_PREFIX_PATH=/opt/homebrew/opt/libomp
```

#### Intel MKL Support

Add the following option to CMake to enable building with IntelMKL.

```bash
cmake -DMKL_ROOT=/opt/intel/oneapi/mkl/latest ...
```

If you want to build the project with both python support and intel MKL, the dependencies of python and c++ on MKL may crash.

To fix this, exporting LD_LIBRARY_PATH=/path/to/intel/oneapi/mkl/latest/lib/intel64:$LD_LIBRARY_PATH works well.

**Note**: Intel MKL has currently problems with GCC 15+. Use Clang for compilation instead as workaround.

## Running

Execute the main viewer via

```bash
./build/character_viewer [--gender [MALE/FEMALE]]
Optionally, you can set a path to samples from InsideHumans
--samples_dir [PATH_TO_CAESAR_FITTED_LAYERS] 
```

## Shortcuts

- **L** for starting playback (and speed up).
- **K** for stoping playback.
- **J** for reverse playback.
- **N** for showing next frame in timeline.
- **B** for showing previous frame in timeline.
- **M** for exporting current frame number into `marked_frames.txt`
- **BACKSPACE** reset Camera.

## Citation

If you use this code in your research, please cite our paper:

```bibtex
@article{nolte2026ndg,
  title   = {Skeletal-Driven Animation of Anatomical Humans via Neural Deformation Gradients},
  author  = {Nolte, Gerrit and Kemper, Fabian and Schwanecke, Ulrich and Botsch, Mario},
  journal = {Computer Graphics Forum},
  volume  = {45},
  number  = {2},
  year    = {2026},
  doi     = {10.1111/cgf.70388}
}
```
