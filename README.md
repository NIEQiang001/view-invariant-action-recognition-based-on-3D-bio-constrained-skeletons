# View-invariant-action-recognition-based-on-3D-bio-constrained-skeletons

This is the code for the paper

Nie, Q., Wang, J., Wang, X. and Liu, Y., 2019. View-Invariant Human Action Recognition Based on a 3D Bio-Constrained Skeleton Model. IEEE Transactions on Image Processing, 28(8), pp.3959-3972. https://ieeexplore.ieee.org/document/8672922

The code in this repository was written by Qiang Nie.

In this repository, we provide some results we achieved and the codes we used in our experiments. As the code for motion visualization related to a patent, we can only offer a partial of it. However, we share some generated motion images for the action recognition. Readers are also suggested to reproduce this part according to our paper.

Currently, only a C++ version of the motion visualization code is provided for a reference.

## Dependencies
* Dlib
* Kinect library
* tensorflow 1.11 or later

## Introduction of the files
* skeleton_visualization: The codes used for motion visualization using skeleton data. Readers can refer to the `visualize_motion_partial.cpp` to generate motion images based on JEAs and JEDM features. Only a c++ version is provided. This part is taken out from one of our project and only offered for reference. Readers are suggested to write their own according to our paper.
* data: This folder contains the generated motion images of the MSRC-12 dataset and the Northwestern_UCLA dataset. Some image samples can also be found in it. 
  * Generated motion images of MSRC-12 dataset can also be obtained from [here](https://pan.baidu.com/s/19U8oUBM25-55lJ7k_fCTAA). PW：95za
  * Generated motion images of NTU RGB+D dataset can be obtained from [here](https://pan.baidu.com/s/1UrTqjqHQpXBp8YCH5XoYJQ). PW：hp51
* traing protocol: according to the cross-view or cross-subject training protocol, we list the sample filenames of each part for NTU RGB+D dataset.

## Training
1. Download the encoded motion images of each dataset
2. Download the training protocol or generate the training protocol for Northwestern-UCLA dataset/MSRC-12 dataset as we do.
3. Modify the related file path in `motionrcg_cs.py` or `motionrcg_cv.py`.
    ```python
    python motionrcg_cs.py
    python motionrcg_cv.py


## Results
We report the prediction results achieved from NTU RGB+D dataset in the `results` folder. You can fuse the predictions from the JEAs stream and the JEDM stream using `fuseResult.py`.
![](https://github.com/NIEQiang001/view-invariant-action-recognition-based-on-3D-bio-constrained-skeletons/raw/master/results/NCMcv_NTU.jpg)


## Citation
To cite our paper, please use the following bibtex record:

@article{nie2019view,
  title={View-Invariant Human Action Recognition Based on a 3D Bio-Constrained Skeleton Model},
  author={Nie, Qiang and Wang, Jiangliu and Wang, Xin and Liu, Yunhui},
  journal={IEEE Transactions on Image Processing},
  volume={28},
  number={8},
  pages={3959--3972},
  year={2019},
  publisher={IEEE}
}
