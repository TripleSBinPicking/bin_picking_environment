# Training DOPE
The Deep Object Pose Estimation (DOPE) AI is an artificial intelligence designed by Nvidia. It can do 6D pose estimation of objects in images. 6D pose estimation is estimation the transformation (x, y, z) and rotation (roll, pitch, yaw) of an object with respect to the camera. In order to detect objects, it is needed to train the AI on how to detect those objects. This document describes how this can be done.

DOPE needs a very powerful NVidia graphics card in order to train. If you don't have such a graphics card available, it is possible to use the online service [kaggle](https://kaggle.com). Kaggle is an online Jupyter notebook/Google Colab like service and gives you access to a NVidia graphics card for 30 hours per week. 

### Running locally
It is assumed that you have already cloned this repository as per the instructions in the [readme](../readme.md), and that CUDA is installed on your system. You also need your dataset.
```bash
rosrun dope train.py --data <path to dataset> --object <object to train> --outf <path to output folder> --epochs <number of epochs>
```
The relevant parameters are explained in more detail in the following table:
| Parameter Name | Example value | Explanation |
| --- | --- | --- |
| data | `/path/to/dataset` | The path to the dataset to train on. It must contain the images to train on, as well as a json file for each file that contains the pose of the object to train on |
| object | `bicycle` | The name of the object to train on. This _must_ be a lowercase name! Even if the name of the object in the json file has capitalized letters |
| outf | `/path/to/dataset-output` | The output folder. The `.pth` files will be placed here |
| epochs | `30` | The amount of epochs to train for. One epoch is one training session on all the data. More epochs will take more time, but also lead to a better trained AI. After each epoch, an `pth` file is created |
| _The following parameters are not required_ |
| save | `true` | Save a batch of images with the cuboids draw on it to validate the dataset |
| net | `/path/to/epoch.pth` | Path to a `.pth` file to continue training |

So, in order to run for 60 epochs the entire command could be:
```bash
rosrun dope train.py --data /path/to/dataset --object bicycle --outf /path/to/output --epochs 60
```
Once the command is run the AI will start training on the new object.

### Running on Kaggle
In order to run on Kaggle some modifications will have to be made. Some of these modifications are a bit _hacky_, but that is necessary because DOPE is not intended to run on Kaggle.

Insert the following code into a Kaggle code field:
```python
!git clone https://github.com/NVlabs/Deep_Object_Pose.git
!pip install -r Deep_Object_Pose/requirements.txt

import configparser
import sys
# DOPE uses an outdated name for the configparser
sys.modules['ConfigParser'] = sys.modules['configparser']

sys.argv = 'train.py --data /kaggle/input/beer-opener2/Bieropener --object bieropener --outf /kaggle/working/result --epochs 15'.split(' ')

import Deep_Object_Pose.scripts.train as train
```
You can place your command parameters in `sys.argv`, but make sure to include `train.py` at the beginning and keep the `.split(' ')` function call at the end.
You can access your dataset by creating a new dataset in Kaggle, uploading your dataset and then adding it to the Kaggle notebook. The dataset will then be located in `/kaggle/input/<dataset name>`. Make sure to put the output in `/kaggle/working`, otherwise you wont be able to access it afterwards.

### Validating your dataset
Before you train DOPE, it is a good idea to validate that your dataset is correctly loaded in. In order to do this, you can add the `save` option to the command. This will output two image files in the `outf` directory. The images contain a subset of your dataset, with cuboids drawn around the object to train on, like in the following image:
![DOPE dataset validation](resources/dope_train_lowercase.png)
Dope has also distorted each image to make the result more robust for different kind of situations. If the dataset is not correctly loaded, you might get an error, or images without cuboids drawn on them:
![DOPE dataset validation2](resources/dope_train_capitalized.png)