# Analyzing DOPE Training
This document describes our experience in training DOPE to detect new images.

### First dataset
To train DOPE to detect new objects, it is necessary to create a robust dataset. For this project [NNDS](NNDS%20Tutorial.md) was used to generate the dataset. For the first training session, a dataset of 30.000 domain-randomized images were used. A subset of these images is shown below. Note that the deformation that can be seen does not have to be present in the dataset. The deformation is automatically applied before the trained dataset is used.

![1st training set](resources/DOPE/1st_dataset_subset.png)

The training on this dataset was run for 44 epochs[^1]. In the images below it shown that the longer the AI is trained on the dataset, the better the AI becomes at detecting objects.

| Detection after 11 epochs | Detection after 22 epochs |
| --- | --- |
| ![Epoch 11](resources/DOPE/beeropener_epoch_11.jpeg) | ![Epoch 22](resources/DOPE/beeropener_epoch_22.jpeg) |

| Detection after 33 epochs | Detection after 44 epochs |
| --- | --- |
| ![Epoch 33](resources/DOPE/beeropener_epoch_33.jpeg) | ![Epoch 44](resources/DOPE/beeropener_epoch_44.jpeg) |

However, the detection still wasn't good enough. The beeropener that is on the left still isn't detected. This happens because the beeropener is placed in the shadow of the bin. It would be possible to train more and more on this dataset. But before this was done, the loss[^2] off the training was checked. This can be seen in the graph below. What can be seen is that the graph flattens out or, in other words, the improvement per epoch becomes less.

![1st training plot](resources/DOPE/1st_beeropener_loss_per_batch.png)

It was concluded that more training on this dataset would not be very beneficial anymore and that it would be better to make a new, improved, dataset.

### Second dataset
The second dataset contained 40.000 domain-randomized images, and 10.000 photo-realistic images. This still isn't close to the 60.000 domain-randomized images and 60.000 photo-realistic images the authors of DOPE suggest, but the GPU that is accessible for this project isn't fast enough. Training on such a big dataset would take weeks. Therefore the dataset was still kept small. The lighting on the domain-randomized images was also tuned more aggressive, so there are more darker images to (hopefully) improve the AI's performance in shadows. A subset of the images is shown below.

![2nd training set](resources/DOPE/2nd_dataset_subset.png)

The AI was again tested, but only after 18 epochs. However, the AI is able to detect the same objects the previous AI was able to detect at 44 epochs.

![Epoch 18, 2nd training](resources/DOPE/beeropener_epoch_18_2nd.jpeg)

Sadly, the AI is still not able to detect the beeropener in the shadow. Hopefully with more training this will become possible. 

The loss values of this training session are also a bit better than the first session. The curve still flattens out at the same rate as the training on the previous dataset, so also this dataset will reach a point when more training won't improve the AI.

![1st and 2nd dataset](resources/DOPE/beeropener_sessions.png)

### Recommendations
It is recommended that the dataset is extended to the 60.000 domain-randomized images and 60.000 photo-realistic images. This will make the AI perform better. It is also recommended to include images where the desired object is close to the camera. Currently our trained models are not able to reliably detect objects when they are very close to the camera.

[^1]: One epoch is one training on all the data

[^2]: The loss used for DOPE is the L2 Loss and is defined as follows:

![L2 Loss function](resources/DOPE/l2_loss_function.png)

The expected outcome (`y_predicted`) and the actual outcome (`y_true`) are used to determine the loss. Note that the loss only says something about the performance of the AI on the dataset that it is training on. A bad dataset can still have good loss values. It cannot be said that one AI is better than another AI solely based on the loss value.

Read next:  
[Configuring DOPE](Configuring%20DOPE.md)