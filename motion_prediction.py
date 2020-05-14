"""predict the label for a single image"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import os
import sys

import tensorflow as tf
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import resnet_model
import motionrcg_saved
import vgg_preprocessing


#check the motion_model and _LABEL_CLASSES first!!!
checkpoint_directory = '/home/qnie/PycharmProjects/ntumotion/motion_model_saved/'
inputimage = '/home/qnie/PycharmProjects/ntumotion/ntu_motion_midsampled/S005C003P010R001A056.skeleton_0.jpg'
_NUM_CHANNELS = 3
_LABEL_CLASSES = 60
params = {
            'resnet_size': 34,
            'data_format': 'channels_last',
            'batch_size': 64,
        }
# img = mpimg.imread(inputimage)
# plt.imshow(img)
# plt.show()

image_string = tf.read_file(inputimage)
image_decoded = tf.image.decode_jpeg(image_string, _NUM_CHANNELS)
image = tf.image.convert_image_dtype(image_decoded, dtype=tf.float32)
image = tf.image.resize_images(image, [224, 224])
image_t = tf.expand_dims(image, 0)  # [224, 224, 3] => [1, 224, 224, 3]

#forward
network = resnet_model.imagenet_resnet_v2(params['resnet_size'], _LABEL_CLASSES, params['data_format'])
logits = network(inputs=image_t, is_training=False)
predictions = {
        'classes': tf.add(tf.argmax(logits, axis=1), 1),
        'probabilities': tf.nn.softmax(logits, name='softmax_tensor')
    }


sess = tf.Session()
sess.run(tf.global_variables_initializer())
#saver = tf.train.import_meta_graph('./motion_model/model.ckpt-11223.meta')
saver = tf.train.Saver()
saver.restore(sess, tf.train.latest_checkpoint('./motion_model_saved/'))

predictedClass = sess.run(predictions['classes'])
probability = sess.run(predictions['probabilities'])
print("Predicted class is:{}, probability: {}".format(predictedClass, probability[0][predictedClass-1]))





