"""Runs a ResNet model on the ntu rgb+d dataset."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import os
import sys
import json

import tensorflow as tf
import numpy as np
from sklearn.metrics import confusion_matrix
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

import resnet_model

os.environ["CUDA_VISIBLE_DEVICES"] = "0, 3"
parser = argparse.ArgumentParser()

# Basic model parameters.
parser.add_argument('--datadir', type=str, default='/home/qnie/PycharmProjects/ntumotion/flattenedEDM_R//',
                    help='The path to the motion data directory.')


parser.add_argument('--model_dir', type=str, default='/home/qnie/PycharmProjects/ntumotion/model_CS_fEDM_R',
                    help='The directory where the model will be stored.')

parser.add_argument('--resnet_size', type=int, default=34,
                    help='The size of the ResNet model to use.')

parser.add_argument('--train_epochs', type=int, default=160,
                    help='The number of epochs to train.')

parser.add_argument('--epochs_per_eval', type=int, default=1,
                    help='The number of epochs to run in between evaluations.')

parser.add_argument('--batch_size', type=int, default=64,
                    help='The number of images per batch.')

parser.add_argument(
    '--data_format', type=str, default='channels_last',
    choices=['channels_first', 'channels_last'],
    help='A flag to override the data format used in the model. channels_first '
         'provides a performance boost on GPU but is not always compatible '
         'with CPU. If left unspecified, the data format will be chosen '
         'automatically based on whether TensorFlow was built for CPU or GPU.')


_DEFAULT_IMAGE_SIZE = 224
_NUM_CHANNELS = 3
_LABEL_CLASSES = 60
# Best validation accuracy seen so far.
best_validation_accuracy = 0.0

# We use a weight decay of 0.0001, which performs better than the 0.0001 that
# was originally suggested.
_WEIGHT_DECAY = 1e-4
_MOMENTUM = 0.9

_NUM_IMAGES = {
    # 'train': 38930,
    # 'validation': 15897,
}

_FILE_SHUFFLE_BUFFER = 1024
_SHUFFLE_BUFFER = 1500



def get_filenames(is_training,datadir):
  """Returns a list of filenames."""
  assert os.path.exists(datadir), (
      'Can not find data at given directory!!')
  if(is_training):
      labels = []
      data_dir = []
      with open('/home/qnie/PycharmProjects/ntumotion/training_protocol/fEDM_R_CS_trainimgs.txt') as f:
          for line in f:
              data_dir.append(datadir+line.strip())
      f.close()

      with open('/home/qnie/PycharmProjects/ntumotion/training_protocol/fEDM_R_CS_trainlabels.txt') as f:
          for line in f:
              labels.append(int(line.strip()))
      f.close()
  else:
      labels = []
      data_dir = []
      with open('/home/qnie/PycharmProjects/ntumotion/training_protocol/fEDM_R_CS_testimgs.txt') as f:
          for line in f:
              data_dir.append(datadir + line.strip())
      f.close()

      with open('/home/qnie/PycharmProjects/ntumotion/training_protocol/fEDM_R_CS_testlabels.txt') as f:
          for line in f:
              labels.append(int(line.strip()))
      f.close()

  return data_dir, labels


def image_preprocessing(image):
    return image

# Reads an image from a file, decodes it into a dense tensor, and resizes it
# to a fixed shape.
def _parse_function(filename, label):
  image_string = tf.read_file(filename)
  image_decoded = tf.image.decode_jpeg(image_string, _NUM_CHANNELS)
  image = tf.image.convert_image_dtype(image_decoded, dtype=tf.float32)
  """image = vgg_preprocessing.preprocess_image(
      image=image,
      output_height=_DEFAULT_IMAGE_SIZE,
      output_width=_DEFAULT_IMAGE_SIZE,
      is_training=is_training)"""

  image = tf.image.resize_images(image, [224, 224])
  label = tf.subtract(label, 1, None)
  return image, tf.one_hot(label, _LABEL_CLASSES)

def jsonSerializer(tfdic):
    preds = {'Class': [], 'probabilities': []}
    preds['Class'] = tfdic['classes']
    for i in range(len(tfdic['probabilities'])):
        preds['probabilities'].append(float(tfdic['probabilities'][i]))
    return preds

def input_fn(is_training, datadir, batch_size, num_epochs=1):
  """Input_fn using the tf.data input pipeline for CIFAR-10 dataset.

  Args:
    is_training: A boolean denoting whether the input is for training.
    data_dir: The directory containing the input data.
    batch_size: The number of samples per batch.
    num_epochs: The number of epochs to repeat the dataset.

  Returns:
    A tuple of images and labels.
  """
  filenames, labels = get_filenames(is_training, datadir)
  data_dir = tf.constant(filenames)
  labels = tf.constant(labels)
  tf.cast(labels, tf.int32)
  dataset = tf.data.Dataset.from_tensor_slices((filenames, labels))
  dataset = dataset.map(_parse_function)
  if is_training:
    # When choosing shuffle buffer sizes, larger sizes result in better
    # randomness, while smaller sizes have better performance. Because CIFAR-10
    # is a relatively small dataset, we choose to shuffle the full epoch.
    dataset = dataset.shuffle(buffer_size=_FILE_SHUFFLE_BUFFER)

  #dataset = dataset.map(
     #lambda image, label: (image, label))
  dataset = dataset.prefetch(2 * batch_size)

  # We call repeat after shuffling, rather than before, to prevent separate
  # epochs from blending together.
  dataset = dataset.repeat(num_epochs)

  # Batch results by up to batch_size, and then fetch the tuple from the
  # iterator.
  dataset = dataset.batch(batch_size)

  iterator = dataset.make_one_shot_iterator()
  images, labels = iterator.get_next()
  return images, labels

def motionrcg_model_fn(features, labels, mode, params):
    tf.summary.image('images', features, max_outputs=6)
    network = resnet_model.imagenet_resnet_v2(
        params['resnet_size'], _LABEL_CLASSES, params['data_format'])

    #inputs = tf.reshape(features, [-1, _HEIGHT, _WIDTH, _DEPTH])
    logits = network(inputs=features, is_training=(mode == tf.estimator.ModeKeys.TRAIN))

    predictions = {
        'classes': tf.argmax(logits, axis=1),
        'probabilities': tf.nn.softmax(logits, name='softmax_tensor')
    }

    if mode == tf.estimator.ModeKeys.PREDICT:
        return tf.estimator.EstimatorSpec(mode=mode, predictions=predictions)
    #print(params['data_format'])

    # Calculate loss, which includes softmax cross entropy and L2 regularization.
    cross_entropy = tf.losses.softmax_cross_entropy(
        logits=logits, onehot_labels=labels)
    # Create a tensor named cross_entropy for logging purposes.
    tf.identity(cross_entropy, name='cross_entropy')
    tf.summary.scalar('cross_entropy', cross_entropy)

    # Add weight decay to the loss.
    loss = cross_entropy + _WEIGHT_DECAY * tf.add_n(
        [tf.nn.l2_loss(v) for v in tf.trainable_variables()])

    if mode == tf.estimator.ModeKeys.TRAIN:
        # Scale the learning rate linearly with the batch size. When the batch size
        # is 128, the learning rate should be 0.1.
        initial_learning_rate = 0.04 * params['batch_size'] / 256
        batches_per_epoch = _NUM_IMAGES['train'] / params['batch_size']
        global_step = tf.train.get_or_create_global_step()

        # Multiply the learning rate by 0.1 at 100, 150, and 200 epochs.
        boundaries = [int(batches_per_epoch * epoch) for epoch in [70, 120, 150]]
        values = [initial_learning_rate * decay for decay in [1, 0.1, 0.05, 0.01, 1e-4]]
        learning_rate = tf.train.piecewise_constant(
            tf.cast(global_step, tf.int32), boundaries, values)

        # Create a tensor named learning_rate for logging purposes
        tf.identity(learning_rate, name='learning_rate')
        tf.summary.scalar('learning_rate', learning_rate)

        optimizer = tf.train.MomentumOptimizer(
            learning_rate=learning_rate,
            momentum=_MOMENTUM)

        # Batch norm requires update ops to be added as a dependency to the train_op
        update_ops = tf.get_collection(tf.GraphKeys.UPDATE_OPS)
        with tf.control_dependencies(update_ops):
            train_op = optimizer.minimize(loss, global_step)
    else:
        train_op = None

    accuracy = tf.metrics.accuracy(
        tf.argmax(labels, axis=1), predictions['classes'])
    metrics = {'accuracy': accuracy}

    # Create a tensor named train_accuracy for logging purposes
    tf.identity(accuracy[1], name='train_accuracy')
    tf.summary.scalar('train_accuracy', accuracy[1])
    return tf.estimator.EstimatorSpec(
      mode=mode,
      predictions=predictions,
      loss=loss,
      train_op=train_op,
      eval_metric_ops=metrics)

def main(unused_argv):
    # Using the Winograd non-fused algorithms provides a small performance boost.
    os.environ['TF_ENABLE_WINOGRAD_NONFUSED'] = '1'

    # Set up a RunConfig to only save checkpoints once per training cycle.
    run_config = tf.estimator.RunConfig().replace(save_checkpoints_secs=1e9)
    motionrcg_classifier = tf.estimator.Estimator(
        model_fn=motionrcg_model_fn, model_dir=FLAGS.model_dir, config=run_config,
        params={
            'resnet_size': FLAGS.resnet_size,
            'data_format': FLAGS.data_format,
            'batch_size': FLAGS.batch_size,
        })

    # ########### start training ##################
    for _ in range(FLAGS.train_epochs // FLAGS.epochs_per_eval):
        tensors_to_log = {
            'learning_rate': 'learning_rate',
            'cross_entropy': 'cross_entropy',
            'train_accuracy': 'train_accuracy'
        }

        logging_hook = tf.train.LoggingTensorHook(
            tensors=tensors_to_log, every_n_iter=100)

        motionrcg_classifier.train(
            input_fn=lambda: input_fn(
                True, FLAGS.datadir, FLAGS.batch_size, FLAGS.epochs_per_eval),
            hooks=[logging_hook])

        # Evaluate the model and print results
        eval_results = motionrcg_classifier.evaluate(
            input_fn=lambda: input_fn(False, FLAGS.datadir, FLAGS.batch_size))
        print(eval_results)


    # ######### uncomment the following codes and comment the training part when used for testing ###########
    # predictlables = list(motionrcg_classifier.predict(
    #     input_fn=lambda: input_fn(False, FLAGS.datadir, FLAGS.batch_size),
    #     checkpoint_path='./model_CS_final/bests/model.ckpt-54189'))
    # print(len(predictlables))
    # _, labels = get_filenames(False, FLAGS.datadir)
    # tcount = 0
    # for i in range(len(predictlables)):
    #     if predictlables[i]['classes'] == (labels[i] - 1):
    #         tcount += 1
    # print(tcount / len(predictlables))
    # jsonPreds = []
    # for i in range(len(predictlables)):
    #     jsonPreds.append(jsonSerializer(predictlables[i]))
    # with open("./model_CS_final/bests/train_predsJson54189.json", "w") as f:
    #     json.dump(jsonPreds, f)


if __name__ == '__main__':
  tf.logging.set_verbosity(tf.logging.INFO)
  FLAGS, unparsed = parser.parse_known_args()
  tf.app.run(argv=[sys.argv[0]] + unparsed)