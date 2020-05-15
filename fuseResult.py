import os
import json
from sklearn.metrics import confusion_matrix
import itertools
import matplotlib.pyplot as plt
import numpy as np

_LABEL_CLASSES = 60

def readtxtdata(filepath):
    result = {'labels': [], 'probability': []}
    assert os.path.exists(filepath), (
        'Can not find data at given directory!!')

    with open(filepath) as f:
        for line in f:
            tmpresult = line.strip()
            l, p = tmpresult.split(',')
            result['labels'].append(int(l))
            result['probability'].append(float(p))
    f.close()
    return result



def loadJasondata(filepath):
    assert os.path.exists(filepath), (
        'Can not find data at given directory!!')
    with open(filepath) as f:
        data = json.load(f)
    return data



def get_truelabel(training_protocol):
  """Returns a list of filenames."""
  if training_protocol == 'cv':
      labelPath = '/home/qnie/PycharmProjects/ntumotion/training_protocol/crossviewTest_labels_s.txt'
  elif training_protocol == 'cs':
      labelPath = '/home/qnie/PycharmProjects/ntumotion/training_protocol/crosssubjectTest_labels_s.txt'
  else:
      print("error training protocol")
  labels = []
  with open(labelPath) as f:
    for line in f:
        labels.append(int(line.strip()))
  f.close()
  return labels



def fuseResults(result1, result2=None, operation='max'):

    preds = []
    if result2 == None:
        for m in range(len(result1)):
            preds.append(result1[m]['Class']+1)
    else:
        assert (len(result1) == len(result2)), ('length of file one is not compatible with length of file two')
        if operation == 'max':
            for m in range(len(result1)):
                if (max(result1[m]['probabilities']) >= max(result2[m]['probabilities'])):
                    preds.append(result1[m]['Class']+1)
                else:
                    preds.append(result2[m]['Class']+1)
        elif operation == 'average':
            for m in range(len(result1)):
                prob = [(x+y) for x, y in zip(result1[m]['probabilities'], result2[m]['probabilities'])]
                preds.append(prob.index(max(prob))+1)
        else:
            pass
    return preds


def calAcc(preds, groundtruth):
    assert (len(preds) == len(groundtruth)), ('length of preds is not compatible with length of groundtruth')
    right = 0
    for i in range(len(groundtruth)):
        if groundtruth[i] == preds[i]:
            right += 1
    acc = float(right) / len(groundtruth)
    return right, acc


def plot_confusion_matrix(cls_pred, cls_true,
                          normalize=False,
                          title='Confusion matrix',
                          cmap=plt.cm.Blues):
    """
    This function prints and plots the confusion matrix.
    Normalization can be applied by setting `normalize=True`.
    """
    cm = confusion_matrix(y_true=cls_true,
                          y_pred=cls_pred)
    # print(cm)
    plt.matshow(cm)
    if normalize:
        cm = cm.astype('float') / cm.sum(axis=1)[:, np.newaxis]
        print("Normalized confusion matrix")
    else:
        print('Confusion matrix, without normalization')

    print(cm)
    plt.imshow(cm, interpolation='nearest', cmap=cmap)
    plt.title(title)
    plt.colorbar()
    tick_marks = np.arange(_LABEL_CLASSES)
    plt.xticks(tick_marks, range(1, _LABEL_CLASSES + 1), rotation=45)
    plt.yticks(tick_marks, range(1, _LABEL_CLASSES + 1))

    fmt = '.2f' if normalize else 'd'
    thresh = cm.max() / 2.
    for i, j in itertools.product(range(cm.shape[0]), range(cm.shape[1])):
        if normalize:
            if cm[i, j] != 0.0:
                plt.text(j, i, format(cm[i, j], fmt),
                         horizontalalignment="center",
                         color="white" if cm[i, j] > thresh else "black", fontsize=5)
        else:
            plt.text(j, i, format(cm[i, j], fmt),
                     horizontalalignment="center",
                     color="white" if cm[i, j] > thresh else "black", fontsize=7)

    plt.tight_layout()
    plt.ylabel('True label')
    plt.xlabel('Predicted label')


def main():

    jsonpath1 = './model_crossview_midf/best/predsJson44491.json'
    jsonpath2 = './model_CV_ntuflattenedEDM/bests/predsJson79864.json'
    jsonpath3 = './model_CS_final/bests/predsJson54189.json'
    jsonpath4 = './model_CS_flattenedEDM/bests/predsJson83050.json'

    resultc = loadJasondata(jsonpath3)
    resultd = loadJasondata(jsonpath4)

    resultcl = []
    # for i in range(len(resultd)):
    #     resultcl.append(resultd[i]['Class']+1)
    preds_avg = fuseResults(resultd, resultc, 'average')
    # preds_max = fuseResults(resultc, resultd, 'max')
    groundtruth = get_truelabel('cs')
    # for n in range(len(groundtruth)):
    #     if groundtruth[n] == 1:
    #         preds_avg[n] = resultc[n]['Class']
    rightAvg, accAvg = calAcc(preds_avg, groundtruth)
    # rightMax, accMax = calAcc(preds_max, groundtruth)
    # right, acc = calAcc(resultcl, groundtruth)
    print("Based on average method, number of right predicted labels: {0}, accuracy is: {Acc}".format(rightAvg, Acc=accAvg))
    # print("Based on max method, number of right predicted labels: {0}, accuracy is: {Acc}".format(rightMax, Acc=accMax))
    # np.set_printoptions(precision=2)
    # plot_confusion_matrix(cls_pred=preds_avg, cls_true=groundtruth, normalize=True,
    #                       title="Normalized Confusion Matrix")
    # plot_confusion_matrix(cls_pred=preds_max, cls_true=groundtruth, normalize=True,
    #                       title="Normalized Confusion Matrix")
    # plt.show()



if __name__ == "__main__":
    main()

# Generally, according to the results of experiments, the result based on average method is slightly better than max method!
