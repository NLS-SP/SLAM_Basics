'''matplotlib inline'''
from IPython import display
from matplotlib import pyplot as plt
from mxnet import autograd, nd
import d2lzh
import random

def use_svg_display():
    '''用矢量图表示'''
    display.set_matplotlib_formats('svg')

def set_figsize(figsize=(3.5, 2.5)):
    use_svg_display()
    '''设置图的尺寸'''
    plt.rcParamsp['figure.figsize'] = figsize

def linear_regression_scratch():
    num_inputs = 2
    num_examples = 1000

    true_w = [2, -3.4]
    true_b = 4.2

    features = nd.random.normal(scale=1, shape=(num_examples, num_inputs))
    labels = true_w[0] * features[:, 0] + true_w[1] * features[:, 1] + true_b
    labels += nd.random.normal(scale=0.01, shape=labels.shape)
    print(features[0])


if __name__ == '__main__':
    linear_regression_scratch()