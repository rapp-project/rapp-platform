#!/usr/bin/env python

#Copyright 2015 RAPP

#Licensed under the Apache License, Version 2.0 (the "License");
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at

    #http://www.apache.org/licenses/LICENSE-2.0

#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS,
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#See the License for the specific language governing permissions and
#limitations under the License.

# Author: Athanassios Kintsakis
# contact: akintsakis@issel.ee.auth.gr
 
import rospy
import sys
import numpy as np
import matplotlib.pyplot as plt
import time
from os.path import expanduser

from rapp_platform_ros_communications.srv import (
  imageClassificationSrv,
  imageClassificationSrvResponse  
  )

from rapp_platform_ros_communications.msg import (
  StringArrayMsg
  )


class ImageClassification:
  
  def classifyImage(self,req):
    res = imageClassificationSrvResponse()
        
    
    caffe_root = expanduser("~")+'/rapp_platform/caffe/'  # this file is expected to be in {caffe_root}/examples
    import sys
    sys.path.insert(0, caffe_root + 'python')
    print np.__version__
    
    import os    
    os.environ['GLOG_minloglevel'] = '2'
    import caffe
    
    start_time = time.time()
    #plt.rcParams['figure.figsize'] = (10, 10)
    #plt.rcParams['image.interpolation'] = 'nearest'
    #plt.rcParams['image.cmap'] = 'gray'



    import os
    if not os.path.isfile(caffe_root + 'models/bvlc_reference_caffenet/bvlc_reference_caffenet.caffemodel'):
        print("Downloading pre-trained CaffeNet model...")
        os.system(caffe_root+"scripts/download_model_binary.py ../models/bvlc_reference_caffenet")
        #!../scripts/download_model_binary.py ../models/bvlc_reference_caffenet




    caffe.set_mode_cpu()
    net = caffe.Net(caffe_root + 'models/bvlc_reference_caffenet/deploy.prototxt',
                    caffe_root + 'models/bvlc_reference_caffenet/bvlc_reference_caffenet.caffemodel',
                    caffe.TEST)

    # input preprocessing: 'data' is the name of the input blob == net.inputs[0]
    transformer = caffe.io.Transformer({'data': net.blobs['data'].data.shape})
    transformer.set_transpose('data', (2,0,1))
    transformer.set_mean('data', np.load(caffe_root + 'python/caffe/imagenet/ilsvrc_2012_mean.npy').mean(1).mean(1)) # mean pixel

    transformer.set_raw_scale('data', 255)  # the reference model operates on images in [0,255] range instead of [0,1]
    transformer.set_channel_swap('data', (2,1,0))  # the reference model has channels in BGR order instead of RGB

    # set net to batch size of 50
    net.blobs['data'].reshape(50,3,227,227)

    
    net.blobs['data'].data[...] = transformer.preprocess('data', caffe.io.load_image(req.objectFileUrl))
    
    out = net.forward()
    print("Predicted class is #{}.".format(out['prob'][0].argmax()))
    

    # load labels
    imagenet_labels_filename = caffe_root + 'data/ilsvrc12/synset_words.txt'
    try:
        labels = np.loadtxt(imagenet_labels_filename, str, delimiter='\t')
    except:
        #!../data/ilsvrc12/get_ilsvrc_aux.sh #script to download labels
        labels = np.loadtxt(imagenet_labels_filename, str, delimiter='\t')

    # sort top k predictions from softmax output
    top_k = net.blobs['prob'].data[0].flatten().argsort()[-1:-6:-1]
    print labels[top_k]
    print "fist choice was :" + labels[top_k][0]
    res.objectClass=labels[top_k][0]
    res.success=True
    #res.objectClass=("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Predicted class is #{}.".format(out['prob'][0].argmax()))
    print("--- %s seconds elapsed---" % (time.time() - start_time))
    
    
    return res




