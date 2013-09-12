''' This python decoder works in the following way:
First, you have to run a classifier using offline data
based on the classifier, the correct features have to be send to the python side
The python side uses a pickle_class file that has the following information
pickle_class:
    Is a file that has a dictionary of classifiers, 
    the key of each dictioanry describes the features used to train such classifier
    The classifier is a sklearn class of logistic regression
    The class is already capable of doing classification and you can also access the features from within
'''
import zmq
import numpy as np
import pickle
import collections
pickle_filename = 'pickle_class_8_12_large_lapl.p'
#pickle_filename = 'pickle_class_1_50_large_lapl.p'
clf_dict = pickle.load(open(pickle_filename, "rb"))
#opening the pickle object to load the classifiers
# ZMQ connections, 3 threads
online_clf = clf_dict['Power']
context = zmq.Context(3)
'''
List of Connections

Graphic Interface: tcp://127.0.0.1:50000
Python Send: tcp://127.0.0.1:50001
Python Receive: tcp://127.0.0.1:50002
EEG_module: tcp://192.168.56.101:5556


'''
bsocket = context.socket(zmq.PUB)
bsocket.bind("tcp://127.0.0.1:5558")

#
fsocket = context.socket(zmq.SUB)
fsocket.connect ("tcp://127.0.0.1:5557")
fsocket.setsockopt(zmq.SUBSCRIBE, '' ) # subscribe with no filter, receive all messages
zmq_msg = "1"
feat_buff = collections.deque(maxlen = 50) #here we set the length of the smoothing
past_value = 0

while True:
    try:
            #bsocket.send(zmq_msg);
            #print 'sent'
            features = fsocket.recv()
            # if received data
            print 'received'
            if features != zmq.EAGAIN:
                    # convert string array to numpy array
                    features_vec = [x for x in features.split(',')]
                    del features_vec[-1] #elminate the end of line character 
                    features_vec_array=np.array([float(x) for x in features_vec])
            zmq_sample = np.atleast_2d(features_vec_array)
            #feat_buff.append(zmq_sample - past_value)#This buffers adds the differences over time
            #data_sample = np.hstack((zmq_sample, np.mean(feat_buff,0))) #here we calculate the mean to calculate the smoothing
            data_sample = zmq_sample[:,0:60]
            print data_sample.shape
            #data_sample = np.mean(feat_buff,0)
            #print feat_buff
            past_value = zmq_sample
            predict_value = online_clf.predict(data_sample)
            print predict_value
            zmq_msg=str(predict_value[0])
            #zmq_msg = str(1.0)
	    print zmq_msg
	    print 'Connection'
            bsocket.send(zmq_msg);
            #raw_input()
    except KeyboardInterrupt:
        pass
        print "No Connections"
        break
    except (KeyboardInterrupt, SystemExit):
        print '\nkeyboardinterrupt found!'
        print '\n...Program Stopped Manually!'
        raise

