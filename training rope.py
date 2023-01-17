#----------------------------------------------------------
#   Neural Network to predict rope like behaviour
#----------------------------------------------------------

import h5py
import keras
import numpy as np
from keras.models import Sequential
from keras.layers import Dense,Activation
from keras.layers import Dropout
from keras import losses
from keras import optimizers
from keras.models import load_model
from sklearn.utils import shuffle
import matplotlib.pyplot as plt
from sklearn import preprocessing
from keras import regularizers
from keras.callbacks import EarlyStopping
import matplotlib.pyplot as plt

def Classify(x):
    if x[0]>0.5:
        p1=1
    else:
        p1=0
    if x[1]>0.5:
        p2=1
    else:
        p2=0
    if x[2]>0.5:
        p3=1
    else:
        p3=0
    return p1, p2, p3


def load_data(add_input, add_output, add_ExCount, num_data):
    for i in range(1, num_data):
        inputDataTemp = np.load(add_input + str(i) + '.npy')
        outputDataTemp = np.load(add_output+ str(i) + '.npy')
        ExCountTemp = np.load(add_ExCount + str(i) + '.npy')
        if i != 1:
            inputData1 = np.append(inputData1, inputDataTemp[0:ExCountTemp - 2], axis=0)
            outputData1 = np.append(outputData1, outputDataTemp[0:ExCountTemp - 2], axis=0)
        else:
            inputData1 = inputDataTemp[0:ExCountTemp - 2]
            outputData1 = outputDataTemp[0:ExCountTemp - 2]
    return inputData1, outputData1

def RemoveNan(inputData):
    shape = np.shape(inputData)
    index = []
    for i in range(0, shape[0]):
        for j in range(0, 23):
            if np.isnan(inputData1[i, j]):
                temp = i
                index = np.append(index, temp)
    if index != []:
        inputData = np.delete(inputData, index)
        
def preprocess(threshold, inputdata, outputdata):
    outputData=np.zeros((4500,6))
    inputData=np.zeros((4500,23))
    count=0
    for i in range(len(outputData1)):
        if (outputData1[i][0]>=threshold or outputData1[i][0]<=-threshold) or (outputData1[i][2]>=threshold or outputData1[i][2]<=-threshold) or (outputData1[i][4]>=threshold or outputData1[i][4]<=-threshold):
            outputData[count]=outputData1[i]
            inputData[count]=inputData1[i]
            count+=1

    outputData=outputData[0:count]
    inputData=inputData[0:count]
    return outputData, inputData

def create_train_data_label(label, input, testDataIn, testDataOut):
    for i in range(len(outputData)):
        if i < len(label):
            input[i] = inputData[i]
            for n in range(0, 3):
                if outputData[i][2 * n + 1] == 1:
                    label[i][n] = 1
                else:
                    label[i][n] = 0
        else:
            if i - len(label) < len(testDataOut):
                testDataIn[i - len(label)] = inputData[i]
                for n in range(0, 3):
                    if outputData[i][2 * n + 1] == 1:
                        testDataOut[i - len(label)][n] = 1
                    else:
                        testDataOut[i - len(label)][n] = -1

#load training data
add_input = 'D:/internship/robot final files/final/samples/new/newexamples/inputs'
add_output = 'D:/internship/robot final files/final/samples/new/newexamples/outputs'
add_ExCount = 'D:/internship/robot final files/final/samples/new/newexamples/ExCount'
inputData1, outputData1 = load_data(add_input, add_output, add_ExCount, 32)


#preprocess
RemoveNan(inputData1)
outputData, inputData = preprocess(0.1, inputData1, outputData1)
print("input size",np.shape(inputData))
print("output size",np.shape(outputData))

#making train data and label
outputData,inputData=shuffle(outputData,inputData)

for i in range(len(outputData)):
    if outputData[i][1]==0:
        outputData[i][1]=-1
    if outputData[i][3] == 0:
        outputData[i][3] = -1
    if outputData[i][5] == 0:
        outputData[i][5] = -1



label=np.zeros((int(0.9*len(outputData)),3))
input=np.zeros((int(0.9*len(outputData)),23))
print('train size:',len(label))
testDataIn=np.zeros((int(0.1*len(outputData)),23))
testDataOut=np.zeros((int(0.1*len(outputData)),3))
print('test size:',len(testDataOut))
create_train_data_label(label, input, testDataIn, testDataOut)

input=preprocessing.normalize(input)
testDataIn=preprocessing.normalize(testDataIn)
name='D:\\internship\\robot final files\\final\\samples\\inputnewexample.npy'
np.save(name,input)
name='D:\\internship\\robot final files\\final\\samples\\labelnewexample.npy'
np.save(name,label)
name='D:\\internship\\robot final files\\final\\samples\\testDataInnewexample.npy'
np.save(name,testDataIn)
name='D:\\internship\\robot final files\\final\\samples\\testDataOutnewexample.npy'
np.save(name,testDataOut)

input=np.load('D:/internship/robot final files/final/samples/inputnewexample.npy')
label=np.load('D:/internship/robot final files/final/samples/labelnewexample.npy')
testDataIn=np.load('D:/internship/robot final files/final/samples/testDataInnewexample.npy')
testDataOut=np.load('D:/internship/robot final files/final/samples/testDataOutnewexample.npy')



testOut=np.zeros((len(testDataOut),3))
for i in range(len(testDataOut)):
    for n in range(0,3):
        if testDataOut[i][n]==1:
            testOut[i][n]=1



#building the NN
model=Sequential()
model.add(Dense(23,input_shape=(23,),activation='relu'))
model.add(Dense(15,activation='relu'))
#model.add(Dense(10,activation='relu'))
model.add(Dense(3,activation='sigmoid'))


optim=keras.optimizers.Adam(lr=0.01)
model.compile(optimizer=optim,loss='binary_crossentropy', metrics=['accuracy'])
early_stopping = EarlyStopping(monitor='val_loss', patience=2000)
history=model.fit(input,label,epochs=5000,validation_split=0.1,batch_size=50, callbacks=[early_stopping])



prediction=model.predict(input[0:int(0.9*len(input))])
predicted=[]
for x in prediction:
    p1, p2, p3 = Classify(x)
    predicted.append([p1,p2,p3])

count=0
force=0
mis=0
for i in range(len(label[0:int(0.9*len(input))])):
    for n in range(0,3):
        if (predicted[i][n]==label[i][n]):
            count+=1
        if label[i][n]==1:
            force+=1
            if predicted[i][n]!=1:
                mis+=1

print('correct_train:',count, 'force:',force, 'mis:',mis)
print('accuracy_train: ',count/(3*len(label[0:int(0.9*len(input))]))*100,'      positive_accuracy_train: ',(force-mis)/force*100)


prediction=model.predict(testDataIn)
predicted=[]
for x in prediction:
    p1, p2, p3 = Classify(x)
    predicted.append([p1,p2,p3])

count=0
force=0
mis=0
for i in range(len(testOut)):
    for n in range(0,3):
        if (predicted[i][n]==testOut[i][n]):
            count+=1
        if testOut[i][n]==1:
            force+=1
            if predicted[i][n]!=1:
                mis+=1

print('correct:',count, 'force:',force, 'mis:',mis)
print('accuracy_test: ',count/(3*len(testOut))*100,'      positive_accuracy_test: ',(force-mis)/force*100)


#saving model
model.save('D:/internship/robot final files/final/samples/new/newexamples/ropeModel.h5')


plt.plot(history.history['loss'])
plt.plot(history.history['val_loss'])
plt.title('Training Losses')
plt.ylabel('Loss')
plt.xlabel('Epoch')
plt.legend(['Training', 'Validation'], loc='upper left')
plt.show()
