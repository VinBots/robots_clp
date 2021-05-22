#STEP 1: Import all the necessary libraries

import os
import csv
from keras.models import Sequential, Model
from keras.layers import Lambda, Flatten, Dense, Cropping2D, Dropout
from keras.layers.convolutional import Convolution2D
from keras.layers.pooling import MaxPooling2D
from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle
import cv2
import numpy as np
import sklearn
import math

#change the brightness of the image
#INPUT RGB Image 320 x 160
#OUTPUT: RGB Image with a different brightness level
#converting images to HSV, scaling up or down the V channel and converting back to the RGB channel

def augment_brightness_camera_images(image):
    
    image1 = cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
    image1 = np.array(image1, dtype = np.float64)
    random_bright = .5+np.random.uniform()
    image1[:,:,2] = image1[:,:,2]*random_bright
    image1[:,:,2][image1[:,:,2]>255]  = 255
    image1 = np.array(image1, dtype = np.uint8)
    image1 = cv2.cvtColor(image1,cv2.COLOR_HSV2RGB)
    return image1

#shadow augmentation where random shadows are cast across the image
#INPUT RGB Image 320 x 160
#OUTPUT: Shadow augmented RGB Image 320 x 160
def add_random_shadow(image):
    top_y = 320*np.random.uniform()
    top_x = 0
    bot_x = 160
    bot_y = 320*np.random.uniform()
    image_hls = cv2.cvtColor(image,cv2.COLOR_RGB2HLS)
    shadow_mask = 0*image_hls[:,:,1]
    X_m = np.mgrid[0:image.shape[0],0:image.shape[1]][0]
    Y_m = np.mgrid[0:image.shape[0],0:image.shape[1]][1]
    shadow_mask[((X_m-top_x)*(bot_y-top_y) -(bot_x - top_x)*(Y_m-top_y) >=0)]=1
    #random_bright = .25+.7*np.random.uniform()
    if np.random.randint(2)==1:
        random_bright = .5
        cond1 = shadow_mask==1
        cond0 = shadow_mask==0
        if np.random.randint(2)==1:
            image_hls[:,:,1][cond1] = image_hls[:,:,1][cond1]*random_bright
        else:
            image_hls[:,:,1][cond0] = image_hls[:,:,1][cond0]*random_bright    
    image = cv2.cvtColor(image_hls,cv2.COLOR_HLS2RGB)
    return image

def flip_image(image,y_steer):
    ind_flip = np.random.randint(2)
    if ind_flip==0:
        image = cv2.flip(image,1)
        y_steer = -y_steer
    return image, y_steer

def augment(image,steering_angle):
    augmented_image = augment_brightness_camera_images(image)
    augmented_image = add_random_shadow (augmented_image)
    augmented_image, augmented_angle = flip_image(augmented_image,steering_angle)
    return augmented_image,augmented_angle


#STEP 2: Load all the records for left, right and center cameras
#Personal data
#csv_filename = './My_IMG/driving_log.csv'
#path_records_images = "./My_IMG/IMG/"

#Udacity data
csv_filename = './data/driving_log.csv'
path_records_images = "./data/IMG/"

# Set our batch size
batch_size=50

correction_steering_left = 0.25
correction_steering_right = -0.25

records = []

with open(csv_filename) as csvfile:
    reader = csv.reader(csvfile)
    next(reader)
    for record_per_timestep in reader:
        records.append(record_per_timestep)   

#split the training set and the validation set
train_records, validation_records = train_test_split(records, test_size=0.2)


def generator(samples, batch_size=32,training=False):
    num_samples = len(samples)
    while 1: # Loop forever so the generator never terminates
        shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]

            images = []
            angles = []
            
            for batch_sample in batch_samples:
                name = path_records_images +batch_sample[0].split('/')[-1]
                #print (name)
                center_image = cv2.cvtColor(cv2.imread(name),cv2.COLOR_BGR2RGB)
                center_angle = float(batch_sample[3])
                
                images.append(center_image)
                angles.append(center_angle)

                if training == True:
                    # add 2 images from the left and right camera
                    
                    name = path_records_images +batch_sample[1].split('/')[-1]
                    side_image = cv2.cvtColor(cv2.imread(name),cv2.COLOR_BGR2RGB)
                    side_angle = float(batch_sample[3]) + correction_steering_left
                    
                    images.append(side_image)
                    angles.append(side_angle)

                    name = path_records_images +batch_sample[2].split('/')[-1]
                    side_image = cv2.cvtColor(cv2.imread(name),cv2.COLOR_BGR2RGB)
                    side_angle = float(batch_sample[3]) + correction_steering_right                    
                
                    images.append(side_image)
                    angles.append(side_angle)
                
                    # add 2 augmented images
                    for i in range(2):
                        new_augmented_image, new_augmented_angle = augment(center_image,center_angle)
                        images.append(new_augmented_image)
                        angles.append(new_augmented_angle)

            # trim image to only see section with road
            X_train = np.array(images)
            y_train = np.array(angles)
            
            yield sklearn.utils.shuffle(X_train, y_train)



# compile and train the model using the generator function
train_generator = generator(train_records, batch_size=batch_size,training=True)
validation_generator = generator(validation_records, batch_size=batch_size)

#set up lamba Layer
model = Sequential()

model.add(Cropping2D(cropping=((70,25),(0,0)),input_shape = (160,320,3)))
#normalization btw -0.5 and 0.5, mean 0
model.add(Lambda(lambda x: x/255.-0.5,input_shape=(65,320,3)))

model.add(Convolution2D(3,1,1,subsample=(1,1),activation="relu"))

model.add(Convolution2D(24,5,5,subsample=(2,2),activation="relu"))
model.add(Dropout(0.5))
model.add(Convolution2D(36,5,5,subsample=(2,2),activation="relu"))
model.add(Dropout(0.5))
model.add(Convolution2D(48,5,5,subsample=(2,2),activation="relu"))
model.add(Convolution2D(64,3,3,activation="relu"))
model.add(Convolution2D(64,3,3,activation="relu"))

model.add(Flatten())
model.add(Dense(100))
model.add(Dense(50))
model.add(Dense(10))
model.add(Dense(1))

model.compile(loss='mse', optimizer='adam')

model.fit_generator(train_generator,
            steps_per_epoch=math.ceil(len(train_records)/batch_size),
            validation_data=validation_generator,
            validation_steps=math.ceil(len(validation_records)/batch_size),
            epochs=5, verbose=1)
model.summary()          
model.save('model11.h5')
print ("Model Saved")
exit()