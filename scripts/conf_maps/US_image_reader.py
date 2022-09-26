#!/usr/bin/env python
import rospy
import imfusion
from sensor_msgs.msg import Image
import sys
import numpy as np
from matplotlib import pyplot as plt

def convert_us_images():
    shared_image_set = imfusion.SharedImageSet()
    while not rospy.is_shutdown():

        us_img = rospy.wait_for_message("/imfusion/cephasonics", Image)

        np_img = np.frombuffer(us_img.data, dtype=np.uint8).reshape(us_img.height, us_img.width, -1)
        print(np_img.shape)
        plt.imshow(np_img, interpolation='nearest') # cmap='gray',
        plt.show()
        
        shared_image = imfusion.SharedImage(np_img)
        shared_image_set.add(shared_image)

        if(len(shared_image_set)==2):
            imfusion.executeAlgorithm('Set Modality', [shared_image_set], imfusion.Properties({ 'modality': 4 }))

            # CHANGE THIS TO THE DEPTH OF THE ULTRASOUND!!!!!!!!
            # us_depth = 40
            # spacing_dict = {'spacing':None, 'isMetric': False}
            # spacing_dict['spacing'] = "%.5f" % (us_depth / us_img.height) + ' ' + "%.5f" % (us_depth / us_img.height) + ' 1'
            # print(spacing_dict['spacing'])
            imfusion.executeAlgorithm('Set Spacing', [shared_image_set], imfusion.Properties({'spacing': '0,06545 0,06545 1', 'isMetric': False}))
            print(shared_image_set[0])
            
            # compute_conf_map(shared_image_set)
            imfusion.executeAlgorithm('Change Selection', [shared_image_set], imfusion.Properties({'first': 0, 'last': 1}))
            print("computing conf maps")
            confidence_maps_image_set = imfusion.executeAlgorithm('Ultrasound;Compute Confidence Maps', [shared_image_set],\
                imfusion.Properties({ 'Working Resolution': 0.5,
                            'Use Superpixels':0,
                            'Fast Mode':0, 
                            'Adaptiveness':100, }))
            print(confidence_maps_image_set)
            shared_image_set = imfusion.SharedImageSet()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


def compute_conf_map(data):
    imfusion.executeAlgorithm('Change Selection', [data], imfusion.Properties({'first': 0, 'last': 1}))
    print("computing conf maps")
    confidence_maps_image_set = imfusion.executeAlgorithm('Ultrasound;Compute Confidence Maps', [data],\
        imfusion.Properties({ 'Working Resolution': 0.5,
                            'Use Superpixels':0,
                            'Fast Mode':0, 
                            'Adaptiveness':100, }))
    print(confidence_maps_image_set)
    confidence_map = confidence_maps_image_set[0]
    arr = np.array(confidence_map[0])
    plt.imshow(arr, interpolation='nearest')
    plt.show()

    for i in range(len(confidence_maps_image_set)-1):
        conf_map_publisher.publish(confidence_maps_image_set[i])


def main(args):
    rospy.init_node('image_converter', anonymous=True)
    convert_us_images()

if __name__ == '__main__':
    imfusion.init()
    conf_map_publisher = rospy.Publisher('confidenceMaps', Image, queue_size=15)
    main(sys.argv)


