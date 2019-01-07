import tensorflow as tf
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib as mpl
import sys
import csv
import gc
from multiprocessing import Pool
import multiprocessing
import time
import os

input_dimension_xy = 64
input_dimension_z = 24

batch_size = 20
learning_rate = 1e-4
total_epoches = 1000
save_every_n_epoch = 100
times_per_file = 1

model_save_path = "/home/ubuntu/chg_workspace/3dcnn/model/auto_encoder/encoder_003/model/"
image_save_path = "/home/ubuntu/chg_workspace/3dcnn/model/auto_encoder/encoder_003/plots/"

path = "/home/ubuntu/chg_workspace/data/new_csvs/new_map/encoder/training"
clouds_filename = ["pcl_data_2018_12_15_10:51:54.csv",
                   "pcl_data_2018_12_15_10:59:07.csv",
                   "pcl_data_2018_12_15_11:33:52.csv",
                   "pcl_data_2018_12_15_11:37:17.csv",
                   "pcl_data_2018_12_15_11:40:32.csv",
                   "pcl_data_2018_12_15_11:47:35.csv",
                   "pcl_data_2018_12_15_11:51:31.csv",
                   "pcl_data_2018_12_15_11:56:45.csv"
                   ]

file_path_list_pcl = [os.path.join(path, cloud) for cloud in clouds_filename]

img_wid = input_dimension_xy
img_height = input_dimension_z

encoder_para = {
    "kernel1": 5,
    "stride_xy1": 2,
    "stride_z1": 3,
    "channel1": 32,
    "pool1": 2,
    "kernel2": 3,
    "stride_xy2": 2,
    "stride_z2": 2,
    "channel2": 64,
    "kernel3": 3,
    "stride_xy3": 2,
    "stride_z3": 2,
    "channel3": 128,
    "out_dia": 2048
}


def conv3d_relu(x, kernel_shape, bias_shape, strides):
    weights = tf.get_variable("weights_con", kernel_shape, initializer=tf.truncated_normal_initializer(stddev=0.1))
    biases = tf.get_variable("bias_con", bias_shape, initializer=tf.constant_initializer(0.0))
    conv = tf.nn.conv3d(x, weights, strides=strides, padding="SAME")
    return tf.nn.relu(conv + biases)


def deconv3d(x, kernel_shape, output_shape, strides):
    weights = tf.get_variable("weights_con", kernel_shape, initializer=tf.truncated_normal_initializer(stddev=0.1))
    return tf.nn.conv3d_transpose(x, filter=weights, output_shape=output_shape, strides=strides)


def max_pool(x, kernel_shape, strides):
    return tf.nn.max_pool3d(x, ksize=kernel_shape, strides=strides, padding='SAME')


def encoder(x):
    k1 = encoder_para["kernel1"]
    sxy1 = encoder_para["stride_xy1"]
    sz1 = encoder_para["stride_z1"]
    d1 = encoder_para["channel1"]
    p1 = encoder_para["pool1"]

    k2 = encoder_para["kernel2"]
    sxy2 = encoder_para["stride_xy2"]
    sz2 = encoder_para["stride_z2"]
    d2 = encoder_para["channel2"]

    k3 = encoder_para["kernel3"]
    sxy3 = encoder_para["stride_xy3"]
    sz3 = encoder_para["stride_z3"]
    d3 = encoder_para["channel3"]

    with tf.variable_scope("encoder"):
        with tf.variable_scope("conv1"):
            conv1 = conv3d_relu(x, [k1, k1, k1, 1, d1], [d1], [1, sxy1, sxy1, sz1, 1])

        with tf.variable_scope("pool1"):
            max_pool1 = max_pool(conv1, [1, p1, p1, p1, 1], [1, p1, p1, p1, 1])

        with tf.variable_scope("conv2"):
            conv2 = conv3d_relu(max_pool1, [k2, k2, k2, d1, d2], [d2], [1, sxy2, sxy2, sz2, 1])

        with tf.variable_scope("conv3"):
            conv3 = conv3d_relu(conv2, [k3, k3, k3, d2, d3], [d3], [1, sxy3, sxy3, sz3, 1])
            return conv3


def decoder(x, batch_size):
    k1 = encoder_para["kernel1"]
    sxy1 = encoder_para["stride_xy1"]
    sz1 = encoder_para["stride_z1"]
    d1 = encoder_para["channel1"]
    p1 = encoder_para["pool1"]

    k2 = encoder_para["kernel2"]
    sxy2 = encoder_para["stride_xy2"]
    sz2 = encoder_para["stride_z2"]
    d2 = encoder_para["channel2"]

    k3 = encoder_para["kernel3"]
    sxy3 = encoder_para["stride_xy3"]
    sz3 = encoder_para["stride_z3"]
    d3 = encoder_para["channel3"]

    size_1 = [batch_size, 64, 64, 24, d1]
    size_2 = [batch_size, 16, 16, 4, d2]
    size_3 = [batch_size, 8, 8, 2, d3]

    special_sxy1 = sxy1 * p1
    special_sz1 = sz1 * p1

    # Use conv to decrease kernel number. Use deconv to enlarge map

    with tf.variable_scope("decoder"):
        with tf.variable_scope("conv0"):  # Middle layer, change nothing
            conv0 = conv3d_relu(x, [k3, k3, k3, d3, d3], [d3], [1, 1, 1, 1, 1])
            print "conv0 ", conv0

        with tf.variable_scope("deconv0"):
            deconv0 = deconv3d(conv0, [sxy3, sxy3, sz3, d3, d3], output_shape=size_3, strides=[1, sxy3, sxy3, sz3, 1])
            print "deconv0", deconv0.get_shape()
        with tf.variable_scope("conv1"):
            conv1 = conv3d_relu(deconv0, [k3, k3, k3, d3, d2], [d2], [1, 1, 1, 1, 1])

        with tf.variable_scope("deconv1"):
            deconv1 = deconv3d(conv1, [sxy2, sxy3, sz3, d2, d2], output_shape=size_2, strides=[1, sxy2, sxy2, sz2, 1])
            print "deconv1", deconv1.get_shape()
        with tf.variable_scope("conv2"):
            conv2 = conv3d_relu(deconv1, [k2, k2, k2, d2, d1], [d1], [1, 1, 1, 1, 1])

        with tf.variable_scope("deconv2"):  # special, consider pooling and stride in conv1
            deconv2 = deconv3d(conv2, [special_sxy1, special_sxy1, special_sz1, d1, d1], output_shape=size_1, strides=[1, special_sxy1, special_sxy1, special_sz1, 1])
            print "deconv2", deconv2.get_shape()
        with tf.variable_scope("conv3"):
            conv3 = conv3d_relu(deconv2, [1, 1, 1, d1, d1], [1], [1, 1, 1, 1, 1])
            print "conv3", conv3.get_shape()

        with tf.variable_scope("conv4"):
            conv4 = conv3d_relu(conv3, [k1, k1, k1, d1, 1], [1], [1, 1, 1, 1, 1])
            return conv4


def generate_shuffled_array(start, stop, shuffle=True):
    """
    Give a length and return a shuffled one dimension array using data from start to stop, stop not included
    Used as shuffled sequence
    """
    array = np.arange(start, stop)
    if shuffle:
        np.random.shuffle(array)
    return array


def get_bacth_step(seq, time_step, data):
    """
    get values of the seq in data(array), together with time_step back values
    :param seq: sequence to get, 0 or positive integers in one dimention array
    :param time_step: 2 at least
    :param data: data to get, must be numpy array!!!, at least 2 dimension
    :return: list [seq_size*time_step, data_size:] typical(if values in seq are all valid).
    """
    shape = list(data.shape)
    shape[0] = seq.shape[0] * time_step
    result = np.zeros(shape)
    step = time_step - 1

    gc.disable()

    for k in range(seq.shape[0]):
        for j in range(-step, 1, 1):
            result[k*time_step+step+j, :] = data[seq[k] + j, :]

    gc.enable()
    return result


def get_bacth(seq, data):
    """
    get values of the seq in data(array), together with time_step back values
    :param seq: sequence to get, 0 or positive integers in one dimention array
    :param data: data to get, must be numpy array!!!, at least 2 dimension
    :return: list [seq_size*time_step, data_size:] typical(if values in seq are all valid).
    """
    shape = list(data.shape)
    shape[0] = seq.shape[0]
    result = np.zeros(shape)

    gc.disable()
    for k in range(seq.shape[0]):
        result[k, :] = data[seq[k], :]
    gc.enable()

    return result


# draw by axis z direction
def compare_img_save_3d_to_2d(data1, data2, min_val, max_val, rows, cols, step, name):
    """
    To compare two 3 dimension array by image slices and save the image
    :param data1: data to compare, 3 dimension array
    :param data2: should have the same size as data1
    :param min_val: minimum value in data1 and data2
    :param max_val: maximum value in data1 and data2
    :param rows: row number of the figure
    :param cols: col number of the figure
    :param step: step in z axis to show
    :param name: path + name of the image to save
    :return:
    """
    colors = ['purple', 'yellow']
    bounds = [min_val, max_val]
    cmap = mpl.colors.ListedColormap(colors)
    norm = mpl.colors.BoundaryNorm(bounds, cmap.N)

    f, a = plt.subplots(rows, cols, figsize=(cols, rows))

    # for scale
    data1_copy = np.array(tuple(data1))
    data2_copy = np.array(tuple(data2))
    data1_copy[0, 0, :] = min_val
    data2_copy[0, 0, :] = min_val
    data1_copy[0, 1, :] = max_val
    data2_copy[0, 1, :] = max_val

    for i in range(cols):
        for j in range(rows / 2):
            a[2 * j][i].imshow(data1_copy[:, :, (j * cols + i) * step])
            a[2 * j + 1][i].imshow(data2_copy[:, :, (j * cols + i) * step])

    #plt.show(cmap=cmap, norm=norm)
    plt.savefig(name)
    plt.cla
    plt.close()


def read_pcl_threading(filename_pcl, flags, house):
    maxInt = sys.maxsize
    decrement = True

    clouds = open(filename_pcl, "r")
    img_num = len(clouds.readlines())
    clouds.close()
    data_pcl = np.zeros([img_num, img_wid, img_wid, img_height, 1])

    while decrement:
        # decrease the maxInt value by factor 10
        # as long as the OverflowError occurs.
        decrement = False
        try:
            print "begin read pcl data.."
            csv.field_size_limit(maxInt)

            with open(filename_pcl, mode='r') as csvfile:
                csv_reader = csv.reader(csvfile, delimiter=',', quotechar='|')
                i_row = 0
                for row in csv_reader:
                    for i in range(img_wid):
                        for j in range(img_wid):
                            for k in range(img_height):
                                data_pcl[i_row, i, j, k, 0] = row[i * img_wid * img_height + j * img_height + k]
                    i_row = i_row + 1
                # list_result.append(data)
        except OverflowError:
            maxInt = int(maxInt / 10)
            decrement = True

    # Set data to house
    isLookingForFreeSpace = True
    while isLookingForFreeSpace:
        time.sleep(0.05)
        for i_flag in range(len(flags)):
            if flags[i_flag] == 0:
                flags[i_flag] = 1
                print "found available space, copy data... "
                house[i_flag] = data_pcl
                flags[i_flag] = 2
                isLookingForFreeSpace = False
                break


def tf_training(data_read_flags, data_house, file_num):
    '''Training'''
    dia_xy = input_dimension_xy
    dia_z = input_dimension_z
    x_ = tf.placeholder("float", shape=[None, dia_xy, dia_xy, dia_z, 1])

    encode_vector = encoder(x_)

    print "encode_vector: ", encode_vector.get_shape()
    decode_result = decoder(encode_vector, batch_size)

    loss = tf.reduce_mean(tf.square(x_ - decode_result))
    train_step = tf.train.AdamOptimizer(learning_rate).minimize(loss)

    print "Start training"

    with tf.Session() as sess:
        saver = tf.train.Saver()
        sess.run(tf.global_variables_initializer())

        for epoch in range(total_epoches):
            print "epoch: " + str(epoch)

            for file_seq in range(file_num):
                # Check flags to find data
                isLookingForData = True
                data_mat = np.array(0)
                while isLookingForData:
                    time.sleep(0.05)
                    for i_flag in range(len(data_read_flags)):
                        if data_read_flags[i_flag] == 2:
                            print "found available data.. "
                            data_mat = data_house[i_flag]
                            data_read_flags[i_flag] = 0
                            isLookingForData = False
                            # print "get data: ", data_mat[0]
                            break
                print "done looking for data.."

                # get a random sequence for this file
                for times in range(times_per_file):
                    sequence = generate_shuffled_array(0, data_mat.shape[0], shuffle=True)
                    batch_num = int(data_mat.shape[0] / batch_size)
                    # start batches
                    for batch_seq in range(batch_num):
                        # print "batch:" + str(batch_seq)
                        # get data for this batch
                        start_position = batch_seq * batch_size
                        end_position = (batch_seq + 1) * batch_size
                        batch_data = get_bacth(sequence[start_position:end_position], data_mat)

                        sess.run(train_step, feed_dict={x_: batch_data})  # training

            print "epoch: " + str(epoch)
            print('loss=%s' % sess.run(loss, feed_dict={x_: batch_data}))

            if epoch % 10 == 0:
                decode_pcl = sess.run(decode_result, feed_dict={x_: batch_data})
                print "batch_data", batch_data[0, :, :, :, 0]
                save_name = image_save_path + "epoch_" + str(epoch) + ".png"
                compare_img_save_3d_to_2d(batch_data[0, :, :, :, 0], decode_pcl[0, :, :, :, 0], 0, 1, 4, 12, 1, save_name)

            if epoch % save_every_n_epoch == 0:
                saver.save(sess,
                           model_save_path + "simulation_autoencoder_" + str(epoch) + ".ckpt")


if __name__ == '__main__':

    '''Data reading'''
    print "Reading data..."
    pool = Pool(processes=8)

    data_read_flags = multiprocessing.Manager().list([0, 0, 0, 0])
    data_house = multiprocessing.Manager().list([0, 0, 0, 0])

    # Training thread
    pool.apply_async(tf_training, args=(data_read_flags, data_house, len(file_path_list_pcl)))

    # Data Reading Thread
    file_list_origin = np.array(file_path_list_pcl)
    for i_epoch in range(total_epoches):
        np.random.shuffle(file_list_origin)
        file_path_list_pcl = file_list_origin.tolist()

        for i_pool in range(file_list_origin.shape[0]):
            # pool.apply_async(test)
            filename_pcl = file_path_list_pcl.pop()
            pool.apply_async(read_pcl_threading, args=(filename_pcl, data_read_flags, data_house))

    pool.close()
    pool.join()



