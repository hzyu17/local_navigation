import os
import time
import multiprocessing.pool as Pool


base_save_path = os.getcwd()


def multi_bagplay_func(play_cmd):
    os.popen("gnome-terminal -x %s" % play_cmd)


def do_one_file(_, dirname, names):
    if not '.bag' in names[0]:
        pass
    else:
        # pool = Pool.Pool(processes=2)
        dir = os.path.split(dirname)[-1]
        bags_name = ''
        if not os.path.exists(dir):
            os.makedirs(dir)
        cd_cmd = "cd %s" % dir
        for file in names:
            file = os.path.join(dirname, file)
            bags_name = bags_name + file + ' '
        bag_play_cmd = "rosbag play -r 0.9 %s" % bags_name
        cmd_comple = "%s && gnome-terminal -x %s && rosrun uavInfo processDataOrigin" % (cd_cmd, bag_play_cmd)

        os.system("%s" % cmd_comple)
        time.sleep(0.05)
        # pool.apply_async(func=multi_bagplay_func, args=bag_play_cmd)
        # pool.close()
        # pool.join()


if __name__ == '__main__':
    base_save_path = os.getcwd()
    top_bag_files = "/home/lucasyu/YU/decision_RNN/test_random_target/short"
    os.path.walk(top_bag_files, do_one_file, ())













    # cur_filename = []
    # for root, dirs, files in os.walk(top_bag_files, topdown=True):
    #     for name in dirs:
    #         if not os.path.exists(name):
    #             os.popen("mkdir %s" % name)
    #         else:
    #             print("path: %s exists, please check !!" % name)
    #     for f_name in files:
    #         dir = os.path.split(root)[-1]
    #         complete_path = os.path.join(root, f_name)
    #         os.system("cd %s" % dir)
    #         time_begin = time.time()
    #         os.popen("rosbag play -r 0.9 %s" % complete_path)
    #         time_elapsed = time.time() - time_begin
    #         print("time used: %s" % time_elapsed)
            # os.popen("source '/home/lucasyu/ros_test_ws/devel/setup.bash' && rosrun uavInfo processDataOrigin")

        # print(root, dirs, files)
        # print (dirs)
        # path_to_save = os.path.join(base_save_path,dirs)
        # os.system("mkdir -p ")
	# ls = os.popen(
	# 	"cd path_mkdir && mkdir test && "
     #    "gnome-terminal -x  rosbag play -r 0.9 '/home/lucasyu/2018-01-28-14-12-19.bag'")
	# print(ls)
	# if ls:
	# 	print("time to exit...")
	# 	os.system("exit")
