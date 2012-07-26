import rosbag
from lfd import bag_proc
bag = rosbag.Bag("/home/joschu/Data/bags/test_rope.bag")
bag_proc.get_button_presses(bag)