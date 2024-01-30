import rosbag
bag = rosbag.Bag('bag_still.bag')
for topic, msg, t in bag.read_messages(topics=['wire']):
    print(msg)
bag.close()
