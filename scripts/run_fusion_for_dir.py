
import re
from pathlib import Path
from argparse import ArgumentParser

import roslaunch
import rospy
from std_msgs.msg import Bool

from bondpy import bondpy

class confirm_callback():
    def __init__(self):
        self.fusion_done = False

        self.confirm_subscriber = \
            rospy.Subscriber("/fusion_complete_confirm",
                Bool, self.callback)

    def reset(self):
        self.fusion_done = False

    def callback(self, data):

        rospy.loginfo("CONFIRM CALLBACK")

        if data is False:
            rospy.logwarn("Fusion ended but was not complete!")

        self.fusion_done = True

def main(source_dir, launch_file):

    package = "panoptic-3d"

    scenes = list(source_dir.glob('scene*'))
    n_scenes = len(scenes)

    #id = 1
    scenes.sort( )

    confirm_listener = confirm_callback()

    for i, d in enumerate(scenes):

        '''
        id += 1
        bond = bondpy.Bond("publisher_bond", str(id))
        bond.start()
        '''

        rospy.init_node('run_fusion', anonymous=True)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        '''
        launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
        '''

        data_arg = 'data_root:='+str(d.resolve())

        '''
        bond_arg = 'bond_id:='+str(id)
        '''

        cli_args = [launch_file, data_arg]#, bond_arg]
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

        launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

        launch.start()
        rospy.loginfo(f"Fusion of {d.name} started")

        '''
        if not bond.wait_until_formed(rospy.Duration(60.0)):
            raise Exception('Bond could not be formed with publisher node')

        bond.wait_until_broken()
        rospy.loginfo(f"{d.name} : bond broken")
        '''

        while not confirm_listener.fusion_done:
            rospy.sleep(1)

        confirm_listener.reset()
        launch.shutdown()
        rospy.sleep(5)

if __name__ == '__main__':

    launch_default = Path("../launch/fusion_from_dir.launch")
    launch_default = str(launch_default.resolve())

    parser = ArgumentParser()

    parser.add_argument("-s", "--source_dir", type=Path, dest="source_dir",
                        help="Path to ScanNet scans root")

    parser.add_argument("-l", "--launch_file", type=str, dest="launch_file",
                        default=launch_default, help="")

    args = parser.parse_args()

    main(**vars(args))
