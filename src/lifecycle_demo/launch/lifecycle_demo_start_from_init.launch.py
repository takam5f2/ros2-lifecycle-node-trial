#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os
import sys

from launch import LaunchDescription
from launch import LaunchIntrospector
from launch import LaunchService

from launch_ros import get_default_launch_description
import launch.substitutions
import launch_ros.actions

import lifecycle_msgs.msg

def generate_launch_description():
    """
    just launch talker and listener node.
    but it does not operate any state transition.
    """
    ld = launch.LaunchDescription()
    namespace = 'ns0'
    
    talker_node = launch_ros.actions.LifecycleNode(
        package='lifecycle_demo',
        node_executable='lc_talker',
        node_name='lifecycle_talker',
        node_namespace=namespace,
        output='screen'
    )
    ld.add_action(talker_node)

    listener_node = launch_ros.actions.Node(
        package='lifecycle_demo',
        node_executable='lc_listener',
        node_namespace=namespace,
        output='screen'
    )
    ld.add_action(listener_node)

    return ld


def main(argv=sys.argv[1:]):
    """Main."""
    ld = generate_launch_description()

    print('launch node from main() function')
    print('')

    print(LaunchIntrospector.format_launch_description(ld))

    ls = LaunchService()
    ls.include_launch_description(get_default_launch_description())
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main()
    
