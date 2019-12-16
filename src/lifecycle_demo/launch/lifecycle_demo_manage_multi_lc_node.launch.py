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

    node_num = 8
    talker_node_array = list()
    listener_node_array = list()
    for i in range(node_num):
        namespace = 'ns{}'.format(i)

        talker_node_array.append(launch_ros.actions.LifecycleNode(
            package='lifecycle_demo',
            node_executable='lc_talker',
            node_name='lifecycle_talker',
            node_namespace=namespace,
            output='screen'
        ))
        ld.add_action(talker_node_array[i])

        listener_node_array.append(launch_ros.actions.Node(
            package='lifecycle_demo',
            node_executable='lc_listener',
            node_namespace=namespace,
            output='screen'
        ))
        ld.add_action(listener_node_array[i])

        # change all node state from unconfigured to inactive.
    for i in range(0, node_num):
        ld.add_action(launch.actions.RegisterEventHandler(
            launch.event_handlers.on_process_start.OnProcessStart(
                target_action=talker_node_array[i],
                on_start=[
                    launch.actions.EmitEvent(
                        event=launch_ros.events.lifecycle.ChangeState(
                            lifecycle_node_matcher=launch.events.matches_action(talker_node_array[i]),
                            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                        )
                    )
                ]
            )
        ))


    # change even number node inactive to active per 5 seconds.
    # change 0th node state from configuring inactive
    ld.add_action(launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=talker_node_array[node_num-1],
            start_state='configuring', goal_state='inactive',
            entities=[
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(talker_node_array[0]),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ]
        )
    ))

    for i in range(2, node_num, 2):
        ld.add_action(launch.actions.RegisterEventHandler(
            launch_ros.event_handlers.OnStateTransition(
                target_lifecycle_node=talker_node_array[i-2],
                start_state='activating', goal_state='active',
                entities=[
                    launch.actions.TimerAction(
                        period=5.0, actions=[
                            launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                                lifecycle_node_matcher=launch.events.matches_action(talker_node_array[i]),
                                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                            )),
                        ],
                    ),
                ],
            )
        ))

    for i in range(0, node_num, 2):
        ld.add_action(launch.actions.RegisterEventHandler(
            launch_ros.event_handlers.OnStateTransition(
                target_lifecycle_node=talker_node_array[i],
                start_state='activating', goal_state='active',
                entities=[
                    launch.actions.TimerAction(
                        period=10.0, actions=[
                            launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                                lifecycle_node_matcher=launch.events.matches_action(talker_node_array[i]),
                                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_DEACTIVATE,
                            )),
                        ],
                    ),
                ],
            )
        ))

    # change odd number node inactive to active per 5 seconds.
    # change 1th node state from configuring to inactive
    ld.add_action(launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=talker_node_array[node_num-4],
            start_state='deactivating', goal_state='inactive',
            entities=[
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(talker_node_array[1]),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                ))
            ]
        )
    ))


    # change 3, 5, 7th node state from inactive to active per 5 seconds.
    def message_output_handler_3(event):
        target_str = "Sending message #5"
        if target_str in event.text.decode():
            return launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                lifecycle_node_matcher=launch.events.matches_action(talker_node_array[3]),
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
            ))

    ld.add_action(launch.actions.RegisterEventHandler(
        launch.event_handlers.OnProcessIO(
            target_action=listener_node_array[1],
            on_stdout=message_output_handler_3,
        )
    ))

    def message_output_handler_5(event):
        target_str = "Sending message #5"
        if target_str in event.text.decode():
            return launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                lifecycle_node_matcher=launch.events.matches_action(talker_node_array[5]),
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
            ))

    ld.add_action(launch.actions.RegisterEventHandler(
        launch.event_handlers.OnProcessIO(
            target_action=listener_node_array[3],
            on_stdout=message_output_handler_5,
        )
    ))

    def message_output_handler_7(event):
        target_str = "Sending message #5"
        if target_str in event.text.decode():
            return launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                lifecycle_node_matcher=launch.events.matches_action(talker_node_array[7]),
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
            ))

    ld.add_action(launch.actions.RegisterEventHandler(
        launch.event_handlers.OnProcessIO(
            target_action=listener_node_array[5],
            on_stdout=message_output_handler_7,
        )
    ))




    for i in range(1, node_num, 2):
        ld.add_action(launch.actions.RegisterEventHandler(
            launch_ros.event_handlers.OnStateTransition(
                target_lifecycle_node=talker_node_array[i],
                start_state='activating', goal_state='active',
                entities=[
                    launch.actions.TimerAction(
                        period=10.0, actions=[
                            launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                                lifecycle_node_matcher=launch.events.matches_action(talker_node_array[i]),
                                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_DEACTIVATE,
                            )),
                        ],
                    ),
                ],
            )
        ))



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
    
