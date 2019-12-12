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
        node_namespace=namespace,
        node_name='lc_talker',
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

    # change state from uncofigured to configured (inactive)
    configure_lc_talker_action = launch.actions.RegisterEventHandler(
        launch.event_handlers.on_process_start.OnProcessStart(
            target_action=talker_node,
            on_start=[
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(talker_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )
    ld.add_action(configure_lc_talker_action)
    
    # change state from inactive to active
    activate_lc_talker_action = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=talker_node,
            start_state='configuring', goal_state='inactive',
            entities=[
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(talker_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )
    ld.add_action(activate_lc_talker_action)

    # change state from active to deactivate after 10 sec passes in active state.
    deactivate_lc_talker_action = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=talker_node,
            start_state='activating', goal_state='active',
            entities=[
                launch.actions.TimerAction(
                    period=10.0, actions=[
                        launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                            lifecycle_node_matcher=launch.events.matches_action(talker_node),
                            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_DEACTIVATE,
                        )),
                    ]),
            ],
        )
    )
    ld.add_action(deactivate_lc_talker_action) 

    # cleanup state
    cleanup_lc_talker_action = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=talker_node,
            start_state='deactivating', goal_state='inactive',
            entities=[
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(talker_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CLEANUP,
                )),
            ],
        )
    )
    ld.add_action(cleanup_lc_talker_action)

    # shutdown this lc_talker node.
    shutdown_lc_talker_action = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=talker_node,
            start_state='cleaningup', goal_state='unconfigured',
            entities=[
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(talker_node),
                    transition_id=(
                        lifecycle_msgs.msg.Transition.TRANSITION_UNCONFIGURED_SHUTDOWN
                    ),
                )),
            ],
        )
    )
    ld.add_action(shutdown_lc_talker_action)

    # terminate this process.
    shutdown_this_process = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=talker_node,
            start_state='shuttingdown', goal_state='finalized',
            entities=[
                # launch.actions.Shutdown(
                #    reason="Terminates this launch file due to landing all states."
                #)
                launch.actions.EmitEvent(event=launch.events.Shutdown(
                    reason="Terminates this launch process due to landing all states."
                ))
            ]
        )
    )
    ld.add_action(shutdown_this_process)

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
    
