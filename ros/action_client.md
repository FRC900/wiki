The Code
--------

```py
#!/usr/bin/env python3
import rospy
from __future__ import print_function
# Brings in the SimpleActionClient
import actionlib
# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import beginner_tutorials.msg
def fibonacci_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('fibonacci', beginner_tutorials.msg.FibonacciAction)
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    # Creates a goal to send to the action server.
    goal = beginner_tutorials.msg.FibonacciGoal(order=20)
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult
if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fibonacci_client_py')
        result = fibonacci_client()
        print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
```

The Code, explained
-------------------

```py
import beginner_tutorials.msg
```

The action specification generates several messages for sending goals, receiving feedback, etc... This line imports the generated messages.

```py
    client = actionlib.SimpleActionClient('fibonacci', beginner_tutorials.msg.FibonacciAction)
```

The action client and server communicate over a set of topics, described in [the actionlib protocol](http://wiki.ros.org/actionlib/DetailedDescription). The action name describes the namespace containing these topics, and the action specification message describes what messages should be passed along these topics.

```py
    client.wait_for_server()
```

Sending goals before the action server comes up would be useless. This line waits until we are connected to the action server.

```py
    # Creates a goal to send to the action server.
    goal = beginner_tutorials.msg.FibonacciGoal(order=20)
    # Sends the goal to the action server.
    client.send_goal(goal)
```

Creates a goal and sends it to the action server.

```py
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult
```

The action server will process the goal and eventually terminate. We want the result from the termination, but we wait until the server has finished with the goal.

Running the Client
------------------

Before running the client, we assume roscore ans Action server are already running from [previous page](action_server.md)

Start the client. It will start up, send a goal to the server, wait for the goal to complete, and then exit.

In a Docker terminal, run:
```bash
rosstd
rosrun beginner_tutorials fibonacci_client.py
```