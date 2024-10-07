Writing a Simple Server
-----------------------

The code and examples used in this tutorial can be found in the [actionlib_tutorials](http://wiki.ros.org/actionlib_tutorials) package. You may want to read about the [actionlib](http://wiki.ros.org/actionlib) package before starting this tutorial.

### The Code

Put this code in `beginner_tutorials/src/fibonacci_server.py`:

```py
#!/usr/bin/env python3
import rospy
import actionlib
import beginner_tutorials.msg
class FibonacciAction(object):
    # create messages that are used to publish feedback/result
    _feedback = beginner_tutorials.msg.FibonacciFeedback()
    _result = beginner_tutorials.msg.FibonacciResult()
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, beginner_tutorials.msg.FibonacciAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # append the seeds for the fibonacci sequence
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)
        
        # publish info to the console for the user
        rospy.loginfo(f"{self._action_name}: Executing, creating fibonacci sequence of order {goal.order} with seeds {self._feedback.sequence[0]}, {self._feedback.sequence[1]}")
        
        # start executing the action
        for i in range(1, goal.order):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo(f"{self._action_name}: Preempted")
                self._as.set_preempted()
                success = False
                break
            rospy.loginfo(f"{self._action_name}: currently on {i}th iteration")
            self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()
        
        if success:
            self._result.sequence = self._feedback.sequence
            rospy.loginfo(f"{self._action_name}: Succeeded")
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('fibonacci')
    server = FibonacciAction(rospy.get_name())
    rospy.spin()
```

**IMPORTANT:** In Linux, files can't be executed/run as code unless they are specifically marked as executable. So, whenever we create a new Python node, we must mark it as executable. We do this by using the `chmod` command with the `+x` flag (`add` e`x`ecutable permissions) and giving it the path to our Python code:

```bash
rosstd
roscd beginner_tutorials
chmod +x src/fibonacci_server.py
```

### The Code, explained

```py
import actionlib
```

This line imports the actionlib library used for implementing simple actions.

```py
import beginner_tutorials.msg
```

The action specification generates several messages for sending goals, receiving feedback, etc... This line imports the generated messages.

```py
        self._as = actionlib.SimpleActionServer(self._action_name, actionlib_tutorials.msg.FibonacciAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
```

Here, the `SimpleActionServer` is created, we pass it a name (used as a namespace), an action type, and optionally an execute callback. Since we've specified an execute callback in this example, a thread will be spun for us which allows us to take long running actions in a callback received when a new goal comes in.

Note you should always set `auto_start` to False explicitly, unless you know what you're doing ([ref](https://github.com/ros/actionlib/pull/60)).

```py
    def execute_cb(self, goal):
```

This is the execute callback function that we'll run everytime a new goal is received.

```py
        r = rospy.Rate(1)
        success = True
        
        # append the seeds for the fibonacci sequence
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)
        
        # publish info to the console for the user
        rospy.loginfo(f"{self._action_name}: Executing, creating fibonacci sequence of order {goal.order} with seeds {self._feedback.sequence[0]}, {self._feedback.sequence[1]}")
```

Here, the internals of the action are created. In this example `rospy.loginfo` is used to let the user know that the action is executing.

```py
        # start executing the action
        for i in range(1, goal.order):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo(f"{self._action_name}: Preempted")
                self._as.set_preempted()
                success = False
```

An important component of an action server is the ability to allow an action client to request that the goal under execution be canceled. When a client requests that the current goal be preempted, the action server should cancel the goal, perform any necessary cleanup, and call the `set_preempted` function, which signals that the action has been preempted by user request. Here, we'll check if we've been preempted every second. We could, alternatively, receive a callback when a preempt request is received.

```py
            self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
            # publish the feedback
            self._as.publish_feedback(self._feedback)
```

Here, the Fibonacci sequence is put into the feedback variable and then published on the feedback channel provided by the action server. Then, the action continues looping and publishing feedback.

```py
        if success:
            self._result.sequence = self._feedback.sequence
            rospy.loginfo(f"{self._action_name}: Succeeded")
            self._as.set_succeeded(self._result)
```

Once the action has finished computing the Fibonacci sequence, the action server notifies the action client that the goal is complete by calling `set_succeeded`.

```py
if __name__ == '__main__':
    rospy.init_node('fibonacci')
    server = FibonacciAction(rospy.get_name())
    rospy.spin()
```

Finally, the `main` function creates the action server and spins the node.

Compiling
---------

Only initially when you just created your tutorial package, you need to compile to generate shell config files.

```bash
rosstd
natbuild
```

Running the Action Server
-------------------------

In a Docker terminal, run:

```bash
roscore -p 5802 &
rosrun actionlib_tutorials fibonacci_server.py
```

Now, make an [action client](./action_client.md) to call the server with!

[Home](/README.md)