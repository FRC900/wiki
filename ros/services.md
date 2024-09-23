Writing a Simple Service and Client
===================================

Writing a Service Node
----------------------

Here we'll create the service (`add_two_ints_server`) node which will receive two ints and return the sum.

Change directory into the `beginner_tutorials` package:

```bash
roscd beginner_tutorials
```

### The Code

Create the **src/add\_two\_ints\_server.py** file within the `beginner_tutorials` package (e.g. through `Ctrl-N` in VS Code) and paste the following inside it:

```py
#!/usr/bin/env python3
from beginner_tutorials.srv import AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse
import rospy

def handle_add_two_ints(req):
    rospy.loginfo(f"Returning [{req.a} + {req.b} = {req.a + req.b}]")
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server', anonymous=True)
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    rospy.loginfo("Ready to add two ints.")
    rospy.spin()
 
if __name__ == "__main__":
    add_two_ints_server()
```

**IMPORTANT:** In Linux, files can't be executed/run as code unless they are specifically marked as executable. So, whenever we create a new Python node, we must mark it as executable. We do this by using the `chmod` command with the `+x` flag (`add` e`x`ecutable permissions) and giving it the path to our Python code:

```bash
rosstd
roscd beginner_tutorials
chmod +x src/add_two_ints_server.py
```

If you do not do this, you will see a helpful error telling you that ROS either couldn't find your script or it was not marked as executable, e.g:

```
[rosrun] Couldn't find executable named add_two_ints_server.py below /home/ubuntu/.2023RobotCode.readonly/zebROS_ws/src/beginner_tutorials
[rosrun] Found the following, but they're either not files,
[rosrun] or not executable:
[rosrun]   /home/ubuntu/.2023RobotCode.readonly/zebROS_ws/src/beginner_tutorials/src/add_two_ints_server.py
```

### The Code Explained

Now, let's break the code down.

There's very little to writing a service using [rospy](https://wiki.ros.org/rospy). We declare our node using `init_node()` and then declare our service:

```py
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
```

This declares a new service named `add_two_ints` with the AddTwoInts service type. All requests are passed to `handle_add_two_ints` function. `handle_add_two_ints` is called with instances of `AddTwoIntsRequest` and returns instances of `AddTwoIntsResponse`.

Just like with the subscriber example, `rospy.spin()` keeps your code from exiting until the service is shutdown.

Writing the Client Node
-----------------------

### The Code

Create the **src/add\_two\_ints\_client.py** file within the `beginner_tutorials` package and paste the following inside it:

```py
#!/usr/bin/env python3
import sys
import rospy
from beginner_tutorials.srv import AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse
def add_two_ints_client(a, b):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(a, b)
        return resp1.sum
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def usage():
    return f"{sys.argv[0]} [a b]"

if __name__ == "__main__":
    if len(sys.argv) == 3:
        a = int(sys.argv[1])
        b = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    rospy.init_node('add_two_ints_client', anonymous=True)
    rospy.loginfo(f"Requesting {a}+{b}")
    result = add_two_ints_client(a, b)
    print(f"{a} + {b} = {result}")
```

Don't forget to make the node executable:

```bash
rosstd
roscd beginner_tutorials
chmod +x src/add_two_ints_client.py
```

### The Code Explained

Now, let's break the code down.

The client code for calling services is also simple. We first call:

```py
    rospy.wait_for_service('add_two_ints')
```

This is a convenience method that blocks until the service named `add_two_ints` is available.

Next we create a handle for calling the service:

```py
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
```

We can use this handle just like a normal function and call it:

```py
        resp1 = add_two_ints(a, b)
        return resp1.sum
```

Because we've declared the type of the service to be `AddTwoInts`, it does the work of generating the `AddTwoIntsRequest` object for you (you're free to pass in your own instead). The return value is an `AddTwoIntsResponse` object. If the call fails, a `rospy.ServiceException` may be thrown, so you should setup the appropriate try/except block.

Try it out!
-----------

(helpful tip: type `terminator` to get a terminal that you can split horizontally and vertically into multiple terminals by right clicking)

In a **new terminal** inside Docker, run

```bash
rosstd
roscore -p 5802 &
rosrun beginner_tutorials add_two_ints_server.py
```

---

In a **new terminal**, run

```bash
rosstd
rosrun beginner_tutorials add_two_ints_client.py
```

*   And you will see the usage information printed, similar to
    `/home/ubuntu/2023RobotCode/zebROS_ws/src/beginner_tutorials/src/add_two_ints_client.py [a b]`
    

In the same terminal, then run

```bash
rosrun beginner_tutorials add_two_ints_client.py 4 5
```

*   And you will get

```
    Requesting 4+5
    4 + 5 = 9
```
*   And the server will print out
```
    Returning [4 + 5 = 9]
```

*   And the turtles will rejoice amid the great integer adding


![ROS Noetic mascot](https://spectrum.ieee.org/media-library/noetic-ninjemys-is-the-last-distribution-release-of-ros-1.jpg?id=25591717)
