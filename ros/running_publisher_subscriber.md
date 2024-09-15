**Note:** This tutorial assumes that you have completed the previous tutorials: writing a simple publisher and subscriber [(python)](./publisher_subscriber.md).

Running the Publisher
---------------------

Make sure that you have sourced ROS (type `rosstd`) and that a `roscore` is up and running:

```
roscore -p 5802 &
```

In the last tutorial we made a publisher called "talker". Let's run it:

```
rosrun beginner_tutorials talker.py
```

You will see something similar to:

```
[INFO] hello world 1314931831.77
[INFO] hello world 1314931832.77
[INFO] hello world 1314931833.78
[INFO] hello world 1314931834.78
[INFO] hello world 1314931835.78
[INFO] hello world 1314931836.79
```
    
The publisher node is up and running. Now we need a subscriber to receive messages from the publisher.

Running the Subscriber
----------------------

In the last tutorial we made a subscriber called "listener". Let's run it:

```
rosrun beginner_tutorials listener.py
```

You will see something similar to:

```
[INFO] /listener_17657_1314931968795I heard hello world 1314931969.26
[INFO] /listener_17657_1314931968795I heard hello world 1314931970.26
[INFO] /listener_17657_1314931968795I heard hello world 1314931971.26
[INFO] /listener_17657_1314931968795I heard hello world 1314931972.27
[INFO] /listener_17657_1314931968795I heard hello world 1314931973.27
[INFO] /listener_17657_1314931968795I heard hello world 1314931974.28
[INFO] /listener_17657_1314931968795I heard hello world 1314931975.28
```

When you are done, press Ctrl-C to terminate both the listener and the talker.