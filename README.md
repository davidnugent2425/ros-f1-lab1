# Lab 1 Tutorial

## Starting Point

To start this lab, you must be able to run the f1tenth simulator in ROS.

## Part 1: Setting up a Workspace

In this step we make a Catkin Workspace. These workspaces ensure we have all of our dependencies (required packages). In the terminal we will execute the following commands:

Set up the file structure for a catkin workspace

```
mkdir -p /sample_ws/src
```

Build the workspace

```
cd /sample_ws
catkin_make
```

Overlay the workspace on our current environment

```
source devel/setup.bash
```

## Part 2: Creating a Package

In this step we make a ROS Package. When we are writing code we will store files in packages so they can be used throughout our workspace.

Navigate to the source directory of your workspace

```
cd /sample_ws/src
```

Create a package called lab1 with dependencies: ```std_msgs```, ```rospy```

```
catkin_create_pkg lab1 std_msgs rospy
```

## Part 3: Making a Subscriber and a Publisher

We will now move on to making a 'subscriber' that listens to data being published to a topic by another node in ROS, and a 'publisher' that publishes data to a particular topic. The simulator must be running in the background during this part (visual output not required). At this point it would be good to have a number of terminals open

Check what 'topics' are currently being broadcast on the network

```
rostopic list
```

Take a look at the output of the ```/scan``` topic

```
rostopic echo /scan
```

Take a look at the information given about the ```/scan``` topic

```
rostopic info /scan
```

The ```/scan``` topic is a stream of ```sensor_msgs/LaserScan``` messages. Take a look at what the contents of a ```LaserScan``` message is

```
rosmsg info sensor_msgs/LaserScan
```

Now that we know what type of message we're trying to listen to, we'll start by making a Subscriber. Navigate to the source directory of our lab1 package and write a new python script. 

```
roscd lab1/src
vim scan_listener.py
```

Python scripts in ROS must all start with this line to ensure they are executed as a python script (this full tutorial could also be completed in C++)

```python
#!/usr/bin/env python
```

We will then add the imports required for this part

```python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import numpy as np
```

We will now make a ```ScanListener``` class. Using a class will work well for us here because we want our subscriber and publisher to be easily accessible. Note that in our main function we initialise our 'rospy process' as a node and we tell rospy to ```spin()```, this will allow our subscriber to keep listening until we stop the program

```python
class ScanListener:
	def __init__(self):
		# more to follow

if __name__ == '__main__':
	rospy.init_node('scan_listener', anonymous=True)
	listener = ScanListener()
	rospy.spin()
```

In the constructor for our class, let's set up a subscriber that listens to the ```/scan``` topic. The paramaters for the subscriber are: the topic we want to listen to, the type of message we will listen to, a callback function which will be run when we receive data from the topic, and the number of messages we are willing to queue at a time

```python
	def __init__(self):
		self.scan_sub = rospy.Subscriber('scan', LaserScan,
				self.scan_callback, queue_size=1)
```

Let's now write the callback function (in our class) that we just passed into our subscriber. The LaserScan message contains an array of ranges which is the distance each LiDAR ray is detecting. Our goal is to ultimately publish messages stating the closest distance and the farthest distance. We will start by pre-processing our LiDAR data. This is a precaution taken to ensure there are no NaN values or inf values in our ranges array. In this case, we use a naive method - we just remove the incorrect values

```python
	def scan_callback(self, data):
		ranges = np.array(data.ranges)
		proc_ranges = ranges[np.logical_and(~np.isnan(ranges), ~np.isinf(ranges))]
```

For the next step, we will need publishers, so let's go back to our constructor and initialise them. Each publisher will take the following parameters: the name of the topic it will publish to, the type of data it will publish, the number of messages it is willing to queue at a time

```python
	def __init__(self):
		...
		self.closest_pub = rospy.Publisher('closest_range', Float64,
					queue_size=10)
		self.farthest_pub = rospy.Publisher('farthest_range', Float64,
					queue_size=10)
```

Now we can finish off our callback function

```python
	def scan_callback(self, data):
		...
		closest_dist = min(proc_ranges)
		farthest_disdt = max(proc_ranges)
		self.closest_pub.publish(closest_dist)
		self.farthest_pub.publish(farthest_dist)
```

The code should now be in a state similar to [checkpoint_1.py](./spoilers/checkpoint_1.py). Now, back in the terminal, let's make our script executable

```
chmod +x scan_listener.py
```

Let's run our script in one terminal

```
rosrun lab1 scan_listener.py
```

And listen to the topics in another terminal

```
rostopic echo /closest_range
OR
rostopic echo /farthest_range
```

## Part 4: Publishing a Custom Message Type

In this part we will modify our previous script to publish both ranges in the same message.

Start by creating file structure for messages and a message file

```
roscd lab1
mkdir msg
cd msg
vim ScanRange.msg
```

We would like our message to have the following contents: a header containing info about the message, the value of the closest range, the value of the furthest range. Here's how we write that in a message file

```
Header header
float64 closest_range
float64 farthest_range
```

Now we must make various (tedious) changes to our package configuration files to account for our custom message.

In package.xml (in our lab1 package directory) we add the following:

```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

In CMakeLists.txt make the following additions to existing functions:

```
find_package(catkin REQUIRED COMPONENTS
  ...
  message_generation
)
```
```
catkin_package(
  CATKIN_DEPENDS message_runtime
  ...
)
```
```
add_message_files(
  FILES
  ScanRange.msg
)
```
```
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

We have now finished the configuration process. Let's build and look at our new message type

```
cd /sample_ws
catkin_make
source devel/setup.bash
rosmsg show lab1/ScanRange
```

Let's now update our script to use ScanRange. Start by adding the import

```python
from lab1.msg import ScanRange
```

Replace our old publishers with one that uses ScanRange

```python
	def __init__(self):
		...
		self.scan_range_pub = rospy.Publisher('scan_range', ScanRange,
					queue_size=10)
```

Update our callback function to publish a ScanRange message. Note that we use the header of the data heard by our subscriber. This means that when we publish our ScanRange message, whoever hears it can compare it to other messages with the same timestamp

```python
	def scan_callback(self, data):
		...
		scan_range_msg = ScanRange(data.header, closest_dist, farthest_dist)
		self.scan_range_pub.publish(scan_range_msg)
```

The final script should be something similar to [scan_listener.py](./spoilers/scan_listener.py). Let's run our script again in a terminal

```
rosrun lab1 scan_listener.py
```

Listen to the published topic in another terminal

```
rostopic echo /scan_range
```

Voila.
