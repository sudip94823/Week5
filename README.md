# Week-5, Sudip Dhungana
##12194823

# **Creating A Simple Publisher and Subscriber**
## 1. Creating a Package
*The package creation is done navigating to ros2_ws/src directory:*
```
ros2 pkg create --build-type ament_python py_pubsub
```
## 2. Writing the Publisher node
Location to navigate: ros2_ws/src/py_pubsub/py_pubsub
Using the given command, we can download the example talker code.
```
wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
```
New files is created after the command: publisher_member_function.py which is adjacent to __init__.py
When the file is opened, it shows:

```
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2.1 Adding Dependencies
Navigate to ros2_ws/src/py_pubsub directory.
There you can see the 3 files: setup.py, setup.cfg, package.xml.
After that, description, maintainer and lisense tags were filled after opening package.xml

```
<description>Examples of minimal publisher/subscriber using rclpy</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```

After the lines above that correspond to the import declarations for your node,
the following dependencies were added.

```
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```
This declares the package needs rclpy and std_msgs when code is being executed.

 With everychanges made, it is important to save the file.
 
 ## 2.2 Adding an entry point.
 Open setup.py. And confirm the maintainer, maintainer email, description, and license columns match on package.xml. 
 
 ```
 maintainer='YourName',
maintainer_email='you@email.com',
description='Examples of minimal publisher/subscriber using rclpy',
license='Apache License 2.0',
```
Within the console scripts brackets of the entry points field, add the following line:

```
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
        ],
},
```

Save the file.

![package_xml](https://user-images.githubusercontent.com/113494159/194859583-d1b67f1d-156b-4465-8d3f-2c5a1dc3c3c6.png)


## 2.3 setup.cfg
The setup.cfg file should automatically contain the following information:

```
[develop]
script-dir=$base/lib/py_pubsub
[install]
install-scripts=$base/lib/py_pubsub
```
Simply instruct setuptools to place your executables in the lib directory, where ros2 run will look for them.

## 3. Writing the subscriber node
 Fill in the following code on your terminal navigating to ros2_ws/src/py_pubsub/py_pubsub

```
wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
```
Now, the directory must include the following files:

```
__init__.py  publisher_member_function.py  subscriber_member_function.py
```

## 3.1 Adding an enrty point
Reopen setup.py and affix the entry point of the subscriber node below that of the publisher. The entry points field now should be:

```
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.subscriber_member_function:main',
        ],
},
```
![image](https://user-images.githubusercontent.com/113494159/194861247-f61879b2-d6bf-4c29-afe2-726a74ab49d5.png)
The system is operational once it is saved.

## 4. Build and Run
*Running rosdep to check on missing dependencies if any.*

```
rosdep install -i --from-path src --rosdistro foxy -y
```

Still in the root of your workspace, ros2_ws, build your new package:

```
colcon build --packages-select py_pubsub
```

After that, navigate to ros2_ws and source the setup files in the new terminal.

```
. install/setup.bash
```
Let's run the talker code now.
```
ros2 run py_pubsub talker
```

Open a fresh terminal, source the configuration files from ros2 ws once more, and then launch the listener node:
```
ros2 run py_pubsub listener
```

Starting at the publisher's current message count, the listener will begin writing messages to the console as follows:
![#4TalkerNode](https://user-images.githubusercontent.com/113494159/194862438-f62e8d96-58f5-4c4d-831d-dd5463b6c027.png)
![#5ListenerNode](https://user-images.githubusercontent.com/113494159/194862451-7c83a8dc-f455-4218-ad13-fa19f0a5adc7.png)


Ctrl+C will stop the nodes.


# A simple Service & Client
Here , the goal is to create and run service and client nodes using Python.

## 1. Creating a Package
Run the package creation command by going to ros2 ws/src:
It is exactly same process to getting into same directory and creating a new package.

```
ros2 pkg create --build-type ament_python py_srvcli --dependencies rclpy example_interfaces
```
The terminal shows the confirmation message of package creation and its necessary files and folders.
![#5Building_new_package](https://user-images.githubusercontent.com/113494159/194864310-6e998d53-15f5-4d44-ab14-d1ff9f7b2155.png)



## 1.1 Update Package.xml
As we used the —dependencies option when generating the package, you don't need to manually add dependencies to package.xml, but as always, It is important to add the description, maintainer's name and email, and licensing information to package.xml

```
<description>Python client server tutorial</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```

## 1.2 Update setup.py
The following details should be added to the setup.py file's description, maintainer, maintainer email, and license fields:
```
maintainer='Your Name',
maintainer_email='you@email.com',
description='Python client server tutorial',
license='Apache License 2.0',
```
***I haven't added the screenshot as its the same process which we did earlier.***

## 2. Write the service node
In the following directories: ros2_ws/src/py_srvcli/py_srvcli, a new file named service_member_function.py is created and the following code is pasted within the file:

And paste the following code inside:
```
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
 ```
 
  
## 2.1 Add an entry point
For the ros2 run command to be able to run your node, the entry point must be added to setup.py (located in the ros2 ws/src/py srvcli directory).

In between the "console scripts" brackets, the following line should be added:
```
'service = py_srvcli.service_member_function:main',
```

## 3 Write the client node
In the ros2 ws/src/py srvcli/py srvcli directory, a new file called client_member_function.py is needed to create and then paste the following code inside:

```
import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
 ```
    

## 3.1 Add an entry point
Similar to how the service node needs an entry point, the client node also needs one.

The entry points column in your setup.py file must be formatted as follows:

```
entry_points={
    'console_scripts': [
        'service = py_srvcli.service_member_function:main',
        'client = py_srvcli.client_member_function:main',
    ],
},
```
![image](https://user-images.githubusercontent.com/113494159/194865968-9fb3a440-7461-4547-b41b-80e4a7ad0a21.png)


## 4 Build and Run

*Running rosdep to check if any dependencies missing.*

```
rosdep install -i --from-path src --rosdistro foxy -y
```

Navigate back, ros2_ws, and build your new package: (All the new packages are built here:)

```
colcon build --packages-select py_srvcli
```

![a2](https://user-images.githubusercontent.com/58104378/194231383-3a6d35dd-c72f-4e9e-ad31-c60b2d414c57.png)

Open a new terminal, navigate to ros2_ws, and source the setup files:

```
. install/setup.bash
```

Now run the service node:

```
ros2 run py_srvcli service
```

The node will watch for the client's request.

Re-source the setup files from ros2 ws in a new terminal. The client node, any two integers, and a space between them.

```
ros2 run py_srvcli client 2 3
```

The client would get a response like this if you selected options 2 and 3 as an example:

```
[INFO] [minimal_client_async]: Result of add_two_ints: for 2 + 3 = 5
```


![a3](https://user-images.githubusercontent.com/58104378/194233544-34c78a69-41a1-4a83-9c5f-631d2d4f845f.png)

You should return to the terminal where your service node is running. As you can see, it published the following log statements after receiving the request:

```
[INFO] [minimal_service]: Incoming request
a: 2 b: 3
```

![a4](https://user-images.githubusercontent.com/58104378/194234334-194efd1b-981d-4994-af7e-08a8992ef3b5.png)


# Creating custom msg and srv files

## 1 Create a new package
Run the package creation command by going to ros2 ws/src:

```
ros2 pkg create --build-type ament_cmake tutorial_interfaces
```

Your terminal will display a message confirming the establishment of your package tutorial interfaces and every file and folder it needs. Due to the fact that pure Python packages cannot currently generate.msg or.srv files, it should be stated that this is a CMake package. A custom interface that you create in a CMake package can be used by a Python node, which will be covered in the last section.

It's best practice to keep the.msg and.srv files separated within a package. Create the directories in the ros2 ws/src/tutorial interfaces.

```
mkdir msg

mkdir srv
```
![c1](https://user-images.githubusercontent.com/58104378/194291243-793b703b-9873-4e46-95d5-ed00f6444dd0.png)


## 2 Create custom definitions
In the tutorial interfaces/msg directory that you just created, make a new file named Num.msg, and then add a single line of code describing the data structure in Num.msg:
```
int64 num
```

This custom message transmits the 64-bit integer num, which is one single value.

In the tutorial interfaces/msg directory that you just created, make a new file called Sphere.msg and put the following information in it:

```
geometry_msgs/Point center
float64 radius
```
This custom message makes use of a message from a different message package (in this case, geometry msgs/Point).

## 2.2 srv definition
In the instructional interfaces/srv directory that you just created, add a new file with the name AddThreeInts.srv with the following request and response structure:
```
int64 a
int64 b
int64 c
---
int64 sum
```

This is a custom service that accepts three integers with names a, b, and c and returns an answer with the integer sum.

## 3 CMakeLists.txt
To convert the interfaces you defined into language-specific code (such as C++ and Python) so they may be used in those languages, add the following lines to CMakeLists.txt:
```
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "msg/Sphere.msg"
  "srv/AddThreeInts.srv"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
```

## 4 package.xml
To package.xml, these lines should be added.
```
<depend>geometry_msgs</depend>

<build_depend>rosidl_default_generators</build_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

## 5 Build the tutorial_interfaces package
Now that all of the pieces of your custom interfaces package are in place, you can build it. Run the following command in the workspace's root directory (/ros2 ws):
```
colcon build --packages-select tutorial_interfaces
```

![c4](https://user-images.githubusercontent.com/58104378/194284059-0da36d3a-6ceb-42ae-9f33-d9769b451b8f.png)

Now, the interfaces will be discoverable by other ROS 2 programs.

## 6 Confirm msg and srv creation
To source it in a new terminal, issue the following command from within your workspace (ros2 ws):

```
. install/setup.bash
```

The ros2 interface show command can now be used to verify that your interface creation was successful:

```
ros2 interface show tutorial_interfaces/msg/Num
```

& it should return
```
int64 num
```
&
```
ros2 interface show tutorial_interfaces/msg/Sphere
```
should return
```
geometry_msgs/Point center
        float64 x
        float64 y
        float64 z
float64 radius
```

&
```
ros2 interface show tutorial_interfaces/srv/AddThreeInts
```

& it should return
```
int64 a
int64 b
int64 c
---
int64 sum
```

![d1](https://user-images.githubusercontent.com/58104378/194285656-fbda7b36-5780-4afd-90bc-048b5165d3fb.png)

## 7 Test the new interfaces
For this step, you can use the packages you created in the prior stages. By making a few simple adjustments to the nodes, CMakeLists, and package files, you can use your new interfaces.
Publisher:

```
import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import Num    # CHANGE


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Num, 'topic', 10)     # CHANGE
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Num()                                           # CHANGE
        msg.num = self.i                                      # CHANGE
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.num)  # CHANGE
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Subscriber:

```
import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import Num        # CHANGE


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Num,                                              # CHANGE
            'topic',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
            self.get_logger().info('I heard: "%d"' % msg.num) # CHANGE


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

CMakeLists.txt:


Add the following lines (C++ only):

```
#...

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)                         # CHANGE

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp tutorial_interfaces)         # CHANGE

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp tutorial_interfaces)     # CHANGE

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```
package.xml:

Add the following line:

```
<exec_depend>tutorial_interfaces</exec_depend>
```

Build the package after making the aforementioned alterations and saving all the modifications:

```
colcon build --packages-select py_pubsub
```

On Windows:

```
colcon build --merge-install --packages-select py_pubsub
```

Then open two new terminals, source ros2_ws in each, and run:

```
ros2 run py_pubsub talker
```

    
![e3](https://user-images.githubusercontent.com/58104378/194287426-ae7f6e32-513f-46d0-9978-88aec26d301b.png)
![e2](https://user-images.githubusercontent.com/58104378/194288329-15cb6f9a-4721-419f-a335-6e0154e9d075.png)

Instead of broadcasting strings as Num.msg only relays an integer, the talker should only be publishing integer values:
```
[INFO] [minimal_publisher]: Publishing: '0'
[INFO] [minimal_publisher]: Publishing: '1'
[INFO] [minimal_publisher]: Publishing: '2'
```

## 7.2 Testing AddThreeInts.srv with service/client

By making a few small changes to the service/client package created in a previous tutorial (in C++ or Python), you may utilize AddThreeInts.srv. You'll be changing from the initial two integer request srv to a three integer request srv, which will have a big impact on the result.

Service:

```
from tutorial_interfaces.srv import AddThreeInts     # CHANGE

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddThreeInts, 'add_three_ints', self.add_three_ints_callback)        # CHANGE

    def add_three_ints_callback(self, request, response):
        response.sum = request.a + request.b + request.c                                                  # CHANGE
        self.get_logger().info('Incoming request\na: %d b: %d c: %d' % (request.a, request.b, request.c)) # CHANGE

        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Client:

```
from tutorial_interfaces.srv import AddThreeInts       # CHANGE
import sys
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddThreeInts, 'add_three_ints')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddThreeInts.Request()                                   # CHANGE

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.req.c = int(sys.argv[3])                  # CHANGE
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of add_three_ints: for %d + %d + %d = %d' %                               # CHANGE
                    (minimal_client.req.a, minimal_client.req.b, minimal_client.req.c, response.sum)) # CHANGE
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
CMakeLists.txt:

Add the following lines (C++ only):
```
#...

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)        # CHANGE

add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server
  rclcpp tutorial_interfaces)                      #CHANGE

add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client
  rclcpp tutorial_interfaces)                      #CHANGE

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```
package.xml:

Add the following line:
```
<exec_depend>tutorial_interfaces</exec_depend>
```
After making the above edits and saving all the changes, build the package:

```
colcon build --packages-select py_srvcli
```
On Windows:

```
colcon build --merge-install --packages-select py_srvcli
```
Then open two new terminals, source ros2_ws in each, and run:

```
ros2 run py_srvcli service

```
![11](https://user-images.githubusercontent.com/58104378/194289956-93714d9c-1fd6-495a-b5eb-38acbc791bd4.png)

```
ros2 run py_srvcli client 2 3 1

```
![12](https://user-images.githubusercontent.com/58104378/194290393-2a42fbe5-f522-4c9c-b7c8-54111e6854e7.png)


    
