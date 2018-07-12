# ros_gammon_rs485
This is a package that provides a ROS interface for a an RS485 node chain. It assumes that there is a master
with an ID number of 0, and several slaves connected to a single line, where each slave reads some device and sends back 
a response to the master when it it requested.

The available method of communication ia based on Nick Gammon's RS485 communication protocol.

The proposed method of operation for this packages is:
- For each slave there is a ```RS485Slave``` instance that:

    - Can subscribe to a specific topic (that can trigger some change in its state)(optional)
    - Send a message request down the line that is addressed to a specific device
    - Receive the response from the slave and process it.
    - Republish the data received as a ROS msg (optional)

The base class ```RS485Slave``` has a ```makeExchange``` method that derived classes should implement if you want to 
implement your own send and receive methods.
Some additional methods that you might want to implement yourself in a new slave class are:

> ```serialize()```: Serializes the message to a byte array so it can be sent to the slave.
>
>```deSerialize()```: Deserializes the response from the slave to a ROS message.
>
> ```sendMsg()```: Creates the crc'd message and sends it down the line.
>
> ```recvMsg()```: Receives a message from the line, and checks it for errors.

An example slave is located in the ```arduino_sketches``` folder. You should edit the ```MY_ID``` constant if you have 
multiple slaves and the way the response message is created if you choose to actually read from the device. In the example
sketch, the slave waits for a message from the master addressed to it, and responds with its iD.

Slaves can be defined using the ROS parameter server and the ```createSlaves()``` method. The method will
query ROS parameter server for configurations like the one below and create the corresponding slave:

```yaml
slave1:
  id: 1 #must be >=1
  sub_type: "std_msgs/Int8"
  sub_topic: ""
  pub_type: "std_msgs/Int8"
  pub_topic: "slave1"
  timeout: 500 #in milliseconds
  mtl: 100  #maximum_transmit_length, must be positive
  mrl: 100  #maximum_receive_length, must be positive
``` 
### TODO
- ~~Fix subscribers.~~
- ~~Add serialization methods for the slaves for different data types.~~

### Maintainers
[Andreas Lydakis](andlydakis@gmail.com)