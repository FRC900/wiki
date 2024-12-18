# Everything You Need to Know About Config Values# 
## Storing Config Values## 
Make a .yaml file in a package's config directory

Write stuff in it. Syntax guides:

https://gettaurus.org/docs/YAMLTutorial/#YAML-Syntax (good one)

http://wiki.ros.org/YAML%20Overview (the less detailed one that ROS wiki provides)


Add it to your launch file with this line:
```
<rosparam file="$(find YOUR_PACKAGE)/config/YOUR_CONFIG_FILE.yaml" command="load"/>
```

## Reading Config Values## 
Make sure you have a `ros::NodeHandle`, we'll refer to it as `nh`. In general, you do config value reading in the `main()` function, but you don't have to.

For basic types:
```
double my_double;
if(!nh.getParam("config value name", my_double)) {
    ROS_ERROR("Couldn't read [config value name] in [filename]");
    my_double = 0; *default
}
```

For lists (`bool`, `int`, `float`, `double`, `std::string` types only):
```
std::vector<my_type> my_list;
if(!nh.getParam("config list name", my_list)) {
    ROS_ERROR("Couldn't read [config list name] in [filename]");
}

*access list item
my_type my_variable = my_list[2]; *gets the item at index 2
```

For dictionaries:
```
XmlRpc::XmlRpcValue my_dictionary;
if(!nh.getParam("config dictionary name", my_dictionary)) {
    ROS_ERROR("Couldn't read [config dictionary name] in [filename]");
}

*access an item in the dictionary - casting is necessary. NOTE: in the yaml file, "0" means int, "0.0" means double. XmlRpc will not automatically cast one to the other
my_double = (double) my_dictionary["name of item"];
my_int = (int) my_dictionary["name of item"];
my_string = my_dictionary["name of item"]; *I'm pretty sure you don't need a cast for strings, might want to check that though

*If it's possible that the user could have put e.g. an int or a double but you want a double, you need to check the type from the yaml file so you don't get casting errors:
if(my_dictionary["name_of_item"].getType() #####  XmlRpc::XmlRpcValue::Type::TypeDouble) {
    my_double = (double) my_dictionary["name of item"];
}
else if(action_data["duration"].getType() #####  XmlRpc::XmlRpcValue::Type::TypeInt) {
    my_double = (double) (int) my_dictionary["name of item"];
}

```

ROS wiki's page on this: http://wiki.ros.org/roscpp/Overview/Parameter%20Server

### A Note on Namespacing (IMPORTANT)### 
Every config value has a namespace it lives in. If you want to get the proper config value, you have to specify its namespace correctly. Example: `/intake/intake_actionlib_params/server_timeout`. The namespace is `/intake/intake_actionlib_params`, the config value is `server_timeout`

The namespace can come from two places:

a) the group the <rosparam> tag is in

b) namespacing in the yaml file


The above example has both of these. The <rosparam> was put in the launch file like this:
```
<group ns="intake">
    <rosparam ...stuff.../>
</group>
```
This gives the config value's namespace the first bit of `/intake`

The yaml file itself looked like:
```
intake_actionlib_params:
    server_timeout: 10
```
This added the `/intake_actionlib_params` bit to the namespace

When you read a config value, there are two ways you can get the correct namespace. The simple way is by specifying the entire namespace+config value, for example:
```
nh.getParam("/intake/intake_actionlib_params/server_timeout", server_timeout_storage_variable)
```

The second way, often used if you're reading a lot of config values from the same namespace, is by using relative namespaces. Each node handle has a namespace, determined by the node it's for. For example, if the node is in the `/intake` namespace, your node handle will have the `/intake` namespace. So, you can read the config value like this:
```
nh.getParam("intake_actionlib_params/server_timeout", server_timeout_storage_variable)
```
Note that this time there's no `/` in front of the config value name - this indicates you're using a name *relative* to the node handle's namespace.
Usually, you use relative namespacing with a second node handle with a lower-down namespace like so:
```
ros::NodeHandle nh_intake_actionlib(nh, "intake_actionlib_params"); *this node handle has the namespace /intake/intake_actionlib_params
nh_intake_actionlib.getParam("server_timeout", server_timeout_storage_variable)
```
