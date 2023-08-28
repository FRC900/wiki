## Why do we use ROS?

### Several benefits:
- We try out all sorts of unusual sensors. Since ROS is an industry standard, a large majority of vendors provide ROS drivers for their hardware. This means we don't have to create them ourselves
- Automatic handling of coprocessor nodes. For the past 5 seasons we've used various coprocessors to offload computationally intensive tasks from the Rio. ROS provides transparent communication of data between different processors.
- We consistently have a large software team. ROS naturally encourages designs which split work into a number of separate processes (node), each of which communicates over well-defined interfaces. Among the many benefits of such a design is that helps us split tasks out among the members of the team rather than concentrating the work into a few larger programs.
- The same code which runs on the robot also runs on team members' laptops. This allows for testing and integration work off the robot. Again, a huge benefit given that we have way more programmers than robots to test on.
- ROS introspection allows reading and writing the data which passes between nodes. This aids in testing and debugging - it naturally leads to unit-testing various nodes, or possibly scripting tests without having to bring up the entire code base
- ROS allows saving of all messages passing between nodes. This allows us to debug exactly what happened during a match, or even replay the data after fixing code to verify the fix.
- ROS has a huge library of robotics-related packages. Since they use standard interfaces to communicate we can easily adapt them to our robot as needed.

### Marshall's distilled bullet points:
- ROS is an industry standard framework with widespread adoption and vendor support for emerging sensor technolog which means we can focus on higher order problem solving instead of reinventing solutions that already exist.
- ROS provides a decentralized framework that supports multi-nodal topologies which enable seamless transparent communication between processors on our robots.
- The design principles and well-defined interfaces of ROS enable our large software development team to work individually while maintaining clear communication among disparate nodes.
- ROS is supported on a wide variety of architectures which means our software development team can develop, deploy, and test code on their laptops as well as the systems on our robot, enabling unit-testing and better software development practices.
- All messages passed between nodes on ROS are capable of being inspected and recorded using standard ROS tools, which means our software development team can bring up portions of the robot using test data or even data recorded during a match to verify our code changes, debug robot problems, and diagnose issues before they result in a fault.
- Due to the rapid adoption of ROS in industry and academia, there is a massive library of packages which utilize standard interfaces and we can adopt quickly to be used on our robots.

[Home](/README.md)