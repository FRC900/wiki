#  Multiple Jetson and ZED setup # 

  - Make sure to delete everything in the known_hosts file in `%%~/.ssh/known_hosts%%` and use `%%ssh -oHostKeyAlgorithms='ssh-rsa' host%%` when ssh’ing into the slave to setup the ssh for the <html><machine></html> tag
  - Make sure the network is setup to be static and you have run `%%. ./ROSJetsonMaster%%` in the master terminal
  - Run `%%roslaunch controller_node controller.launch%%`
