#= Setting the Number of CPU Cores Used by your VM #=

Computers today have multiple cores 1. essentially multiple separate processors which all run in parallel.  Using multiple cores allows things to run faster. Each core can work on something different so processes happen in parallel rather than serially.

Your VM needs to be configured to use a specific number of cores. The default settings are conservative so they work on all systems.  You can check to see if your system has more CPU cores than the default and if so, set up the VM to take advantage of them.

==== How many cores do you have? ====

In windows, open up task manager using Ctrl-Shift-Esc.  Click on "more details" if you're not already in the more details mode.  Click on performance. Look in the lower right for the "Logical Processors:" value. This is the number of cores your system has available to run tasks on.

==== Setting the VM to use those cores ====

The number of cores can only be changed when the VM is inactive, so shut down / power off the VM.

Make sure the Ubuntu VM is selected, then click on VM -> Settings.  Select "Processors" under device.  On the right, set "Number of cores per processor".  You can set this to the "Logical Processors:" value minus 1. This will leave a full core completely free for native Windows processes.  

Note that this is the maximum number of cores used by the VM. If the VM is idle, those cores will be free for other work on Windows. And even if the VM is using the cores, Windows will still be able to share those cores with native windows work.  