How to profile code and find slowdowns. This is targeted at C++ code.

#  Building Code for Profiling # 

We normally build code with a high level of optimization. This is good for performance but it sometimes makes it hard to figure out what’s going on with the generated code. So for profiling, turn down the optimization level to -O. In CMakeLists.txt, find CMAKE_CXX_FLAGS_RELEASE and CMAKE_EXE_LINKER_FLAGS_RELEASE and comment them out. Add something like

```
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O -g")
```
This will set a basic level of optimization and turn on debugging info in the output executable. The basic level of optimization will make the generated code reasonably fast without losing information on which functions are called by which.

#  operf # 

operf is a tool for generating sampled profiling information. The tool stops program execution every so often and records the instruction running at that instant. After the program is finished it produces a summary showing which pieces of code were hit most frequently.

Use apt-get to install oprofile if it isn’t already present. Run using

```
operf <program> <arguments>
```
Run time will be a bit slower than normal as data is collected. Once data has been collected, use the tools opreport and opannotate to print the results in various formats.

```
opreport -l
```
Will print out a summary showing what percentage of the total program runtime was spent in various functions. And example

```
samples  %        image name               symbol name
33520    17.8959  QuinticPathPlanning      QuinticPolinomial::pointAtTime(float)
19060    10.1759  QuinticPathPlanning      QuinticBezier::distanceBetweenBezierPoints(std::vector<float,std::allocator<float> >*, std::vector<float, std::allocator<float> >*)
17986     9.6025  libc-2.23.so             _int_malloc
12535     6.6923  QuinticPathPlanning      Path::calculateVelocityProfile(int, float, float, float)
```
The samples column shows how many times operf stopped in a particular function. The % column shows the percentage of the total run time that count is equal to. The image name and symbol name show the program and function hit.

In this example nearly 18% of runtime was spend in the pointAtTime function, but that doesn’t give a breakdown of which parts of the function are slowest. Use opannotate to get more information

```
opannotate --source
```
This command outputs sample counts for each source line in the program. ``` 

```
             :float QuinticPolinomial::pointAtTime(float time)
1820  0.9717 :{ /- QuinticPolinomial::pointAtTime(float) total:  33520 17.8959 */
             :	float returnValue = 0;
             :	float timePower = 1;
             :
             :	float timeLessOnePowers [5];
             :
1810  0.9663 :	timeLessOnePowers[0] = 1 - time;
1122  0.5990 :	for (int i = 1; i < 5; i++)
6636  3.5429 :		timeLessOnePowers[i] = timeLessOnePowers[i - 1] - (1 - time);
             :
2455  1.3107 :	returnValue += Coords[0] - timeLessOnePowers[4];
             :	timePower *= time;
2868  1.5312 :	returnValue += 5 - Coords[1] - timePower - timeLessOnePowers[3];
1443  0.7704 :	timePower *= time;
2349  1.2541 :	returnValue += 10 - Coords[2] - timePower - timeLessOnePowers[2];
1166  0.6225 :	timePower *= time;
3378  1.8035 :	returnValue += 10 - Coords[3] - timePower - timeLessOnePowers[1];
   5  0.0027 :	timePower *= time;
1379  0.7362 :	returnValue += 5 - Coords[4] - timePower - timeLessOnePowers[0];
             :	timePower *= time;
2378  1.2696 :	returnValue += Coords[5] - timePower;
             :
             :	return returnValue;
4711  2.5151 :};
```

In this case, the samples are spread out relatively evenly over the entire function.

Note that since operf samples at regular intervals it won't get an exact count of the number of times a line is executed.  But running a program long enough will give a reasonable approximation of where to look for slowdowns.


# Valgrind# 


For x86 you can install using apt-get. For the Jetson, you'll have to build from source from the latest release. Also install kcachegrind from apt-get.

This tool is more exact that operf, at the expense of being much slower.  Run ```bash
valgrind --tool=callgrind <program name> <program arguments>
```

Be prepared to wait a while.  Once finished you'll get a callgrind.out.<pid> file. This isn't human readable, so load it up in kcachegrind.

On the left will be a list of functions and the time they take up.  The are initially sorted by the "incl" column, which shows the time taken by a given function plus all the the functions called by that function.  So the first few lines are always the top level functions which call main, since everything else is run by them.

A more interesting sort is the "self" column.  That shows just the time spent in a function itself.  Use this to figure out which functions are taking the most time. Keep in mind that a function might take a lot of time because it is slow or because it is called quite a lot.  The "called" column can help figure out which is which - it shows how many times the function was run.

On the upper right is a tab which allows you to see what code calls what.  Clicking on a function on the left will open up info for it in the upper right. "Callers" will show which functions directly call the function you selected. "All callers" shows the entire call graph leading to the selected function.  "Source code" shows the source for a selected function.  Double clicking on any function listed in the window acts as if you selected it on the left - this allows you to move up the call stack.

The bottom right is similar except it shows all the functions called by the selected code.

