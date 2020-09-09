# ROS Practice

This practice guide covers:

-   Docker images
-   ROS workspaces & automating moving code
-   Compiling C/C++ and using Python programs within ROS
-   Running your programs
-   Visualizing the system with `rqt_graph`

This should hopefully be useful too in case you need to refer to the code/file structure used here. You can also screw with the code here, but when done please don't try to commit your changes! If needed, remember that you can discard all changes from GitHub Desktop or Atom.

For a more in-depth guide to ROS, you'll want to check out their [core tutorials](http://wiki.ros.org/action/fullsearch/ROS/Tutorials?action=fullsearch&context=180&value=linkto%3A%22ROS%2FTutorials%22).

# What you'll need

-   A way to clone this GitHub repository
-   Docker
-   Windows PowerShell (or Mac/Linux terminal) for all windows commands and for accessing Docker containers
-   MobaXterm
-   GNU Make installed and on you path (you should be able to run `make` from your command line)

* * *

# The Guide

## Setup

Run docker if you have not already. Nothing should open, but there should be a notification eventually that says Docker is Running.

Run MobaXterm. This should automatically start an X11 Server (the X in the top right will become colorful after a few seconds). X11 is what allows you to see a GUI that is actually running in the Docker container.

If you're on Windows, you'll want to use PowerShell, rather than Command Prompt.

## The Directory Structure

Look at the directory structure. This is important for how files will get moved around. Type `tree` from within the cloned repository.

This is what it should look like, with some things omitted:

```*
U:\PRACTICE-ROS
│   ...
│   README.md
│
├───cpp-publisher
│   │   CMakeLists.txt
│   │   package.xml
│   │
│   └───src
│           main.cpp
│
├───py-subscriber
│   │   CMakeLists.txt
│   │   package.xml
│   │
│   └───src
│           main.py
│
└───ros-docker
    │   ...
    │   base.dockerfile
    │   git.dockerfile
    │   local.dockerfile
    └───Makefile
```

The projects for this are in `cpp-publisher` and in `py-subscriber`. Note that both of these projects have the same two files in their base directory: `CMakeLists.txt` and `package.xml`. These are used to configure and build the packages. Many changes applied to one must be applied to the other. Take a look at the `CMakeLists.txt` file for both projects, and note the difference between the python and C++ version. This file is the script that dictates how to get the source files to be executable in ROS. It also does library/package management.

The other directory is ros-docker. This is actually another GitHub repository, included in this one automatically. It contains everything we need to get docker going with our projects.

## The Code

Contained are two programs, one in Python and one in C++. This project, when complete, will have the C++ code sending messages to the Python code, over the pre-defined channel, called `/chatter`.

The code in the projects is copied directly from the ROS tutorials, so all the original documentation is there. I've also added some notes on unclear parts.

### The C++ Code

The C++ code is the talker. It will put data out on a data channel, for any listener to hear. This is also referred to as a _publisher_. The channel, in this case, is called a _topic_.

The program, when run, will publish messages to the `/chatter` topic.

### The Python Code

The Python code is the listener, also called a _subscriber_.

The program, when run, will read messages published to the `/chatter` topic and print them out. It does this will a _callback_ function, which will be called any time new information is posted to `/chatter`.

## Docker

From the command line (PowerShell), test that docker is running: `docker ps`. You should get a (blank) list of all running containers:

```shell
> docker ps
CONTAINER ID        IMAGE               COMMAND                  CREATED             STATUS              PORTS                NAMES
```

In PowerShell, `cd` into the cloned repository (`practice-ros`). You're now in the main repository, but you'll actually want to be in the `ros-docker` repository from here on out, so `cd` there.

_Type `make` into PowerShell._ If it's your first time running one of our docker containers, this command will take a while to finish. It is downloading the base image, ROS, and many helpers and programs to get all our docker images on the same page. Remember that `make` is just being used as a convenience; it is actually just running more commands for you.

Once docker is done building the image, there will be no exiting; it will print this and continuously run:

    Container started successfully.
    Container ID: 465e402cbe28

From here, you can type `^C` (`^` = `ctrl key`), which will kill the container and stop the OS. If you do that, then type `make` again, you'll see docker run through all the layers it ran through before, only much faster, since it does not need to re-download anything, as those layers were saved.

More useful for us, though, is another shortcut: _after seeing that the container finished building, enter `^P` + `^Q`._ This will detach from the container while it runs in the background, allowing you to continue using your PowerShell.

_After detaching, you can enter the container using the command `docker exec -it piranhabot_container bash`._

-   This will execute (`exec`) the command `bash` in the `piranhabot_container` (it was named by the `Makefile` automatically), using interactive mode (`-it`). In other words, it will start a bash session in the container. (Note that you can technically use any command instead of `bash`, like `ls`, `nano`, or others, including local executables.)

You should now be in the container. Your prompt will have changed from the windows `>` to a Linux prompt (in this case, `#`, since you are running as root)

You can look around, and you will find that you are in a barebones Ubuntu system. Type `neofetch` for a fancy display of your system, as the container sees it.

## Catkin Workspaces

_Enter the catkin workspace: `cd ~/catkin_ws`_ (you can type `cd ~/c` then hit `tab`).

Type `ls` and look at the folders:

```*
build  devel  src  temp
```

-   `build/` contains all the auto-generated files used to build your code. We won't be messing with this.
-   `devel/` Contains the information from the builds that actually ran. You will find executables from compiled C/C++ programs here, which can be useful for debugging.
-   `src/` will contain our code.
-   `temp/` is not normally in `catkin_ws/`, but was added by Docker, by copying files in.

Take a closer look at `temp/`; type `tree temp` into the console. This should look familiar, since it's an exact copy of this repository! This is how we are moving files into the repositories. It is entirely up to us to get the correct files from `temp` into `src`. _For now, just type these commands:_

-   `mv temp/python-subscriber/ src/python-subscriber`
-   `mv temp/cpp-publisher/ src/cpp-publisher`

Remember that the `tab` key will autocomplete files/directories/commands for you if you type out enough for it to know what you mean.

**Challenge 1! The answer is at the [bottom of the page](#Challenge-1).**

> Take a look at the output from when you ran `make`. Just above those last two lines that indicate success, there was an error! This error can be fixed by creating the file `file-setup.sh` in the repository you cloned. That is, it should be in the directory `practice-ros/file-setup.sh`. This file actually exists in any repository that needs files to be built, and you'll need to modify it to get your code working repeatedly!
>
> -   The purpose of `file-setup.sh` is to do the two move commands you just types for you. It will be run every time you create the container.
> -   Put the above commands into the file, BUT CHANGE THE FILE PATHS! I wrote the Docker container to put some environment variables into the system just for this.
> -   `$temporary_package_directory` will refer to `temp/`
> -   `$final_package_directory` will refer to `src/`
> -   **PLEASE** use these variables in the file! If we ever need to move the temporary directory, `$temporary_package_directory` will be updated, but your file won't change, so the variable will keep it up to date.
> -   You can test any of these variables by typing `echo $var_name` into the container.
> -   Use this template:
>     ```shell
>     # file-setup.sh
>     echo Copying files...
>     cd $temporary_package_directory
>     # Update this line, and have one per project in the repository:
>     mv project_name $final_package_directory/project_name
>     ```
> -   **IMPORTANT: If you're working on windows or mac, creating a file and typing this in will cause a error when it runs.** You need to change the line endings of the file. In Atom Editor, type `ctrl-shift-L`, in Notepad++, go through the menus `Edit > EOL Conversions > Unix (LF)`. Save after changing.
> -   This error will show issues with unrecognized commands, and will contain `\r`. If you see that error, this is the problem.

## Building the Files

Ok, so you're in the docker container, you (hopefully) did the challenge above, you've gone into the workspace, and when you type `ls src` it shows the two projects in this repository. All this preperation means it's finally time to build. _Type `catkin_make`. A few things will go past the screen._

Take a minute to look at the output; first, it tells what directories are important. Then, it dumps some configuration items, then it lists the packages that it found. These are what we've been referring to as projects, which is the other name for them. It then processes the packages and after finishing, builds the C++ file. Python does not need to be built, so the Python work happens in the background.

If you run `tree devel`, you will see the file tree for the built files. The most interesting is `cpp_publisher_node`, which should be green (indicating an executable file) and is the built output of the C++ code.

## Running the Code

Finally, we can get these working.

You'll want a few more PowerShell windows open. Open a window, type in the `docker exec ...` command above. (You may just be able to hit the up arrow to bring up the last command).

_In the second bash session, type `roscore`._ This will start ROS, which allows the two programs to communicate with each other. It will give some information about the startup and network and such.

_In the third bash session, type `rqt_graph`._ Make sure that MobaXterm is open! This will start a GUI. You won't see anything special yet.

```shell
# If you see this when running rqt_graph, you need to run MobaXterm and make sure the X11 server is running!
QXcbConnection: Could not connect to display host.docker.internal:0
Could not connect to any X display.
```

Time to run the programs! We'll start with the Python code. Remember, this is the _subscriber_, so it will listen and wait for messages on the topic. If it gets one, it prints it out. The code is referred to by the name of the project (according to the `CMakeLists.txt`) and the Python file you want to run.

All programs are run using `rosrun`. The format of the command is `rosrun [project_name] [node_name] <args>`. Project name is the name defined in `CMakeLists.txt`, not by the folder name. `node_name` is either the Python program or the node name given for a C++ executable in `CMakeLists.txt`. `args` are the arguments to pass to the program, but we won't need those now.

_Challenge 2: In a fourth bash session, use `rosrun` to run the python program. Use the project name defined in the Python project's `CMakeLists.txt`. (Answers are still at bottom, but try to figure this out!)_

Refresh the rqt_graph from the GUI using the button in the top left. You should see `/py_listener`. Since it is a node (an instance of the program), it is a circle.

_Challenge 3: In a fifth bash session, use `rosrun` to run the C++ program._

Look at the output of both the C++ program and the Python program. They should match! This means you've just gotten two programs to talk using ROS!

Refresh the rqt_graph from the GUI. Now you can see the connection, both by name (`\chatter`) and by visual connections to the nodes.

## ROS Tools

Now you've got these running and talking. But what if something goes wrong? This is where a number of extra helper commands come in. This touches on some slightly more advanced topics in ROS than those we've been going though.

Go back to your first bash session. Type `rostopic`. There are a few things worth trying right now (type `rostopic [command]`):

-   `list`: List all topics. Note that our `chatter` topic is not the only one.
-   `echo <topic>`: Echo a topic's outputs. This is very similar to what our Python program is doing, but it's built in to `rostopic`. Use our topic name, `/chatter`. Interestingly, this has a datafield called `data`. Use `type` (below) to see more about topic's datatype.
-   `type <topic>`: Print the topic's datatype. This can be customized!
-   The type is a string, but it's not just a regular string. The field `data` had to be defined as a string somewhere.
    -   `rosmsg show std_msgs/String` will read the file that was used to define the `std_msgs/String` topic type. Now we see the base definition: the String message is a message which contains `data`, of type String. That might sound obvious, but think about why we can't just transport whatever type we want, without existing messages like `std_msgs/String`, which have pre-defined objects.
-   `hz <topic>`
-   `info <topic>` basically what rqt_graph shows (but more boring)

Note that there are a LOT of ROS commands; type `ros` then hit `tab` twice to see a full list. The same goes for `rqt`.

One last interesting command is `rqt_bag`. There's too much here to go over, since we likely won't use this much. It opens a GUI which will allow you to use ROS bagfiles, which store the state of the session recorded. You can later take the recorded file and play it back. If you recorded the output of the C++ publisher, then closed it, then played it back, with the Python subscriber running, the subscriber will write out the same data as it would if the C++ file were running.

# Challenges

## Challenge 1

```shell
# file-setup.sh
echo Copying files...
cd $temporary_package_directory
# All variables used here are set by the dockerfile when it moves files into $temporary_package_directory
mv python-subscriber $final_package_directory/python-subscriber
mv cpp-publisher $final_package_directory/cpp-publisher
```

## Challenge 2

The command to run the Python executable is `rosrun rosrun py_subscriber main.py`

## Challenge 3

The command to run the C++ executable is `rosrun cpp_publisher cpp_publisher_node`
