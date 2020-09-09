# ROS Practice

This practice guide covers:

-   Docker images
-   ROS workspaces & automating moving code
-   Compiling C/C++ and using Python programs within ROS
-   Running your programs
-   Visualizing the system with `rqt_graph`

It should hopefully be useful too in case you need to refer to the code/file structure used here.

For a more in-depth guide to ROS, you'll want to check out their [core tutorials](http://wiki.ros.org/action/fullsearch/ROS/Tutorials?action=fullsearch&context=180&value=linkto%3A%22ROS%2FTutorials%22).

# What you'll need

-   A way to clone this GitHub repository
-   Docker
-   Windows PowerShell (or Mac/Linux terminal) for all windows commands and for accessing Docker containers
-   MobaXterm
-   GNU Make installed and on you path (you should be able to run `make` from your command line)

* * *

# The Guide

### Setup

Run docker if you have not already. Nothing should open, but there should be a notification eventually that says Docker is Running.

Run MobaXterm. This should automatically start an X11 Server (the X in the top right will become colorful after a few seconds). X11 is what allows you to see a GUI that is actually running in the Docker container.

If you're on Windows, you'll want to use PowerShell, rather than Command Prompt.

### The Directory Structure

Look at the directory structure. This is important for how files will get moved around. Type `tree` from within the cloned repository.

This is what it should look like, with some things ommitted:

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

### Docker

From the command line (PowerShell), test that docker is running: `docker ps`. You should get a (blank) list of all running containers:

```shell
> docker ps
CONTAINER ID        IMAGE               COMMAND                  CREATED             STATUS              PORTS                NAMES
```

In powershell, `cd` into the cloned repository (`practice-ros`). You're now in the main repository, but you'll actually want to be in the `ros-docker` repository from here on out, so `cd` there.

_Type `make` into powershell._ If it's your first time running one of our docker containers, this command will take a while to finish. It is downloading the base image, ROS, and many helpers and programs to get all our docker images on the same page. Remember that `make` is just being used as a convenience; it is actually just running more commands for you.

Once docker is done building the image, there will be no exiting; it will print this and continuously run:

    Container started successfully.
    Container ID: 465e402cbe28

From here, you can type `^C` (`^` = `ctrl key`), which will kill the container and stop the OS. If you do that, then type `make` again, you'll see docker run through all the layers it ran through before, only much faster, since it does not need to re-download anything, as those layers were saved.

More useful for us, though, is another shortcut: _after seeing that the container finished building, enter `^P` + `^Q`._ This will detach from the container while it runs in the background, allowing you to continue using your PowerShell.

_After detaching, you can enter the container using the command `docker exec -it piranhabot_container bash`._

-   This will execute (`exec`) the command `bash` in the `piranhabot_container` (it was named by the `Makefile` automatically), using interactive mode (`-it`). In other words, it will start a bash session in the container. (Note that you can technically use any command instead of `bash`, like `ls`, `nano`, or others, including local executables.)

You should now be in the container. Your pompt will have changed from the windws `>` to a linux prompt (in this case, `#`, since you are running as root)

You can look around, and you will find that you are in a barebones Ubuntu system. Type `neofetch` for a fancy display of your system, as the container sees it.

### Catkin Workspaces

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

**Challenge 1! The answer is at the [bottom of the page](#Challenges).**

> Take a look at the output from when you ran `make`. Just above those last two lines that indicate success, there was an error! This error can be fixed by creating the file `file-setup.sh` in the repository you cloned. That is, it should be in the directory `practice-ros/file-setup.sh`. This file actually exists in any repository that needs files to be built, and you'll need to modify it to get your code working repeatedly!
>
> -   The purpose of `file-setup.sh` is to do the two move commands you just types for you. It will be run every time you create the container.
> -   Put the above commands into the file, BUT CHANGE THE FILE PATHS! I wrote the Docker container to put some environment variables into the system just for this.
> -   `$temporary_package_directory` will refer to `temp/`
> -   `$final_package_directory` will refer to `src/`
> -   **PLEASE** use these variables in the file! If we ever need to move the temporary directory, `$temporary_package_directory` will be updated, but your file won't change, so the variable will keep it up to date.
> -   You can test any of these variables by typing `echo $var_name` into the container.
> -   Use this template
>     ```shell
>     # file-setup.sh
>     echo Copying files...
>     cd $temporary_package_directory
>     # Update this line, and have one per project in the repository:
>     mv project_name $final_package_directory/project_name
>     ```

### Building the Files

# Challenges

```shell
# file-setup.sh
echo Copying files...
cd $temporary_package_directory
# All variables used here are set by the dockerfile when it moves files into $temporary_package_directory
mv python-subscriber $final_package_directory/python-subscriber
mv cpp-publisher $final_package_directory/cpp-publisher
```
