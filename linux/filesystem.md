## **Linux Filesystem**

### **Filesystem Basics**
- **directory**: a folder
- **`..` aka parent directory**: the directory "above" a directory (e.g. the parent directory of `~/2023RobotCode/docker`, represented by `~/2023RobotCode/docker/..`, is `~/2023RobotCode`)
- **subdirectory**: the opposite of a parent directory, a subdirectory is a directory below another directory
- **path**: the location of a file or directory, components are separated by slashes (`/`)
    - e.g. to get to `/home/ubuntu/2023RobotCode/zebROS_ws/src`, go to `/`, then `home`, then `ubuntu`, then `2023RobotCode`, then `zebROS_ws`, then `src`
- **`.`**: your current directory, to run scripts in your current directory you will need to put `./` before them (e.g. `./docker-run`)
- **`~` aka home directory**: usually located at `/home/[username]`, this is where you put all of your stuff
- **`/` at the start of a path**: the root directory (if you visualize your directories as a tree, this is the directory at the "top" of the tree)

### [`ls`](#ls)
- **Description**: <ins>l</ins>i<ins>s</ins>ts files in a directory
- **Usage**: `ls [directory]`
    - `[directory]` is optional, if not provided `ls` will list the contents of the current directory
- **Example**:
    ```bash
    bean@bean-ThinkPad-X1-Yoga-Gen-6:~/linux_commands_intro$ ls
    hello  world  zebra
    ```

    Advanced version (`ls -al` shows hidden files and a lot more info):
    ```bash
    bean@bean-ThinkPad-X1-Yoga-Gen-6:~/linux_commands_intro$ ls -al
    total 28
    drwxrwxr-x  3 bean bean  4096 Aug 24 13:07 .
    drwxr-x--- 73 bean bean 12288 Aug 24 13:07 ..
    -rw-rw-r--  1 bean bean    22 Aug 24 11:24 hello
    -rw-rw-r--  1 bean bean    43 Aug 24 13:07 hello_world.py
    drwxrwxr-x  2 bean bean  4096 Aug 24 13:00 world
    -rw-rw-r--  1 bean bean     0 Jul  9 16:44 zebra
    ```

### [`cd`](#cd)
- **Description**: <ins>c</ins>hange <ins>d</ins>irectory
- **Usage**: `cd [directory]`
    - Special cases:
        - `cd ..` = go to the parent directory
        - `cd .` = stay in the current directory (why would you want to do this?)
        - `cd` = go to your home directory (`~`)
        - `cd -` = go to the last directory you were in
- **Example**:
    ```bash
    bean@bean-ThinkPad-X1-Yoga-Gen-6:~/linux_commands_intro$ cd world
    bean@bean-ThinkPad-X1-Yoga-Gen-6:~/linux_commands_intro/world$ cd ..
    bean@bean-ThinkPad-X1-Yoga-Gen-6:~/linux_commands_intro$ cd
    bean@bean-ThinkPad-X1-Yoga-Gen-6:~$ cd -
    /home/bean/linux_commands_intro
    bean@bean-ThinkPad-X1-Yoga-Gen-6:~/linux_commands_intro$
    ```

### [`mv`](#mv)
- **Description**: <ins>m</ins>o<ins>v</ins>es things
- **Usage**: `mv [source] [destination]`
- **Example**:
    ```bash
    bean@bean-ThinkPad-X1-Yoga-Gen-6:~/linux_commands_intro$ ls
    hello  move_me  world  zebra
    bean@bean-ThinkPad-X1-Yoga-Gen-6:~/linux_commands_intro$ mv move_me world/
    bean@bean-ThinkPad-X1-Yoga-Gen-6:~/linux_commands_intro$ ls
    hello  world  zebra
    bean@bean-ThinkPad-X1-Yoga-Gen-6:~/linux_commands_intro$ cd world/
    bean@bean-ThinkPad-X1-Yoga-Gen-6:~/linux_commands_intro/world$ ls
    another_file  move_me  this.is.a.file.txt
    ```

### [`pwd`](#pwd)
- **Description**: <ins>p</ins>rint <ins>w</ins>orking <ins>d</ins>irectory (prints the path to the directory you are in)
- **Usage**: `pwd`
- **Example**:
    ```bash
    bean@bean-ThinkPad-X1-Yoga-Gen-6:~/linux_commands_intro$ pwd
    /home/bean/linux_commands_intro
    ```

### [`cat`](#cat)
- **Description**: prints out a file's contents
- **Usage**: `cat [file]`
- **Example**:
    ```bash
    bean@bean-ThinkPad-X1-Yoga-Gen-6:~/linux_commands_intro$ ls
    hello  world  zebra
    bean@bean-ThinkPad-X1-Yoga-Gen-6:~/linux_commands_intro$ cat hello
    Hello striped zebras!
    ```

[Home](/README.md)