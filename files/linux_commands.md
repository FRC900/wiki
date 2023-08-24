# Overview of Linux Commands

ls, cd, mv, python3, apt, git, bash, echo, cat, less, vim, code

## **Filesystem Commands**

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

## **Text Editing/Programming**

### [`python3`](#python3)
- **Description**: the Python interpreter!
- **Usage**: `python3 [program]`
    - If `[program]` is not provided, a Python interpreter will show up. Otherwise, your file will be executed.
- **Example**:
    ```bash
    bean@bean-ThinkPad-X1-Yoga-Gen-6:~/linux_commands_intro$ python3
    Python 3.10.12 (main, Jun 11 2023, 05:26:28) [GCC 11.4.0] on linux
    Type "help", "copyright", "credits" or "license" for more information.
    >>> print("Hello World from Python")
    Hello World from Python
    ```

    ```bash
    bean@bean-ThinkPad-X1-Yoga-Gen-6:~/linux_commands_intro$ cat hello_world.py
    print("Hello World from a Python program")
    bean@bean-ThinkPad-X1-Yoga-Gen-6:~/linux_commands_intro$ python3 hello_world.py
    Hello World from a Python program
    ```

### [`vim`](#vim)
- **Description**: a text editor
- **Usage**: `vim [file]`
    - For example, `vim test.py` would edit the file `test.py`
    - Vim has a bunch of really cool features -- this is the minimum you need to know to use it:
        - `i` = <ins>i</ins>nsert mode, lets you type and move around the file
        - escape key = exit insert mode
        - escape key + `:wq` = save and quit
        - escape key + `:qa!` = quit without saving


