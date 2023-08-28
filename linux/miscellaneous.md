## **Miscellaneous**

### [`sudo`](#sudo)
- **Description**: runs commands as the root user/superuser (<ins>su</ins>peruser <ins>do</ins>), **use caution when using `sudo`**
- **Usage**: `sudo [command]`

### [`apt`](#apt)
- **Description**: the Debian Linux package manager, you can use it to install things
- **Usage**:
    - `sudo apt install [package]` installs a package/program, you need root permissions to do this (hence the `sudo`)
    - `apt search [query]` searches for packages matching `[query]`
    - `apt show [package]` shows specific details about `[package]`
- **Example**:
    ```bash
    bean@bean-ThinkPad-X1-Yoga-Gen-6:~$ sudo apt install python3
    [sudo] password for bean:
    Reading package lists... Done
    Building dependency tree... Done
    Reading state information... Done
    python3 is already the newest version (3.10.6-1~22.04).
    python3 set to manually installed.
    The following packages were automatically installed and are no longer required:
    chromium-codecs-ffmpeg-extra ethtool gstreamer1.0-vaapi libflashrom1 libftdi1-2 libgstreamer-plugins-bad1.0-0 libllvm13 wmdocker
    Use 'sudo apt autoremove' to remove them.
    0 upgraded, 0 newly installed, 0 to remove and 221 not upgraded.
    ```

    ```bash
    bean@bean-ThinkPad-X1-Yoga-Gen-6:~$ apt search python
    Sorting...
    Full Text Search...
    2to3/jammy-updates,jammy-updates,jammy-security,jammy-security 3.10.6-1~22.04 all
    2to3 binary using python3

    accerciser/jammy,jammy 3.38.0-1 all
    interactive Python accessibility explorer for the GNOME desktop

    ... many results somewhat related to Python ...
    ```

    ```bash
    bean@bean-ThinkPad-X1-Yoga-Gen-6:~$ apt show python3
    Package: python3
    Version: 3.10.6-1~22.04

    ... more stuff ...
    ```

### [`bash`](#bash)
- **Description**: the program that runs all of these commands, called a shell
- **Usage**: `bash`
- **Example**:
    ```bash
    bean@bean-ThinkPad-X1-Yoga-Gen-6:~$ bash
    bean@bean-ThinkPad-X1-Yoga-Gen-6:~$ exit # a new shell!
    exit
    bean@bean-ThinkPad-X1-Yoga-Gen-6:~$ # same shell as original
    ```

### [`grep`](#grep)
- **Description**: searches for text
- **Usage**: `grep "string to search for"`
    - Often you pipe (`|`) the output of other commands into grep (see the example).
- **Example**:
    ```bash
    bean@bean-ThinkPad-X1-Yoga-Gen-6:~/linux_commands_intro$ cat hello
    Hello striped zebras!
    Here is another sentence
    And another one!
    bean@bean-ThinkPad-X1-Yoga-Gen-6:~/linux_commands_intro$ cat hello | grep zebras
    Hello striped zebras!
    bean@bean-ThinkPad-X1-Yoga-Gen-6:~/linux_commands_intro$
    ```

[Home](/README.md)