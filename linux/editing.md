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


