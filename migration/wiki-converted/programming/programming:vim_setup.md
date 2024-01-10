#  Vim Setup # 
This tutorial is intended for Ubuntu 16.04, although it should work on most flavors of Debian Linux. This vim configuration is optimized for ROS development. 

###  Installation ### 
####  Step 1: Ensure you have the correct version of Vim #### 
I strongly suggest you remove your current Vim configuration to ensure that everything works smoothly. Before you start, run any uninstall scripts for Vim packages and plugin managers. When you're done, run:

```bash
rm -rf ~/.vim
sudo apt-get purge *vim*
sudo apt-get install python-dev
sudo apt-get install vim-nox-py2
```

####  Step 2: Installing vim plugins #### 
We will be using Vim-Plug, as well as installing YouCompleteMe, vim-ros, vim-cpp-enhanced-highlight, and vim-sensible. If you use a different plugin manager then just install them with that; it won't affect the other steps.
```bash
curl -fLo ~/.vim/autoload/plug.vim --create-dirs \
    https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim
```
Then copy this into your ~/.vimrc
<code vim>
call plug#begin('~/.vim/plugged')
Plug 'tpope/vim-sensible'
Plug 'taketwo/vim-ros'
Plug 'Valloric/YouCompleteMe'
Plug 'octol/vim-cpp-enhanced-highlight'
call plug#end()
let g:ycm_extra_conf_globlist = ['~/path/to/2018RobotCode/.ycm_extra_conf.py']
let g:ycm_semantic_triggers = {
\   'roslaunch' : ['="', '$(', '/'],
\   'rosmsg,rossrv,rosaction' : ['re!^', '/'],
\ }
```
**Replace /path/to with your actual path to 2018RobotCode**

Install the plugins by opening vim and running ```:PlugInstall```
####  Step 3: Installing ycmd (YouCompleteMe server component) #### 
Run the following:
```bash
cd ~/.vim/plugged/YouCompleteMe
./install.py --clang-completer
```

##  Usage ## 
###  YouCompleteMe ### 
YouCompleteMe will automatically compile your code after you exit insert mode and highlight potential errors in red. Note that many errors are from files that are nonexistent until you compile everything with catkin_make. To auto-complete, simply select the desired auto-completion with your arrow keys and keep typing. 

##  Vim-ROS ### 
Use :Roscd to move between packages with tab completion. Use :Rosed to open files within a package with tab completion.