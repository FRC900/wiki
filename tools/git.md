### Table of Contents

*   [Using Git](#using_git)
    
    *   [Branches](#branches)
            
    *   [How to Use Git for Team Code Development](#how_to_use_git_for_team_code_development)
        
    *   [Resolving Conflicts](#resolving_conflicts)
        
    *   [Rebasing](#rebasing)
        
        *   [Squashing a Branch](#squashing_a_branch)
            
        *   [TODO](#todo)
            
    *   [Git LFS (Large File Storage)](#git_lfs_large_file_storage)
        
        *   [Usage](#usage)
            
        *   [How it actually works](#how_it_actually_works)
            

Using Git
=========

The purpose of this page is to introduce the most useful git commands and best practices when using git.

Presentation with introductory Git information: [https://docs.google.com/presentation/d/19Ptx7wfajie3Bin_6Ovp5f2JD2-9qRCeMK6ctcNWlek/edit#slide=id.g179a4ab37e_0_25](https://docs.google.com/presentation/d/19Ptx7wfajie3Bin_6Ovp5f2JD2-9qRCeMK6ctcNWlek/edit#slide=id.g179a4ab37e_0_25 "https://docs.google.com/presentation/d/19Ptx7wfajie3Bin_6Ovp5f2JD2-9qRCeMK6ctcNWlek/edit#slide=id.g179a4ab37e_0_25")


### Branches

**Create new branch:**  
Run the below 2 commands.

`git checkout -b new_branch_name`: this creates a new branch on your laptop.

`git push -u origin new_branch_name`: this adds the branch to the GitHub repository on the internet.

**Create a pull request:**

If you intend to eventually merge the branch back into the main set of code, this might be the time to create a pull request. A pull request is part of the process for getting the code merged into the main branch. See [here](/doku.php?id=programming:code_development_process "programming:code_development_process") for details.

**Getting a local copy of a branch**

If someone else has created a branch and pushed it, you can get a local copy on your machine to work from.

`git branch -r`

will list remote branches - that is, branches which exist on github but not your local machine. They will all start with `origin/` followed by the branch name. Use

`git checkout -t [branch name including origin/]`

to make a local branch which tracks that remote copy. Your local copy of the branch will not include the origin/ in the name.

From this point, you can work in the branch as if you created it yourself.

**Deleting a Branch**  
Run the below 2 commands.
```
git push --delete origin branch_name
```
This deletes a branch from the internet GitHub repository.
```
git branch -d branch_name
```
This deletes a branch from your computer (your computer forgets it exists until you do git checkout on it again). If the branch has been deleted from the internet repository, it's gone from your world completely.


How to Use Git for Team Code Development
----------------------------------------

Developing code on Team 900, multiple students at any time could be changing related or even the same file on their computer. If not done correctly, this has the potential to create a huge mess in the file and potentially lose someone's important changes. However, there are a few simple ways to avoid the majority of these issues with some important best practices. Before editing a file, it is important to make sure all of your local code is up-to-date by doing `git pull -r` and resolving any conflicts. Once everything is up-to-date without conflicts, You should be safe enough to make changes to any file, but you should attempt to be aware of what the other programmers are changing to minimize any conflicts. After you are done making changes, it is incredibly important to

1.  Compile (`natbuild` and `crossbuild`)
    
2.  Test (run code in sim or on a robot)
    
3.  Commit (`git commit [filename]`)
    
4.  Push (`git push`)
    

#### NEVER PUSH UNTESTED CODE TO MAIN!!! (Or really never push code to main, instead, create pull request off a branch you've pushed)


Resolving Conflicts
-------------------

Often when pulling code, there will be some form of merge conflict where your local changes conflict with someone else's. Resolving merge conflicts really isn't difficult, but is confusing at first.

Documented in the other git page under cirruculum TODO - combine the two


Rebasing
--------

While working in a branch, code will also be merged into main. At times, you'll want that new code incorporated into your branch. There are several ways to do this. The recommended approach is to _rebase_.

A bit of background. A branch is a piece of code history which branches off the main branch.

```
A -- B -- C -- D   <-- main
                \ 
                 \-- X -- Y    <- branch
```

Here, the commits A, B, C, D happened on main. Then a branch was created with commits X and Y. The base of the branch with X&Y is commit D - the last place it shared a common history with main

Now, if commit E is added to main, the tree looks like this :

```
A -- B -- C -- D  -- E   <-- main
                \ 
                 \-- X -- Y  <-- branch
```

D is still the base of the branch with X&Y, but main has moved forward and now has newer code.

If we need to have the new code from commit E as part of the work on branch X&Y, the rebase command can be used. Basically what this does is move the base of the branch in question to start at the new commit and then applies all of the branch's commits on top of that. So after `git rebase main` the tree would look like this:

```
A -- B -- C -- D  -- E   <-- main
                      \ 
                       \-- X -- Y  <-- branch
```

This is called “changing history” - it has revised the git history to make it that the branch started from commit E and then added commits X and Y.

Note that this will sometimes require resolving conflicts, just like any other time code is revised.

At this point, the local branch will be way out of sync with the remote version of the branch. To resolve this issue, use the command `git push --force-with-lease`.

Also note - rebase will update to the most recent copy of main on your local repo. To make sure you get the absolute latest, be sure to pull from main before rebasing.

```
git checkout main
git pull -r
git checkout my_branch
git rebase main
git push --force-with-lease
```

#### Rewriting history is dangerous

As anyone who has seen a sci-fi time travel movie knows, rewriting history is dangerous. While git won't let you accidentally kill your great-grandparents thus dooming your very existence before it even started, it will let you mess up the main branch. Which is almost as bad.

The short version - if you ever think the solution to a problem on main is to do a force push, think again. You'll be corrupting the history of everyone else's copy of the main robot code.

**Only do force pushes on your own development branches.**


### Squashing a Branch

There will be times where a branch ends up with a lot of commits. Code will get changed, tested, changed, changed back, changed back again, and so on. Lots of these changes undo or totally remove other changes. This is totally normal for code development - but can clutter up the git commit history a bit.

This is even worse when rebasing that branch. Rebase moves to the current head of the branch you're rebasing to and then applies each commit from your branch one at a time. If there's a conflict in a branch, it stops and makes you fix them. That's all good - until you end up fixing conflicts in an intermediate commit for code which you know doesn't even exist in the most recent version of your code.

Here, a technique called squashing can help. What squashing the commit history does is take the net change from all the commits you've made and combine them into a single one. This is nice because it removes all of the intermediate back-and-forth that sometimes happens. And you'll only need to resolve one set of conflicts rather than lots of them when rebasing.

The downside is that it gets rid of all the intermediate history. If you think you might need to refer back to an intermediate commit, do not squash the commit history because those intermediate commits will no longer be there. Post-squashing, only the final state of the code will be available.

To do a git squash, identify the commit just prior to the one you wish to combine. For example, to squash this entire branch

```
A -- B -- C -- D   <-- main
                \ 
                 \-- X -- Y --- Z -- XX -- YY -- ZZ   <- branch
```

you'd use `git rebase -i D`, where D is the commit ID of D. That tells git you want to combine every commit starting at X up to the current member of the branch ZZ into one final commit.

git will open up an editor with a list of commits along the path from X to ZZ. Before each of them is `pick`. For every commit **except the first** change `pick` to `squash`. Save and exit the editor.

You will then get another editor window. This one lets you decide which commit messages to use for the new squashed commit. By default, it will be all of the commit messages each commit being squashed. You can use that as is, but sometimes it makes sense to clean up ones which don't matter (e.g. fixing bugs in code which doesn't exist anymore). Anything in comments (starting with the `#` character) is left out of the message.

After saving and exiting, the branch will be squashed:

```
A -- B -- C -- D   <-- main
                \ 
                 \-- Q   <- branch
```

One commit holding all of the net changes from the old series of commits.


### TODO


Git LFS (Large File Storage)
----------------------------

Firstly, it is important to recognize that the name “Large File Storage” is not entirely accurate. LFS works best for binary, non-textual files, most commonly multimedia files, such as video, audio, or pictures. This is largely because LFS lacks built in support for many git commands designed for textual data, such as `git diff`.


### Usage

*   Install it
    
    *   [https://github.com/git-lfs/git-lfs/wiki/Installation](https://github.com/git-lfs/git-lfs/wiki/Installation "https://github.com/git-lfs/git-lfs/wiki/Installation")
        
*   Adding items to LFS
    
    *   `git lfs track "<your file>"`
        
        *   `“\<your file>”` can either be a file extension (e.g. “*.txt”), or a file (e.g. “test.txt”)
            
*   Retrieving items from LFS
    
    *   This is handled automatically, as `git clone` pulls LFS files once installed. **However**, if the repository was cloned prior to installing LFS, it will not be properly initialized. If this is the case, simply run `git lfs fetch` while inside the repo dir.
        
*   Overall, LFS is highly integrated into git, so there is little to no effect on workflow.
    
*   [For further information/examples, the official LFS tutorial is an excellent resource](https://github.com/git-lfs/git-lfs/wiki/Tutorial "https://github.com/git-lfs/git-lfs/wiki/Tutorial")
    


### How it actually works

Git LFS works by replacing the file tracked with LFS in the git repository with a placeholder file, which directs the git LFS client to download the file from an external source called the LFS store. This avoids the issue of local repository balooning by storing large files remotely and downloading them when needed, instead of storing everything locally.

[Home](/README.md)