#  Code Development Process # 

Our programming team is large. Without some sort of process to follow, there would be even more than the typical amount of chaos.

The process is designed to make sure that code which makes it onto the robot is tested reasonably well and has been checked by a few sets of eyes.  Code will always have bugs, but this should help reduce the number that end up on the robot.

#  Using Git Branches # 

The set of code which will be run on the robot at competitions is on the `master` branch.  We try to not develop code on that branch.  Instead, code is developed, tested, debugged and so on in other branches.  Once everyone is happy (enough) with it, the code is *merged* into master.  The process for requesting this merge is called a *pull request*.

#  Pull Requests ## 

###  Pull Request Overview ### 

A pull request (often shortened to PR, or even MR for merge request) is used to start the process of merging a branch into master.  A pull request is associated with a given branch.  It also has a place for the programmer submitting the request to explain what the code does.  It lets people easily review changes between the branch and master.  And it can let others comment on code - requesting changes or asking questions about it.

###  Starting a pull request ### 

There are a number of ways to start a pull request.

The first is offered when initially pushing a new branch. As an example :

`kjaget@ubuntu:~/2020Offseason$ git push origin pr_test \\
remote: \\
remote: Create a pull request for 'pr_test' on GitHub by visiting://\
remote:      https://github.com/FRC900/2020Offseason/pull/new/pr_test\\
remote: \\
To github.com:FRC900/2020Offseason.git\\
 - [new branch]        pr_test -> pr_test\\
`

Here, a new branch called pr_test was pushed to github for the first time.  Part of the status message includes a link which will open the PR create dialog for that branch. Enterprising students will notice that the only thing that changes in the link is the branch name...

Another option is to go to the `Pull requests` tab in our github repo (direct link [here](https://github.com/FRC900/2020Offseason/pulls).  There, click on the green `New pull request` button.  From this window, use the `compare:` pulldown to select the desired source branch. Leave the base: setting as master.  This will show the code differences between the source branch and master. Verify they are correct and click on `Create Pull Request`.

Another possibility from the `Pull requests tab` depending on the timing : there might be a yellow bar near the top listing a recently pushed branch. Click on the `Compare & pull request` button to start a PR for that branch.  This bypasses the step of having to select the source branch but otherwise ends up in the same place as the previous paragraph's method.  Note that the main repo code window will also have a similar `Compare & pull request` button.

Another option is to click on the `Branches` button near the top of the main FRC/2020Offseason page. This will list branches that you have created. Use the `New pull request` button to start a pull request for that branch.

###  Pull Request Dialog ### 

Regardless of the method used to get there, eventually you'll end up at the same `Open a pull request` window.  The important fields here 

  - A title field near the top.  This will be auto-filled from the commit message, but make sure it is succinct, meaningful, educational and so on.
  - The description field below it.  Explain the 5 W's - what was broken, why it was changed, where in the code we're looking, etc.  
  - Reviewers. Who you want to look at the code. Sometimes github will recommend people based on who else has looked at  the code. Programming mentors and leadership are always fair game, as are students you know have seen the code before, or even just people you want to show something to.
  - The list of differences in the files between the branch and master. There will be two columns - the one on the left is master, the one on the right the new branch.  Green lines are additions, red lines are deletions or changes.  Do a check to make sure all the changes you expect are in there. And that there aren't extra changes you don't want.  

After that, hit `'Create pull request`'.

###  It is software, nothing is permanent ### 

A pull request will automatically update to the latest commit in the branch it was created with. That means that you can update code on your branch, push it, and the pull request will be updated to match. This is a normal process - no one gets code right the first time through, and part of the process is making the code better.

Likewise, the fields in the pull request can be edited freely.  Do so if you need to - having accurate information is way more important than making us think you never make mistakes. We all do, no big deal, fix it and move on to the next problem.

###  Draft Pull Request ### 

If you want people to review code that is a work in progress to get feedback, that's a good use of a pull request too. In that situation, you know it isn't ready to merge but are looking for feedback.  In that case, use the `Create draft pull request` option - click the arrow next to `Create pull request` to find it.  That way people will know that this is a work in progress rather than your final masterpiece and not accidentally merge it.

It would also be good to note the fact prominently in the description. Or add WIP: (work in progress) to the title.

At some point, hopefully, the branch will be ready to merge. Use the "Ready For Review" button to promote it to a real, fully ready PR.

##  Code Reviews ## 

Once a PR is created, the next step is a *code review*.  Here, other students and mentors look over the code an offer feedback.  This could be anything from questions about why the code was written this way, ideas to test to make sure the code is OK, suggestions for rearranging code ... pretty much anything that comes to mind when reviewers read it.

This might sound scary. Do not worry.  Do not take it personally. Everyone on the team has the same goal - make a better robot through better code.  More sets of eyes on a piece of code helps us get to that goal.

Use the feedback from a code review as a learning opportunity. Maybe someone knows of a cool trick to make life easier for you. Or has seen a bug in similar code they wrote and can save you time that way.

Or if the reviewer isn't sure about something, a chance to teach. It goes both ways ... no one is perfect, including people reviewing the code.  Explaining how the code is correct will help to teach another team member something new.

So in summary - there are only opportunities for improvement here.  Take advantage of them.

###  As a reviewer ### 

Read the above section, then make sure what you're saying fits in with that approach.  Your goal is to make the software, and software people, better. Mentoring doesn't have to come from adult mentors, here's your chance to be part of the process.

There are two options for code review.  Both are started from the `Files Changed` tab in the PR.  This will bring up a list of difference between the two branches (the PR and master in this case).

Hovering over a source code line will show a blue plus. Click on that to bring up a comment field.  Enter comments. After that, there are two choices.

`Add Single Comment` creates the comment immediately.  You can do this multiple times for different parts of the code.

`Start a review` queues up this comment with others.  They are only seen once you click on the `Finish your review` button at the top right.  That allows you to add another comment (overall summary?) and then publishes the review comments.


###  Dealing with code reviews ### 

Update code as needed on your local machine. Commit and push the changes to the branch.  

Then, reply to comments explaining the changes in reaction to those comments.  Resolve simple comments, let the code reviewer resolve more involved discussions.

##  Merging ## 

Once all review comments have been resolved, the branch can be merged into master. This will be done by a reviewer, likely a mentor or leadership student.  

If a PR has been reviewed and is ready to merge but is sitting around, please ping one of us to get it merged.  It probably just got lost in the shuffle so remind us to take a look.
