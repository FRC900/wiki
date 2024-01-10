#= Design Reviews #=
We often hold design reviews to look at designs during different stages of the design process to catch potential trouble spots and make revisions. It's important to remember that design reviews aren't someone just sitting there criticizing your work; someone is helping you make your design better and you should be open to and appreciate of any feedback received.

# Guidelines for Design Reviews #
Following are some guidelines for design reviews (adapted from [[https:*developers.redhat.com/blog/2019/07/08/10-tips-for-reviewing-code-you-dont-like/|here]]):
 ==== Rephrase your objection as a question ====
  - Bad: “This change will make XXX impossible.” (This is hyperbole; is it really impossible?)
  - Good: “How can we do XXX with your change?”
 ==== Avoid hyperbole ====
  - Simply state your concerns and ask questions to help get to the desired outcome.
  - Bad: “This change will destroy our robot performance.”
  - Good: “It seems like doing X might be slower than existing Y; have you measured/gathered data to show it isn’t?”
  - Better (if you have time): “In the meantime, I am gathering data to try to verify that X is not slower than Y.”
  - Also good: “This change changes this simple mechanism to a more complicated mechanism; won’t this affect performance?”
 ==== Keep snide comments to yourself ====
  - Some thoughts are better kept to yourself. If you can’t be civil, don’t engage.
  - Bad: “I think this change is bad and will ruin everything.”
  - Bad: “Are you sure that mechanical engineering is the right career path for you?”
 ==== Engage positively ====
  - Maybe you had a different idea about how to solve a problem? If you engage positively, you might end up discovering a solution that is better than either original option.
  - Bad: “This change sucks, my version is better.”
  - Good: “I also have a similar change at this location XXX: maybe we can compare and/or combine ideas.”
  - Also good: “I have a similar change in progress, but I chose to do X because ZZZ; why did you choose Y?”
 ==== Remember that not everybody’s experience is identical to yours ====
  - An otherwise completely competent engineer could go for years without knowing some fact that you take as common sense. It’s okay to state the obvious, as long as you aren’t patronizing or snide about it. A great thing to do is to turn a mistake into a learning experience. 
  - Bad: “Can’t you see that this is obviously wrong?”
  - Good: “This is incorrect because it causes interference with another mechanism.”
 ==== Don’t diminish the complexity of something that’s not obvious ====
  - Remember that things that are obvious to you may not be obvious to everyone. Suggesting alternative approaches and pointing out useful examples can help get everyone on the same page.
  - Bad: “Why not simply get a bigger piston?”
  - Good: “It might be possible to get a bigger piston, which would simplify this part (see XXX for an example).”
 ==== Be respectful ====
  - Sometimes a submission just doesn’t meet a minimum standard for quality. It’s okay to say so, but it doesn’t cost anything extra to be respectful.
  - Bad: “This is a stupid design.”
  - Good: “Thanks for your contribution. However, it cannot be accepted in its current form; there are multiple problems (as outlined above).”
  - Also good: “As outlined above, there are multiple problems with this design.  Maybe we could back up a step and talk about the design requirements instead?  That could help us find a path forward.”
 ==== Manage expectations (and your time) ====
  - If a design is too large to be reasonably reviewed, it is okay to let the submitter know right away. There is never any shame in asking for help if you need it.
  - Bad: “I’m not reviewing this, it’s too big.”
  - Also bad: Ignoring it until it goes away.
  - Good: “Could you please break this down into smaller parts? I do not have a lot of time for design reviews and this one is just too large/complex to review in one pass.”
 ==== Say please ====
  - Just saying “please” goes a long way toward showing that you respect the designer’s time, especially when you want something to be different due to formatting or style, which might seem to be a minor detail of the change. Examples:
  - “Could you please use a master sketch instead of performing multiple extrusions?"
  - “Could you please use sketch relations instead of manually defining so many dimensions?”
 ==== Start a conversation ====
  - If, after all this, you still don’t like something but you’re not sure why, you might have to just live with it. But it’s also okay to say, “I don’t like this and I’m not sure why, can we talk about it?” It’s a reasonable thing to ask, and even though it might take a little time, it’s often worth the investment because now you have two people who are both learning (one by explaining and one by listening) rather than two people who are opposed to each other.
  - Even skilled and experienced engineers should be able to say “I don’t understand why I don’t like this”; it’s not an invitation to attack the position of the reviewer but rather an honest quest for knowledge.