#  Jenkins Setup Stuff # 
  - Installed Jenkins on Azure using MarketPlace: https://azuremarketplace.microsoft.com/en-us/marketplace/apps/azure-oss.jenkins
  - Installed a new inbound security rule for port 443
  - Installed Letsencrypt using these directions: https://jenkins.io/blog/2017/04/20/secure-jenkins-on-azure/
  - Accessible here now: https://jenkins900.eastus.cloudapp.azure.com
  - Install Docker (instructions for 16.04, but should work for any): https://askubuntu.com/questions/938700/how-do-i-install-docker-on-ubuntu-16-04-lts
  - Allow jenkins user access to docker without needing sudo (pretty sketch): 
    - sudo groupadd docker
    - sudo usermod -aG docker jenkins
    - sudo service jenkins restart
  - Add credentials for Jenkins to get access to git repos


#  Additional Plugins to Install # 
  - Blue Ocean
  - GitHub Authentication
  - Green Balls
  - Slack Notification
  - Pipeline (?)

#  Required Credentials # 

  - Zebrabuild GitHub user account (bot account)
  - Slack notifications

#  Setting up the Multibranch Pipeline # 
  - Use zebrabuild credentials
  - Owner: FRC900
  - Discover Branches: All branches
  - Additional Behaviors: Checkout to matching local branch, Clean before/after checkout

#  GitHub OAuth # 
  - Configure global security
  - Leave GitHub web URI/API alone
  - Copy secret info from GitHub OAuth page in GitHub here
  - Minimum OAuth scopes acceptable
  - Authorization strategy follows:
  - ![](../../wiki-resources/programming/jenkins_auth_strategy.png)