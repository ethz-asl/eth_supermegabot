# Setting up user account on the robot
### Connect battery
### Turn on the nuc
### Connect to the computer
```
ssh smb@10.0.0.5
``` 
(ping it `ping 10.0.0.5` if it does not work to check connectivity)
### Create user
```
sudo adduser username
usermod -aG sudo username
``` 
### Add the user to the developers group
```
sudo adduser username developers
```
### Generate ssh keys
```
ssh-keygen
eval `ssh-agent`
ssh-add ~/.ssh/id_rsa
```
* Copy the key: `cat ~/.ssh/id_rsa.pub` and copy the printed key
* Go to github, add the ssh key (https://github.com/settings/keys)


