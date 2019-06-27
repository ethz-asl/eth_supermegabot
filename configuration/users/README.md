# Setting up a user account on the robot
### Connect the battery
### Turn on the NUC
### Connect to the computer
```
ssh smb@10.0.0.5
``` 
(ping it `ping 10.0.0.5` if it does not work to check connectivity)
### Create a user 
```
sudo adduser your_username
```
### Add the user to the developers group
```
sudo usermod -aG sudo,developers,dialout,plugdev,adm,lpadmin,cdrom,dip,sambashare your_username
``` 
### Generate ssh keys
```
ssh-keygen
eval `ssh-agent`
ssh-add ~/.ssh/id_rsa
```
* Copy the key: `cat ~/.ssh/id_rsa.pub` and copy the printed key
* Go to github, add the ssh key (https://github.com/settings/keys)


