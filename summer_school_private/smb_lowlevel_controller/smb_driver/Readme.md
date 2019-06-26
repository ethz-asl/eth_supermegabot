# SMB driver


## udev 
To allow the automated detection of the smb-serial connector, copy the 
contents of the udev folder to your local udev-rules directory:

**commands**:

```bash
roscd smb_driver
sudo cp ./udev/* /etc/udev/rules.d/
```

These rules generate a symlink for the usb to serial adapter when plugging it in to 
```/dev/ttySMB```. 