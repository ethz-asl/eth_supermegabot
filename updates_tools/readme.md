## Abstract   
Package contains basic tools for updating the whole summerschool meta-pkg    

## Requirement  
* python (>=3.5)  
* install dependencies  

## Install 
``` 
pip install -r requirements.txt  
```

## Usage  
### Packages updating  
The app would load specified package name from given yaml then first delete old ones, then copy the new ones to certain path.  
Inputs arguments:  
* packages_list: list of (private) packages will be forked and updated.   
* source_ws: the catkin workspace where all source codes copied from  
* target_dir: the folder that will be released (copied to), it will create one if not exist  

```
cd updates_tools/   
python packages_updates.py [--packages_list PACKAGES_LIST] [--source_ws SOURCE_WS] [--target_dir TARGET_DIR]  

example:  
python3 packages_updates.py --packages_list ./packages_list --source_ws ~/smb_ws/ --target_dir ../summer_school_private
```



