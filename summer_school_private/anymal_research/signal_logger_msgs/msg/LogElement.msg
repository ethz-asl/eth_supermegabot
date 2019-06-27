## Contains the properties of a logger element with name
## 'name'. The flag 'is_logged' determines whether the 
## element is currently logged. The collection frequency is
## given by 'divider' -> update_freq = controller_freq/divider.
## The 'action' defines if the data logged should be published,
## saved or both. The buffer size is defined by 'buffer_size'.

string name
bool is_logged
uint32 divider

uint8 ACTION_SAVE_AND_PUBLISH =  0
uint8 ACTION_SAVE			  =  1
uint8 ACTION_PUBLISH	      =  2
uint8 action

uint8 BUFFERTYPE_FIXED_SIZE 	 	   =  0
uint8 BUFFERTYPE_LOOPING	 	 	   =  1
uint8 BUFFERTYPE_EXPONENTIALLY_GROWING =  2
uint8 buffer_type

uint32 buffer_size

uint32 no_items_in_buffer
uint32 no_unread_items_in_buffer