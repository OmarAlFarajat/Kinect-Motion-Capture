# comp477-assignment
Code base is owned by Prof. Tiberiu Popa, Concordia University, Canada
* Make sure to run in Release mode and x86 configuration.       
* Inputs are mixed between the console window and the glut window. Be mindful of which window is currently focused. Enter the key presses while focused on glut window, then switch to console window if prompted to enter a file name.     

+ Inputs:
    * __q__ : Quit. Suggested to use this instead of closing window or stopping VS debugger manually.
    * __r__ : Reset skeleton position/orientation. 
    * __l__ : Load animation.
    * __s__ : Save animation.
    * __1__ : Select matrix interpolation
    * __2__ : Select euler interpolation
    * __3__ : Select quaternion lerp
    * __4__ : Select quaternion slerp
    * __p__ : Play animation. Must have loaded an animation first and selected interpolation type.
    * __j__ : Speed up animation
    * __k__ : Slow down animation
    * __m__ : Switch between animation and keyframe editing modes. 
    > __The following inputs are problematic, be careful with the order of these operations!__ 
    * __=__ : Add keyframes (must be in editing mode or it will throw an error). Press m when done to switch back to animation mode. Press P to play the animation you made. 
    * __-__ : Goes back to first frame, keep pressing = to move to next keyframe and edit (and make sure you're in edit mode) 
    * __space__ : To record, then make sure to save the .anima file. May require a restart to play back the anima file. 
	
    + Mouse (camera and joint control): 
        * __Left-click and drag__ to rotate camera (however, clicking and dragging directly on skeleton joints  will move them)
        * __Right-click and drag__ to pan
        * __Middle-click and drag__ (or scroll) to zoom in. 
        