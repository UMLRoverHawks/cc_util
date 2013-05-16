cc_util
=======

Color Calibration Tool “CCUtil” README

author: Adam DeFelice <ad.defelice@gmail.com>

What is this thing?

    The CCUtil program will subscribe to a node publishing raw image, and allow the user 
    to select rocks on that raw image. These selections will be used to create HSV color 
    thresholds, which will be stored in a .yaml file. This file can be easily read by 
    the rock detection software. 
    Code for reading the .yaml file is in “yaml_io_code_snippets”. 


How to build/run:

    1. Make a package, “cc_util” or something
    2. Use the manifest.xml and CMakeLists.txt supplied by me.
    3. Make sure the variable TOPIC[] on line 40 is set to the correct raw image topic coming from a camera.
    4. Make sure the variable PATH[] is set where you want the output yaml files to go.
    5. compile
    6. $ rosrun cc_util cc_util


What do I do now?:

    - If TOPIC[] is correct, and that topic is currently publishing video, then a new 
    window titled “Color Calibration Utility” should have popped up.
    
    - Controls will be displayed on the terminal window, along with some state information 
    at the bottom. This information is: “current color,” the color of the box you are currently 
    drawing, and “now editing,” which is the yaml file the output will be saved to. There are three 
    calibration files that can be modified, so you can have different calibrations for different 
    lighting scenarios.
    
    - Select a color with the number keys, 1 through 6.
    
    - Select a calibration file to save to with ‘[‘, ‘]’, and ‘\’.
    
    - Draw boxes with the mouse!
        - please just draw a bunch of boxes inside of the rocks, so as to not pick up the colors of the background.
        - you should be really put the same colors in the same color boxes.
        - the names of the box colors are just names, feel free to highlight brown rocks with the purple boxes, 
        just be consistent.

    - If you make a mistake, press ‘u’ to undo.
    
    - When you’re done, press the spacebar to do calculations and output a yaml file. 
    The yaml file will go where you told it to with the path variable.
    
    - Test your calibration, make more, save to different yaml files (of the three presets), that’s really it.


If you have questions, you could email me. My email’s at the top.
