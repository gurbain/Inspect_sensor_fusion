INSPECT SENSOR FUSION DEV SOFTWARE
==================================

This software provides tools for testing and improving sensor fusion with the following cameras:
- Optical Range Finder MESA SwissRanger 4000K (ethernet)
- MIT SSL VERTIGO Goggles made with 2 IDS-imaging uEye LE 1225-M-HQ (USB) with Fujinon 2.8mm, f/1.3 lenses
- Thermocam FLIR A5
Alternatively, it is possible to postprocess images in a folder with the same functions if the cameras are not plugged to your computer.

It has been developped to run on Linux Ubuntu 14.04 but can there shouldn't be any problem to extend its compatibility to any recent version of Linux or even WIndows and MacOS.

1. Download:
------------

To download the software on a Linux or a Mac computer, type in a shell:
```
cd ~
git clone https://github.com/Gabs48/Inspect_sensor_fusion
```

2. Install Drivers:
-------------------
If it is the first time you are installing the software on your computer, you may need to install the ORF and the vertigo cameras drivers! Note that you don't need them of you are processing images from a folder but then you shall comment the related lines in the *CMakeFile.txt* file.

**Install MESA-IMAGING 3D CAM Drivers**
- Go to http://www.mesa-imaging.ch/support/driver-downloads/
- Download the linux driver you need (VERTIGO computer runs in 32bits)
- Double-click on the .deb file you intalled and follow the instructions
- **WARNING: ** There is a conflict in the *defines* provided from MESA-IMAGING and OpenCV. To solve that conflict, simply replace the files *defineSR.h* and *libMesaSR.h* by new one provided in the folder ~/Inspect_sensor_fusion/Utilities/drivers/mesa:
```
cd ~/Inspect_sensor_fusion/Utilities/drivers/mesa
sudo mv libMesaSR.h defineSR.h /usr/include
```

**Install uEye Drivers**
To be sure to have the last version, follow the instructions on the IDS-imaging website: http://fr.ids-imaging.com/download-ueye.html

3. Install Libraries:
---------------------

To run this project, you will need third-part libraries in addition to the standard libraries pre-installed on Ubuntu:
- OpenCV (tested with versions 2.3 and 2.4)
- PCL (tested with version 1.7)
- Eigen (tested with version 3.2)
- Boost (tested with version 1.54)
The installation of those libraries can be done easily with your package manager (*apt-get* on Ubuntu) or thanks to tutoriels on internet forums.

4. Compile:
-----------

To compile the software, create a build directory, config with cmake and compile with make using make:
```
cd ~/Inspect_sensor_fusion
mkdir build
cd build
cmake ..
make
```
If there is an error during this compilation, *CMAKE* or *MAKE* should give you more precision about the missing libraries. If you need further information, contact me by mail or create a topic on my github forum.

5. Configure IP addresses
-------------------------

To facilitate the ethernet communication between each device, we  will give every devices a fix IP address on the network 192.168.1.* with a submask 255.255.255.0. If you already performed this step previously, you can skip it. If it is the first time, we need to fix the address for every devices:

- **ORF sensor**: By default, the IP address it takes automatically the value 192.168.1.42. If you want to change it, you have to open a telnet communication with it (being on the network 192.168.1.*) and then change it. However, it doesn't seem to work, so we'll keep the automatic address to avoid any problem.

- **Your own computer**: If you are running on Windows: Go in Control Panel/Network and Internet/Network Connections. Find your Local Area Wired Network. Click right then Properties. Select IPV4 settings then edit and set:
```
IP address : 192.168.1.4
Submask: 255.255.255.0
Gateway: /
```
Finsish with OK. Then you can plug you ethernet cable.


6. Execute:
-----------

To improve the comprehension, differents examples have been created in the repertory *Examples*:
- *haloCalib*: shows how to calibrate all cameras on Halo Hardware
- *haloFileCalib*: shows how to calibrate all cameras on Halo Hardware with images in a directory
- *haloCapture*: shows how to capture and display ORF and STEREO images together
- *haloSave*: shows how to capture and save ORF and STEREO images together
- *orfCalib*: shows how to calibrate ORF
- *orfCapture*: shows how to capture and display ORF images
- *orfSave*: shows how to capture and save ORF images
- *orfMeasDepth*: shows how to capture ORF images and compute the depth for a given point in the picture
- *orfMeasXYZ*: shows how to capture ORF images and compute the depth for a given point in the picture
- *orf3DCloud*: shows how to capture ORF images and display the 3D cloud
- *orfFile3DCloud*: shows how to load ORF images and display the 3D cloud
- *stereoCalib*: shows how to calibrate STEREO cameras
- *stereoCapture*: shows how to capture and display STEREO images
- *stereoSave*: shows how to capture and save STEREO images
- *stereoProject*: shows how to project and triangulate point with a simple method for STEREO cameras
- *stereoProjectHaloPoint*: shows how to project XYZ from ORF to stereo images then reproject and move them to XYZ ORF coordinate system again
- *stereoProjectIterative*: shows how to project and triangulate point with an iterative method for STEREO cameras
- *stereoProjectLinear*: shows how to project and triangulate point with a linear method for STEREO cameras

To compile them, uncomment all the lines below this one:
```
# Build examples
```
And perform the compilation step again.

To launch the main software, do:
```
cd ~/Inspect_sensor_fusion/build
./Main
```
To launch any example, do:
```
cd ~/Inspect_sensor_fusion/build
./The_name_of_my_example
```

7. Known issues:
-------------

During compilation:
- The defines *WORD* and *DWORD* are already defined: **You need to install the MESA-IMAGING driver correctly (step 2)**

During execution:
- The ORF cannot be initialized: **The IP addresses are badly configured (step 7)**
- The OpticsMount cannot be initialized: **Verify the OpticsMount IDs in /opt/GogglesOptics/CAMERA_FILE (see VERTIGO installation)**
