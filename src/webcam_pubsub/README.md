# Ros2_webcam_publisher-subscriber

ros2 package for publish and subscribe image from webcam

# How to build package

1.Create your workspace with a ```src``` sub-directory.

2.Inside a ```src``` ,clone this git with ```git clone https://github.com/Dr2546/Ros2_webcam_publisher-subscriber.git```.

3.In the root of your workspace,run ```rosdep install -i --from-path src --rosdistro humble -y``` to check dependencies.

> Note:Ros distro may vary depends on you,this project use Ros2 foxy.

4.Run ```colcon build``` or ```colcon build --packages-select webcam_pubsub``` if your workspace has many packages and you only want to build this package.

# How to use/run package
1.Open up your workspace in 2 terminal.

2.Run ```. install/setup.bash``` in both terminals.

3.First terminal run ```ros2 run webcam_pubsub webcam_pub``` to publish image.

4.Second terminal run ```ros2 run webcam_pubsub webcam_sub``` to subscribe image.
