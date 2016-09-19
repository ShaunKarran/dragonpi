ROS_SCRIPTS_DIR=dragonpi/target_acquisition/ros_pyscripts
CLASSIFIERS=($ROS_SCRIPTS_DIR/classifiers/acid.xml $ROS_SCRIPTS_DIR/classifiers/flammable.xml $ROS_SCRIPTS_DIR/classifiers/misc.xml)

python $ROS_SCRIPTS_DIR/camera.py > $ROS_SCRIPTS_DIR/logs/camera.log & \
python $ROS_SCRIPTS_DIR/image_processor.py -c $CLASSIFIERS -r 600 > $ROS_SCRIPTS_DIR/logs/image_processor.log & \
python $ROS_SCRIPTS_DIR/image_writer.py -t processed_images -o $ROS_SCRIPTS_DIR/processed_images/ > $ROS_SCRIPTS_DIR/logs/image_writer.log & \
