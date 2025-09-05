# ros2_bag_record
## 功能
自定义采集ros2数据, 防止ros2 bag record无法采集所有数据
## 编译
```bash
sudo apt install libyaml-cpp-dev
colcon build
```
## 运行
```bash
ros2 launch ros2_bag_record ros2_bag_record.launch.py
```