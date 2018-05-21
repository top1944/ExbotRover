#!/bin/bash
echo "
#    一键提升下载速度      #
## Top Liu @ Exbot.net ##
##     2018.05.18      ##
脚本自动配置内容：
1. 更新github的IP 速度可从30KB/s提升到2M/s
2. 更新pipy服务器为清华大学
3. 更新ROS服务器为中国科技大学

####################################################
另外新手推荐使用图形界面配置更改deb软件源为阿里云： 
系统设置 -> 软件和更新 选择下载服务器 -> China ->  mirrors.aliyun.com 

手动更改
用你熟悉的编辑器打开：
/etc/apt/sources.list

替换默认的
http://archive.ubuntu.com/

为
mirrors.aliyun.com

以Ubuntu 16.04LTS为例，最后的效果如下：
deb http://mirrors.aliyun.com/ubuntu/ xenial main
deb-src http://mirrors.aliyun.com/ubuntu/ xenial main

deb http://mirrors.aliyun.com/ubuntu/ xenial-updates main
deb-src http://mirrors.aliyun.com/ubuntu/ xenial-updates main

deb http://mirrors.aliyun.com/ubuntu/ xenial universe
deb-src http://mirrors.aliyun.com/ubuntu/ xenial universe
deb http://mirrors.aliyun.com/ubuntu/ xenial-updates universe
deb-src http://mirrors.aliyun.com/ubuntu/ xenial-updates universe

deb http://mirrors.aliyun.com/ubuntu/ xenial-security main
deb-src http://mirrors.aliyun.com/ubuntu/ xenial-security main
deb http://mirrors.aliyun.com/ubuntu/ xenial-security universe
deb-src http://mirrors.aliyun.com/ubuntu/ xenial-security universe
"

# 1.github
sudo echo -e "# github
151.101.44.249 github.global.ssl.fastly.net
192.30.253.113 github.com
103.245.222.133 assets-cdn.github.com
23.235.47.133 assets-cdn.github.com
203.208.39.104 assets-cdn.github.com
204.232.175.78 documentcloud.github.com
204.232.175.94 gist.github.com
107.21.116.220 help.github.com
207.97.227.252 nodeload.github.com
199.27.76.130 raw.github.com
107.22.3.110 status.github.com
204.232.175.78 training.github.com
207.97.227.243 www.github.com
185.31.16.184 github.global.ssl.fastly.net
185.31.18.133 avatars0.githubusercontent.com
185.31.19.133 avatars1.githubusercontent.com 
" >> /etc/hosts 

#2. pipy
mkdir -p ~/.pip/
touch ~/.pip/pip.conf 
sudo echo -e "
[global]
index-url = https://pypi.tuna.tsinghua.edu.cn/simple
" >> ~/.pip/pip.conf 


# 3.ROS mirror
sudo sh -c 'echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-get update
 
