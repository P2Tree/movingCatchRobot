#!/bin/bash
updir=/home/pwe/catkin_ws/src/dobot    #要上传的文件夹
todir=catkin_ws/src/dobot          #目标文件夹，必须用相对路径
ip=192.168.1.43      #服务器
user=ubuntu          #ftp用户名
password=1406        #ftp密码
sss=`find $updir -type d -printf $todir/'%P\n'| awk '{if ($0 == "")next;print "mkdir " $0}'`
aaa=`find $updir -type f -printf 'put %p %P \n'`
ftp -nv $ip <<EOF
user $user $password
type binary
prompt
passive on
$sss
cd $todir
$aaa
quit
EOF
