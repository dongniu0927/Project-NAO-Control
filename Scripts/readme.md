# 环境搭建

## conda环境安装
`set CONDA_FORCE_32BIT=1`
`conda env create -f environment.yml`

## naoqi
### 下载地址
1. https://community.ald.softbankrobotics.com/en/resources/software/language/en-gb // 需翻墙，可下载各个版本
2. 链接: https://pan.baidu.com/s/1lWAT1SS7OhmCKKtpiIDp7Q 密码: ukcs // 提供window32的二进制版本

### 安装
1. 从上面地址1处下载并安装
2. 对于conda方式的安装：下载链接2的文件，解压将其中的所有文件放入特定的conda envs中（如我的是：`C:\Users\dongn\Anaconda3\envs\robotpy2\Lib\site-packages`）

## vrep
从remoteApis寻找对应系统的版本放入本文件所在目录

## 运行
```
开启vrep,并加载Vrep-Scene
sh start_nao.sh  // 开启一个nao,要求naoqi-bin已经放入环境变量
activate robotpy2 // 进入环境
sh start_connect.sh // 开启连接
```