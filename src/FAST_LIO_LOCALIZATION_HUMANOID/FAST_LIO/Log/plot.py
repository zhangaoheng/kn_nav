# import matplotlib
# matplotlib.use('Agg')
# ============================================================
# FAST-LIO 日志分析脚本（上游开源算法配套的离线工具）
# 作用：读取 FAST-LIO 运行期间 dump 出的 mat_pre.txt / mat_out.txt
#       等日志文件，绘制姿态、平移、外参、速度、零偏、重力等状态曲线，
#       用于离线查看 ESKF 预测(前验)与更新(后验)状态的差异。
# 工作区内仅作为里程计前端的日志后处理脚本使用，不参与实时导航。
# ============================================================
import numpy as np
import matplotlib.pyplot as plt


#######for ikfom
# ---- 第一部分：绘制 ikfom 状态对比曲线 ----
# mat_pre.txt / mat_out.txt 分别记录 ESKF 预测(前验)与更新后(后验)的状态，
# 按 4 行 x 2 列的子图布局展示：姿态角、平移、外参旋转/平移、速度、
# 陀螺零偏 bg、加计零偏 ba 与重力估计等分量随时间的变化。
fig, axs = plt.subplots(4,2)
lab_pre = ['', 'pre-x', 'pre-y', 'pre-z']
lab_out = ['', 'out-x', 'out-y', 'out-z']
plot_ind = range(7,10)
a_pre=np.loadtxt('mat_pre.txt')
a_out=np.loadtxt('mat_out.txt')
time=a_pre[:,0]
axs[0,0].set_title('Attitude')
axs[1,0].set_title('Translation')
axs[2,0].set_title('Extrins-R')
axs[3,0].set_title('Extrins-T')
axs[0,1].set_title('Velocity')
axs[1,1].set_title('bg')
axs[2,1].set_title('ba')
axs[3,1].set_title('Gravity')
for i in range(1,4):
    for j in range(8):
        axs[j%4, j/4].plot(time, a_pre[:,i+j*3],'.-', label=lab_pre[i])
        axs[j%4, j/4].plot(time, a_out[:,i+j*3],'.-', label=lab_out[i])
for j in range(8):
    # axs[j].set_xlim(386,389)
    axs[j%4, j/4].grid()
    axs[j%4, j/4].legend()
plt.grid()
#######for ikfom#######


#### Draw IMU data
# ---- 第二部分（注释模板）：绘制 IMU 原始数据曲线 ----
# 读取 imu.txt，绘制陀螺仪与加速度计的三轴原始测量曲线，默认关闭，
# 需要时取消对应代码块的注释即可启用。
# fig, axs = plt.subplots(2)
# imu=np.loadtxt('imu.txt')
# time=imu[:,0]
# axs[0].set_title('Gyroscope')
# axs[1].set_title('Accelerameter')
# lab_1 = ['gyr-x', 'gyr-y', 'gyr-z']
# lab_2 = ['acc-x', 'acc-y', 'acc-z']
# for i in range(3):
#     # if i==1:
#     axs[0].plot(time, imu[:,i+1],'.-', label=lab_1[i])
#     axs[1].plot(time, imu[:,i+4],'.-', label=lab_2[i])
# for i in range(2):
#     # axs[i].set_xlim(386,389)
#     axs[i].grid()
#     axs[i].legend()
# plt.grid()

# #### Draw time calculation
# ---- 第三部分（注释模板）：绘制有效特征点数量与计算耗时箱线图 ----
# 对比室外/室内不同场景下每帧有效特征点数量与单帧计算时间，默认关闭。
# plt.figure(3)
# fig = plt.figure()
# font1 = {'family' : 'Times New Roman',
# 'weight' : 'normal',
# 'size'   : 12,
# }
# c="red"
# a_out1=np.loadtxt('Log/mat_out_time_indoor1.txt')
# a_out2=np.loadtxt('Log/mat_out_time_indoor2.txt')
# a_out3=np.loadtxt('Log/mat_out_time_outdoor.txt')
# # n = a_out[:,1].size
# # time_mean = a_out[:,1].mean()
# # time_se   = a_out[:,1].std() / np.sqrt(n)
# # time_err  = a_out[:,1] - time_mean
# # feat_mean = a_out[:,2].mean()
# # feat_err  = a_out[:,2] - feat_mean
# # feat_se   = a_out[:,2].std() / np.sqrt(n)
# ax1 = fig.add_subplot(111)
# ax1.set_ylabel('Effective Feature Numbers',font1)
# ax1.boxplot(a_out1[:,2], showfliers=False, positions=[0.9])
# ax1.boxplot(a_out2[:,2], showfliers=False, positions=[1.9])
# ax1.boxplot(a_out3[:,2], showfliers=False, positions=[2.9])
# ax1.set_ylim([0, 3000])

# ax2 = ax1.twinx()
# ax2.spines['right'].set_color('red')
# ax2.set_ylabel('Compute Time (ms)',font1)
# ax2.yaxis.label.set_color('red')
# ax2.tick_params(axis='y', colors='red')
# ax2.boxplot(a_out1[:,1]*1000, showfliers=False, positions=[1.1],boxprops=dict(color=c),capprops=dict(color=c),whiskerprops=dict(color=c))
# ax2.boxplot(a_out2[:,1]*1000, showfliers=False, positions=[2.1],boxprops=dict(color=c),capprops=dict(color=c),whiskerprops=dict(color=c))
# ax2.boxplot(a_out3[:,1]*1000, showfliers=False, positions=[3.1],boxprops=dict(color=c),capprops=dict(color=c),whiskerprops=dict(color=c))
# ax2.set_xlim([0.5, 3.5])
# ax2.set_ylim([0, 100])

# plt.xticks([1,2,3], ('Outdoor Scene', 'Indoor Scene 1', 'Indoor Scene 2'))
# # # print(time_se)
# # # print(a_out3[:,2])
# plt.grid()
# plt.savefig("time.pdf", dpi=1200)
plt.show()
