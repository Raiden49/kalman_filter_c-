# 卡尔曼滤波器C++

## 使用方法

​		首先确定状态维度与测量量维度，以4维度$(x，y，vx，vy)$为状态量例子，以2维度$(x,y)$为测量量例子

​		然后创建协方差矩阵Q与R，这两个矩阵是调参的主要矩阵，优劣很大程度取决于QR参数的选取。

```c++
	Eigen::MatrixXd Q(4, 4); Eigen::MatrixXd R(2, 2);
	Q << 0.1, 0, 0, 0,
     	0, 0.1, 0, 0,
     	0, 0, 0.1, 0,
     	0, 0, 0, 0.1;
	R << 0.1, 0,
     	0, 0.1;
```

​		再然后创建控制量矩阵B，如果没有控制，让控制向量u为0向量即可

```c++
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(4,1);
    Eigen::VectorXd u = Eigen::VectorXd::Zero(1);
```

​		如果是有控制量，假如是匀加速或者匀减速运动模型，为了方便，假设x y方向加速度一致，即u就为加速度量a；如果不一致u就是2x1的向量，x方向加速度与y方向加速度

```c++
    Eigen::MatrixXd B(4,1);
    B << 0.5 * pow(t, 2), 0.5 * pow(t, 2), t, t;
    Eigen::VectorXd u = Eigen::VectorXd::Zero(1);
    u(0,0) = 1;
```

​		构造卡尔曼类，并初始化

```c++
    auto kf = std::make_shared<proj_kalman::KalmanFilter>(4, 2, 0, Q, R, B);
    Eigen::VectorXd x_0 = Eigen::VectorXd::Zero(4);
    kf->init(x_0);
```

​		在循环中不断进行预测与更新

```c++
    for (int i = 0; i < iter; i++) {
        Eigen::VectorXd x_p_k = kf->predict(u, t);
        Eigen::VectorXd x_k = kf->update(z_k);
    }
```

