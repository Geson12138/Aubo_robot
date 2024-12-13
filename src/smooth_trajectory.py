 
import matplotlib.pyplot as plt
import numpy as np
 
 
# 贝塞尔曲线
def Bezier(t, points):
    n = len(points) - 1  # n可以取到
    res = 0
    c = 1  # 基函数结果
    for i in range(n+1):  # n可以取到
        if i > 0:
            c = c * (n - i + 1) / i  # 更新贝塞尔基函数的结果系数
        _1_t = (1-t)**i  # (1-t)^i
        _t = t**(n-i)  # t^(n-i)
        res += c * _1_t * _t * points[i]
    return res
 
 
def show(x, y, xs, ys, ax, plot_c='g',plot_c_2='b', scater_c='r'):
    ax.plot(x, y, c=plot_c)
    ax.scatter(x,y,color='r',marker='o', linewidths=4)
    ax.scatter(x,y,color='w',marker='o', linewidths=1) 
    ax.plot(xs, ys, c=plot_c_2)
    for i in range(len(x)):
        ax.annotate(f'{x[i]}, {y[i]}', (x[i]+0.2, y[i]-0.2))
 
 
def runBezier(x, y, n=100):
    xs = []
    ys = []
    
    for u in np.linspace(0, 1, num=n):
        xs.append(Bezier(u, points=x))
        ys.append(Bezier(u, points=y))
    return xs, ys
 
 
if __name__ == '__main__':
    x = [0, 3, 4, 10, 5, 6]
    y = [0, 6, 10, 0, 5, 3]
    
    nodeVector = [0, 0, 0.2, 0.4, 0.6, 0.8, 1, 1, 1, 1] 
    fig, axs = plt.subplots(2, 2)
 
    xs, ys = runBezier(x, y)
    show(x, y, xs, ys, axs[0, 0])
    axs[0, 0].set_title('Bezier') 
 
    plt.tight_layout()
    plt.show()