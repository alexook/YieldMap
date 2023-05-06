import matplotlib.pyplot as plt
from graphviz import Digraph

# 创建Digraph对象
dot = Digraph(comment='YOLOv7-tiny Network')

# 添加节点
dot.node('input', 'Input Image')
dot.node('backbone', 'Backbone')
dot.node('neck', 'Neck')
dot.node('head', 'Head')
dot.node('output', 'Output')

# 添加分支
dot.edge('input', 'backbone')
dot.edge('backbone', 'neck')
dot.edge('neck', 'head')
dot.edge('head', 'output')

# 输出流程图
dot.render('yolov7-tiny', view=True)

# 绘制流程图
plt.axis('off')
plt.show()