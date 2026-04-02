# ubuntu24.04环境运行
import pandas as pd
import folium
import yaml
import webbrowser
import os

# 从YAML文件加载数据
with open("./config.yaml", "r", encoding="utf-8") as file:
    data = yaml.safe_load(file)

# 加载数据
csv_file = str(data['ubuntu24.04_path'])+str(data['bagdir'])+"/"+str(data['bag_name'])+"/"+str(data['bag_name'])+".csv"
html_file = str(data['ubuntu24.04_path'])+str(data['bagdir'])+"/"+str(data['bag_name'])+"/"+str(data['bag_name'])+".html"

data = pd.read_csv(csv_file)
# data = pd.read_csv('/home/pbj/docker_ws/data/2025-03-25/analysis/2025-03-25-20-39-22.csv')

# 创建地图对象
m = folium.Map(location=[data['latitude'].mean(), data['longitude'].mean()], zoom_start=12)

# 添加卫星图层
folium.TileLayer(
    tiles='https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',  # Google卫星图
    attr='Google',
    name='Satellite',
    overlay=True,
    control=True
).add_to(m)

# 提取经纬度坐标
coords = data[['latitude', 'longitude']].values.tolist()

# 绘制轨迹线
folium.PolyLine(coords, color="blue", weight=2.5, opacity=1).add_to(m)

# 添加起点和终点标记
folium.Marker(location=coords[0], popup="Start", icon=folium.Icon(color='green')).add_to(m)
folium.Marker(location=coords[-1], popup="End", icon=folium.Icon(color='red')).add_to(m)

# 保存地图到HTML文件
m.save(html_file)

webbrowser.open(html_file)