import matplotlib.pyplot as plt 
from commonroad.visualization.draw_dispatch_cr import draw_object
from commonroad.common.file_reader import CommonRoadFileReader

scenario, pp = CommonRoadFileReader("./maps_lanelet/CHN_Merging_ZS_repaired.xml").open()

plt.figure()
draw_object(scenario)
plt.gca().set_aspect("equal")
plt.autoscale()
plt.show()