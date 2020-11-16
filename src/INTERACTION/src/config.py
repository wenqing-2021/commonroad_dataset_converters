import os


def get_list_info_dataset(directory_maps = "./maps_lanelet/",directory_output_scenrios = "scenarios_converted/"):
    # "data/"
    directory_dataset = "./dataset/"
    # directory_output_scenrios = "scenarios_converted/"

    print(os.getcwd())

    assert os.path.exists(
        os.path.join(os.getcwd(), directory_dataset)), "<dataset> folder not found under current working directory!"
    assert os.path.exists(
        os.path.join(os.getcwd(), directory_maps)), "<maps_lanelet> folder not found under current working directory!"

    # list to hold the info of maps of scenarios
    list_info_maps = []

    list_info_maps.append({
        'prefix_name': 'CHN_Merging-1_',
        'path_map': os.path.join(os.getcwd(), directory_maps, "CHN_Merging_ZS_repaired.xml"),
        'directory_data': os.path.join(os.getcwd(), directory_dataset,
                                       "INTERACTION-Dataset-DR-v1_0/recorded_trackfiles/DR_CHN_Merging_ZS/"),
        'directory_output': os.path.join(os.getcwd(), directory_output_scenrios, "CHN_Merging-1/"),
        'flag_same_direction_problems': True,
        'tags': 'highway multi_lane parallel_lanes merging_lanes',
        'x_offset_lanelets': -504580,
        'y_offset_lanelets': 965,
        'x_offset_tracks': 1056,
        'y_offset_tracks': 954.5})

    list_info_maps.append({
        'prefix_name': 'CHN_Roundabout-1_',
        'path_map': os.path.join(os.getcwd(), directory_maps, "CHN_Roundabout_LN_repaired.xml"),
        'directory_data': os.path.join(os.getcwd(), directory_dataset,
                                       "INTERACTION-Dataset-DR-v1_0/recorded_trackfiles/DR_CHN_Roundabout_LN/"),
        'directory_output': os.path.join(os.getcwd(), directory_output_scenrios, "CHN_Roundabout-1/"),
        'flag_same_direction_problems': True,
        'tags': 'urban roundabout multi_lane comfort',
        'x_offset_lanelets': -504580,
        'y_offset_lanelets': 965,
        'x_offset_tracks': 1056,
        'y_offset_tracks': 954.5})

    list_info_maps.append({
        'prefix_name': 'DEU_Merging-1_',
        'path_map': os.path.join(os.getcwd(), directory_maps, "DEU_Merging_MT_repaired.xml"),
        'directory_data': os.path.join(os.getcwd(), directory_dataset,
                                       "INTERACTION-Dataset-DR-v1_0/recorded_trackfiles/DR_DEU_Merging_MT/"),
        'directory_output': os.path.join(os.getcwd(), directory_output_scenrios, "DEU_Merging-1/"),
        'flag_same_direction_problems': True,
        'tags': 'highway merging_lanes comfort no_oncoming_traffic',
        'x_offset_lanelets': -504739.17,
        'y_offset_lanelets': 1015.95,
        'x_offset_tracks': 904,
        'y_offset_tracks': 1004.5})

    list_info_maps.append({
        'prefix_name': 'DEU_Roundabout-1_',
        'path_map': os.path.join(os.getcwd(), directory_maps, "DEU_Roundabout_OF_repaired.xml"),
        'directory_data': os.path.join(os.getcwd(), directory_dataset,
                                       "INTERACTION-Dataset-DR-v1_0/recorded_trackfiles/DR_DEU_Roundabout_OF/"),
        'directory_output': os.path.join(os.getcwd(), directory_output_scenrios, "DEU_Roundabout-1/"),
        'flag_same_direction_problems': True,
        'tags': 'urban roundabout two_lane comfort',
        'x_offset_lanelets': -504739.17,
        'y_offset_lanelets': 1015.95,
        'x_offset_tracks': 897,
        'y_offset_tracks': 1004})

    list_info_maps.append({
        'prefix_name': 'USA_Intersection-1_',
        'path_map': os.path.join(os.getcwd(), directory_maps, "USA_Intersection_EP0_repaired.xml"),
        'directory_data': os.path.join(os.getcwd(), directory_dataset,
                                       "INTERACTION-Dataset-DR-v1_0/recorded_trackfiles/DR_USA_Intersection_EP0/"),
        'directory_output': os.path.join(os.getcwd(), directory_output_scenrios, "USA_Intersection-1/"),
        'flag_same_direction_problems': True,
        'tags': 'urban multi_lane intersection comfort',
        'x_offset_lanelets': -504690,
        'y_offset_lanelets': 1005,
        'x_offset_tracks': 945,
        'y_offset_tracks': 993.5})

    list_info_maps.append({
        'prefix_name': 'USA_Intersection-2_',
        'path_map': os.path.join(os.getcwd(), directory_maps, "USA_Intersection_EP1_repaired.xml"),
        'directory_data': os.path.join(os.getcwd(), directory_dataset,
                                       "INTERACTION-Dataset-DR-v1_0/recorded_trackfiles/DR_USA_Intersection_EP1/"),
        'directory_output': os.path.join(os.getcwd(), directory_output_scenrios, "USA_Intersection-2/"),
        'flag_same_direction_problems': True,
        'tags': 'urban multi_lane intersection comfort',
        'x_offset_lanelets': -504690,
        'y_offset_lanelets': 1005,
        'x_offset_tracks': 945,
        'y_offset_tracks': 993.5})

    list_info_maps.append({
        'prefix_name': 'USA_Intersection-3_',
        'path_map': os.path.join(os.getcwd(), directory_maps, "USA_Intersection_GL_repaired.xml"),
        'directory_data': os.path.join(os.getcwd(), directory_dataset,
                                       "INTERACTION-Dataset-DR-v1_0/recorded_trackfiles/DR_USA_Intersection_GL/"),
        'directory_output': os.path.join(os.getcwd(), directory_output_scenrios, "USA_Intersection-2/"),
        'flag_same_direction_problems': True,
        'tags': 'urban multi_lane intersection comfort',
        'x_offset_lanelets': -504690,
        'y_offset_lanelets': 1005,
        'x_offset_tracks': 946,
        'y_offset_tracks': 995})

    list_info_maps.append({
        'prefix_name': 'USA_Intersection-4_',
        'path_map': os.path.join(os.getcwd(), directory_maps, "USA_Intersection_MA_repaired.xml"),
        'directory_data': os.path.join(os.getcwd(), directory_dataset,
                                       "INTERACTION-Dataset-DR-v1_0/recorded_trackfiles/DR_USA_Intersection_MA/"),
        'directory_output': os.path.join(os.getcwd(), directory_output_scenrios, "USA_Intersection-4/"),
        'flag_same_direction_problems': True,
        'tags': 'urban multi_lane intersection comfort',
        'x_offset_lanelets': -504690,
        'y_offset_lanelets': 1005,
        'x_offset_tracks': 945.5,
        'y_offset_tracks': 993.5})

    list_info_maps.append({
        'prefix_name': 'USA_Roundabout-1_',
        'path_map': os.path.join(os.getcwd(), directory_maps, "USA_Roundabout_EP_repaired.xml"),
        'directory_data': os.path.join(os.getcwd(), directory_dataset,
                                       "INTERACTION-Dataset-DR-v1_0/recorded_trackfiles/DR_USA_Roundabout_EP/"),
        'directory_output': os.path.join(os.getcwd(), directory_output_scenrios, "USA_Roundabout-1/"),
        'flag_same_direction_problems': True,
        'tags': 'urban roundabout multi_lane comfort',
        'x_offset_lanelets': -504690,
        'y_offset_lanelets': 1005,
        'x_offset_tracks': 945.5,
        'y_offset_tracks': 993.5})

    list_info_maps.append({
        'prefix_name': 'USA_Roundabout-2_',
        'path_map': os.path.join(os.getcwd(), directory_maps, "USA_Roundabout_FT_repaired.xml"),
        'directory_data': os.path.join(os.getcwd(), directory_dataset,
                                       "INTERACTION-Dataset-DR-v1_0/recorded_trackfiles/DR_USA_Roundabout_FT/"),
        'directory_output': os.path.join(os.getcwd(), directory_output_scenrios, "USA_Roundabout-2/"),
        'flag_same_direction_problems': True,
        'tags': 'urban roundabout multi_lane comfort',
        'x_offset_lanelets': -504690,
        'y_offset_lanelets': 1005,
        'x_offset_tracks': 945.5,
        'y_offset_tracks': 993.5})

    list_info_maps.append({
        'prefix_name': 'USA_Roundabout-3_',
        'path_map': os.path.join(os.getcwd(), directory_maps, "USA_Roundabout_SR_repaired.xml"),
        'directory_data': os.path.join(os.getcwd(), directory_dataset,
                                       "INTERACTION-Dataset-DR-v1_0/recorded_trackfiles/DR_USA_Roundabout_SR/"),
        'directory_output': os.path.join(os.getcwd(), directory_output_scenrios, "USA_Roundabout-3/"),
        'flag_same_direction_problems': True,
        'tags': 'urban roundabout multi_lane comfort',
        'x_offset_lanelets': -504690,
        'y_offset_lanelets': 1005,
        'x_offset_tracks': 945.5,
        'y_offset_tracks': 993.5})

    list_info_maps.append({
        'prefix_name': 'BGR_Interaction-1_',
        'path_map': os.path.join(os.getcwd(), directory_maps, "BGR_Intersection_VA_repaired.xml"),
        'directory_data': os.path.join(os.getcwd(), directory_dataset,
                                       "INTERACTION-Dataset-TC-v1_0/recorded_trackfiles/TC_BGR_Intersection_VA/"),
        'directory_output': os.path.join(os.getcwd(), directory_output_scenrios, "BGR_Interaction-1/"),
        'flag_same_direction_problems': True,
        'tags': 'urban multi_lane intersection comfort',
        'x_offset_lanelets': -504690,
        'y_offset_lanelets': 965,
        'x_offset_tracks': 945,
        'y_offset_tracks': 954})

    return list_info_maps
