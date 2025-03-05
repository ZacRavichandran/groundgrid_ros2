#ifndef GROUND_GRID_CONFIG_H
#define GROUND_GRID_CONFIG_H

struct GroundGridConfig
{
    int point_count_cell_variance_threshold = 10;
    int max_ring = 1024;
    double groundpatch_detection_minimum_threshold = 0.01;
    double distance_factor = 0.0001;
    double minimum_distance_factor = 0.0005;
    double miminum_point_height_threshold = 0.3;
    double minimum_point_height_obstacle_threshold = 0.1;
    double outlier_tolerance = 0.1;
    double ground_patch_detection_minimum_point_count_threshold = 0.25;
    double patch_size_change_distance = 20.0;
    double occupied_cells_decrease_factor = 5.0;
    double occupied_cells_point_count_factor = 20.0;
    double min_outlier_detection_ground_confidence = 1.25;
    int thread_count = 8;
};

#endif // GROUND_GRID_CONFIG_H
