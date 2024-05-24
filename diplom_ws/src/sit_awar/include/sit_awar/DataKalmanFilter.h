#pragma once

#include "DataFilter.h"
#include "Kalman.h"
#include <Eigen/Dense>
#include "Hungarian.h"
#include "consts.h"
#include <fstream>

class KalmanObject {
public:
    std::shared_ptr<KalmanFilter> filter;
    SensorData object;
    uint16_t live_time;
    uint32_t id;
    int state_size;
    int meas_size;
    bool already_pred;

    KalmanObject(uint32_t id, SensorData object);
    void predict();
    ObjectState update_measurement(SensorData object);
    Eigen::MatrixXd create_F3();
    Eigen::MatrixXd create_F6();
};

class DataKalmanFilter : public DataFilter {
    std::vector<KalmanObject> objects;
    HungarianAlgorithm hungariangSolver;
    std::ofstream logger;

public:
    std::unordered_map<uint32_t, ObjectState> get_filtered_objects(
        std::vector<SensorData> &aggregatedObjects,
        // std::shared_ptr<WorldModel> worldModel,
        std::shared_ptr<IdGenerator> IdGenerator 
        ) override;
    
    std::vector<std::vector<double>> make_cost_matrix_dist_az(
                            std::vector<KalmanObject> &seq1,
                            std::vector<SensorData> &seq2
                            );

    std::vector<std::vector<double>> make_cost_matrix_az(
                            std::vector<KalmanObject> &seq1,
                            std::vector<SensorData> &seq2 
                            );
    void dist_az(
        std::vector<SensorData> &fillObjects, std::vector<SensorData> &objects, 
        std::vector<int> &fillIdxs, std::vector<bool> alreagyAggObj
    );
    void az(
        std::vector<SensorData> &fillObjects, std::vector<SensorData> &objects, 
        std::vector<int> &fillIdxs, std::vector<bool> alreagyAggObj
    );
    void dist_az(
        std::vector<KalmanObject> &fillObjects, std::vector<KalmanObject> &objects, 
        std::vector<int> &fillIdxs, std::vector<bool> alreagyAggObj
    );
    void az(
        std::vector<KalmanObject> &fillObjects, std::vector<KalmanObject> &objects, 
        std::vector<int> &fillIdxs, std::vector<bool> alreagyAggObj
    );
};

