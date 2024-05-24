#pragma once

#include "DataAggregation.h"
#include "Hungarian.h"
#include <Eigen/Dense>
#include <fstream>
#include <time.h>
#include "consts.h"

class DataHungarianAggregation : public DataAggregation {
    HungarianAlgorithm hungariangSolver;
    std::ofstream logger;
    
    std::vector<std::vector<double>> make_cost_matrix_dist_az_ty_cl(
                            std::vector<SensorData> &seq1,
                            std::vector<SensorData> &seq2    
                            );
    
    std::vector<std::vector<double>> make_cost_matrix_az_ty_cl(
                            std::vector<SensorData> &seq1,
                            std::vector<SensorData> &seq2    
                            );

    std::vector<std::vector<double>> make_cost_matrix_dist_az_ty(
                            std::vector<SensorData> &seq1,
                            std::vector<SensorData> &seq2    
                            );
    
    std::vector<std::vector<double>> make_cost_matrix_az_ty(
                            std::vector<SensorData> &seq1,
                            std::vector<SensorData> &seq2    
                            );

    void aggregate_pair(
                    std::vector<SensorData> &aggregatedObjects, 
                    std::vector<SensorData> &objects,
                    int agg_idx, 
                    int obj_idx
                    );

    void dist_az_ty_cl(
                    std::vector<SensorData> &fillObjects, std::vector<SensorData> &objects, 
                    std::vector<int> &fillIdxs, std::vector<bool> alreagyAggObj
                    );

    void dist_az_ty(
                std::vector<SensorData> &fillObjects, std::vector<SensorData> &objects, 
                std::vector<int> &fillIdxs, std::vector<bool> alreagyAggObj
                );
    void az_ty_cl(
                std::vector<SensorData> &fillObjects, std::vector<SensorData> &objects, 
                std::vector<int> &fillIdxs, std::vector<bool> alreagyAggObj
                );
    void az_ty(
            std::vector<SensorData> &fillObjects, std::vector<SensorData> &objects, 
            std::vector<int> &fillIdxs, std::vector<bool> alreagyAggObj
            );

public:
    DataHungarianAggregation();
    std::vector<SensorData> do_aggregation(std::vector<SensorPack::SharedPtr>& sensorsSequence) override;

};