#include "DataHungarianAggregation.h"

DataHungarianAggregation::DataHungarianAggregation()
{
    this->hungariangSolver = HungarianAlgorithm();
}

std::vector<SensorData> DataHungarianAggregation::do_aggregation(std::vector<SensorPack::SharedPtr> &sensorsSequence)
{
    // TPR AND FPR LOGGING
    this->logger.open("/media/sf_ros2dir/diplom_ws/logger_agg.txt",std::ios::app);
    
    // TIMER LOGGING
    // this->logger.open("/media/sf_ros2dir/diplom_ws/logger_timer_agg.txt", std::ios::app);
    
    std::vector<SensorData> aggregatedObjects = std::vector<SensorData>();

    //--------------FOR TIME LOGGER------------------
    // clock_t tStart = clock();
    //----------------------------------------------

    if(!sensorsSequence.empty()) {
        for(int i = 0; i < sensorsSequence.size(); i++){

            std::cout << "-----------------AGGREGATE SensorPack №" << i << "------------\n";
            // if SensorPack empty -> get next Pack
            if(sensorsSequence[i]->objects.empty())
                continue;
            
            //------------------------FOR FP NP LOGGER FILE----------------------------
            for(int j = 0; j < sensorsSequence[i]->objects.size(); j++){
                if(j == sensorsSequence[i]->objects.size()-1) {
                    this->logger << sensorsSequence[i]->objects[j].scen_id;
                } else {
                    this->logger << sensorsSequence[i]->objects[j].scen_id << ",";
                }
            }
            this->logger << "|";
            //------------------------------------------------------------------

            // if current aggregatedObject empty -> get objects from current SensorPack
            if(aggregatedObjects.empty()) {
                aggregatedObjects = sensorsSequence[i]->objects;
                continue;
            }
            
            std::vector<SensorData> objects = sensorsSequence[i]->objects;
            
            // tracking already aggregated objects with objects from the current pack
            std::vector<bool> alreadyAggS1 = std::vector<bool>(objects.size(), 0);
            std::vector<bool> alreadyAggAgg = std::vector<bool>(aggregatedObjects.size(), 0);
            
            // counters for TPR and FPR metrics
            int TP = 0;
            int FP = 0;
            int TN = 0;
            int FN = 0;

            // iterate over all combinations of object characteristics
            for (int variant = 0; variant < 4; variant++) {
                std::vector<SensorData> pretendentObjS1 = std::vector<SensorData>();
                std::vector<int> pretendentIdxS1 = std::vector<int>();

                std::vector<SensorData> pretendentObjAgg = std::vector<SensorData>();
                std::vector<int> pretendentIdxAgg = std::vector<int>();

                std::vector<int> hungAssignment = std::vector<int>();
                if(variant == 0){
                    dist_az_ty_cl(
                        pretendentObjS1, objects,
                        pretendentIdxS1, alreadyAggS1);

                    dist_az_ty_cl(
                        pretendentObjAgg, aggregatedObjects,
                        pretendentIdxAgg, alreadyAggAgg);
                } else if(variant == 1){
                    dist_az_ty(
                        pretendentObjS1, objects,
                        pretendentIdxS1, alreadyAggS1);

                    dist_az_ty(
                        pretendentObjAgg, aggregatedObjects,
                        pretendentIdxAgg, alreadyAggAgg);
                } else if(variant == 2){
                    az_ty_cl(
                        pretendentObjS1, objects,
                        pretendentIdxS1, alreadyAggS1);

                    az_ty_cl(
                        pretendentObjAgg, aggregatedObjects,
                        pretendentIdxAgg, alreadyAggAgg);
                } else {
                    az_ty(
                        pretendentObjS1, objects,
                        pretendentIdxS1, alreadyAggS1);

                    az_ty(
                        pretendentObjAgg, aggregatedObjects,
                        pretendentIdxAgg, alreadyAggAgg);
                }

                if ((!pretendentObjAgg.empty() and !pretendentObjS1.empty()))
                {
                    std::vector<std::vector<double>> costMatrix = std::vector<std::vector<double>>();
                    if(variant == 0)
                        costMatrix = make_cost_matrix_dist_az_ty_cl(pretendentObjAgg, pretendentObjS1);
                    else if(variant == 1)
                        costMatrix = make_cost_matrix_dist_az_ty(pretendentObjAgg, pretendentObjS1);
                    else if(variant == 2)
                        costMatrix = make_cost_matrix_az_ty_cl(pretendentObjAgg, pretendentObjS1);
                    else 
                        costMatrix = make_cost_matrix_az_ty(pretendentObjAgg, pretendentObjS1);

                    double hungCost = hungariangSolver.solve(costMatrix, hungAssignment);
                    
                    int range = pretendentObjS1.size();
                    // aggregate pack object to aggregatedObjects by assignment
                    for (int x = range - 1; x >= 0 ; x--)
                    {
                        int ass_idx = hungAssignment[x];
                        int obj_idx = pretendentIdxS1[x];
                        double cost = costMatrix[x][ass_idx];

                        //-----------------------FOR TP FP LOGGER-----------------------------
                        if (ass_idx == -1) {
                            bool flag = 1;
                            for(int m = 0; m < pretendentIdxAgg.size(); m++){
                                if(objects[obj_idx].scen_id == aggregatedObjects[pretendentIdxAgg[m]].scen_id){
                                    FN++;
                                    flag=0;
                                    break;
                                }
                            }
                            if(flag)
                                TN++;
                        }
                        if (ass_idx != -1 && cost < AGG_COST_TRESHHOLD[variant]){
                                int agg_idx = pretendentIdxAgg[ass_idx];
                                if(objects[obj_idx].scen_id == aggregatedObjects[agg_idx].scen_id) {
                                    TN++;
                                } else {
                                    FN++;
                                }
                            }
                        //----------------------------------------------------------------

                        if((ass_idx == -1 || cost > AGG_COST_TRESHHOLD[variant]) && variant == 3) {
                            aggregatedObjects.push_back(objects[obj_idx]);
                            alreadyAggS1[obj_idx] = 1;
                        } else {
                            if(ass_idx != -1 && cost < AGG_COST_TRESHHOLD[variant]){
                                int agg_idx = pretendentIdxAgg[ass_idx];
                                //-----------------------FOR TP FP LOGGER-----------------------------
                                if(objects[obj_idx].scen_id == aggregatedObjects[agg_idx].scen_id) {
                                    TP++;
                                } else {
                                    FP++;
                                }
                                //---------------
                                aggregate_pair(aggregatedObjects, objects, agg_idx, obj_idx);
                                alreadyAggS1[obj_idx] = 1;
                                alreadyAggAgg[agg_idx] = 1;
                                
                            }
                        }
                        
                    }
                }
            }
            
            //--------------------FOR FP TP LOGGER--------------------------
            this->logger << "TP_" << TP << "_TN_" << TN << "_FP_" << FP << "_FN_" << FN << "|";
            std::cout << "TP_" << TP << "_TN_" << TN << "_FP_" << FP << "_FN_" << FN << "\n";
            std::cout << "TPR_" << TP/(TP+FN) << "_FPR_" << FP/(FP+TN) << "\n";
        }

    }
    //----------------------FOR FP TP LOGGER-----------------------------------
    if(!aggregatedObjects.empty()){
        for(int i = 0; i < aggregatedObjects.size(); i++){
            if(i == aggregatedObjects.size()-1) {
                this->logger << aggregatedObjects[i].agg_id;
            } else {
                this->logger << aggregatedObjects[i].agg_id << ",";
            }
        }
        this->logger << "\n";
    }
    //------------------------


    // --------------FOR TIME LOGGER------------------
    // logging if aggregatedObjects not empty
    // if(!aggregatedObjects.empty()){
    //     this->logger << sensorsSequence.size() << "|";
    //     this->logger << (double)(clock() - tStart)/CLOCKS_PER_SEC << "\n";
    //     std::cout << "Agg Time: " << (double)(clock() - tStart)/CLOCKS_PER_SEC <<"\n";
    // }
    // --------------

    this->logger.close();
    return aggregatedObjects;
}

// get from vector of SensorData only with azimuth and type
void DataHungarianAggregation::az_ty(
                            std::vector<SensorData> &fillObjects, std::vector<SensorData> &objects, 
                            std::vector<int> &fillIdxs, std::vector<bool> alreagyAggObj
                            ){
    for(int j = 0; j < objects.size();j++){
        if(alreagyAggObj[j])
            continue;
        fillObjects.push_back(objects[j]);
        fillIdxs.push_back(j);
    }
};
// get from vector of SensorData only with azimuth, type and class
void DataHungarianAggregation::az_ty_cl(
                            std::vector<SensorData> &fillObjects, std::vector<SensorData> &objects, 
                            std::vector<int> &fillIdxs, std::vector<bool> alreagyAggObj
                            ){
    for(int j = 0; j < objects.size();j++){
        if(alreagyAggObj[j])
            continue;
        if(objects[j].obj_class < 0)
            continue;
        fillObjects.push_back(objects[j]);
        fillIdxs.push_back(j);
    }
};

// get from vector of SensorData only with distance, azimuth and type
void DataHungarianAggregation::dist_az_ty(
                            std::vector<SensorData> &fillObjects, std::vector<SensorData> &objects, 
                            std::vector<int> &fillIdxs, std::vector<bool> alreagyAggObj
                            ){
    for(int j = 0; j < objects.size();j++){
        if(alreagyAggObj[j])
            continue;
        if(objects[j].distance < 0)
            continue;

        fillObjects.push_back(objects[j]);
        fillIdxs.push_back(j);
    }
};

// get from vector of SensorData only with distance, azimuth, type and class
void DataHungarianAggregation::dist_az_ty_cl(
                            std::vector<SensorData> &fillObjects, std::vector<SensorData> &objects, 
                            std::vector<int> &fillIdxs, std::vector<bool> alreagyAggObj
                            ){
    for(int j = 0; j < objects.size();j++){
        if(alreagyAggObj[j])
            continue;
        if(objects[j].distance < 0)
            continue;
        if(objects[j].obj_class < 0)
            continue;

        fillObjects.push_back(objects[j]);
        fillIdxs.push_back(j);
    }
};

// aggregate object
void DataHungarianAggregation::aggregate_pair(
                    std::vector<SensorData> &aggregatedObjects, 
                    std::vector<SensorData> &objects,
                    int agg_idx, 
                    int obj_idx
                    ){

    double agg_dist = aggregatedObjects[agg_idx].distance;
    double agg_azim = aggregatedObjects[agg_idx].azimuth;
    double sen_obj_dist = objects[obj_idx].distance;
    double sen_obj_azim = objects[obj_idx].azimuth;
    
    // if the object does not have a distance, then choose from a pair; 
    // otherwise, take the average
    if(agg_dist < 0) {
        aggregatedObjects[agg_idx].distance = sen_obj_dist;
    } else if(agg_dist > 0 && sen_obj_dist > 0) {
        aggregatedObjects[agg_idx].distance = (sen_obj_dist + agg_dist)/2;
    }
    
    // if the object does not have a azimuth, then choose from a pair; 
    // otherwise, take the average
    if(agg_azim < 0) {
        aggregatedObjects[agg_idx].azimuth = sen_obj_azim;
    } else if(agg_azim > 0 && sen_obj_azim > 0) {
        aggregatedObjects[agg_idx].azimuth = (sen_obj_azim + agg_azim)/2;
    }

    int agg_type = aggregatedObjects[agg_idx].obj_type;
    double agg_type_conf = aggregatedObjects[agg_idx].type_conf;
    int sen_obj_type = objects[obj_idx].obj_type;
    double sen_obj_type_conf = objects[obj_idx].type_conf;  

    int agg_class = aggregatedObjects[agg_idx].obj_class;
    double agg_class_conf = aggregatedObjects[agg_idx].class_conf;
    int sen_obj_class = objects[obj_idx].obj_class;
    double sen_obj_class_conf = objects[obj_idx].class_conf;  

//----------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------FOR TPR AND FPR MERTICS------------------------------------------------------
    aggregatedObjects[agg_idx].agg_id = aggregatedObjects[agg_idx].scen_id + "_" + objects[obj_idx].scen_id;
//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------

    // if the object does not have a type, then we choose from a pair; 
    // otherwise, if we take the data with more confidence in the type;
    // if the types are the same, then we choose the class with more confidence
    if(agg_type == -1) {
        aggregatedObjects[agg_idx].obj_type = sen_obj_type;
        aggregatedObjects[agg_idx].type_conf = sen_obj_type_conf;
        aggregatedObjects[agg_idx].obj_class = sen_obj_class;
        aggregatedObjects[agg_idx].class_conf = sen_obj_class_conf;
    } else if (agg_type != -1 && sen_obj_type != -1) {
        if(agg_type == sen_obj_type){
            if(sen_obj_type_conf > agg_type_conf) {
//----------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------FOR TPR AND FPR MERTICS-------------------------------------------------------------------
                aggregatedObjects[agg_idx].scen_id = objects[obj_idx].scen_id;
//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
                aggregatedObjects[agg_idx].type_conf = sen_obj_type_conf;
                if(sen_obj_class_conf > agg_class_conf){
                    aggregatedObjects[agg_idx].obj_class = sen_obj_class;
                    aggregatedObjects[agg_idx].class_conf = sen_obj_class_conf;
                }
            }
        } else {
            if(sen_obj_type_conf > agg_type_conf) {
//----------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------FOR TPR AND FPR MERTICS--------------------------------------------------------------------
                aggregatedObjects[agg_idx].scen_id = objects[obj_idx].scen_id;
//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
                aggregatedObjects[agg_idx].obj_type = sen_obj_type;
                aggregatedObjects[agg_idx].type_conf = sen_obj_type_conf;
                aggregatedObjects[agg_idx].obj_class = sen_obj_class;
                aggregatedObjects[agg_idx].class_conf = sen_obj_class_conf;
            }
        }
        
    }
}

// make cost martix by azimuth and object type
std::vector<std::vector<double>> DataHungarianAggregation::make_cost_matrix_az_ty(
                            std::vector<SensorData> &seq1,
                            std::vector<SensorData> &seq2
                            ){
    std::vector<std::vector<double>> costMatrix = std::vector<std::vector<double>>();

    for(int i =0; i < seq2.size(); i++){
        std::vector<double> col = std::vector<double>();
        for(int j =0; j < seq1.size();j++){

            double cost = 100;

            if(seq1[j].friendly == seq2[i].friendly){
                double s1_az = seq1[j].azimuth;
                double s2_az = seq2[i].azimuth;
                
                double manh_azim_dist = abs(s1_az - s2_az);
                if(manh_azim_dist <= AGG_MAX_OBJECT_AZIM) {
                    cost = exp(-manh_azim_dist*AGG_AZIM_SCALE_KOEFF);

                    double s1_type = seq1[j].obj_type;
                    double s1_type_conf = seq1[j].type_conf;
                    double s2_type = seq2[i].obj_type;
                    double s2_type_conf = seq2[i].type_conf;

                    double m_type = 0.1;

                    if(s1_type == s2_type) {
                        m_type = s1_type_conf * s2_type_conf;
                    } else {
                        m_type = AGG_TYPE_SCALE_KOEFF *(1-s1_type_conf * s2_type_conf);
                    }
                    cost = cost *  m_type;
                    cost = 1 - cost;
                }   
            }
            
            col.push_back(cost);

        }
        costMatrix.push_back(col);
    }
    return costMatrix;
};

// make cost martix by azimuth, object type and object class
std::vector<std::vector<double>> DataHungarianAggregation::make_cost_matrix_az_ty_cl(
                            std::vector<SensorData> &seq1,
                            std::vector<SensorData> &seq2
                            ){
    std::vector<std::vector<double>> costMatrix = std::vector<std::vector<double>>();

    for(int i =0; i < seq2.size(); i++){
        std::vector<double> col = std::vector<double>();
        for(int j =0; j < seq1.size();j++){

            double cost = 100;

            if(seq1[j].friendly == seq2[i].friendly){
                double s1_az = seq1[j].azimuth;
                double s2_az = seq2[i].azimuth;

                double manh_azim_dist = abs(s1_az - s2_az);
                if(manh_azim_dist <= AGG_MAX_OBJECT_AZIM) {
                    cost = exp(-manh_azim_dist*AGG_AZIM_SCALE_KOEFF);

                    double s1_type = seq1[j].obj_type;
                    double s1_type_conf = seq1[j].type_conf;
                    double s2_type = seq2[i].obj_type;
                    double s2_type_conf = seq2[i].type_conf;

                    double m_type = 0.1;

                    if(s1_type == s2_type) {
                        m_type = s1_type_conf * s2_type_conf;
                    } else {
                        m_type = AGG_TYPE_SCALE_KOEFF *(1-s1_type_conf * s2_type_conf);
                    }
                    cost = cost *  m_type;

                    double s1_class = seq1[j].obj_class;
                    double s1_class_conf = seq1[j].class_conf;
                    double s2_class = seq2[i].obj_class;
                    double s2_class_conf = seq2[i].class_conf;

                    double m_class = 0.1;

                    if(s1_class == s2_class && s1_type == s2_type) {
                        m_class = s1_class_conf * s2_class_conf;
                    } else {
                        m_class = AGG_CLASS_SCALE_KOEFF *(1-s1_class_conf * s2_class_conf);
                    }
                    cost = cost * m_class;
                    cost = 1 - cost;
                }
            }
            col.push_back(cost);

        }
        costMatrix.push_back(col);
    }
    return costMatrix;
};

// make cost martix by distance, azimuth and object type
std::vector<std::vector<double>> DataHungarianAggregation::make_cost_matrix_dist_az_ty(
                            std::vector<SensorData> &seq1,
                            std::vector<SensorData> &seq2
                            ){
    std::vector<std::vector<double>> costMatrix = std::vector<std::vector<double>>();

    for(int i =0; i < seq2.size(); i++){
        std::vector<double> col = std::vector<double>();
        for(int j =0; j < seq1.size();j++){

            double cost = 100;

            if(seq1[j].friendly == seq2[i].friendly){
                double s1_d = seq1[j].distance;
                double s1_az = seq1[j].azimuth;
                double s2_d = seq2[i].distance;
                double s2_az = seq2[i].azimuth;
                double evkl_dist = sqrt(pow(s1_d*cos(s1_az) - s2_d*cos(s2_az), 2) +
                                        pow(s1_d*sin(s1_az) - s2_d*sin(s2_az), 2));

                if(evkl_dist <= AGG_MAX_OBJECT_DIST) {
                    cost = exp(-evkl_dist*AGG_DIST_SCALE_KOEFF);

                    double s1_type = seq1[j].obj_type;
                    double s1_type_conf = seq1[j].type_conf;
                    double s2_type = seq2[i].obj_type;
                    double s2_type_conf = seq2[i].type_conf;

                    double m_type = 0.1;

                    if(s1_type == s2_type) {
                        m_type = s1_type_conf * s2_type_conf;
                    } else {
                        m_type = AGG_TYPE_SCALE_KOEFF *(1-s1_type_conf * s2_type_conf);
                    }
                    cost = cost * m_type;

                    cost = 1 - cost;
                }
            }
            
            col.push_back(cost);

        }
        costMatrix.push_back(col);
    }
    return costMatrix;
};

// make cost martix by distance, azimuth, object type and object class
std::vector<std::vector<double>> DataHungarianAggregation::make_cost_matrix_dist_az_ty_cl(
                            std::vector<SensorData> &seq1,
                            std::vector<SensorData> &seq2
                            ){
    std::vector<std::vector<double>> costMatrix = std::vector<std::vector<double>>();

    for(int i =0; i < seq2.size(); i++){
        std::vector<double> col = std::vector<double>();
        for(int j =0; j < seq1.size();j++){

            double cost = 100;

            if(seq1[j].friendly == seq2[i].friendly){
                double s1_d = seq1[j].distance;
                double s1_az = seq1[j].azimuth;
                double s2_d = seq2[i].distance;
                double s2_az = seq2[i].azimuth;
                double evkl_dist = sqrt(pow(s1_d*cos(s1_az) - s2_d*cos(s2_az), 2) +
                                        pow(s1_d*sin(s1_az) - s2_d*sin(s2_az), 2));

                if(evkl_dist <= AGG_MAX_OBJECT_DIST) {
                    cost = exp(-evkl_dist*AGG_DIST_SCALE_KOEFF);

                    double s1_type = seq1[j].obj_type;
                    double s1_type_conf = seq1[j].type_conf;
                    double s2_type = seq2[i].obj_type;
                    double s2_type_conf = seq2[i].type_conf;

                    double m_type = 0.1;

                    if(s1_type == s2_type) {
                        m_type = s1_type_conf * s2_type_conf;
                    } else {
                        m_type = AGG_TYPE_SCALE_KOEFF *(1-s1_type_conf * s2_type_conf);
                    }
                    cost = cost * m_type;

                    double s1_class = seq1[j].obj_class;
                    double s1_class_conf = seq1[j].class_conf;
                    double s2_class = seq2[i].obj_class;
                    double s2_class_conf = seq2[i].class_conf;

                    double m_class = 0.1;

                    if(s1_class == s2_class && s1_type == s2_type) {
                        m_class = s1_class_conf * s2_class_conf;
                    } else {
                        m_class = AGG_CLASS_SCALE_KOEFF *(1- s1_class_conf * s2_class_conf);
                    }
                    cost = cost * m_class;
                    cost = 1 - cost;
                }
            }
            col.push_back(cost);

        }
        costMatrix.push_back(col);
    }
    return costMatrix;
};