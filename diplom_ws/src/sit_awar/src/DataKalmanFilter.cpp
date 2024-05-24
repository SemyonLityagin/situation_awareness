
#include "DataKalmanFilter.h"

// to track object state in filter module
KalmanObject::KalmanObject(uint32_t id, SensorData object) {
    this->id = id;
    this->object = object;
    this->state_size = 0;
    this->meas_size = 0;
    
    // if no distance in object state -> for kalman init state vector 3 and meas vector 1
    // else -> for kalman init state vector 6 and meas vector 2
    if(this->object.distance < 0) {
        this->state_size = 3;
        this->meas_size = 1;
    } else {
        this->state_size = 6;
        this->meas_size = 2;
    }
    this->live_time = LIVE_TIME;
    this->already_pred = false;

    // init dynamic model F matrix and fill state vector x
    Eigen::MatrixXd F;
    Eigen::VectorXd x;
    x.resize(this->state_size);
    if(this->state_size == 3) {
        F = this->create_F3();
        x << this->object.azimuth, 0,0;
    } else {
        F = this->create_F6();
        x << this->object.distance,this->object.azimuth,0,0,0,0;
    }

    // init KalmanFicter
    this->filter = std::make_shared<KalmanFilter>(this->state_size, this->meas_size);
    this->filter->init(x, F);
};

// create F 3x3 matrix for filtering
Eigen::MatrixXd KalmanObject::create_F3(){
    Eigen::MatrixXd F;
    F.resize(3,3);
    F << 1,delta_t,delta_t*delta_t/2,
             0,1,delta_t,
             0,0,1;
    return F;
};
// create F 6x6 matrix for filtering
Eigen::MatrixXd KalmanObject::create_F6(){
    Eigen::MatrixXd F;
    F.resize(6,6);
    F << 1,0,delta_t,0,delta_t*delta_t/2,0,
             0,1,0,delta_t,0,delta_t*delta_t/2,
             0,0,1,0,delta_t,0,
             0,0,0,1,0,delta_t,
             0,0,0,0,1,0,
             0,0,0,0,0,1;
    return F;
};

// predict object state
void KalmanObject::predict(){
    if(this->already_pred){
        this->live_time--;
        return;
    }
    Eigen::VectorXd x = this->filter->predict();
    this->already_pred = true;
    if (this->state_size == 3) {
        this->object.azimuth = x[0];
    } else if(this->state_size == 6){
        this->object.distance = x[0];
        this->object.azimuth = x[1];
    }
    this->live_time--;
}

// correct state by meas
// if get more characteristic -> reinit Filter for big state vector x
ObjectState KalmanObject::update_measurement(SensorData state){
    this->live_time = LIVE_TIME;
    this->already_pred = false;

    Eigen::VectorXd meas;
    // if state vector for distance and azimuth
    // but in meas only azimuth -> use current state vector distance
    if(this->state_size == 3){
        meas.resize(1);
        meas << state.azimuth;
    } else if(this->state_size == 6){
        meas.resize(2);
        if(state.distance < 0){
            meas << this->object.distance, state.azimuth;
        } else {
            meas << state.distance, state.azimuth;
        }
    }

    Eigen::VectorXd corr_state = this->filter->update(meas);
    ObjectState newObjState = ObjectState();
    if (this->state_size == 3) {
        state.azimuth = corr_state[0];
        newObjState.vel_azim = corr_state[1];

        if(state.distance >= 0) {
            this->state_size = 6;
            this->meas_size = 2;
            Eigen::MatrixXd F;
            Eigen::VectorXd x;
            x.resize(this->state_size);
            F = this->create_F6();
            x << state.distance,state.azimuth,0,0,0,0;
            this->filter = std::make_shared<KalmanFilter>(this->state_size, this->meas_size);
            this->filter->init(x, F);
        }
    } else if(this->state_size == 6){
        state.distance = corr_state[0];
        state.azimuth = corr_state[1];
        newObjState.vel_dist = corr_state[2];
        newObjState.vel_azim = corr_state[3];
    }
    state.fil_id = this->object.scen_id + "_" + state.scen_id;
    this->object = state;
    newObjState.obj_sensor_data = state;
    return newObjState;
}


std::unordered_map<uint32_t, ObjectState> DataKalmanFilter::get_filtered_objects(
        std::vector<SensorData> &aggregatedObjects,
        std::shared_ptr<IdGenerator> idGenerator 
        ){
        this->logger.open("/media/sf_ros2dir/diplom_ws/logger_fil.txt",std::ios::app);
        // this->logger.open("/media/sf_ros2dir/diplom_ws/logger_timer_fil.txt", std::ios::app);
        
        std::cout << "-----------------FILTERING------------\n";

        //--------------------FOR TIMER LOG----------------------------
        // clock_t tStart = clock();
        //------------------------------------------------------------

        std::unordered_map<uint32_t, ObjectState> filteredObjects = std::unordered_map<uint32_t, ObjectState>();
        
        // if there are no object states in the current tracking -> do it from aggregatedObjects
        if(this->objects.empty()){
            for(SensorData aggObj: aggregatedObjects){
                uint32_t id = idGenerator->get_id();
                KalmanObject newObj = KalmanObject(id, aggObj);
                this->objects.push_back(newObj);
                ObjectState newObjState = ObjectState();
                newObjState.obj_sensor_data = aggObj;
                filteredObjects[id] = newObjState;
            }
        } else {
            // if no new object states -> return empty map
            if(aggregatedObjects.empty()){
                return filteredObjects;
            }

            // first - predict new objects state
            for(int i = 0; i < this->objects.size(); i++){
                //-----------------------FOR TP FP LOGGER-----------------------------
                if(i == this->objects.size()-1) {
                    this->logger << this->objects[i].object.scen_id;
                } else {
                    this->logger << this->objects[i].object.scen_id << ",";
                }
                //------------------------
                this->objects[i].predict();
            }

            //-----------------------FOR TP FP LOGGER-----------------------------
            this->logger << "|";
            for(int i = 0; i < aggregatedObjects.size(); i++){
                if(i == aggregatedObjects.size()-1) {
                    this->logger << aggregatedObjects[i].scen_id;
                } else {
                    this->logger << aggregatedObjects[i].scen_id << ",";
                }
            }
            this->logger << "|";
            //---------------------------------------
            
            // tracking already associated objects state in filter 
            // with objects state from the aggregatedObjects
            std::vector<bool> alreadyAssFil = std::vector<bool>(this->objects.size(), 0);
            std::vector<bool> alreadyAssAgg = std::vector<bool>(aggregatedObjects.size(), 0);

            int TP = 0;
            int FP = 0;
            int TN = 0;
            int FN = 0;

            // associate in two iteration: by distance and azimuth; by asimuth
            for (int variant = 0; variant < 2; variant++) {
                std::vector<KalmanObject> pretendentObjFil = std::vector<KalmanObject>();
                std::vector<int> pretendentIdxFil = std::vector<int>();

                std::vector<SensorData> pretendentObjAgg = std::vector<SensorData>();
                std::vector<int> pretendentIdxAgg = std::vector<int>();

                std::vector<int> hungAssignment = std::vector<int>();
                if(variant == 0){
                    dist_az(
                        pretendentObjFil, objects,
                        pretendentIdxFil, alreadyAssFil);

                    dist_az(
                        pretendentObjAgg, aggregatedObjects,
                        pretendentIdxAgg, alreadyAssAgg);
                } else {
                    az(
                        pretendentObjFil, objects,
                        pretendentIdxFil, alreadyAssFil);

                    az(
                        pretendentObjAgg, aggregatedObjects,
                        pretendentIdxAgg, alreadyAssAgg);
                }             

                if ((!pretendentObjAgg.empty() and !pretendentObjFil.empty()))
                {
                    std::vector<std::vector<double>> costMatrix = std::vector<std::vector<double>>();
                    if(variant == 0)
                        costMatrix = make_cost_matrix_dist_az(pretendentObjFil, pretendentObjAgg);
                    else
                        costMatrix = make_cost_matrix_az(pretendentObjFil, pretendentObjAgg);

                    double hungCost = this->hungariangSolver.solve(costMatrix, hungAssignment);

                    // correct objects state and init new in filter
                    int range = pretendentObjAgg.size();
                    for (int x = range - 1; x >= 0 ; x--)
                    {
                        int ass_idx = hungAssignment[x];
                        int fill_idx = pretendentIdxFil[ass_idx];
                        int agg_idx = pretendentIdxAgg[x];
                        double cost = costMatrix[x][ass_idx];

                        //-----------------------FOR TP FP LOGGER-----------------------------
                        if (ass_idx == -1) {
                             bool flag = 1;
                             for(int m = 0; m < pretendentIdxFil.size(); m++){
                                 int fill_p_idx = pretendentIdxFil[m];
                                 if(objects[fill_p_idx].object.scen_id == aggregatedObjects[agg_idx].scen_id){
                                     FN++;
                                     flag=0;
                                     break;
                                 }
                             }
                             if(flag)
                                 TN++;
                         }
                         if (ass_idx != -1 && cost < FIL_COST_TRESHHOLD[variant]){
                                 if(objects[fill_idx].object.scen_id == aggregatedObjects[agg_idx].scen_id) {
                                     TN++;
                                 } else {
                                     TN++;
                                 }
                             }
                        //------------------------------------------

                        if((ass_idx == -1 || cost > FIL_COST_TRESHHOLD[variant]) && variant == 1) {
                            uint32_t new_idx = idGenerator->get_id();
                            this->objects.push_back(KalmanObject(new_idx, aggregatedObjects[agg_idx]));

                            ObjectState newObjState = ObjectState();
                            newObjState.obj_sensor_data = aggregatedObjects[agg_idx];
                            filteredObjects[new_idx] = newObjState;

                            alreadyAssAgg[agg_idx] = 1;
                        } else {
                            if(ass_idx != -1 && cost < FIL_COST_TRESHHOLD[variant]){
                                
                                //-----------------------FOR TP FP LOGGER-----------------------------
                                if(objects[fill_idx].object.scen_id == aggregatedObjects[agg_idx].scen_id) {
                                    TP++;
                                } else {
                                    FP++;
                                }
                                //--------------------------------------------
                                
                                uint32_t new_idx = idGenerator->get_id();
                                ObjectState newObjState = objects[fill_idx].update_measurement(aggregatedObjects[agg_idx]);
                                uint32_t idx = objects[fill_idx].id;
                                filteredObjects[idx] = newObjState;

                                alreadyAssFil[fill_idx] = 1;
                                alreadyAssAgg[agg_idx] = 1;
                            }
                        }
                    }
                }
            }
            
            //-------------------------FOR TP NP LOGGER------------------------
            std::cout << "FILTER RES:\n";
            this->logger << "TP_" << TP << "_TN_" << TN << "_FP_" << FP << "_FN_" << FN << "|";
            std::cout << "TP_" << TP << "_TN_" << TN << "_FP_" << FP << "_FN_" << FN << "\n";
            std::cout << "TPR_" << TP/(TP+FN) << "_FPR_" << FP/(FP+TN) << "\n";

            for(int i = 0; i < this->objects.size(); i++){
                if(i == this->objects.size()-1) {
                    this->logger << this->objects[i].object.fil_id;
                } else {
                    this->logger << this->objects[i].object.fil_id << ",";
                }
            }
            this->logger << "\n";
            //---------------------------------
        
            // if objects state too old - delete it
            for(int i = alreadyAssFil.size()-1; i >= 0; i--){
                if(!alreadyAssFil[i]){
                    if(this->objects[i].live_time == 0){
                        std::swap(this->objects[i], this->objects[this->objects.size()-1]);
                        this->objects.pop_back();
                    }
                }
            }
        }
        //--------------FOR TIME LOGGER------------------
        // if(!filteredObjects.empty()){
        //     this->logger << filteredObjects.size() << "|";
        //     this->logger << (double)(clock() - tStart)/CLOCKS_PER_SEC << "\n";
        //     std::cout << "Fil Time: " << (double)(clock() - tStart)/CLOCKS_PER_SEC <<"\n";
        // }
        //-----------------------
        this->logger.close();
        return filteredObjects;
};

// make cost matrix by distance and azimuth
std::vector<std::vector<double>> DataKalmanFilter::make_cost_matrix_dist_az(
                            std::vector<KalmanObject> &seq1,
                            std::vector<SensorData> &seq2 
                            ){

    std::vector<std::vector<double>> costMatrix = std::vector<std::vector<double>>();

    for(int i =0; i < seq2.size(); i++){
        std::vector<double> col = std::vector<double>();
        for(int j =0; j < seq1.size();j++){

            double cost = 100;

            if(seq1[j].object.friendly == seq2[i].friendly){
                double s1_az = seq1[j].object.azimuth;
                double s2_az = seq2[i].azimuth;
                
                double manh_azim_dist = abs(s1_az - s2_az);
                if(manh_azim_dist <= FIL_MAX_OBJECT_AZIM) {
                    cost = exp(-manh_azim_dist*FIL_AZIM_SCALE_KOEFF);

                    double s1_type = seq1[j].object.obj_type;
                    double s2_type = seq2[i].obj_type;
                    double s1_class = seq1[j].object.obj_class;
                    double s2_class = seq2[i].obj_class;

                    if(s1_type != s2_type) {
                        cost = 100;
                    } else {
                        if(s1_class == -1 || s2_class == -1 || (s1_class == s2_class)){
                            cost = 1 - cost;
                        } else{
                            cost = 100;
                        }
                    }
                }   
            }
            
            col.push_back(cost);

        }
        costMatrix.push_back(col);
    }
    return costMatrix;
};

// make cost matrix by azimuth
std::vector<std::vector<double>> DataKalmanFilter::make_cost_matrix_az(
                            std::vector<KalmanObject> &seq1,
                            std::vector<SensorData> &seq2
                            ){
    std::vector<std::vector<double>> costMatrix = std::vector<std::vector<double>>();

    for(int i =0; i < seq2.size(); i++){
        std::vector<double> col = std::vector<double>();
        for(int j =0; j < seq1.size();j++){

            double cost = 100;

            if(seq1[j].object.friendly == seq2[i].friendly){
                double s1_d = seq1[j].object.distance;
                double s1_az = seq1[j].object.azimuth;
                double s2_d = seq2[i].distance;
                double s2_az = seq2[i].azimuth;
                double evkl_dist = sqrt(pow(s1_d*cos(s1_az) - s2_d*cos(s2_az), 2) +
                                        pow(s1_d*sin(s1_az) - s2_d*sin(s2_az), 2));

                if(evkl_dist <= FIL_MAX_OBJECT_DIST) {
                    cost = exp(-evkl_dist*FIL_DIST_SCALE_KOEFF);

                    double s1_type = seq1[j].object.obj_type;
                    double s2_type = seq2[i].obj_type;
                    double s1_class = seq1[j].object.obj_class;
                    double s2_class = seq2[i].obj_class;

                    if(s1_type != s2_type) {
                        cost = 100;
                    } else {
                        if(s1_class == -1 || s2_class == -1 || (s1_class == s2_class)){
                            cost = 1 - cost;
                        } else{
                            cost = 100;
                        }
                    }
                }
            }
            
            col.push_back(cost);

        }
        costMatrix.push_back(col);
    }
    return costMatrix;
};

// get aggregatedObjects by distance and azimuth
void DataKalmanFilter::dist_az(
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

// get aggregatedObjects by azimuth
void DataKalmanFilter::az(
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

// get filtering objects state by distance and azimuth
void DataKalmanFilter::dist_az(
        std::vector<KalmanObject> &fillObjects, std::vector<KalmanObject> &objects, 
        std::vector<int> &fillIdxs, std::vector<bool> alreagyAggObj
        ){
            for(int j = 0; j < objects.size();j++){
                if(alreagyAggObj[j])
                    continue;
                if(objects[j].object.distance < 0)
                    continue;

                fillObjects.push_back(objects[j]);
                fillIdxs.push_back(j);
            }
        };

// get filtering objects state by azimuth
void DataKalmanFilter::az(
        std::vector<KalmanObject> &fillObjects, std::vector<KalmanObject> &objects, 
        std::vector<int> &fillIdxs, std::vector<bool> alreagyAggObj
        ){
            for(int j = 0; j < objects.size();j++){
                if(alreagyAggObj[j])
                    continue;

                fillObjects.push_back(objects[j]);
                fillIdxs.push_back(j);
            }
        };