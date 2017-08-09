#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include <iostream>
#include <math.h>
#include <vector>
#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

typedef struct car_state {
    int id;
    double x;
    double y;
    double s;
    double d;
    double yaw_radians;
    double v;
    double dist2ego;
} CAR_STATE;

class PathPlanner {

    public:
        // Constructor
        PathPlanner(vector<double> map_s, vector<double> map_x, vector<double> map_y, vector<double> map_dx, vector<double> map_dy);

        // Destructor
		~PathPlanner() {};

        // create new path
        vector<vector<double>> MakePath(CAR_STATE &ego, vector<double> &prev_path_x, vector<double> &prev_path_y, vector<vector<double>> &sensor_fusion);

    private:
        CAR_STATE ego;
        int curr_lane;
        double curr_v;
        string bstate;  // behavior state
        vector<vector<double>> path_hist;
        vector<double> velocity_hist;
        int n_completed_points;
        int prev_path_size;
        int sf_size;

        int n_points = 50;
        int n_waypoints_previous = 5;
        int n_spline_waypoints = 15;
        double max_v = 45 * 0.44704;
        double target_s;
        double target_d;

        double lane_width = 4.0;
        int global_step = 0;
        int n_lanes = 3;
        double laneCostWeight = 1.0;
        double distanceCostWeight = 1.0;
        double vDiffCostWeight = 1.0;
        int laneChangeVotes = 0.0;
        int numVotesNeeded = 10;
        double min_gap = 50;
        double lane_change_dist = 50;

        int n_tot_waypoints;

        vector<double> map_s;
        vector<double> map_x;
        vector<double> map_y;
        vector<double> map_dx;
        vector<double> map_dy;

        vector<double> prev_path_x;
        vector<double> prev_path_y;

        vector<vector<double>> sensor_fusion;

        vector<vector<CAR_STATE>> lanes;
        vector<CAR_STATE*> closestCars;
        vector<pair<double, int>> costs;

        void makeLaneSpline(tk::spline &lane_spline);
        void initVelocitySpline(tk::spline &velocity_spline);

        void initPath(tk::spline &lane_spline, vector<vector<double>> &results);
        void continuePath(tk::spline &lane_spline, vector<vector<double>> &results);

        int ClosestWaypoint();
        vector<vector<double>> getLocalWaypoints();
        vector<double> getLocalXY(double global_x, double global_y);
        vector<double> getWorldXY(double local_x, double local_y);
        vector<vector<double>> convertPoints(string toRef, vector<double> from_xs, vector<double> from_ys);

        void PrepSensorFusion();
        void CalcLaneCosts();
        void UpdateState();
        void ClosestCars();

};

#endif /* PATHPLANNER_H_ */