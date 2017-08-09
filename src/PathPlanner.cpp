#include <iostream>
#include "PathPlanner.h"

using namespace std;

double distance(double x1, double y1, double x2, double y2) {
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

PathPlanner::PathPlanner(vector<double> map_s, vector<double> map_x, vector<double> map_y, vector<double> map_dx, vector<double> map_dy) {
    this->curr_lane = 1;
    this->curr_v = 0.0;
    this->bstate = "KL";
    this->target_s = max_v / n_points;
    this->target_d = curr_lane * lane_width + lane_width / 2;

    this->map_s = map_s;
    this->map_x = map_x;
    this->map_y = map_y;
    this->map_dx = map_dx;
    this->map_dy = map_dy;

    this->n_tot_waypoints = map_x.size();

    this->closestCars.resize(n_lanes);
};

vector<vector<double>> PathPlanner::MakePath(CAR_STATE &ego, vector<double> &prev_path_x, vector<double> &prev_path_y, vector<vector<double>> &sensor_fusion) {
    vector<vector<double>> results;

    this->global_step++;
    cout << "STEP: " << global_step << endl;

    this->ego = ego;
    this->curr_lane = (int)(round(round(ego.d - 2.0) / 4.0));

    this->prev_path_x = prev_path_x;
    this->prev_path_y = prev_path_y;
    this->sensor_fusion = sensor_fusion;

    this->prev_path_size = prev_path_x.size();
    this->sf_size = sensor_fusion.size();

    this->n_completed_points = n_points - prev_path_size;

    tk::spline lane_spline;
    makeLaneSpline(lane_spline);

    if (prev_path_size == 0) {
        initPath(lane_spline, results);
    } else {
        cout << "continuePath" << endl;
        continuePath(lane_spline, results);
    }

    if (bstate == "KL") {
        PrepSensorFusion();
        CalcLaneCosts();
        UpdateState();
    } else {
        if (fabs(ego.d - target_d) < 0.2) {
            laneChangeVotes = 0;
            bstate = "KL";
        }
    }

    return results;
};

void PathPlanner::makeLaneSpline(tk::spline &lane_spline) {
    vector<vector<double>> local_waypoints = getLocalWaypoints();
    lane_spline.set_points(local_waypoints[0], local_waypoints[1]);
    return;
}

void PathPlanner::initPath(tk::spline &lane_spline, vector<vector<double>> &results) {
    vector<double> local_xs;
    vector<double> local_ys;

    double local_x = 0.0;
    double local_y = 0.0;

    tk::spline velocity_spline;
    initVelocitySpline(velocity_spline);

    for (int i = 0; i < n_points; i++) {
        local_x += velocity_spline((double)i);
        local_xs.push_back(local_x);
        local_ys.push_back(lane_spline(local_x));
    }

    this->curr_v = velocity_spline((double)n_points);

    results = convertPoints("world", local_xs, local_ys);
    this->path_hist = results;

    local_x = 0.0;
    local_y = 0.0;
    for (int i = 0; i < n_points; i++) {
        this->velocity_hist.push_back(local_xs[i] - local_x);
        local_x = local_xs[i];
    }

    return;
}

void PathPlanner::continuePath(tk::spline &lane_spline, vector<vector<double>> &results) {
    vector<vector<double>> local_path = convertPoints("local", prev_path_x, prev_path_y);
    
    velocity_hist.erase(velocity_hist.begin(), velocity_hist.begin() + n_completed_points);
    path_hist[0].erase(path_hist[0].begin(), path_hist[0].begin() + n_completed_points);
    path_hist[1].erase(path_hist[1].begin(), path_hist[1].begin() + n_completed_points);

    // new lane spline including previous path
    tk::spline new_lane_spline;
    makeLaneSpline(new_lane_spline);

    vector<double> local_xs;
    vector<double> local_ys;
    for (int i = 0; i < prev_path_size; i++) {
        local_xs.push_back(local_path[0][i]);
        local_ys.push_back(local_path[1][i]);
    }

    double next_x = local_path[0][prev_path_size - 1] + lane_change_dist;
    for (int i = 0; i < prev_path_size; i++)
    {
        local_xs.push_back(next_x);
        local_ys.push_back(new_lane_spline(next_x));
        next_x += target_s;
    }

    lane_spline.set_points(local_xs, local_ys);

    // new velocity tracker
    vector<double> dt, ds;
    for (int i = 0; i < prev_path_size; i++) {
        dt.push_back(i);
        ds.push_back(velocity_hist[i]);
    }
    dt.push_back((double)n_points * 5);
    ds.push_back(target_s);

    tk::spline velocity_spline;
    velocity_spline.set_points(dt, ds);

    for(int i = prev_path_size; i < n_points; i++) {
        local_path[0].push_back(local_path[0][i - 1] + velocity_spline((double)i));
        local_path[1].push_back(lane_spline(local_path[0][i]));
    }

    // cout << "N Points: " << n_points << endl;
    // for (int i = 0; i < n_points; i++) {
    //     cout << local_path[0][i] << "," << ends;
    // }
    // cout << endl;

    vector<vector<double>> world_xy = convertPoints("world", local_path[0], local_path[1]);
    results = {prev_path_x, prev_path_y};
    for (int i = prev_path_size; i < n_points; i++) {
        results[0].push_back(world_xy[0][i]);
        results[1].push_back(world_xy[1][i]);
    }

    this->path_hist = results;

    for (int i = prev_path_size; i < n_points; i++) {
        double local_x = local_path[0][i-1];
        double local_y = local_path[0][i-1];
        this->velocity_hist.push_back(local_path[0][i] - local_x);
    }

    // cout << "V size: " << velocity_hist.size() << endl;
    // for (int i = 0; i < velocity_hist.size(); i++) {
    //     cout << velocity_hist[i] << "," << ends;
    // }
    // cout << endl;

    return;
}

void PathPlanner::initVelocitySpline(tk::spline &velocity_spline) {
    vector<double> dt, ds;

    dt.push_back(-1.0);
    dt.push_back((double)(n_points * 0.3));
    dt.push_back((double)(n_points * 0.5));
    dt.push_back((double)(n_points * 1.0));
    dt.push_back((double)(n_points * 2.0));

    ds.push_back(target_s * 0.01);
    ds.push_back(target_s * 0.10);
    ds.push_back(target_s * 0.15);
    ds.push_back(target_s * 0.25);
    ds.push_back(target_s * 0.30);

    velocity_spline.set_points(dt, ds);
    return;
}

int PathPlanner::ClosestWaypoint() {
	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < map_x.size(); i++)
	{
		double dist = distance(ego.x,ego.y,map_x[i],map_y[i]);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;
}

vector<double> PathPlanner::getLocalXY(double global_x, double global_y) {
    double dx = (global_x - ego.x);
    double dy = (global_y - ego.y);

    double local_x = dx * cos(ego.yaw_radians) + dy * sin(ego.yaw_radians);
    double local_y = -dx * sin(ego.yaw_radians) + dy * cos(ego.yaw_radians);

    return {local_x, local_y};
}

vector<double> PathPlanner::getWorldXY(double local_x, double local_y) {
    double global_x = local_x * cos(ego.yaw_radians) - local_y * sin(ego.yaw_radians) + ego.x;
    double global_y = local_x * sin(ego.yaw_radians) + local_y * cos(ego.yaw_radians) + ego.y;

    return {global_x, global_y};
}

vector<vector<double>> PathPlanner::getLocalWaypoints() {
    cout << "getLocalWaypoints" << endl;

    vector<double> local_waypoints_x;
    vector<double> local_waypoints_y;

    int closest = ClosestWaypoint();
    int wp_start = closest - n_waypoints_previous;;
    if (wp_start < 0) 
    {
        wp_start += n_tot_waypoints;
    }

    int wp_next;
    double global_x, global_y;
    vector<double> local_xy;
    for (int i = 0; i < n_spline_waypoints; i++) {
        wp_next = (wp_start + i) % n_tot_waypoints;
        global_x = map_x[wp_next] + target_d * map_dx[wp_next];
        global_y = map_y[wp_next] + target_d * map_dy[wp_next];
        local_xy = getLocalXY(global_x, global_y);

        local_waypoints_x.push_back(local_xy[0]);
        local_waypoints_y.push_back(local_xy[1]);

        // cout << local_waypoints_x[i] << "," << local_waypoints_y[i] << endl;
    }
    
    return {local_waypoints_x, local_waypoints_y};
}

vector<vector<double>> PathPlanner::convertPoints(string toRef, vector<double> from_xs, vector<double> from_ys) {
    vector<double> to_xs;
    vector<double> to_ys;

    int path_size = from_xs.size();
    vector<double> to_xy;
    for (int i = 0; i < path_size; i++) {
        if (toRef == "world") {
            to_xy = getWorldXY(from_xs[i], from_ys[i]);
        } else {
            to_xy = getLocalXY(from_xs[i], from_ys[i]);
        }
        to_xs.push_back(to_xy[0]);
        to_ys.push_back(to_xy[1]);
    }

    return {to_xs, to_ys};
}

void PathPlanner::PrepSensorFusion() {
    cout << "PrepSensorFusion" << endl;

    vector<vector<CAR_STATE>> car_lanes(n_lanes);

    for (int i = 0; i < sf_size; i++) {
        CAR_STATE other_car = {
            (int)sensor_fusion[i][0],
            sensor_fusion[i][1],
            sensor_fusion[i][2],
            sensor_fusion[i][5],
            sensor_fusion[i][6],
            0.0,
            distance(0.0, 0.0, sensor_fusion[i][3], sensor_fusion[i][4]) * 0.020,
            sensor_fusion[i][5] - ego.s
        };

        int car_lane = (int)(round(round(other_car.d - 2.0) / 4.0));
        if (car_lane >= 0 and car_lane < n_lanes) {
            car_lanes[car_lane].push_back(other_car);
        }
    }

    for (int i = 0; i < n_lanes; i++) {
        cout << car_lanes[i].size() << "," << ends;
        sort(car_lanes[i].begin(), car_lanes[i].end(), [](CAR_STATE &a, CAR_STATE &b) {
            return a.dist2ego < b.dist2ego;
        });
    }
    cout << endl;

    this->lanes = car_lanes;

    return;
}

void PathPlanner::CalcLaneCosts() {
    vector<pair<double, int>> costs;

    ClosestCars();

    double lane_cost, dist_cost, v_diff_cost, tot_cost;
    for (int i = 0; i < n_lanes; i++) {
        lane_cost = laneCostWeight * fabs(i - curr_lane) / (n_lanes - 1);

        if (!closestCars[i]) {
            dist_cost = 0;
            v_diff_cost = 0;
        } else {
            dist_cost = distanceCostWeight * max(0.0, 1 - fabs(closestCars[i]->dist2ego) / 100);
            v_diff_cost = vDiffCostWeight * max(0.0, 1 - closestCars[i]->v / (max_v/n_points));
        }
        
        tot_cost = lane_cost + dist_cost + v_diff_cost;

        cout << "Lane: " << i << ", TOT: " << tot_cost << ", LC: " << lane_cost << ", DC: " << dist_cost << ", VC: " << v_diff_cost << endl;

        costs.push_back(make_pair(lane_cost + dist_cost + v_diff_cost, i));
    }

    sort(costs.begin(), costs.end());

    this->costs = costs;
}

void PathPlanner::ClosestCars() {
    for (int i = 0; i < n_lanes; i++) {
        this->closestCars[i] = nullptr;

        int n_cars = this->lanes[i].size();
        for (int j = 0; j < n_cars; j++) {
            if (this->lanes[i][j].dist2ego > 0) {
                this->closestCars[i] = &this->lanes[i][j];
                cout << "closest car in lane " << i << ": id=" << this->closestCars[i]->id << ", dist2ego=" << this->closestCars[i]->dist2ego << endl;
                break;
            }
        }
    }
}

void PathPlanner::UpdateState() {
    int dest_lane = curr_lane;

    for (int i = 0; i < costs.size(); i++) {
        int lane_id = costs[i].second;

        cout << "checking lane: " << lane_id << endl;

        if (lane_id == curr_lane) {
            cout << "in best lane already" << lane_id << endl;
            laneChangeVotes = 0;
            break;
        }

        int n_changes = lane_id - curr_lane;
        int direction = n_changes / fabs(n_changes);

        bool safe = true;

        if (safe) {
            laneChangeVotes++;
            if (laneChangeVotes > numVotesNeeded) {
                bstate = "LC";
                dest_lane = lane_id;
                laneChangeVotes = 0;
            }
            break;
        }
    }

    cout << "dest_lane: " << dest_lane << endl;
    cout << "bstate: " << bstate << endl;

    target_d = dest_lane * lane_width + 0.5 * lane_width;

    CAR_STATE *car_ahead = closestCars[dest_lane];
    if (car_ahead) {
        if (car_ahead->dist2ego > min_gap) {
            target_s = max_v / n_points;
        } else {
            target_s = target_s > car_ahead->v ? car_ahead->v : target_s;
        }
    } else {
        target_s = max_v / n_points;
    }
}