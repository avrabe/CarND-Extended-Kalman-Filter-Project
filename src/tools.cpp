#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;


Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    if (estimations.size() != ground_truth.size()
        || estimations.size() == 0) {
        cout << "Invalid estimation or ground_truth data" << endl;
        return rmse;
    }

    for (int i = 0; i < estimations.size(); ++i) {
        VectorXd e = estimations[i] - ground_truth[i];
        e = e.array() * e.array();
        rmse += e;
    }
    double f = 1.0 / estimations.size();
    rmse = f * rmse;
    rmse = rmse.array().sqrt();
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    MatrixXd Hj(3, 4);
    //recover state parameters
    double_t px = x_state(0);
    double_t py = x_state(1);
    double_t vx = x_state(2);
    double_t vy = x_state(3);

    //TODO: YOUR CODE HERE

    double_t px2py2 = px * px + py * py;
    //check division by zero
    if (px2py2 == 0) {
        Hj << 0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0;
    } else {

        double_t sqrtpx2py2 = sqrt(px2py2);
        double_t powpx2py2 = pow(px2py2, 3 / 2);

        //compute the Jacobian matrix
        Hj << px / sqrtpx2py2, py / sqrtpx2py2, 0, 0,
                -py / px2py2, px / px2py2, 0, 0,
                py * (vx * py - vy * px) / powpx2py2, px * (vy * px - vx * py) / powpx2py2, px / sqrtpx2py2, py /
                                                                                                             sqrtpx2py2;
    }
    return Hj;
}