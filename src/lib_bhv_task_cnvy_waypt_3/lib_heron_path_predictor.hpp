

/*
    Heron path predictor
    Take the dynamic model of a heron, take initial conditions, and simulate the path the heron will take to reach its target

    Inputs
        Initial conditions
        Desired target
        Time steps
    Outputs
        Vector of points


*/

#include <vector> 
#include <armadillo> 

class HeronWaypointSimulator {
    public:

    //Heron Properties
    arma::mat H(3,3);
    arma::mat D(3,3);
    double b = 0.38;

    //Heron state-dependent parameters
    arma::mat Rot(3,3); //Rotation matrix from local frame to world frame
    arma::vec x_w(3); //x, y, theta in world frame
    arma::vec x_w_dot(3); //first time derivative of x_w
    arma::vec x_w_dotdot(3); //second time derivative of x_w
    arma::vec x_l_dot(3); //Instantaneous derivative of the local state (u, v, phi_dot) - we employ differential kinematics so its integral means nothing directly
    arma::vec x_l_dotdot(3); //Instantaneous local acceleration
    arma::mat C(3,3); //Coriolis acceleration - a function of x_l_dot and entries in H

        HeronWaypointSimulator() {
            arma::vec h = {35, 35, 30};
            H = arma::diagmat(h);
            arma::vec d = {6, 6, 9};
            D = arma::diagmat(d);
        };

        void set_H(arma::mat new_H) {
            H = new_H;
        };

        void set_D(arma::mat new_D) {
            D = new_D;
        };
        void set_C() {
            C = {}
        }
        void set_rot() {
            Rot = {}
        }

};