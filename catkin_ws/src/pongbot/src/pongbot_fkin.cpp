#include "ros/ros.h"
#include <pongbot/JointGoal.h>
#include <stdlib.h>
#include <iostream>

const double RADIAN = M_PI/180;
const double DEGREE = .29;
std::vector<double> thetalist(3, -1);

void updatePos(const pongbot::JointGoal::ConstPtr& msg);
std::vector<std::vector<double>> matrixMult(std::vector<std::vector<double>> a, std::vector<std::vector<double>> b);

//MR library functions
std::vector<std::vector<double>> VecTose3(std::vector<double> V);
std::vector<std::vector<double>> VecToso3(std::vector<double> omg);
std::vector<std::vector<double>> MatrixExp3(std::vector<std::vector<double>> so3mat);
std::vector<std::vector<double>> MatrixExp6(std::vector<std::vector<double>> se3mat);
std::vector<double> so3ToVec(std::vector<std::vector<double>> so3mat);
bool NearZero(double near);
double Norm(std::vector<double> V);
std::vector<double> AxisAng3 (std::vector<double> expc3);

int JOINT_PAN_ZERO_CONFIG, JOINT_TILT_ZERO_CONFIG, JOINT_ELBOW_ZERO_CONFIG;
int zero_offset[3];

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pongbot_fkin");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("joint_pos", 1000, updatePos);

  n.getParam("joint_pan/zero_config", JOINT_PAN_ZERO_CONFIG);
  n.getParam("joint_tilt/zero_config", JOINT_TILT_ZERO_CONFIG);
  n.getParam("joint_elbow/zero_config", JOINT_ELBOW_ZERO_CONFIG);

  zero_offset[0] = JOINT_PAN_ZERO_CONFIG * DEGREE;
  zero_offset[1] = JOINT_TILT_ZERO_CONFIG * DEGREE;
  zero_offset[2] = JOINT_ELBOW_ZERO_CONFIG * DEGREE;

  // M matrix for the arm
  std::vector<std::vector<double>> M    {
                                            {0, 0, 1, 379.93},
                                            {1, 0, 0, 0},
                                            {0, 1, 0, 84.455},
                                            {0, 0, 0, 1}
                                        };

  std::vector<double> B1 {0, 1, 0, 379.93, 0, 0};
  std::vector<double> B2 {-1, 0, 0, 0, 379.93, 65.745};
  std::vector<double> B3 {-1, 0, 0, 0, 235.6, 25};
  std::vector<std::vector<double>> Blist2 {{B1},{B2},{B3}};


  int count = 0;
  ros::Rate r(10.0);
  while (ros::ok())
  {
    pongbot::JointGoal armmsg;

    // T will be the homogeneous transfrom matrix output by fkin
    std::vector<std::vector<double>> T = M;
    std::vector<std::vector<double>> tmp0;
    std::vector<std::vector<double>> tmp1;
    std::vector<std::vector<double>> se3mat;
    
    //forward kinematics
    for (size_t i=0; i<thetalist.size(); i++)
    {
        std::vector<double> tmp2 (6, -1);
        for (size_t j=0; j<Blist2[i].size(); j++)
            tmp2[j] = Blist2[i][j] * thetalist[i];

        se3mat = VecTose3(tmp2);
        tmp0 = T;
        tmp1 = MatrixExp6(se3mat);
        T = matrixMult(tmp0, tmp1);
    }
    ROS_INFO("\n%f   %f   %f   %f\n"
                "%f   %f   %f   %f\n"
                "%f   %f   %f   %f\n"
                "%f   %f   %f   %f\n"
            ,  T[0][0],T[0][1],T[0][2],T[0][3],
                T[1][0],T[1][1],T[1][2],T[1][3],
                T[2][0],T[2][1],T[2][2],T[2][3],
                T[3][0],T[3][1],T[3][2],T[3][3]);


    ros::spinOnce();
    r.sleep();
    count++;
  }
  return 0;
}

/** Callback function for getting the goal position from subscribed topic
 * */
void updatePos(const pongbot::JointGoal::ConstPtr& msg)
{
    // need to convert joint pos (0-1023) to radian angle
    // each interval is .29 degrees
    // finally subtract an offset so that it matches the dxl motors definition of 0 degrees
    // Joint 2 (tilt) is interesting because the body moves the arm rather than the wheel
    // Okay so ^^ is not correct..
    for (size_t i=0; i<3; i++)
        thetalist[i] = ((msg->joints.at(i) * DEGREE) - zero_offset[i]) * RADIAN;
        //if (i==1)
            //thetalist[i] = ((msg->joints.at(i) * DEGREE) + zero_offset[i]) * RADIAN;
        //else
            //thetalist[i] = ((msg->joints.at(i) * DEGREE) - zero_offset[i]) * RADIAN;
}


std::vector<std::vector<double>> VecTose3(std::vector<double> V)
{
    std::vector<double> omg {V.at(0), V.at(1), V.at(2)};
    std::vector<std::vector<double>> so3mat = VecToso3(omg);
    std::vector<std::vector<double>> se3mat {
                                                {so3mat[0][0], so3mat[0][1], so3mat[0][2], V.at(3)},
                                                {so3mat[1][0], so3mat[1][1], so3mat[1][2], V.at(4)},
                                                {so3mat[2][0], so3mat[2][1], so3mat[2][2], V.at(5)},
                                                {0, 0, 0, 0}
                                            };
    return se3mat;
}

std::vector<std::vector<double>> VecToso3(std::vector<double> omg)
{
    std::vector<std::vector<double>> so3mat {
                                                {0, -omg.at(2), omg.at(1)},
                                                {omg.at(2), 0, -omg.at(0)},
                                                {-omg.at(1), omg.at(0), 0}
                                            };
    return so3mat;
}

std::vector<double> so3ToVec(std::vector<std::vector<double>> so3mat)
{
    std::vector<double> omg {so3mat[2][1], so3mat[0][2], so3mat[1][0]};
    return omg;
}

std::vector<std::vector<double>> MatrixExp3(std::vector<std::vector<double>> so3mat)
{
    std::vector<std::vector<double>> R (3, std::vector<double> (3, -1));
    std::vector<double> omgtheta = so3ToVec(so3mat);
    R = {
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1}
        };
    if (NearZero(Norm(omgtheta)))
    {
        return R;
    }else
    {
        //first 3 are omghat last element is theta
        std::vector<double> omghat_theta = AxisAng3(omgtheta);
        double theta = omghat_theta.back();
        //vectors in c++ can copy using `=`? cool!
        std::vector<std::vector<double>> omgmat = so3mat;
        for (size_t i=0; i<so3mat.size(); i++)
            for (size_t j=0; j<so3mat.at(i).size(); j++)
                omgmat[i][j] /= theta;
        
        double sintheta = sin(theta);
        double costheta = cos(theta);
        std::vector<std::vector<double>> omgmat_sqrd = matrixMult(omgmat, omgmat);
        for (size_t i=0; i<R.size(); i++)
            for (size_t j=0; j<R.at(i).size(); j++) {
                double tmp = omgmat[i][j] * sintheta;
                double tmp1 = 1-costheta;
                R[i][j] += tmp + tmp1 * omgmat_sqrd[i][j]; 
            }
    }
    return R;
}

std::vector<std::vector<double>> MatrixExp6(std::vector<std::vector<double>> se3mat)
{
    std::vector<std::vector<double>> T  {
                                            {1, 0, 0, se3mat[0][3]},
                                            {0, 1, 0, se3mat[1][3]},
                                            {0, 0, 1, se3mat[2][3]},
                                            {0, 0, 0, 1}
                                        };
    std::vector<std::vector<double>> threerows  {
                                                    {se3mat[0][0], se3mat[0][1], se3mat[0][2]},
                                                    {se3mat[1][0], se3mat[1][1], se3mat[1][2]},
                                                    {se3mat[2][0], se3mat[2][1], se3mat[2][2]}
                                                };
    std::vector<double> omgtheta = so3ToVec(threerows);
    if (NearZero(Norm(omgtheta))) {
        return T;
    }else {
        //first 3 are omghat last element is theta
        std::vector<double> omghat_theta = AxisAng3(omgtheta);
        double theta = omghat_theta.back();
        std::vector<std::vector<double>> omgmat {
                                                    {se3mat[0][0], se3mat[0][1], se3mat[0][2]},
                                                    {se3mat[1][0], se3mat[1][1], se3mat[1][2]},
                                                    {se3mat[2][0], se3mat[2][1], se3mat[2][2]},
                                                };
        std::vector<std::vector<double>> Rin (3, std::vector<double> (3, -1));
        //divide the rotation by theta
        for (size_t i=0; i<3; i++)
            for (size_t j=0; j<3; j++) {
                Rin[i][j] = omgmat[i][j];
                omgmat[i][j] /= theta;
            }

        double sintheta = sin(theta);
        double costheta = cos(theta);
        std::vector<std::vector<double>> omgmat_sqrd = omgmat;
        std::vector<std::vector<double>> omgmatOG = omgmat;

        //T matrix
        std::vector<std::vector<double>> R = MatrixExp3(Rin);
        std::vector<std::vector<double>> Itheta {
                                                {theta, 0, 0},
                                                {0, theta, 0},
                                                {0, 0, theta}
                                             };
        double tmp = 1-costheta;
        double tmp1 = theta - sintheta;
        for (size_t i=0; i<omgmat.size(); i++) 
            for (size_t j=0; j<omgmat.at(i).size(); j++) {
                omgmat[i][j] *= tmp;
                omgmat_sqrd[i][j] *= tmp1;
            }
        omgmat_sqrd = matrixMult(omgmat_sqrd, omgmatOG);

        for (size_t i=0; i<omgmat_sqrd.size(); i++) 
            for (size_t j=0; j<omgmat_sqrd.at(i).size(); j++)
                Itheta[i][j] += omgmat[i][j] + omgmat_sqrd[i][j];
                
        std::vector<double> tmp3 = {se3mat[0][3]/theta, se3mat[1][3]/theta, se3mat[2][3]/theta};
        
        std::vector<double> tmp4 {0,0,0};
        for (size_t i=0; i<Itheta.size(); i++)
            for (size_t k=0; k<tmp3.size(); k++)
                tmp4[i] += Itheta[i][k] * tmp3[k];

        T = {
                {R[0][0], R[0][1], R[0][2], tmp4[0]},
                {R[1][0], R[1][1], R[1][2], tmp4[1]},
                {R[2][0], R[2][1], R[2][2], tmp4[2]},
                {0, 0, 0, 1}
            };
        return T;     
    }
}

bool NearZero(double near)
{
    return std::abs(near) < 0.000001;
}

//DOES NORM MEANS abs() OR C++ NORM HUH???
double Norm(std::vector<double> V)
{
    double sum;
    for (size_t i=0; i<V.size(); i++)
        sum += pow(V.at(i),2);

    return sqrt(sum);
}

std::vector<double> AxisAng3 (std::vector<double> expc3)
{
    std::vector<double> omghat_theta;
    double theta = Norm(expc3);
    for (size_t i=0; i<expc3.size(); i++)
        omghat_theta.push_back(expc3[i]/theta);
    omghat_theta.push_back(theta);
    return omghat_theta;
}

std::vector<std::vector<double>> matrixMult(std::vector<std::vector<double>> a, std::vector<std::vector<double>> b)
{
  std::vector<std::vector<double>> c(a.size(), std::vector<double>(b.at(0).size(), -1));
  int N = a.size();
  int i, j,k;
  int limit = N-1;
  double acc0; //two accumulators
  double acc1;

  for(i=0; i<N; i++)
    for(j=0; j<N; j++){
      acc0 = 0;
      acc1 = 0;
      for(k=0; k<limit; k+=2){
        acc0 += a[i][k] * b[k][j]; //stores the "original"
        acc1 += a[i][k+1] * b[k+1][j]; //stores the "unroll"
      }
      //compute finishing cases
      for (; k<N; k++) {
        acc0 += a[i][k] * b[k][j];
      }

      c[i][j] = acc0 + acc1; //we add the accumulators
    }
  return c;
}
