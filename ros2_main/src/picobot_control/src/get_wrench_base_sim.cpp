#include "ros/ros.h"
// Note these are removed as iiwa_driver/tools are ROS
// We also seperate the iiwa commands ro the kuka arm container
//code will be disabled and will need replacing with node calls
// via redis to the other container
// TODO: Fix this
//#include "iiwa_driver/AdditionalOutputs.h"
//#include <iiwa_tools/GetFK.h>
//#include <iiwa_tools/GetJacobian.h>
#include "sensor_msgs/JointState.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/Float64MultiArray.h"
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
//#include <tf/tf.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include "tf/transform_listener.h"
#include <tf_conversions/tf_eigen.h>

double joint_positions_[6];
double joint_velocities_[6];
Eigen::VectorXd ext_torques(7);
int n;
using namespace std;
Eigen::MatrixXd J(6,7);
Eigen::MatrixXd J_(7,6);
Eigen::MatrixXd Ext_torq(7,1);
geometry_msgs::WrenchStamped eef_wrench;
Eigen::Matrix<double, 6, 1> cartesian_wrench_;



ros::ServiceClient *fk_clientPtr;
ros::ServiceClient *J_clientPtr;
ros::Publisher pub_ee_wrench;
template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> pseudo_inverse(const MatT& mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) // choose appropriately
{
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto& singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance) {
            singularValuesInv(i, i) = Scalar{1} / singularValues(i);
        }
        else {
            singularValuesInv(i, i) = Scalar{0};
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}   
    

// Helper function to set value in a MultiArray
void set_multi_array(std_msgs::Float64MultiArray& array, size_t i, size_t j, double val)
{
    assert(array.layout.dim.size() == 2);
    size_t offset = array.layout.data_offset;

    array.data[offset + i * array.layout.dim[0].stride + j] = val;
}


bool transformWrench(Eigen::Matrix<double, 6, 1> cartesian_wrench,
                                                        const std::string &from_frame, const std::string &to_frame)  {
    try
    {
      tf::StampedTransform transform; tf::TransformListener tf_listener_;
      tf_listener_.waitForTransform(to_frame,from_frame,ros::Time(), ros::Duration(0.5));
      tf_listener_.lookupTransform(to_frame, from_frame, ros::Time(0), transform);
      tf::Vector3 v_f(cartesian_wrench(3), cartesian_wrench(4), cartesian_wrench(5));
      tf::Vector3 v_t(cartesian_wrench(0), cartesian_wrench(1), cartesian_wrench(2));
      tf::Vector3 v_f_rot = tf::quatRotate(transform.getRotation(), v_f);
      tf::Vector3 v_t_rot = tf::quatRotate(transform.getRotation(), v_t);
      cartesian_wrench_(0) =  v_f_rot[0]; // beacause eigentowrench will reverse this
      cartesian_wrench_(1) =  v_f_rot[1];
      cartesian_wrench_(2) =  v_f_rot[2];
      cartesian_wrench_(3) =  v_t_rot[0];
      cartesian_wrench_(4) =  v_t_rot[1];
      cartesian_wrench_(5) =  v_t_rot[2];
      return true;
    }
    catch (const tf::TransformException &ex)
    {
      ROS_ERROR_THROTTLE(1, "%s", ex.what());
      return false;
    }
  }

void iiwa_output_callback(std_msgs::Float64MultiArray incoming_msg){

    std::vector<double> command_tor = incoming_msg.data;
    // for (int i = 0; i < ext_torques.size(); i++)
    // {
    //    std::cout << "i:::" << i<< command_tor[i] << std::endl;
       
    //    //ext_torques[i] = command_tor[i];
    // }
    int count=0;
    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 1; j++)
        {
         Ext_torq(i,j) = command_tor[count];
         
         //std::cout<< i << ":::"<< j << ":::"<< J(i,j) << std::endl;
         count++;
         
        }
        
    }
    
        
   
    //std::cout << Ext_torq << std::endl;
    const Eigen::MatrixXd J_t_pinv = pseudo_inverse(J.transpose());
    
    Eigen::VectorXd external_ee_wrench = J_t_pinv*Ext_torq;
    // Eigen::Matrix<double, 6, 1> cartesian_wrench;
    // for (size_t i = 0; i < 6; i++)
    // {
    //    cartesian_wrench(i) =  external_ee_wrench[i];
    //    std::cout << cartesian_wrench(i) << std::endl;
    // }

    eef_wrench.header.frame_id = "iiwa_link_0";
    eef_wrench.header.stamp = ros::Time::now();
    eef_wrench.wrench.force.x =  external_ee_wrench[3];
    eef_wrench.wrench.force.y =  external_ee_wrench[4];
    eef_wrench.wrench.force.z =  external_ee_wrench[5];
    eef_wrench.wrench.torque.x = external_ee_wrench[0];
    eef_wrench.wrench.torque.y = external_ee_wrench[1];
    eef_wrench.wrench.torque.z = external_ee_wrench[2];
    // std::cout << eef_wrench.wrench.force.x << std::endl;
 
    pub_ee_wrench.publish(eef_wrench);
    //std::cout << ext_torques << std::endl;
    //incoming_msg.external_torques;
    //incoming_msg.commanded_positions;

}
void iiwa_jointstates_callback(sensor_msgs::JointState inflow_j_states){ 
    //ROS_INFO("Subscribing to the joint_states");
    //std::cout <<  joint_positions << std::endl;
    //std::cout <<  positions_joints << std::endl;

    int n = inflow_j_states.name.size();
    //joint_positions.name.resize(n);
    //joint_positions.position.resize(n);
    for (int i = 0; i < n; ++i)
    {
        joint_positions_[i] = inflow_j_states.position[i];
        joint_velocities_[i] = inflow_j_states.velocity[i];
        //  std::cout << joint_positions_[i] << std::endl;
        //  std::cout << joint_velocities_[i] << std::endl;
    
    } 

    iiwa_tools::GetJacobian get_J_srv;
    for (int i = 0; i < n; ++i)
    {
            
    get_J_srv.request.joint_angles.push_back(joint_positions_[i]);
    get_J_srv.request.joint_velocities.push_back(joint_velocities_[i]);
    }
   
    if (J_clientPtr->call(get_J_srv))
    {
        // ROS_INFO("Jacobian: " get_J_srv.response.jacobian.data[0]);
        // std::cout << get_J_srv.response.jacobian << std::endl;
    }
    else
    {
        ROS_ERROR("Failed to call service iiwa_jacobian_server");
    }
    std::vector<double> j_data = get_J_srv.response.jacobian.data;
    J_ = Eigen::Map<Eigen::MatrixXd>(j_data.data(), 7, 6);
    J=J_.transpose();
    // std::cout << "J" << std::endl; 
    // for (int i = 0; i < J.rows(); ++i) {
    //         for (int j = 0; j < J.cols(); ++j) {
    //         std::cout << J(i, j) << " ";
    //         }
    //     std::cout << std::endl;  // Move to the next row
    //     }

    //Old Method to get Jacobian
    // int count = 0;
    // std::cout << "J_second" << std::endl; 
    // for (int i = 0; i < 6; i++)
    // {
    //     for (int j = 0; j < 7; j++)
    //     {
    //      J(i,j) = j_data[count];
         
    //     //  std::cout<< i << ":::"<< j << ":::"<< J(i,j) << std::endl;
    //      count++;
    //      std::cout << J(i, j) << " ";
    //     }
    //     std::cout << std::endl;
    // }
    //cout << J << endl;
    
    
}

int main(int argc, char **argv){
    ros::init(argc, argv, "get_wrench_base_sim");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    ROS_INFO("into the get_plot Node");
    ros::ServiceClient fk_client = nh.serviceClient<iiwa_tools::GetFK>("/iiwa/iiwa_fk_server");
    fk_clientPtr = &fk_client;
    fk_client.waitForExistence();
    ros::ServiceClient J_client = nh.serviceClient<iiwa_tools::GetJacobian>("/iiwa/iiwa_jacobian_server");
    J_clientPtr = &J_client;
    J_client.waitForExistence();
    ros::Subscriber sub_joint_states = nh.subscribe("/iiwa/joint_states", 100, iiwa_jointstates_callback);
    ros::Subscriber sub_add = nh.subscribe("/iiwa/CartesianImpedance_trajectory_controller/commanded_torques", 100, iiwa_output_callback);
    pub_ee_wrench = nh.advertise<geometry_msgs::WrenchStamped>("/cartesian_wrench",1);



    ros::waitForShutdown();

}