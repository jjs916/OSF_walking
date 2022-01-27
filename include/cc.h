#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"


class CustomController
{
public:
    CustomController(RobotData &rd);
    Eigen::VectorQd getControl();

    //void taskCommandToCC(TaskCommand tc_);
    
    void computeSlow();
    void computeFast();
    void computePlanner();
    void copyRobotData(RobotData &rd_l);

    RobotData &rd_;
    RobotData rd_cc_;

    const std::string FILE_NAMES[6]={
        "../jjs_data/RH.txt",
        "../jjs_data/LH.txt",
        "../jjs_data/Joint.txt",
        "../jjs_data/walking.txt",
        "../jjs_data/torque.txt",
        "../jjs_data/common.txt"
    };
    ofstream file[6];

    //walking - by MJ
    ////////////////////////////////////////////////////////////////////////////
    void parameterSetting();
    void updateInitialState();
    void calculateFootStepTotal_MJ();
    void getRobotState();
    void floatToSupportFootstep();

    void getZmpTrajectory();
    void zmpGenerator(const unsigned int norm_size, const unsigned planning_step_num);
    void addZmpOffset();
    void onestepZmp(unsigned int current_step_number, Eigen::VectorXd& temp_px, Eigen::VectorXd& temp_py);
  
    void getComTrajectory();
    void previewcontroller(double dt, int NL, int tick, double x_i, double y_i, Eigen::Vector3d xs, Eigen::Vector3d ys, double& UX, double& UY, 
    Eigen::MatrixXd Gi, Eigen::VectorXd Gd, Eigen::MatrixXd Gx, Eigen::MatrixXd A, Eigen::VectorXd B, Eigen::MatrixXd C, Eigen::Vector3d &XD, Eigen::Vector3d &YD);  
    void preview_Parameter(double dt, int NL, Eigen::MatrixXd& Gi, Eigen::VectorXd& Gd, Eigen::MatrixXd& Gx, Eigen::MatrixXd& A, Eigen::VectorXd& B, Eigen::MatrixXd& C);
    void SC_err_compen(double x_des, double y_des);
    void CLIPM_ZMP_compen_MJ(double XZMP_ref, double YZMP_ref);
    
    void getFootTrajectory();
    void getPelvTrajectory();
    void updateNextStepTime();
    void walkingContactState();
    void walkingFstarJacSet();
    void ContactStateChangeTorque();
    ////////////////////////////////////////////////////////////////////////////
    //WholebodyController &wbc_;
    //TaskCommand tc;
    ////////////////////////////////////////walking variables////////////////////////////////////
    const double hz_ = 2000.0;
    unsigned int walking_tick_mj = 0;
    double aa = 0;
    double del_t = 0.0005;
    bool walking_enable_ ;
    unsigned int initial_flag = 0;
    double wn = 0;
    double walking_end_flag = 0;

    double target_x_;
    double target_y_;
    double target_z_;
    double com_height_;
    double target_theta_;
    double step_length_x_;
    double step_length_y_;
    int is_right_foot_swing_;
    double t_rest_init_;
    double t_rest_last_;  
    double t_double1_;
    double t_double2_;
    double t_total_;
    double t_temp_;
    double t_last_;
    double t_start_;
    double t_start_real_;
    double current_step_num_;
    double foot_height_; // 실험 제자리 0.04 , 전진 0.05 시뮬 0.04
    double t_step_change; // 지지발 바꿀때의 walkingtick

    Eigen::Vector3d pelv_rpy_current_;
    Eigen::Isometry3d pelv_yaw_rot_current_from_global_;
    Eigen::Isometry3d pelv_float_init_;
    Eigen::Isometry3d lfoot_float_init_;
    Eigen::Isometry3d rfoot_float_init_;
    Eigen::Vector3d com_float_init_;
    Eigen::MatrixXd foot_step_;
    Eigen::Isometry3d lfoot_support_init_;
    Eigen::Isometry3d rfoot_support_init_;
    Eigen::Isometry3d pelv_support_init_;
    Eigen::Vector3d com_support_init_;
    Eigen::Vector3d pelv_support_euler_init_;
    Eigen::Vector3d rfoot_support_euler_init_;
    Eigen::Vector3d lfoot_support_euler_init_;
    Eigen::Vector6d supportfoot_float_init_;
    Eigen::Vector6d swingfoot_float_init_;
    Eigen::Isometry3d pelv_support_start_;
    double total_step_num_;
    Eigen::Vector3d rfoot_rpy_current_;
    Eigen::Vector3d lfoot_rpy_current_;
    Eigen::Isometry3d rfoot_roll_rot_;
    Eigen::Isometry3d lfoot_roll_rot_;
    Eigen::Isometry3d rfoot_pitch_rot_;
    Eigen::Isometry3d lfoot_pitch_rot_;
    Eigen::Vector3d com_float_current_;
    Eigen::Vector3d com_float_current_dot;

    double R_angle = 0;
    double P_angle = 0;
    Eigen::Isometry3d pelv_float_current_;
    Eigen::Isometry3d lfoot_float_current_;
    Eigen::Isometry3d rfoot_float_current_;
    Eigen::Vector3d com_float_current_dot_LPF;
    Eigen::Vector3d com_float_current_dot_prev;
    Eigen::Isometry3d supportfoot_float_current_;
    Eigen::Isometry3d pelv_support_current_;
    Eigen::Isometry3d lfoot_support_current_;
    Eigen::Isometry3d rfoot_support_current_;
    Eigen::Vector3d com_support_current_;
    Eigen::Vector6d l_ft_;
    Eigen::Vector6d r_ft_;
    Eigen::Vector6d l_ft_LPF;
    Eigen::Vector6d r_ft_LPF;
    Eigen::Vector2d zmp_measured_;
    Eigen::Vector2d zmp_measured_LPF_;
    double zc_;

    Eigen::MatrixXd foot_step_support_frame_;
    Eigen::Vector6d swingfoot_support_init_;
    Eigen::Vector6d supportfoot_support_init_;
    Eigen::MatrixXd foot_step_support_frame_offset_;
    Eigen::Vector6d supportfoot_support_init_offset_;
    Eigen::MatrixXd ref_zmp_;

    Eigen::MatrixXd Gi_;
    Eigen::MatrixXd Gx_;
    Eigen::VectorXd Gd_;
    Eigen::MatrixXd A_;
    Eigen::VectorXd B_;
    Eigen::MatrixXd C_;

    Eigen::Vector3d xs_;
    Eigen::Vector3d ys_;
    Eigen::Vector3d xd_;
    Eigen::Vector3d yd_; 
    double UX_, UY_;
    double zmp_start_time_;
    Eigen::Vector3d com_desired_;
    double ZMP_X_REF = 0;
    double ZMP_Y_REF = 0;
    Eigen::Vector3d preview_x, preview_y, preview_x_b, preview_y_b;
    Eigen::Vector2d zmp_err_;

    Eigen::Vector2d cp_desired_;
    Eigen::Vector3d com_support_cp_;
    Eigen::Vector2d cp_measured_;
    Eigen::Vector2d del_zmp;

    Eigen::MatrixXd A_x_ssp;
    Eigen::MatrixXd B_x_ssp;
    Eigen::MatrixXd Ad_x_ssp;
    Eigen::MatrixXd Bd_x_ssp;
    Eigen::MatrixXd C_x_ssp;
    Eigen::MatrixXd D_x_ssp;
    Eigen::MatrixXd A_y_ssp;
    Eigen::MatrixXd B_y_ssp;
    Eigen::MatrixXd Ad_y_ssp;
    Eigen::MatrixXd Bd_y_ssp;
    Eigen::MatrixXd C_y_ssp;
    Eigen::MatrixXd D_y_ssp;
    Eigen::MatrixXd ff_gain_y_ssp;
    Eigen::MatrixXd ff_gain_x_ssp;
    Eigen::MatrixXd K_y_ssp;
    Eigen::MatrixXd Y_y_ssp;
    Eigen::Vector2d X_y_ssp;
    Eigen::MatrixXd K_x_ssp;
    Eigen::MatrixXd Y_x_ssp;
    Eigen::Vector2d X_x_ssp;
    double U_ZMP_y_ssp = 0;
    double U_ZMP_y_ssp_LPF = 0;
    double U_ZMP_x_ssp = 0;
    double U_ZMP_x_ssp_LPF = 0;
    double damping_x = 0;
    double damping_y = 0;

    Eigen::Vector3d rfoot_trajectory_euler_support_;
    Eigen::Vector3d lfoot_trajectory_euler_support_;
    double F_T_L_x_input = 0;
    double F_T_R_x_input = 0;
    double F_T_L_y_input = 0;
    double F_T_R_y_input = 0;

    double P_angle_input_dot = 0;
    double P_angle_input = 0;
    double R_angle_input = 0;

    Eigen::Vector2d SC_com;
    Eigen::Vector2d sc_err_before;
    Eigen::Vector2d sc_err_after;
    Eigen::Vector2d sc_err;

    double xi_;
    double yi_;

    Eigen::Vector3d pelv_float_current_dot;
    Eigen::Vector3d rfoot_float_current_dot;
    Eigen::Vector3d lfoot_float_current_dot;
    Eigen::Vector3d rfoot_support_current_dot;
    Eigen::Vector3d lfoot_support_current_dot;
    Eigen::Vector3d step_error_before;
    Eigen::Vector3d step_error_after;
    Eigen::Vector3d step_error_comp;
    /////////////////////////////////////////////////////////////////////////////////////////////

private:
//control variables
    bool task_init;
    Eigen::VectorQd ControlVal_;
    Eigen::VectorQd Task_torque;
    Eigen::VectorQd Gravity_torque;
    Eigen::VectorQd Extra_torque;
    Eigen::VectorQd Contact_torque;
    Eigen::VectorQd Total_torque;
    Eigen::VectorQd Test_torque;
//task variables
    Eigen::VectorXd f_star;
    Eigen::Vector3d COM_init;
    Eigen::Vector3d RH_init;
    Eigen::Vector3d LH_init;
    Eigen::Vector3d RF_init;
    Eigen::Vector3d LF_init;
    Eigen::VectorXd F_contact;
    double fc_ratio;
    Eigen::Vector3d COM_pos_local;
    Eigen::Vector3d RH_pos_local;
    Eigen::Vector3d LH_pos_local;
    Eigen::Vector3d COM_vel_local;
    Eigen::Vector3d RH_vel_local;
    Eigen::Vector3d LH_vel_local;
//joint variables //for IK
    Eigen::VectorQd q_init;
    Eigen::VectorQd q_desired;
    Eigen::VectorQd q_desired_pre;
    Eigen::VectorQd qdot_desired;
    Eigen::VectorXd R_xdot;
    Eigen::VectorXd L_xdot;

    Eigen::Isometry3d pelv_trajectory_float_; //pelvis frame
    Eigen::Isometry3d rfoot_trajectory_float_;
    Eigen::Isometry3d lfoot_trajectory_float_;
    Eigen::Isometry3d pelv_vel_float_;
    Eigen::Isometry3d rfoot_vel_float_;
    Eigen::Isometry3d lfoot_vel_float_;

    Eigen::Isometry3d pelv_trajectory_support_; //local frame
    Eigen::Isometry3d rfoot_trajectory_support_;  //local frame
    Eigen::Isometry3d lfoot_trajectory_support_;
    Eigen::Isometry3d pelv_vel_support_; 
    Eigen::Isometry3d rfoot_vel_support_;  //local frame
    Eigen::Isometry3d lfoot_vel_support_;

    Eigen::Vector3d pelv_traj_euler;
    Eigen::Vector3d lfoot_traj_euler;
    Eigen::Vector3d rfoot_traj_euler;

    Eigen::Vector12d q_d;//desired from ik
    Eigen::MatrixXd Jr;//leg jacobian for velocity
    Eigen::MatrixXd Jl;

    double zmp_x;
    double zmp_y;

    //control gains
    Eigen::VectorQd Kp;
    Eigen::VectorQd Kd;
    Eigen::Vector3d Kp_com;
    Eigen::Vector3d Kd_com;
    Eigen::Vector3d Kp_com_rot;
    Eigen::Vector3d Kd_com_rot;
    Eigen::Vector3d Kp_ub;
    Eigen::Vector3d Kd_ub;
    Eigen::Vector3d Kp_hand;
    Eigen::Vector3d Kd_hand;
    Eigen::Vector3d Kp_hand_rot;
    Eigen::Vector3d Kd_hand_rot;
    Eigen::Vector3d Kp_foot;
    Eigen::Vector3d Kd_foot;
    Eigen::Vector3d Kp_foot_rot;
    Eigen::Vector3d Kd_foot_rot;

    //task parameters
    bool task_state_init = true;
    bool D2S = true;
    bool first_step = true;
    bool prefoot = true; // true : left contact - false : right contact
    int state;
    int prestate;
    double presupport;// 1 = left 0 = right
    double cursupport;
    int task_state_n;
    int task_number;
    double task_time1;
    double task_time2;
    double task_time3;

    //motor variable
    Eigen::MatrixVVd motor_inertia;
    Eigen::MatrixVVd motor_inertia_inv;
};
