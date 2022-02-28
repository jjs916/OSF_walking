#include "cc.h"
#define PI 3.141592653589793238462643
#define controltype 2 //1:original 2:mosf 3:ik

using namespace TOCABI;

CustomController::CustomController(RobotData &rd) : rd_(rd) //, wbc_(dc.wbc_)
{
    ControlVal_.setZero();
    Task_torque.setZero();
    Gravity_torque.setZero();
    Extra_torque.setZero();
    Contact_torque.setZero();
    Total_torque.setZero();
    task_init = true;

    //motor_inertia.resize(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);
    motor_inertia.setZero();

    for (int i = 0; i < 3; ++i)
    {
        motor_inertia(i, i) = 94.0;
        motor_inertia(i + 3, i + 3) = 20.0;
    }
    motor_inertia(5, 5) = 10.0;

    for (int i = 0; i < 2; ++i)
    {
        motor_inertia(6 + 6 * i, 6 + 6 * i) = 0.56;
        motor_inertia(7 + 6 * i, 7 + 6 * i) = 0.8;
        motor_inertia(8 + 6 * i, 8 + 6 * i) = 1.08;
        motor_inertia(9 + 6 * i, 9 + 6 * i) = 1.08;
        motor_inertia(10 + 6 * i, 10 + 6 * i) = 1.08;
        motor_inertia(11 + 6 * i, 11 + 6 * i) = 0.306;

        motor_inertia(21 + 10 * i, 21 + 10 * i) = 0.185;
        motor_inertia(22 + 10 * i, 22 + 10 * i) = 0.184;
        motor_inertia(23 + 10 * i, 23 + 10 * i) = 0.192;
        motor_inertia(24 + 10 * i, 24 + 10 * i) = 0.184;
        motor_inertia(25 + 10 * i, 25 + 10 * i) = 0.056;
        motor_inertia(26 + 10 * i, 26 + 10 * i) = 0.05;
        motor_inertia(27 + 10 * i, 27 + 10 * i) = 0.015;
        motor_inertia(28 + 10 * i, 28 + 10 * i) = 0.015;
    }
    motor_inertia(18, 18) = 1.01;
    motor_inertia(19, 19) = 1.01;
    motor_inertia(20, 20) = 1.27;
    motor_inertia(29, 29) = 0.015;
    motor_inertia(30, 30) = 0.015;

    motor_inertia_inv = motor_inertia.llt().solve(MatrixXd::Identity(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL));

    for (int i = 0; i < 3; ++i)
    {
        //walking gain
        // Kp_com(i) = 300.0;
        // Kd_com(i) = 10.0;
        // Kp_ub(i) = 900.0;
        // Kd_ub(i) = 60.0;
        // Kp_hand(i) = 1600.0;
        // Kd_hand(i) = 40.0;
        // Kp_hand_rot(i) = 400.0;
        // Kd_hand_rot(i) = 40.0;
        // Kp_foot(i) = 1600.0;
        // Kd_foot(i) = 80.0;
        // Kp_foot_rot(i) = 2500.0;
        // Kd_foot_rot(i) = 60.0;
        //real robot gains
        if(controltype == 1){
            Kp_com(i) = 50.0;//60.0;
            Kd_com(i) = 5.0;//5.0;
            Kp_com_rot(i) = 550.0;
            Kd_com_rot(i) = 5.0;
            Kp_ub(i) = 100.0;
            Kd_ub(i) = 10.0;
            Kp_hand(i) = 10.0;
            Kd_hand(i) = 1.0;
            Kp_hand_rot(i) = 10.0;
            Kd_hand_rot(i) = 1.0;
            Kp_foot(i) = 100.0;
            Kd_foot(i) = 5.0;
            Kp_foot_rot(i) = 100.0;
            Kd_foot_rot(i) = 5.0;
        }
        else if (controltype == 2)
        {
            Kp_com(i) = 110.0;
            Kd_com(i) = 7.0;//40 60 100
            Kp_com_rot(i) = 100.0;
            Kd_com_rot(i) = 1.0;
            Kp_ub(i) = 50.0;
            Kd_ub(i) = 1.0;
            Kp_hand(i) = 100.0;
            Kd_hand(i) = 5.0;
            Kp_hand_rot(i) = 100.0;
            Kd_hand_rot(i) = 5.0;
            Kp_foot(i) = 400.0;
            Kd_foot(i) = 40.0;
            Kp_foot_rot(i) = 100.0;
            Kd_foot_rot(i) = 5.0;
        }
    }
    //////simulation
    // Kp << 1800.0, 2100.0, 2100.0, 2100.0, 1800.0, 1800.0,           //left foot
    //     1800.0, 2100.0, 2100.0, 2100.0, 1800.0, 1800.0,             //right foot
    //     2200.0, 2200.0, 2200.0,                                  //waist
    //     400.0, 800.0, 400.0, 400.0, 250.0, 250.0, 50.0, 50.0, //left hand
    //     50.0, 50.0,                                           //neck
    //     400.0, 800.0, 400.0, 400.0, 250.0, 250.0, 50.0, 50.0; //right hand

    // Kd << 70.0, 90.0, 90.0, 90.0, 80.0, 80.0,
    //     70.0, 90.0, 90.0, 90.0, 80.0, 80.0,
    //     90.0, 90.0, 1590.0,
    //     10.0, 10.0, 10.0, 10.0, 2.5, 2.0, 2.0, 2.0,
    //     2.0, 2.0,
    //     10.0, 10.0, 10.0, 10.0, 2.5, 2.0, 2.0, 2.0;
    /////real gain
    Kp << 2000.0, 5000.0, 4000.0, 3700.0, 5000.0, 5000.0,           //left foot
        2000.0, 5000.0, 4000.0, 3700.0, 5000.0, 5000.0,             //right foot
        6000.0, 10000.0, 10000.0,                                  //waist
        400.0, 800.0, 400.0, 400.0, 250.0, 250.0, 50.0, 50.0, //left hand
        50.0, 50.0,                                           //neck
        400.0, 800.0, 400.0, 400.0, 250.0, 250.0, 50.0, 50.0; //right hand

    Kd << 15.0, 50.0, 20.0, 25.0, 30.0, 30.0,
        15.0, 50.0, 20.0, 25.0, 30.0, 30.0,
        200.0, 100.0, 100.0,
        10.0, 10.0, 10.0, 10.0, 2.5, 2.0, 2.0, 2.0,
        2.0, 2.0,
        10.0, 10.0, 10.0, 10.0, 2.5, 2.0, 2.0, 2.0;

    file[0].open(FILE_NAMES[5].c_str());
    file[1].open(FILE_NAMES[4].c_str());

    step_error_before.setZero();
    step_error_after.setZero();
    step_error_comp.setZero();
}

Eigen::VectorQd CustomController::getControl()
{
    return ControlVal_;
}

// void CustomController::taskCommandToCC(TaskCommand tc_)
// {
//     tc = tc_;
// }

void CustomController::computeSlow()
{
    if(rd_.tc_.mode == 10)
    {
        WBC::SetContact(rd_, 1, 1);

        if (task_init)
        {
            rd_.link_[Upper_Body].rot_desired = Matrix3d::Identity();
            
            cout << "DSP1" << endl;
            task_number = 6 + 3 + 6 + 6;
            f_star.resize(task_number);
            f_star.setZero();
            rd_.J_task.resize(task_number, MODEL_DOF_VIRTUAL);
            rd_.J_task.setZero();
            task_time1 = 3.0;

            for (int i = 0; i < 3; ++i)
            {
                //realrobot-mosf
                // Kp_com(i) = 90.0;
                // Kd_com(i) = 5.0; //40 60 100
                // Kp_com_rot(i) = 100.0;
                // Kd_com_rot(i) = 1.0;
                // Kp_ub(i) = 50.0;
                // Kd_ub(i) = 1.0;
                // Kp_hand(i) = 100.0;
                // Kd_hand(i) = 5.0;
                // Kp_hand_rot(i) = 100.0;
                // Kd_hand_rot(i) = 5.0;
                // Kp_foot(i) = 400.0;
                // Kd_foot(i) = 40.0;
                // Kp_foot_rot(i) = 100.0;
                // Kd_foot_rot(i) = 5.0;
                //realrobot-osf
                // Kp_com(i) = 700.0;
                // Kd_com(i) = 6.0; //40 60 100
                // Kp_com_rot(i) = 100.0;
                // Kd_com_rot(i) = 1.0;
                // Kp_ub(i) = 50.0;
                // Kd_ub(i) = 1.0;
                // Kp_hand(i) = 100.0;
                // Kd_hand(i) = 5.0;
                // Kp_hand_rot(i) = 100.0;
                // Kd_hand_rot(i) = 5.0;
                // Kp_foot(i) = 400.0;
                // Kd_foot(i) = 40.0;
                // Kp_foot_rot(i) = 100.0;
                // Kd_foot_rot(i) = 5.0;
                //simulation-mosf
                Kp_com(i) = 90.0;
                Kd_com(i) = 5.0; //40 60 100
                Kp_com_rot(i) = 200.0;
                Kd_com_rot(i) = 1.0;
                Kp_ub(i) = 50.0;
                Kd_ub(i) = 1.0;
                Kp_hand(i) = 100.0;
                Kd_hand(i) = 5.0;
                Kp_hand_rot(i) = 100.0;
                Kd_hand_rot(i) = 5.0;
                Kp_foot(i) = 400.0;
                Kd_foot(i) = 40.0;
                Kp_foot_rot(i) = 100.0;
                Kd_foot_rot(i) = 5.0;
                //simulation-osf
                // Kp_com(i) = 400.0;
                // Kd_com(i) = 6.0; //40 60 100
                // Kp_com_rot(i) = 900.0;
                // Kd_com_rot(i) = 10.0;
                // Kp_ub(i) = 900.0;
                // Kd_ub(i) = 10.0;
                // Kp_hand(i) = 100.0;
                // Kd_hand(i) = 5.0;
                // Kp_hand_rot(i) = 100.0;
                // Kd_hand_rot(i) = 5.0;
                // Kp_foot(i) = 400.0;
                // Kd_foot(i) = 40.0;
                // Kp_foot_rot(i) = 100.0;
                // Kd_foot_rot(i) = 5.0;
            }

            rd_.link_[Right_Foot].SetGain(Kp_foot(0), Kd_foot(0), 0.0, Kp_foot_rot(0), Kd_foot_rot(0), 0.0);
            rd_.link_[Left_Foot].SetGain(Kp_foot(0), Kd_foot(0), 0.0, Kp_foot_rot(0), Kd_foot_rot(0), 0.0);
            rd_.link_[Right_Hand].SetGain(Kp_hand(0), Kd_hand(0), 0.0, Kp_hand_rot(0), Kd_hand_rot(0), 0.0);
            rd_.link_[Left_Hand].SetGain(Kp_hand(0), Kd_hand(0), 0.0, Kp_hand_rot(0), Kd_hand_rot(0), 0.0);
            rd_.link_[Pelvis].SetGain(Kp_com(0), Kd_com(0), 0.0, Kp_com_rot(0), Kd_com_rot(0), 0.0);
            rd_.link_[Upper_Body].SetGain(0.0, 0.0, 0.0, Kp_ub(0), Kd_ub(0), 0.0);
            rd_.link_[COM_id].SetGain(Kp_com(0), Kd_com(0), 0.0, Kp_com_rot(0), Kd_com_rot(0), 0.0);

            COM_init = rd_.link_[Pelvis].xpos - rd_.link_[Right_Foot].xpos; //rd_.link_[Pelvis].xpos;//
            RH_init = rd_.link_[Right_Hand].xpos - rd_.link_[Upper_Body].xpos;
            LH_init = rd_.link_[Left_Hand].xpos - rd_.link_[Upper_Body].xpos;

            rd_.link_[Pelvis].x_desired = COM_init;//
            rd_.link_[Pelvis].x_desired(0) = COM_init(0) + rd_.tc_.l_x;//
            rd_.link_[Pelvis].x_desired(1) = COM_init(1) + rd_.tc_.l_y;//               //rd_.link_[Pelvis].xpos(1) + tc.l_y;//
            rd_.link_[Pelvis].x_desired(2) = COM_init(2) + rd_.tc_.l_z;//
            rd_.link_[Pelvis].rot_desired = Matrix3d::Identity();

            rd_.link_[Right_Hand].x_desired = RH_init;
            rd_.link_[Right_Hand].rot_desired = rd_.link_[Upper_Body].rot_desired;
            rd_.link_[Left_Hand].x_desired = LH_init;
            rd_.link_[Left_Hand].rot_desired = rd_.link_[Upper_Body].rot_desired;

            task_init = false;
        }

        rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Pelvis].Jac();
        rd_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac().block(3, 0, 3, MODEL_DOF_VIRTUAL);
        rd_.J_task.block(9, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Right_Hand].Jac();
        rd_.J_task.block(15, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Left_Hand].Jac();

        COM_pos_local = rd_.link_[Pelvis].xpos - rd_.link_[Right_Foot].xpos;
        RH_pos_local = rd_.link_[Right_Hand].xpos - rd_.link_[Upper_Body].xpos;
        LH_pos_local = rd_.link_[Left_Hand].xpos - rd_.link_[Upper_Body].xpos;
        COM_vel_local = rd_.link_[Pelvis].v - rd_.link_[Right_Foot].v;
        RH_vel_local = rd_.link_[Right_Hand].v - rd_.link_[Upper_Body].v;
        LH_vel_local = rd_.link_[Left_Hand].v - rd_.link_[Upper_Body].v;

        //Eigen::Vector3d test = rd_.link_[Pelvis].xpos - rd_.link_[14].xpos;
        //double rr = test.transpose()*test;
        //cout << sqrt(17000/(rd_.total_mass_*rr)) << endl;
        //cout << test << endl << endl;


        for (int i = 0; i < 3; ++i)
        {
            rd_.link_[Pelvis].x_traj(i) = DyrosMath::QuinticSpline(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + task_time1, COM_init(i), 0.0, 0.0, rd_.link_[Pelvis].x_desired(i), 0.0, 0.0)(0);
            rd_.link_[Pelvis].v_traj(i) = DyrosMath::QuinticSpline(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + task_time1, COM_init(i), 0.0, 0.0, rd_.link_[Pelvis].x_desired(i), 0.0, 0.0)(1);

            rd_.link_[Right_Hand].x_traj(i) = DyrosMath::QuinticSpline(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + task_time1, RH_init(i), 0.0, 0.0, RH_init(i), 0.0, 0.0)(0);
            rd_.link_[Right_Hand].v_traj(i) = DyrosMath::QuinticSpline(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + task_time1, RH_init(i), 0.0, 0.0, RH_init(i), 0.0, 0.0)(1);

            rd_.link_[Left_Hand].x_traj(i) = DyrosMath::QuinticSpline(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + task_time1, LH_init(i), 0.0, 0.0, LH_init(i), 0.0, 0.0)(0);
            rd_.link_[Left_Hand].v_traj(i) = DyrosMath::QuinticSpline(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + task_time1, LH_init(i), 0.0, 0.0, LH_init(i), 0.0, 0.0)(1);
        }
        rd_.link_[Pelvis].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);
        rd_.link_[Upper_Body].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

        // f_star(0) = Kp_com(0) * (rd_.link_[Pelvis].x_traj(0) - (rd_.link_[Pelvis].xpos(0) - rd_.link_[Right_Foot].xpos(0))) + Kd_com(0) * (rd_.link_[Pelvis].v_traj(0) - rd_.link_[Pelvis].v(0));
        // f_star(1) = Kp_com(1) * (rd_.link_[Pelvis].x_traj(1) - (rd_.link_[Pelvis].xpos(1) - rd_.link_[Right_Foot].xpos(1))) + Kd_com(1) * (rd_.link_[Pelvis].v_traj(1) - rd_.link_[Pelvis].v(1));
        // f_star(2) = Kp_com(2) * (rd_.link_[Pelvis].x_traj(2) - (rd_.link_[Pelvis].xpos(2) - rd_.link_[Right_Foot].xpos(2))) + Kd_com(2) * (rd_.link_[Pelvis].v_traj(2) - rd_.link_[Pelvis].v(2));
        f_star.segment(0, 3) = WBC::GetFstarPosJS(rd_.link_[Pelvis], rd_.link_[Pelvis].x_traj, COM_pos_local, rd_.link_[Pelvis].v_traj, COM_vel_local);
        f_star.segment(3, 3) = WBC::GetFstarRot(rd_.link_[Pelvis]);
        f_star.segment(6, 3) = WBC::GetFstarRot(rd_.link_[Upper_Body]);
        // f_star(9) = Kp_hand(0) * (rd_.link_[Right_Hand].x_traj(0) - (rd_.link_[Right_Hand].xpos(0) - rd_.link_[Upper_Body].xpos(0))) + Kd_hand(0) * (rd_.link_[Right_Hand].v_traj(0) - rd_.link_[Upper_Body].v(0));
        // f_star(10) = Kp_hand(1) * (rd_.link_[Right_Hand].x_traj(1) - (rd_.link_[Right_Hand].xpos(1) - rd_.link_[Upper_Body].xpos(1))) + Kd_hand(1) * (rd_.link_[Right_Hand].v_traj(1) - rd_.link_[Upper_Body].v(1));
        // f_star(11) = Kp_hand(2) * (rd_.link_[Right_Hand].x_traj(2) - (rd_.link_[Right_Hand].xpos(2) - rd_.link_[Upper_Body].xpos(2))) + Kd_hand(2) * (rd_.link_[Right_Hand].v_traj(2) - rd_.link_[Upper_Body].v(2));
        f_star.segment(9, 3) = WBC::GetFstarPosJS(rd_.link_[Right_Hand], rd_.link_[Right_Hand].x_traj, RH_pos_local, rd_.link_[Right_Hand].v_traj, RH_vel_local);
        f_star.segment(12, 3) = WBC::GetFstarRot(rd_.link_[Right_Hand]);
        // f_star(15) = Kp_hand(0) * (rd_.link_[Left_Hand].x_traj(0) - (rd_.link_[Left_Hand].xpos(0) - rd_.link_[Upper_Body].xpos(0))) + Kd_hand(0) * (rd_.link_[Left_Hand].v_traj(0) - rd_.link_[Upper_Body].v(0));
        // f_star(16) = Kp_hand(1) * (rd_.link_[Left_Hand].x_traj(1) - (rd_.link_[Left_Hand].xpos(1) - rd_.link_[Upper_Body].xpos(1))) + Kd_hand(1) * (rd_.link_[Left_Hand].v_traj(1) - rd_.link_[Upper_Body].v(1));
        // f_star(17) = Kp_hand(2) * (rd_.link_[Left_Hand].x_traj(2) - (rd_.link_[Left_Hand].xpos(2) - rd_.link_[Upper_Body].xpos(2))) + Kd_hand(2) * (rd_.link_[Left_Hand].v_traj(2) - rd_.link_[Upper_Body].v(2));
        f_star.segment(15, 3) = WBC::GetFstarPosJS(rd_.link_[Left_Hand], rd_.link_[Left_Hand].x_traj, LH_pos_local, rd_.link_[Left_Hand].v_traj, LH_vel_local);
        f_star.segment(18, 3) = WBC::GetFstarRot(rd_.link_[Left_Hand]);
// const clock_t begin_time = clock();
        fc_ratio = 1.0;
        
        Gravity_torque = WBC::GravityCompensationTorque(rd_);
        //Task_torque = WBC::TaskControlTorqueMotor(rd_, f_star, motor_inertia, motor_inertia_inv);
        Extra_torque = WBC::TaskControlTorqueExtra(rd_, f_star, motor_inertia, motor_inertia_inv);
        //Task_torque = WBC::TaskControlTorque(rd_, f_star);
        //Test_torque = WBC::ContactForceRedistributionTorqueJS(rd_, Extra_torque, fc_ratio, 0);//0이 오른발

        //Eigen::VectorXd test_force;
        //test_force = WBC::Forcecompute(rd_, Extra_torque, motor_inertia, motor_inertia_inv);
        //cout << "Torque"<< endl;
        //cout << test_force << endl << endl;
        
        //  file[0] << rd_.control_time_
        //  << "\t" << Extra_torque(0) + Test_torque(0) << "\t" << Extra_torque(1) + Test_torque(1) << "\t" << Extra_torque(2) + Test_torque(2) << "\t" << Extra_torque(3) + Test_torque(3) 
        //  << "\t" << Extra_torque(6) + Test_torque(6) << "\t" << Extra_torque(7) + Test_torque(7) << "\t" << Extra_torque(8) + Test_torque(8) << "\t" << Extra_torque(9) + Test_torque(9)
        //  << endl;
        // file[0] << rd_.control_time_
        //  << "\t" << test_force(0) << "\t" << test_force(1) << "\t" << test_force(2) << "\t" << test_force(3) 
        //  << "\t" << test_force(4) << "\t" << test_force(5) << "\t" << test_force(6) << "\t" << test_force(7) 
        //  << endl;
        // if (rd_.control_time_ >= rd_.tc_time_ + task_time1 - 0.01) //only for going to SSP
        // {
        //     fc_ratio = DyrosMath::QuinticSpline(rd_.control_time_, rd_.tc_time_ + task_time1 - 0.01, rd_.tc_time_ + task_time1, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0)(0);
        // }
        // else
        // {
        //     fc_ratio = 1.0;
        // }
        
        Contact_torque = WBC::ContactForceRedistributionTorqueJS(rd_, Task_torque + Gravity_torque + Extra_torque, fc_ratio, 0);//0이 오른발
        
        Total_torque = Gravity_torque + Task_torque + Contact_torque + Extra_torque;
        rd_.torque_desired = Total_torque;
        //rd_.torque_desired = WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_) + WBC::TaskControlTorque(rd_, fstar));
    }
    else if (rd_.tc_.mode == 11)
    {
      if (rd_.tc_init)
      {
        std::cout << "mode 11 init" << std::endl;
        rd_.tc_init = false;

        rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;
      }

      WBC::SetContact(rd_, 1, 1);

      rd_.J_task.setZero(9, MODEL_DOF_VIRTUAL);
      rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[COM_id].Jac();
      rd_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac().block(3, 0, 3, MODEL_DOF_VIRTUAL);

      rd_.link_[COM_id].x_desired = rd_.tc_.ratio * rd_.link_[Left_Foot].x_init + (1 - rd_.tc_.ratio) * rd_.link_[Right_Foot].x_init;
      rd_.link_[COM_id].x_desired(2) = rd_.tc_.height;

      rd_.link_[Upper_Body].rot_desired = DyrosMath::Euler2rot(rd_.tc_.roll, rd_.tc_.pitch, rd_.tc_.yaw + rd_.link_[Pelvis].yaw_init);

      Eigen::VectorXd fstar;
      rd_.link_[COM_id].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);
      rd_.link_[Upper_Body].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

      fstar.setZero(9);
      fstar.segment(0, 6) = WBC::GetFstar6d(rd_.link_[COM_id]);
      fstar.segment(6, 3) = WBC::GetFstarRot(rd_.link_[Upper_Body]);

      rd_.torque_desired = WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_) + WBC::TaskControlTorque(rd_, fstar));

      WBC::SetContact(rd_, 1, 1);

      if (rd_.control_time_ >= rd_.tc_time_ + rd_.tc_.time)
      {
        if (initial_flag == 0)
        {
          walking_enable_ = true;
          initial_flag = 1;
          walking_tick_mj = 0;
          walking_end_flag = 0;
          cout << "init\n"
               << endl;
        }

        Gravity_torque = WBC::GravityCompensationTorque(rd_);
        Total_torque = Gravity_torque;
        rd_.torque_desired = Total_torque;
      }
    }
    else if (rd_.tc_.mode == 12){
      if (walking_enable_ == true)
      {
        if (walking_tick_mj == 0)
        {
          task_number = 6 + 3; //com + upperbody orientation
          f_star.resize(task_number);
          f_star.setZero();
          rd_.J_task.resize(task_number, MODEL_DOF_VIRTUAL);
          rd_.J_task.setZero();
          rd_.link_[Pelvis].rot_desired = Matrix3d::Identity();
          rd_.link_[Upper_Body].rot_desired = Matrix3d::Identity();

          parameterSetting();
          for (int i = 0; i < 3; ++i)
          {
            //simulation-mosf
            Kp_com(i) = 100.0;//390.0;
            Kd_com(i) = 5.0;
            Kp_com_rot(i) = 200.0;//500.0
            Kd_com_rot(i) = 10.0;//50.0;
            Kp_ub(i) = 500.0;
            Kd_ub(i) = 50.0;
            Kp_hand(i) = 100.0;
            Kd_hand(i) = 5.0;
            Kp_hand_rot(i) = 100.0;
            Kd_hand_rot(i) = 5.0;
            Kp_foot(i) = 700.0;
            Kd_foot(i) = 5.0;
            Kp_foot_rot(i) = 400.0;
            Kd_foot_rot(i) = 10.0;
          }

          rd_.link_[Right_Foot].SetGain(Kp_foot(0), Kd_foot(0), 0.0, Kp_foot_rot(0), Kd_foot_rot(0), 0.0);
          rd_.link_[Left_Foot].SetGain(Kp_foot(0), Kd_foot(0), 0.0, Kp_foot_rot(0), Kd_foot_rot(0), 0.0);
          rd_.link_[Right_Hand].SetGain(Kp_hand(0), Kd_hand(0), 0.0, Kp_hand_rot(0), Kd_hand_rot(0), 0.0);
          rd_.link_[Left_Hand].SetGain(Kp_hand(0), Kd_hand(0), 0.0, Kp_hand_rot(0), Kd_hand_rot(0), 0.0);
          rd_.link_[Pelvis].SetGain(Kp_com(0), Kd_com(0), 0.0, Kp_com_rot(0), Kd_com_rot(0), 0.0);
          rd_.link_[Upper_Body].SetGain(0.0, 0.0, 0.0, Kp_ub(0), Kd_ub(0), 0.0);
          rd_.link_[COM_id].SetGain(Kp_com(0), Kd_com(0), 0.0, Kp_com_rot(0), Kd_com_rot(0), 0.0);

          RH_init = rd_.link_[Right_Hand].xpos - rd_.link_[Upper_Body].xpos;
          LH_init = rd_.link_[Left_Hand].xpos - rd_.link_[Upper_Body].xpos;

          rd_.link_[Right_Hand].x_desired = RH_init;
          rd_.link_[Right_Hand].rot_desired = rd_.link_[Upper_Body].rot_desired;
          rd_.link_[Left_Hand].x_desired = LH_init;
          rd_.link_[Left_Hand].rot_desired = rd_.link_[Upper_Body].rot_desired;

          cout << "parameter setting OK" << endl;
          cout << "mode = 12" << endl;
        }
        walkingContactState();
        updateInitialState();
        getRobotState();
        floatToSupportFootstep();

        if (current_step_num_ < total_step_num_)
        {
          //cout << total_step_num_ << endl;
          getZmpTrajectory();
          getComTrajectory();
          getFootTrajectory();
          getPelvTrajectory();

          rd_.link_[Pelvis].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + 1.0);
          rd_.link_[Upper_Body].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + 1.0);

          for (int i = 0; i < 3; ++i)
          {
            rd_.link_[Right_Hand].x_traj(i) = DyrosMath::QuinticSpline(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + 1.0, RH_init(i), 0.0, 0.0, RH_init(i), 0.0, 0.0)(0);
            rd_.link_[Right_Hand].v_traj(i) = DyrosMath::QuinticSpline(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + 1.0, RH_init(i), 0.0, 0.0, RH_init(i), 0.0, 0.0)(1);
            rd_.link_[Left_Hand].x_traj(i) = DyrosMath::QuinticSpline(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + 1.0, LH_init(i), 0.0, 0.0, LH_init(i), 0.0, 0.0)(0);
            rd_.link_[Left_Hand].v_traj(i) = DyrosMath::QuinticSpline(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + 1.0, LH_init(i), 0.0, 0.0, LH_init(i), 0.0, 0.0)(1);
          }

          // WBC::SetContact(rd_, 1, 1);
          walkingFstarJacSet();
          Eigen::VectorXd force;
          Task_torque = WBC::TaskControlTorqueMotor(rd_, f_star, motor_inertia, motor_inertia_inv, force);
          Extra_torque = WBC::TaskControlTorqueExtra(rd_, f_star, motor_inertia, motor_inertia_inv);
          Task_torque = Task_torque + Extra_torque;
          // Task_torque = WBC::TaskControlTorque(rd_, f_star);

          Gravity_torque = WBC::GravityCompensationTorque(rd_);
          ContactStateChangeTorque(); //Contact_torque 계산
          Total_torque = Task_torque + Gravity_torque + Contact_torque;
          //updateNextStepTime();
        }
        else
        {
          if (walking_end_flag == 0)
          {
            cout << "walking finish" << endl;
            walking_end_flag = 1;
            initial_flag = 0;
          }
        }

        rd_.torque_desired = Total_torque;
        DyrosMath::rot2Euler_tf2(rd_.link_[Left_Foot].r_traj, left_r, left_p, left_y);
        DyrosMath::rot2Euler_tf2(rd_.link_[Right_Foot].r_traj, right_r, right_p, right_y);
        DyrosMath::rot2Euler_tf2(rd_.link_[Left_Foot].rotm, lr, lp, ly);
        DyrosMath::rot2Euler_tf2(rd_.link_[Right_Foot].rotm, rr, rp, ry);

        file[0] << walking_tick_mj // << "\t" << rd_.J_C.rows()
                //<< "\t" << current_step_num_ << "\t" << total_step_num_-1
                << "\t" << foot_step_(current_step_num_,6)
                // << "\t" << t_start_ + t_rest_init_ << "\t" << t_start_ + t_rest_init_ + t_double1_ << "\t" << t_start_ + t_total_ - t_rest_last_ - t_double2_
                // << "\t" << t_start_ + t_total_ - t_rest_last_ << "\t" << t_start_ + t_total_// << "\t" << t_last_
                //<< "\t" << force(6) << "\t" << force(7) << "\t" << force(8)
                // << "\t" << force(9) << "\t" << force(10) << "\t" << force(11) << "\t" << force(12) << "\t" << force(13) << "\t" << force(14)
                // << "\t" << step_error_before(0) << "\t" << step_error_before(1) << "\t" << step_error_before(2)
                << "\t" << step_error_comp(0) << "\t" << step_error_comp(1) << "\t" << step_error_comp(2)
                << "\t" << pelv_trajectory_support_.translation()(0) << "\t" << pelv_trajectory_support_.translation()(1) << "\t" << pelv_trajectory_support_.translation()(2)
                // << "\t" << pelv_trajectory_support_.translation()(0) - step_error_comp(0) << "\t" << pelv_trajectory_support_.translation()(1) - step_error_comp(1) << "\t" << pelv_trajectory_support_.translation()(2) - step_error_comp(2)
                << "\t" << pelv_support_current_.translation()(0) << "\t" << pelv_support_current_.translation()(1) << "\t" << pelv_support_current_.translation()(2)
                << "\t" << lfoot_trajectory_support_.translation()(0) << "\t" << lfoot_trajectory_support_.translation()(1) << "\t" << lfoot_trajectory_support_.translation()(2)
                << "\t" << current_lfoot_control.translation()(0) << "\t" << current_lfoot_control.translation()(1) << "\t" << current_lfoot_control.translation()(2)
                << "\t" << rfoot_trajectory_support_.translation()(0) << "\t" << rfoot_trajectory_support_.translation()(1) << "\t" << rfoot_trajectory_support_.translation()(2)
                << "\t" << current_rfoot_control.translation()(0) << "\t" << current_rfoot_control.translation()(1) << "\t" << current_rfoot_control.translation()(2)
                // << "\t" << left_r << "\t" << left_p << "\t" << left_y
                // << "\t" << lr << "\t" << lp << "\t" << ly
                // << "\t" << right_r << "\t" << right_p << "\t" << right_y
                // << "\t" << rr << "\t" << rp << "\t" << ry
                << endl;
        file[1] << walking_tick_mj
                << "\t" << Total_torque(0) << "\t" << Total_torque(1) << "\t" << Total_torque(2) << "\t" << Total_torque(3) << "\t" << Total_torque(4) << "\t" << Total_torque(5)
                << "\t" << Total_torque(6) << "\t" << Total_torque(7) << "\t" << Total_torque(8) << "\t" << Total_torque(9) << "\t" << Total_torque(10) << "\t" << Total_torque(11)
                << endl;
                
        if (current_step_num_ < total_step_num_)
        {
          updateNextStepTime();
        }
      }
      else
      {
        WBC::SetContact(rd_, 1, 1);

        walkingFstarJacSet();
        Eigen::VectorXd force;
        Task_torque = WBC::TaskControlTorqueMotor(rd_, f_star, motor_inertia, motor_inertia_inv, force);
        Extra_torque = WBC::TaskControlTorqueExtra(rd_, f_star, motor_inertia, motor_inertia_inv);
        Task_torque = Task_torque + Extra_torque;

        Gravity_torque = WBC::GravityCompensationTorque(rd_);
        if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
        {
          Contact_torque = WBC::ContactForceRedistributionTorqueJS(rd_, Gravity_torque + Task_torque, 1.0, 1);
        }
        else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
        {
          Contact_torque = WBC::ContactForceRedistributionTorqueJS(rd_, Gravity_torque + Task_torque, 1.0, 0);
        }
        //Contact_torque 계산

        Total_torque = Task_torque + Gravity_torque + Contact_torque;

        if (walking_end_flag == 0)
        {
          cout << "walking finish" << endl;
          walking_end_flag = 1;
          initial_flag = 0;
        }
        rd_.torque_desired = Total_torque;
      }
    }
    else if (rd_.tc_.mode == 13)
    {
        if (rd_.tc_init)
        {
            //Initialize settings for Task Control! 

            rd_.tc_init = false;
            std::cout<<"cc mode 13"<<std::endl;

            //rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;
        }

        WBC::SetContact(rd_, 1, 1);

        rd_.J_task.setZero(9, MODEL_DOF_VIRTUAL);
        rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[COM_id].Jac();
        rd_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac().block(3, 0, 3, MODEL_DOF_VIRTUAL);

        rd_.link_[COM_id].x_desired = rd_.tc_.ratio * rd_.link_[Left_Foot].x_init + (1 - rd_.tc_.ratio) * rd_.link_[Right_Foot].x_init;
        rd_.link_[COM_id].x_desired(2) = rd_.tc_.height;

        rd_.link_[Upper_Body].rot_desired = DyrosMath::rotateWithX(rd_.tc_.roll) * DyrosMath::rotateWithY(rd_.tc_.pitch) * DyrosMath::rotateWithZ(rd_.tc_.yaw + rd_.link_[Pelvis].yaw_init);

        Eigen::VectorXd fstar;
        rd_.link_[COM_id].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

        rd_.link_[Upper_Body].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

        fstar.setZero(9);
        fstar.segment(0, 6) = WBC::GetFstar6d(rd_.link_[COM_id]);
        fstar.segment(6, 3) = WBC::GetFstarRot(rd_.link_[Upper_Body]);

        rd_.torque_desired = WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_) + WBC::TaskControlTorque(rd_, fstar));
    }
}

void CustomController::computeFast()
{
    // if (tc.mode == 10)
    // {
    // }
    // else if (tc.mode == 11)
    // {
    // }
}

void CustomController::computePlanner()
{
}

void CustomController::copyRobotData(RobotData &rd_l)
{
    std::memcpy(&rd_cc_, &rd_l, sizeof(RobotData));
}

void CustomController::parameterSetting()
{
    target_x_ = 5.0;
    target_y_ = 0.0;
    target_z_ = 0.0;
    com_height_ = 0.71;
    target_theta_ = 0.0;
    step_length_x_ = 0.20;
    step_length_y_ = 0.0;
    is_right_foot_swing_ = 1;

    // 1.4 Hz 실험
    t_rest_init_ = 0.27*hz_;
    t_rest_last_ = 0.27*hz_;  
    t_double1_ = 0.03*hz_;
    t_double2_ = 0.03*hz_;
    t_total_= 1.3*hz_;

    // t_rest_init_ = 0.23*hz_;
    // t_rest_last_ = 0.23*hz_;  
    // t_double1_ = 0.02*hz_;
    // t_double2_ = 0.02*hz_;
    // t_total_= 1.2*hz_;

    t_temp_ = 4.0*hz_;
    t_last_ = t_total_ + t_temp_ ;
    t_start_ = t_temp_ + 1 ;
    t_start_real_ = t_start_ + t_rest_init_;

    current_step_num_ = 0;
    foot_height_ = 0.055; // 실험 제자리 0.04 , 전진 0.05 시뮬 0.04
}

void CustomController::updateNextStepTime()
{
    if(walking_tick_mj == t_last_)
    {
        if(current_step_num_ != total_step_num_-1)
        {
          t_start_ = t_last_ + 1 ;
          t_start_real_ = t_start_ + t_rest_init_;
          t_last_ = t_start_ + t_total_ -1;
          current_step_num_ ++;
        }
    }
   if(current_step_num_ == total_step_num_-1 && walking_tick_mj >= t_last_ + t_total_)
   {
     walking_enable_ = false; cout << "Last " << pelv_float_init_.translation()(0) << "," << lfoot_float_init_.translation()(0) << "," << rfoot_float_init_.translation()(0) << "," << pelv_rpy_current_(2)*180/3.141592 << endl;
   }
   walking_tick_mj ++;
}

void CustomController::updateInitialState()
{
  if(walking_tick_mj == 0)
  {
    //calculateFootStepTotal();
    calculateFootStepTotal_MJ();
        
    pelv_rpy_current_.setZero();
    pelv_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm); //ZYX multiply
        
    pelv_yaw_rot_current_from_global_ = DyrosMath::rotateWithZ(pelv_rpy_current_(2));   
    
    pelv_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Pelvis].rotm;

    pelv_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[Pelvis].xpos);
    //pelv_float_init_.translation()(0) += 0.11;
    
    // pelv_float_init_.translation()(0) = 0;
    // pelv_float_init_.translation()(1) = 0;

    lfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Left_Foot].rotm;
    lfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[Left_Foot].xpos);  // 지면에서 Ankle frame 위치
    
    // lfoot_float_init_.translation()(0) = 0;
    // lfoot_float_init_.translation()(1) = 0.1225;

    rfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Right_Foot].rotm;
    rfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[Right_Foot].xpos); // 지면에서 Ankle frame
    
    // rfoot_float_init_.translation()(0) = 0;
    // rfoot_float_init_.translation()(1) = -0.1225;

    com_float_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[COM_id].xpos); // 지면에서 CoM 위치   
    
    // com_float_init_(0) = 0;
    // com_float_init_(1) = 0;

    // if(aa == 0)
    // {
    //   lfoot_float_init_.translation()(1) = 0.1025;
    //   rfoot_float_init_.translation()(1) = -0.1025;
    //   aa = 1;
    // }
    cout << "First " << pelv_float_init_.translation()(0) << "," << lfoot_float_init_.translation()(0) << "," << rfoot_float_init_.translation()(0) << "," << pelv_rpy_current_(2)*180/3.141592 << endl;
    
    Eigen::Isometry3d ref_frame;

    if(foot_step_(0, 6) == 0)  //right foot support
    { ref_frame = rfoot_float_init_; }
    else if(foot_step_(0, 6) == 1)
    { ref_frame = lfoot_float_init_; }
         
    lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),lfoot_float_init_);
    rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),rfoot_float_init_);
    pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame) * pelv_float_init_;
    com_support_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(ref_frame), com_float_init_);

    pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear());
    rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
    lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear());
     
    supportfoot_float_init_.setZero();
    swingfoot_float_init_.setZero();
     
    if(foot_step_(0,6) == 1) //left suppport foot
    {
      for(int i=0; i<2; i++)
        supportfoot_float_init_(i) = lfoot_float_init_.translation()(i);
      for(int i=0; i<3; i++)
        supportfoot_float_init_(i+3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

      for(int i=0; i<2; i++)
        swingfoot_float_init_(i) = rfoot_float_init_.translation()(i);
      for(int i=0; i<3; i++)
        swingfoot_float_init_(i+3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

      supportfoot_float_init_(0) = 0.0;
      swingfoot_float_init_(0) = 0.0;
    }
    else
    {
      for(int i=0; i<2; i++)
        supportfoot_float_init_(i) = rfoot_float_init_.translation()(i);
      for(int i=0; i<3; i++)
        supportfoot_float_init_(i+3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

      for(int i=0; i<2; i++)
        swingfoot_float_init_(i) = lfoot_float_init_.translation()(i);
      for(int i=0; i<3; i++)
        swingfoot_float_init_(i+3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

      supportfoot_float_init_(0) = 0.0;
      swingfoot_float_init_(0) = 0.0;
    }

    pelv_support_start_ = pelv_support_init_;
    total_step_num_ = foot_step_.col(1).size();
    
    xi_ = com_support_init_(0); // preview parameter
    yi_ = com_support_init_(1);
    zc_ = com_support_init_(2);
    
  }
  else if(current_step_num_ != 0 && walking_tick_mj == t_start_) // step change 
  { 
    pelv_rpy_current_.setZero();
    pelv_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm); //ZYX multiply

    pelv_yaw_rot_current_from_global_ = DyrosMath::rotateWithZ(pelv_rpy_current_(2));
    
    rfoot_rpy_current_.setZero();
    lfoot_rpy_current_.setZero();
    rfoot_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Right_Foot].rotm);  
    lfoot_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Left_Foot].rotm);

    rfoot_roll_rot_ = DyrosMath::rotateWithX(rfoot_rpy_current_(0));
    lfoot_roll_rot_ = DyrosMath::rotateWithX(lfoot_rpy_current_(0));
    rfoot_pitch_rot_ = DyrosMath::rotateWithY(rfoot_rpy_current_(1));
    lfoot_pitch_rot_ = DyrosMath::rotateWithY(lfoot_rpy_current_(1));
    
    pelv_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Pelvis].rotm;

    pelv_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[Pelvis].xpos);
    
    // lfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Left_Foot].rotm;
    lfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * DyrosMath::inverseIsometry3d(lfoot_pitch_rot_) * DyrosMath::inverseIsometry3d(lfoot_roll_rot_) * rd_.link_[Left_Foot].rotm; // 기울기 반영
    lfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[Left_Foot].xpos);  // 지면에서 Ankle frame 위치
    
    // rfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Right_Foot].rotm;
    rfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * DyrosMath::inverseIsometry3d(rfoot_pitch_rot_) * DyrosMath::inverseIsometry3d(rfoot_roll_rot_) * rd_.link_[Right_Foot].rotm; // 기울기 반영
    rfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[Right_Foot].xpos); // 지면에서 Ankle frame
    
    com_float_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[COM_id].xpos); // 지면에서 CoM 위치   
    
    Eigen::Isometry3d ref_frame;

    if(foot_step_(current_step_num_, 6) == 0)  //right foot support
    { ref_frame = rfoot_float_init_; }
    else if(foot_step_(current_step_num_, 6) == 1)
    { ref_frame = lfoot_float_init_; }     

    pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame) * pelv_float_init_;
    com_support_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(ref_frame), com_float_init_);
    pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear());

    lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),lfoot_float_init_);
    rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),rfoot_float_init_);
    rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
    lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear());
      
  }
}

void CustomController::getRobotState()
{
  pelv_rpy_current_.setZero();
  pelv_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm); //ZYX multiply

  R_angle = pelv_rpy_current_(0);
  P_angle = pelv_rpy_current_(1); 
  pelv_yaw_rot_current_from_global_ = DyrosMath::rotateWithZ(pelv_rpy_current_(2));
    
  rfoot_rpy_current_.setZero();
  lfoot_rpy_current_.setZero();
  rfoot_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Right_Foot].rotm);  
  lfoot_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Left_Foot].rotm);
  rfoot_roll_rot_ = DyrosMath::rotateWithX(rfoot_rpy_current_(0));
  lfoot_roll_rot_ = DyrosMath::rotateWithX(lfoot_rpy_current_(0));
  rfoot_pitch_rot_ = DyrosMath::rotateWithY(rfoot_rpy_current_(1));
  lfoot_pitch_rot_ = DyrosMath::rotateWithY(lfoot_rpy_current_(1));

  pelv_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Pelvis].rotm;

  pelv_float_current_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[Pelvis].xpos);
  pelv_float_current_dot = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[Pelvis].v); 
  // lfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * DyrosMath::inverseIsometry3d(lfoot_pitch_rot_) * DyrosMath::inverseIsometry3d(lfoot_roll_rot_) * rd_.link_[Left_Foot].rotm; // 기울기 반영
  // lfoot_float_current_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[Left_Foot].xpos);  // 지면에서 Ankle frame 위치
  
  // rfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * DyrosMath::inverseIsometry3d(rfoot_pitch_rot_) * DyrosMath::inverseIsometry3d(rfoot_roll_rot_) * rd_.link_[Right_Foot].rotm; // 기울기 반영
  // rfoot_float_current_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[Right_Foot].xpos); // 지면에서 Ankle frame

  lfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Left_Foot].rotm; // 기울기 반영안함
  lfoot_float_current_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[Left_Foot].xpos);  // 지면에서 Ankle frame 위치
  lfoot_float_current_dot = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[Left_Foot].v); 
  
  rfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Right_Foot].rotm; // 기울기 반영안함
  rfoot_float_current_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[Right_Foot].xpos); // 지면에서 Ankle frame
  rfoot_float_current_dot = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[Right_Foot].v); 
   
  com_float_current_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[COM_id].xpos); // 지면에서 CoM 위치   
  com_float_current_dot = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[COM_id].v); 
  
  if(walking_tick_mj == 0)
  { com_float_current_dot_LPF = com_float_current_dot; com_float_current_dot_prev = com_float_current_dot; }

  com_float_current_dot_prev = com_float_current_dot;
  com_float_current_dot_LPF = 1/(1+2*M_PI*6.0*del_t)*com_float_current_dot_LPF + (2*M_PI*6.0*del_t)/(1+2*M_PI*6.0*del_t)*com_float_current_dot; 

  if(foot_step_(current_step_num_, 6) == 0)
  { supportfoot_float_current_ = rfoot_float_current_; }
  else if(foot_step_(current_step_num_, 6) == 1)
  { supportfoot_float_current_ = lfoot_float_current_; }
  
  pelv_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_) * pelv_float_current_;   
  lfoot_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_) * lfoot_float_current_;
  rfoot_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_) * rfoot_float_current_;
  
  current_lfoot_control.translation() = lfoot_float_current_.translation() - supportfoot_float_current_.translation();
  current_lfoot_control.linear() = Matrix3d::Identity();
  current_rfoot_control.translation() = rfoot_float_current_.translation() - supportfoot_float_current_.translation();
  current_rfoot_control.linear() = Matrix3d::Identity();
  
  com_support_current_ =  DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_), com_float_current_);
  //com_support_current_LPF = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_), com_float_current_LPF); // deleted by MJ (1025)

  l_ft_ = rd_.LF_FT;
  r_ft_ = rd_.RF_FT;
  
  if(walking_tick_mj == 0)
  { l_ft_LPF = l_ft_; r_ft_LPF = r_ft_; }
  
  l_ft_LPF = 1/(1+2*M_PI*8.0*del_t)*l_ft_LPF + (2*M_PI*8.0*del_t)/(1+2*M_PI*8.0*del_t)*l_ft_;
  r_ft_LPF = 1/(1+2*M_PI*8.0*del_t)*r_ft_LPF + (2*M_PI*8.0*del_t)/(1+2*M_PI*8.0*del_t)*r_ft_;    

  Eigen::Vector2d left_zmp, right_zmp;

  left_zmp(0) = l_ft_(4)/l_ft_(2) + lfoot_support_current_.translation()(0);
  left_zmp(1) = l_ft_(3)/l_ft_(2) + lfoot_support_current_.translation()(1);

  right_zmp(0) = r_ft_(4)/r_ft_(2) + rfoot_support_current_.translation()(0);
  right_zmp(1) = r_ft_(3)/r_ft_(2) + rfoot_support_current_.translation()(1);

  zmp_measured_(0) = (left_zmp(0) * l_ft_(2) + right_zmp(0) * r_ft_(2))/(l_ft_(2) + r_ft_(2)); // ZMP X
  zmp_measured_(1) = (left_zmp(1) * l_ft_(2) + right_zmp(1) * r_ft_(2))/(l_ft_(2) + r_ft_(2)); // ZMP Y
   
  wn = sqrt(GRAVITY/zc_);
  
  if(walking_tick_mj == 0)
  { zmp_measured_LPF_.setZero(); }

  zmp_measured_LPF_ = (2*M_PI*8.0*del_t)/(1+2*M_PI*8.0*del_t)*zmp_measured_ + 1/(1+2*M_PI*8.0*del_t)*zmp_measured_LPF_;
   
}

void CustomController::floatToSupportFootstep()
{
    Eigen::Isometry3d reference;

    if(current_step_num_ == 0)
    {
        if(foot_step_(0,6) == 0)
        {
          reference.translation() = rfoot_float_init_.translation();
          reference.translation()(2) = 0.0;
          reference.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(rfoot_float_init_.linear())(2));
          reference.translation()(0) = 0.0;
        }
        else
        {
          reference.translation() = lfoot_float_init_.translation();
          reference.translation()(2) = 0.0;
          reference.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(lfoot_float_init_.linear())(2));
          reference.translation()(0) = 0.0;
        }
    }
    else
    {
        reference.linear() = DyrosMath::rotateWithZ(foot_step_(current_step_num_-1,5));
        for(int i=0 ;i<3; i++)
        { reference.translation()(i) = foot_step_(current_step_num_-1,i); }        
    }

    Eigen::Vector3d temp_local_position;
    Eigen::Vector3d temp_global_position;

    for(int i = 0; i < total_step_num_; i++)
    {
        for(int j = 0; j < 3; j ++)
        {temp_global_position(j) = foot_step_(i,j);}

        temp_local_position = reference.linear().transpose()*(temp_global_position - reference.translation());

        for(int j=0; j<3; j++)
        { foot_step_support_frame_(i,j) = temp_local_position(j); }

        foot_step_support_frame_(i,3) = foot_step_(i,3);
        foot_step_support_frame_(i,4) = foot_step_(i,4);
        if(current_step_num_ == 0)
        { foot_step_support_frame_(i,5) = foot_step_(i,5) - supportfoot_float_init_(5); }
        else
        { foot_step_support_frame_(i,5) = foot_step_(i,5) - foot_step_(current_step_num_-1,5); }
    }

    for(int j = 0; j < 3; j ++)
    temp_global_position(j) = swingfoot_float_init_(j); // swingfoot_float_init_은 Pelvis에서 본 Swing 발의 Position, orientation.
 
    temp_local_position = reference.linear().transpose()*(temp_global_position - reference.translation());
 
    for(int j=0;j<3;j++)
      swingfoot_support_init_(j) = temp_local_position(j);

    swingfoot_support_init_(3) = swingfoot_float_init_(3);
    swingfoot_support_init_(4) = swingfoot_float_init_(4);

    if(current_step_num_ == 0)
      swingfoot_support_init_(5) = swingfoot_float_init_(5) - supportfoot_float_init_(5);
    else
      swingfoot_support_init_(5) = swingfoot_float_init_(5) - foot_step_(current_step_num_-1,5);
    
    for(int j=0;j<3;j++)
        temp_global_position(j) = supportfoot_float_init_(j);

    temp_local_position = reference.linear().transpose()*(temp_global_position - reference.translation());

    for(int j=0;j<3;j++)
        supportfoot_support_init_(j) = temp_local_position(j);

    supportfoot_support_init_(3) = supportfoot_float_init_(3);
    supportfoot_support_init_(4) = supportfoot_float_init_(4);

    if(current_step_num_ == 0)
        supportfoot_support_init_(5) = 0;
    else
        supportfoot_support_init_(5) = supportfoot_float_init_(5) - foot_step_(current_step_num_-1,5);
}

void CustomController::getZmpTrajectory()
{
  unsigned int planning_step_number = 3;
  unsigned int norm_size = 0;
 
  if(current_step_num_ >= total_step_num_ - planning_step_number)
    norm_size = (t_last_ - t_start_ + 1)*(total_step_num_ - current_step_num_) + 3.0*hz_;
  else
    norm_size = (t_last_ - t_start_ + 1)*(planning_step_number);
  if(current_step_num_ == 0)
    norm_size = norm_size + t_temp_ + 1;
  addZmpOffset();  
  zmpGenerator(norm_size, planning_step_number);
}

void CustomController::addZmpOffset()
{
  double lfoot_zmp_offset_, rfoot_zmp_offset_;
 
  lfoot_zmp_offset_ = -0.01;
  rfoot_zmp_offset_ = 0.01;

  foot_step_support_frame_offset_ = foot_step_support_frame_;

  supportfoot_support_init_offset_ = supportfoot_support_init_;
 

  if(foot_step_(0,6) == 0) //right support foot
  {
    supportfoot_support_init_offset_(1) = supportfoot_support_init_(1) + rfoot_zmp_offset_;
    //swingfoot_support_init_offset_(1) = swingfoot_support_init_(1) + lfoot_zmp_offset_;
  }
  else
  {
    supportfoot_support_init_offset_(1) = supportfoot_support_init_(1) + lfoot_zmp_offset_;
    //swingfoot_support_init_offset_(1) = swingfoot_support_init_(1) + rfoot_zmp_offset_;
  }

  for(int i=0; i<total_step_num_; i++)
  {
    if(foot_step_(i,6) == 0)//right support, left swing
    {
      foot_step_support_frame_offset_(i,1) += lfoot_zmp_offset_;
    }
    else
    {
      foot_step_support_frame_offset_(i,1) += rfoot_zmp_offset_;
    }
  }
}

void CustomController::zmpGenerator(const unsigned int norm_size, const unsigned planning_step_num)
{
  ref_zmp_.resize(norm_size, 2);
  Eigen::VectorXd temp_px;
  Eigen::VectorXd temp_py;
 
  unsigned int index = 0;
  // 매 tick 마다 zmp가 3발 앞까지 계산 된다.

  if(current_step_num_ == 0) // Walking을 수행 할 때, 정지 상태 일때 3초 동안 Ref X ZMP를 0으로 보냄. Y ZMP는 제자리 유지.  
  {
    for (int i = 0; i <= t_temp_; i++) //600 tick
    {
      if(i < 1.0*hz_)
      {
        ref_zmp_(i,0) = com_support_init_(0) ;
        ref_zmp_(i,1) = com_support_init_(1) ;
      }
      else if(i < 2.0*hz_)
      {
        double del_x = i - 1.0*hz_;
        ref_zmp_(i,0) = com_support_init_(0) - del_x * com_support_init_(0)/(1.0*hz_);
        ref_zmp_(i,1) = com_support_init_(1) ;
      }
      else
      {
        ref_zmp_(i,0) = 0.0;
        ref_zmp_(i,1) = com_support_init_(1) ;
      }      
      index++;
    }    
  }
  /////////////////////////////////////////////////////////////////////
  if(current_step_num_ >= total_step_num_ - planning_step_num)
  {  
    for(unsigned int i = current_step_num_; i < total_step_num_; i++)
    {
      onestepZmp(i, temp_px, temp_py);
     
      for(unsigned int j = 0; j < t_total_; j++)
      {
        ref_zmp_(index + j, 0) = temp_px(j);
        ref_zmp_(index + j, 1) = temp_py(j);    
      }
      index = index + t_total_;
    }
    
    for(unsigned int j = 0; j < 3.0*hz_; j++)
    {
      ref_zmp_(index + j, 0) = ref_zmp_(index -1, 0);
      ref_zmp_(index + j, 1) = ref_zmp_(index -1, 1);
    }
    index = index + 3.0*hz_;      
  }
  else // 보행 중 사용 하는 Ref ZMP
  {
    for(unsigned int i = current_step_num_; i < current_step_num_ + planning_step_num; i++)  
    {
      onestepZmp(i, temp_px, temp_py);
      for (unsigned int j = 0; j < t_total_; j++) // 1 step 보행은 1.2초, 240 tick
      {
        ref_zmp_(index+j,0) = temp_px(j);
        ref_zmp_(index+j,1) = temp_py(j);
      }      
      index = index + t_total_; // 참조 zmp가 이만큼 쌓였다.      
      // 결국 실제 로봇 1Hz마다 720개의 ref_zmp를 생성함. 3.6초
    }   
  }   
}

void CustomController::onestepZmp(unsigned int current_step_number, Eigen::VectorXd& temp_px, Eigen::VectorXd& temp_py)
{
  temp_px.resize(t_total_); // 함수가 실행 될 때 마다, 240 tick의 참조 ZMP를 담는다. Realtime = 1.2초
  temp_py.resize(t_total_);
  temp_px.setZero();
  temp_py.setZero();

  double Kx = 0, Ky = 0, A = 0, B = 0, wn = 0, Kx2 = 0, Ky2 = 0;
  if(current_step_number == 0)
  {
    Kx = 0;
    Ky = supportfoot_support_init_offset_(1) - com_support_init_(1);
    Kx2 = foot_step_support_frame_offset_(current_step_number,0) / 2 - supportfoot_support_init_offset_(0);
    Ky2 = foot_step_support_frame_offset_(current_step_number,1) / 2 - supportfoot_support_init_offset_(1) ;
    
    for(int i = 0; i < t_total_; i++)
    {
      if(i >= 0 && i < t_rest_init_ + t_double1_) //0.05 ~ 0.15초 , 10 ~ 30 tick
      {
        temp_px(i) = Kx / (t_double1_ + t_rest_init_) * (i+1);
        temp_py(i) = com_support_init_(1) + Ky / (t_double1_ + t_rest_init_) * (i+1);
      }
      else if(i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_ ) //0.15 ~ 1.05초 , 30 ~ 210 tick
      {
        temp_px(i) = 0;
        temp_py(i) = supportfoot_support_init_offset_(1);
      }
      else if(i >= t_total_ - t_rest_last_ - t_double2_  && i < t_total_ ) //1.05 ~ 1.15초 , 210 ~ 230 tick
      {
        temp_px(i) = 0 + Kx2 / (t_rest_last_ + t_double2_) * (i+1 - (t_total_ - t_rest_last_ - t_double2_));
        temp_py(i) = supportfoot_support_init_offset_(1) + Ky2 / (t_rest_last_ + t_double2_) * (i+1 - (t_total_ - t_rest_last_ - t_double2_));
      }
    }  
  }
  else if(current_step_number == 1)
  {
    Kx = foot_step_support_frame_offset_(current_step_number-1, 0) - (foot_step_support_frame_offset_(current_step_number-1, 0) + supportfoot_support_init_(0))/2;
    Ky = foot_step_support_frame_offset_(current_step_number-1, 1) - (foot_step_support_frame_offset_(current_step_number-1, 1) + supportfoot_support_init_(1))/2;
    Kx2 = (foot_step_support_frame_offset_(current_step_number, 0) + foot_step_support_frame_offset_(current_step_number-1, 0))/2 - foot_step_support_frame_offset_(current_step_number-1, 0);
    Ky2 = (foot_step_support_frame_offset_(current_step_number, 1) + foot_step_support_frame_offset_(current_step_number-1, 1))/2 - foot_step_support_frame_offset_(current_step_number-1, 1);

    for(int i = 0; i < t_total_; i++)
    {
      if(i < t_rest_init_ + t_double1_) //0.05 ~ 0.15초 , 10 ~ 30 tick
      {
        temp_px(i) = (foot_step_support_frame_offset_(current_step_number-1, 0) + supportfoot_support_init_(0))/2 + Kx / (t_rest_init_ + t_double1_) * (i+1);
        temp_py(i) = (foot_step_support_frame_offset_(current_step_number-1, 1) + supportfoot_support_init_(1))/2 + Ky / (t_rest_init_ + t_double1_) * (i+1);
      }
      else if(i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_) //0.15 ~ 1.05초 , 30 ~ 210 tick
      {
        temp_px(i) = foot_step_support_frame_offset_(current_step_number-1, 0);
        temp_py(i) = foot_step_support_frame_offset_(current_step_number-1, 1);
      }
      else if(i >= t_total_ - t_rest_last_ - t_double2_ && i < t_total_ ) //1.05 ~ 1.2초 , 210 ~ 240 tick
      {
        temp_px(i) = foot_step_support_frame_offset_(current_step_number-1, 0) + Kx2/(t_double2_ + t_rest_last_)*(i+1 - (t_total_ - t_rest_last_ - t_double2_));
        temp_py(i) = foot_step_support_frame_offset_(current_step_number-1, 1) + Ky2/(t_double2_ + t_rest_last_)*(i+1 - (t_total_ - t_rest_last_ - t_double2_));
      }
    }
  }
  else
  {
    Kx = foot_step_support_frame_offset_(current_step_number-1, 0) - ((foot_step_support_frame_offset_(current_step_number-2, 0) + foot_step_support_frame_offset_(current_step_number-1, 0))/2);
    Ky = foot_step_support_frame_offset_(current_step_number-1, 1) - ((foot_step_support_frame_offset_(current_step_number-2, 1) + foot_step_support_frame_offset_(current_step_number-1, 1))/2);
    Kx2 = (foot_step_support_frame_offset_(current_step_number, 0) + foot_step_support_frame_offset_(current_step_number-1, 0))/2 - foot_step_support_frame_offset_(current_step_number-1, 0);
    Ky2 = (foot_step_support_frame_offset_(current_step_number, 1) + foot_step_support_frame_offset_(current_step_number-1, 1))/2 - foot_step_support_frame_offset_(current_step_number-1, 1);

    for(int i = 0; i < t_total_; i++)
    {
      if(i < t_rest_init_ + t_double1_) //0 ~ 0.15초 , 0 ~ 30 tick
      {
        temp_px(i) = (foot_step_support_frame_offset_(current_step_number-2, 0) + foot_step_support_frame_offset_(current_step_number-1, 0))/2 + Kx/(t_rest_init_+t_double1_)*(i+1);
        temp_py(i) = (foot_step_support_frame_offset_(current_step_number-2, 1) + foot_step_support_frame_offset_(current_step_number-1, 1))/2 + Ky/(t_rest_init_+t_double1_)*(i+1);
      }
      else if(i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_) //0.15 ~ 1.05초 , 30 ~ 210 tick
      {
        temp_px(i) = foot_step_support_frame_offset_(current_step_number-1, 0);
        temp_py(i) = foot_step_support_frame_offset_(current_step_number-1, 1);
      }
      else if(i >= t_total_ - t_rest_last_ - t_double2_ && i < t_total_ ) //1.05 ~ 1.2초 , 210 ~ 240 tick
      {
        temp_px(i) = foot_step_support_frame_offset_(current_step_number-1, 0) + Kx2/(t_double2_ + t_rest_last_)*(i+1 - (t_total_ - t_rest_last_ - t_double2_));
        temp_py(i) = foot_step_support_frame_offset_(current_step_number-1, 1) + Ky2/(t_double2_ + t_rest_last_)*(i+1 - (t_total_ - t_rest_last_ - t_double2_));
      }      
    }
  }
}

void CustomController::getComTrajectory()
{
  if(walking_tick_mj == 0)  
  {
    Gi_.setZero(); Gx_.setZero(); Gd_.setZero();
    preview_Parameter(1.0/hz_, 16*hz_/10, Gi_, Gd_, Gx_, A_, B_, C_);
    xs_(0) = xi_; xs_(1) = 0; xs_(2) = 0;
    ys_(0) = yi_; ys_(1) = 0; xs_(2) = 0;
    UX_ = 0; UY_ = 0;
    xd_ = xs_;
  }

  if(current_step_num_ == 0)
  { zmp_start_time_ = 0.0; }
  else
  { zmp_start_time_ = t_start_; }
       
  previewcontroller(0.0005, 3200, walking_tick_mj - zmp_start_time_, xi_, yi_, xs_, ys_, UX_, UY_, Gi_, Gd_, Gx_, A_, B_, C_, xd_, yd_);
   
  xs_ = xd_; ys_ = yd_;

  com_desired_(0) = xd_(0);
  com_desired_(1) = yd_(0);
  com_desired_(2) = pelv_support_start_.translation()(2);

  //SC_err_compen(com_desired_(0), com_desired_(1)); 

  if (walking_tick_mj == t_start_ + t_total_-1 && current_step_num_ != total_step_num_-1)  
  {
    Eigen::Vector3d com_pos_prev;
    Eigen::Vector3d com_pos;
    Eigen::Vector3d com_vel_prev;
    Eigen::Vector3d com_vel;
    Eigen::Vector3d com_acc_prev;
    Eigen::Vector3d com_acc;
    Eigen::Matrix3d temp_rot;
    Eigen::Vector3d temp_pos;
    
    temp_rot = DyrosMath::rotateWithZ(-foot_step_support_frame_(current_step_num_,5));
    for(int i=0; i<3; i++)
      temp_pos(i) = foot_step_support_frame_(current_step_num_,i);     
    
    com_pos_prev(0) = xs_(0);
    com_pos_prev(1) = ys_(0);
    com_pos = temp_rot*(com_pos_prev - temp_pos);
     
    com_vel_prev(0) = xs_(1);
    com_vel_prev(1) = ys_(1);
    com_vel_prev(2) = 0.0;
    com_vel = temp_rot*com_vel_prev;

    com_acc_prev(0) = xs_(2);
    com_acc_prev(1) = ys_(2);
    com_acc_prev(2) = 0.0;
    com_acc = temp_rot*com_acc_prev;

    xs_(0) = com_pos(0);
    ys_(0) = com_pos(1);
    xs_(1) = com_vel(0);
    ys_(1) = com_vel(1);
    xs_(2) = com_acc(0);
    ys_(2) = com_acc(1);
  }
 
}

void CustomController::preview_Parameter(double dt, int NL, Eigen::MatrixXd& Gi, Eigen::VectorXd& Gd, Eigen::MatrixXd& Gx, Eigen::MatrixXd& A, Eigen::VectorXd& B, Eigen::MatrixXd& C)
{
    A.resize(3,3);
    A(0,0) = 1.0;
    A(0,1) = dt;
    A(0,2) = dt*dt*0.5;
    A(1,0) = 0;
    A(1,1) = 1.0;
    A(1,2) = dt;
    A(2,0) = 0;
    A(2,1) = 0;
    A(2,2) = 1;
    
    B.resize(3);
    B(0) = dt*dt*dt/6;
    B(1) = dt*dt/2;
    B(2) = dt;
    
    C.resize(1,3);
    C(0,0) = 1;
    C(0,1) = 0;
    C(0,2) = -0.71/9.81;

    Eigen::MatrixXd A_bar;
    Eigen::VectorXd B_bar;

    B_bar.resize(4);    
    B_bar.segment(0,1) = C*B;
    B_bar.segment(1,3) = B;
    
    Eigen::Matrix1x4d B_bar_tran;
    B_bar_tran = B_bar.transpose();
    
    Eigen::MatrixXd I_bar;
    Eigen::MatrixXd F_bar;
    A_bar.resize(4,4);
    I_bar.resize(4,1);
    F_bar.resize(4,3);
    F_bar.setZero();

    F_bar.block<1,3>(0,0) = C*A;
    F_bar.block<3,3>(1,0) = A;
    
    I_bar.setZero();
    I_bar(0,0) = 1.0;

    A_bar.block<4,1>(0,0) = I_bar;
    A_bar.block<4,3>(0,1) = F_bar;
    
    Eigen::MatrixXd Qe;
    Qe.resize(1,1);
    Qe(0,0) = 1.0;

    Eigen::MatrixXd R;
    R.resize(1,1);
    R(0,0) = 0.000001;

    Eigen::MatrixXd Qx;
    Qx.resize(3,3);
    Qx.setZero();

    Eigen::MatrixXd Q_bar;
    Q_bar.resize(3,3);
    Q_bar.setZero();
    Q_bar(0,0) = Qe(0,0);

    Eigen::Matrix4d K;
    
    K(0,0) = 1083.572780788710;
    K(0,1) = 586523.188429418020;  
    K(0,2) = 157943.283121116518;
    K(0,3) = 41.206077691894;
    K(1,0) = 586523.188429418020;
    K(1,1) = 319653984.254277825356;
    K(1,2) = 86082274.531361579895;
    K(1,3) = 23397.754069026785;
    K(2,0) = 157943.283121116518;
    K(2,1) = 86082274.531361579895;
    K(2,2) = 23181823.112113621086;
    K(2,3) = 6304.466397614751;
    K(3,0) = 41.206077691894;
    K(3,1) = 23397.754069026785;
    K(3,2) = 6304.466397614751;
    K(3,3) = 2.659250532188;
    
    Eigen::MatrixXd Temp_mat;
    Eigen::MatrixXd Temp_mat_inv;
    Eigen::MatrixXd Ac_bar;
    Temp_mat.resize(1,1);
    Temp_mat.setZero();
    Temp_mat_inv.resize(1,1);
    Temp_mat_inv.setZero();
    Ac_bar.setZero();
    Ac_bar.resize(4,4);

    Temp_mat = R + B_bar_tran * K * B_bar;
    Temp_mat_inv = Temp_mat.inverse();
    
    Ac_bar = A_bar - B_bar * Temp_mat_inv * B_bar_tran * K * A_bar;
    
    Eigen::MatrixXd Ac_bar_tran(4,4);
    Ac_bar_tran = Ac_bar.transpose();
    
    Gi.resize(1,1); Gx.resize(1,3);
    Gi(0,0) = 872.3477 ; //Temp_mat_inv * B_bar_tran * K * I_bar ;
    //Gx = Temp_mat_inv * B_bar_tran * K * F_bar ;  
    Gx(0,0) = 945252.1760702;
    Gx(0,1) = 256298.6905049;
    Gx(0,2) = 542.0544196;
    Eigen::MatrixXd X_bar;
    Eigen::Vector4d X_bar_col;
    X_bar.resize(4, NL);
    X_bar.setZero();
    X_bar_col.setZero();
    X_bar_col = - Ac_bar_tran * K * I_bar;

    for(int i = 0; i < NL; i++)
    {
        X_bar.block<4,1>(0,i) = X_bar_col;
        X_bar_col = Ac_bar_tran*X_bar_col;
    }           

    Gd.resize(NL);
    Eigen::VectorXd Gd_col(1);
    Gd_col(0) = -Gi(0,0);
    
    for(int i = 0; i < NL; i++)
    {
        Gd.segment(i,1) = Gd_col;
        Gd_col = Temp_mat_inv * B_bar_tran * X_bar.col(i) ;
    }
    
}

void CustomController::previewcontroller(double dt, int NL, int tick, double x_i, double y_i, Eigen::Vector3d xs, Eigen::Vector3d ys, double& UX, double& UY,
       Eigen::MatrixXd Gi, Eigen::VectorXd Gd, Eigen::MatrixXd Gx, Eigen::MatrixXd A, Eigen::VectorXd B, Eigen::MatrixXd C, Eigen::Vector3d &XD, Eigen::Vector3d &YD)
{
    
    int zmp_size;
    zmp_size = ref_zmp_.col(1).size();
    Eigen::VectorXd px_ref, py_ref;
    px_ref.resize(zmp_size);
    py_ref.resize(zmp_size);
    
    for(int i = 0; i < zmp_size; i++)
    {
        px_ref(i) = ref_zmp_(i,0);
        py_ref(i) = ref_zmp_(i,1);
    }

    ZMP_X_REF = px_ref(tick);
    ZMP_Y_REF = py_ref(tick);
        
    Eigen::VectorXd px, py;
    px.resize(1); py.resize(1);
    
    if(tick == 0 && current_step_num_ == 0)
    {
        preview_x_b.setZero(); preview_y_b.setZero();
        preview_x.setZero(); preview_y.setZero();
        preview_x_b(0) = x_i;  
        preview_y_b(0) = y_i;   
        preview_x(0) = x_i;
        preview_y(0) = y_i;
        UX = 0; UY = 0;
        cout << "preview X state : " << preview_x(0) << "," << preview_x(1) << "," << preview_x(2) << endl;
        cout << "preview Y state : " << preview_y(0) << "," << preview_y(1) << "," << preview_y(2) << endl;
    }
    else
    {     
        preview_x = xs; preview_y = ys;
            
        preview_x_b(0) = preview_x(0) - preview_x(1)*0.0005;  
        preview_y_b(0) = preview_y(0) - preview_y(1)*0.0005;
        preview_x_b(1) = preview_x(1) - preview_x(2)*0.0005;
        preview_y_b(1) = preview_y(1) - preview_y(2)*0.0005;
        preview_x_b(2) = preview_x(2) - UX*0.0005;
        preview_y_b(2) = preview_y(2) - UY*0.0005;
        
    }      
    px = C*preview_x;
    py = C*preview_y;
    
    double sum_Gd_px_ref = 0, sum_Gd_py_ref = 0;

    for(int i = 0; i < NL; i++)
    {
        sum_Gd_px_ref = sum_Gd_px_ref + Gd(i)*(px_ref(tick + 1 + i) - px_ref(tick + i));
        sum_Gd_py_ref = sum_Gd_py_ref + Gd(i)*(py_ref(tick + 1 + i) - py_ref(tick + i));
    }
    
    Eigen::MatrixXd del_ux(1,1);
    Eigen::MatrixXd del_uy(1,1);
    del_ux.setZero();
    del_uy.setZero();
    
    Eigen::VectorXd GX_X(1);
    GX_X = Gx * (preview_x - preview_x_b);
    Eigen::VectorXd GX_Y(1);
    GX_Y = Gx * (preview_y - preview_y_b);

    if(walking_tick_mj == 0)
    {
      del_zmp.setZero(); cout << "del_zmp : " << del_zmp(0) << "," << del_zmp(1) << endl;
    }
    
    del_ux(0,0) = -(px(0) - px_ref(tick))*Gi(0,0) - GX_X(0) - sum_Gd_px_ref;
    del_uy(0,0) = -(py(0) - py_ref(tick))*Gi(0,0) - GX_Y(0) - sum_Gd_py_ref;
    
    UX = UX + del_ux(0,0);
    UY = UY + del_uy(0,0);

    XD = A*preview_x + B*UX;
    YD = A*preview_y + B*UY;
    //SC_err_compen(XD(0), YD(0));
    if(walking_tick_mj == 0)
    {      
      zmp_err_(0) = 0;
      zmp_err_(1) = 0;
    }
    else
    {
      zmp_err_(0) = zmp_err_(0) + (px_ref(tick) - zmp_measured_LPF_(0))*0.0005;
      zmp_err_(1) = zmp_err_(1) + (py_ref(tick) - zmp_measured_LPF_(1))*0.0005;
    }    

    cp_desired_(0) = XD(0) + XD(1)/wn;
    cp_desired_(1) = YD(0) + YD(1)/wn;

    SC_err_compen(com_support_current_(0), com_support_current_(1));

    cp_measured_(0) = com_support_cp_(0) + com_float_current_dot_LPF(0)/wn;
    cp_measured_(1) = com_support_current_(1) + com_float_current_dot_LPF(1)/wn;      

    del_zmp(0) = 1.2*(cp_measured_(0) - cp_desired_(0));
    del_zmp(1) = 1.3*(cp_measured_(1) - cp_desired_(1));

    CLIPM_ZMP_compen_MJ(del_zmp(0), del_zmp(1));

}

void CustomController::SC_err_compen(double x_des, double y_des)
{ 
  if(walking_tick_mj == 0)
  {
    SC_com.setZero();
  }
  if (walking_tick_mj == t_start_ + t_total_-1 && current_step_num_ != total_step_num_-1) // step change 1 tick 이전
  {  
    sc_err_before.setZero();
    sc_err_before(0) = com_support_current_(0) - foot_step_support_frame_(current_step_num_,0) ; // 1.3으로 할꺼면 마지막에 더해야됨. SC_com을 이 함수보다 나중에 더하기 때문에
    // sc_err_before(1) = y_des - com_support_current_(1);
  }

  if(current_step_num_ != 0 && walking_tick_mj == t_start_) // step change 
  { 
    sc_err_after.setZero();
    sc_err_after(0) = com_support_current_(0) ; 
    // sc_err_after(1) = y_des - com_support_current_(1);
    sc_err = sc_err_after - sc_err_before;
  } 

  if(current_step_num_ != 0)
  { 
    SC_com(0) = DyrosMath::cubic(walking_tick_mj, t_start_, t_start_ + 0.05*hz_, sc_err(0), 0, 0.0, 0.0);
    SC_com(1) = DyrosMath::cubic(walking_tick_mj, t_start_, t_start_ + 0.05*hz_, sc_err(1), 0, 0.0, 0.0);
  }

  if(current_step_num_ != total_step_num_ - 1)
  {
    if(current_step_num_ != 0 && walking_tick_mj >= t_start_ && walking_tick_mj < t_start_ + t_total_)
    {
      com_support_cp_(0) = com_support_current_(0) - SC_com(0);
    }
    else
    {
      com_support_cp_(0) = com_support_current_(0); 
    }
  }  
  else if(current_step_num_ == total_step_num_ - 1)
  {
    if(walking_tick_mj >= t_start_ && walking_tick_mj < t_start_ + 2*t_total_)
    { com_support_cp_(0) = com_support_current_(0) - SC_com(0); }
  }
}

void CustomController::CLIPM_ZMP_compen_MJ(double XZMP_ref, double YZMP_ref)
{ 
  double Kp_x_ssp, Kv_x_ssp;
  double Kp_y_ssp, Kv_y_ssp;
  Kp_x_ssp = 30; Kv_x_ssp = 2;
  Kp_y_ssp = 40; Kv_y_ssp = 2; 
  double del_t = 0.0005;

  if(walking_tick_mj == 0)
  {
    A_x_ssp.resize(2,2);
    B_x_ssp.resize(2,1);
    Ad_x_ssp.resize(2,2);
    Bd_x_ssp.resize(2,1);
    C_x_ssp.resize(1,2);
    D_x_ssp.resize(1,1); 
    
    A_y_ssp.resize(2,2);
    B_y_ssp.resize(2,1);
    Ad_y_ssp.resize(2,2);
    Bd_y_ssp.resize(2,1);
    C_y_ssp.resize(1,2);
    D_y_ssp.resize(1,1);

    ff_gain_x_ssp.resize(1,1);
    ff_gain_y_ssp.resize(1,1); 

    K_x_ssp.resize(1,2);
    K_y_ssp.resize(1,2); 

    X_x_ssp.setZero(); 
    Y_x_ssp.resize(1,1);

    X_y_ssp.setZero(); 
    Y_y_ssp.resize(1,1); 

    K_x_ssp(0,0) = 0.0083;      
    K_x_ssp(0,1) = 0.19; 
    // Control pole : -5 , damping : 0.7 (실제 로봇) // Control pole : -7 , damping : 0.9 (시뮬레이션)
    K_y_ssp(0,0) = -0.375;  
    K_y_ssp(0,1) = 0.125; 
    
    // Define the state space equation 
    A_x_ssp(0,0) = 0;
    A_x_ssp(0,1) = 1;
    A_x_ssp(1,0) = - Kp_x_ssp;
    A_x_ssp(1,1) = - Kv_x_ssp;

    B_x_ssp(0,0) = 0;
    B_x_ssp(1,0) = Kp_x_ssp;

    Ad_x_ssp(0,0) = 1 - 0.5 * Kp_x_ssp * del_t * del_t;
    Ad_x_ssp(0,1) = del_t - 0.5 * Kv_x_ssp * del_t * del_t;
    Ad_x_ssp(1,0) = - Kp_x_ssp * del_t;
    Ad_x_ssp(1,1) = 1 - Kv_x_ssp * del_t;

    Bd_x_ssp(0,0) = 0.5*Kp_x_ssp*del_t*del_t;
    Bd_x_ssp(1,0) = Kp_x_ssp*del_t;

    C_x_ssp(0,0) = 1 + zc_/GRAVITY * Kp_x_ssp;
    C_x_ssp(0,1) = zc_/GRAVITY * Kv_x_ssp;

    D_x_ssp(0,0) = -zc_/GRAVITY * Kp_x_ssp;
 
    ff_gain_x_ssp = (-(C_x_ssp - D_x_ssp*K_x_ssp)*((A_x_ssp - B_x_ssp*K_x_ssp).inverse())*B_x_ssp + D_x_ssp).inverse(); 

    A_y_ssp(0,0) = 0;
    A_y_ssp(0,1) = 1;
    A_y_ssp(1,0) = - Kp_y_ssp;
    A_y_ssp(1,1) = - Kv_y_ssp;

    B_y_ssp(0,0) = 0;
    B_y_ssp(1,0) = Kp_y_ssp;

    Ad_y_ssp(0,0) = 1 - 0.5 * Kp_y_ssp * del_t * del_t;
    Ad_y_ssp(0,1) = del_t - 0.5 * Kv_y_ssp * del_t * del_t;
    Ad_y_ssp(1,0) = - Kp_y_ssp * del_t;
    Ad_y_ssp(1,1) = 1 - Kv_y_ssp * del_t;

    Bd_y_ssp(0,0) = 0.5*Kp_y_ssp*del_t*del_t;
    Bd_y_ssp(1,0) = Kp_y_ssp*del_t;

    C_y_ssp(0,0) = 1 + zc_/GRAVITY * Kp_y_ssp;
    C_y_ssp(0,1) = zc_/GRAVITY * Kv_y_ssp;

    D_y_ssp(0,0) = -zc_/GRAVITY * Kp_y_ssp;
 
    ff_gain_y_ssp = (-(C_y_ssp - D_y_ssp*K_y_ssp)*((A_y_ssp - B_y_ssp*K_y_ssp).inverse())*B_y_ssp + D_y_ssp).inverse();     
  }  

  //X_x_ssp(0) = com_float_current_(0);
  X_x_ssp(0) = com_support_current_(0); 

  if(foot_step_(current_step_num_, 6) == 1) // 왼발 지지
  { X_y_ssp(0) = com_support_current_(1) - rfoot_support_current_.translation()(1)*0.5; } 
  else if(foot_step_(current_step_num_, 6) == 0)
  { X_y_ssp(0) = com_support_current_(1) - lfoot_support_current_.translation()(1)*0.5; } 
  
  U_ZMP_x_ssp = - (K_x_ssp(0,0)*X_x_ssp(0) + K_x_ssp(0,1)*preview_x(1)) + XZMP_ref * ff_gain_x_ssp(0,0);
  U_ZMP_y_ssp = - (K_y_ssp(0,0)*X_y_ssp(0) + K_y_ssp(0,1)*preview_y(1)) + YZMP_ref * ff_gain_y_ssp(0,0); 
  
  U_ZMP_x_ssp_LPF =  1/(1+2*M_PI*3.0*del_t)*U_ZMP_x_ssp_LPF + (2*M_PI*3.0*del_t)/(1+2*M_PI*3.0*del_t)*U_ZMP_x_ssp;
  U_ZMP_y_ssp_LPF =  1/(1+2*M_PI*6.0*del_t)*U_ZMP_y_ssp_LPF + (2*M_PI*6.0*del_t)/(1+2*M_PI*6.0*del_t)*U_ZMP_y_ssp;       
  if(walking_tick_mj == 0)
  { U_ZMP_x_ssp_LPF = U_ZMP_x_ssp; U_ZMP_y_ssp_LPF = U_ZMP_y_ssp; }

  damping_x = U_ZMP_x_ssp_LPF ;
  damping_y = U_ZMP_y_ssp_LPF ;

  if(damping_x > 0.02)  
  { damping_x = 0.02; }
  else if(damping_x < - 0.02)
  { damping_x = -0.02; } 

  if(damping_y > 0.03) // 로봇 0.03, 시뮬 0.02
  { damping_y = 0.03; }
  else if(damping_y < - 0.03)
  { damping_y = -0.03; }
  
  // MJ_graph << cp_desired_(1) << "," << cp_measured_(1) << "," << com_float_current_(1) << "," << X_y_ssp(0) << "," << damping_y << endl;  
}

void CustomController::getFootTrajectory()
{
  Eigen::Vector6d target_swing_foot;

  for(int i=0; i<6; i++)
  { target_swing_foot(i) = foot_step_support_frame_(current_step_num_,i); }
 
  if(walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
  {  
    if(foot_step_(current_step_num_,6) == 1) // 왼발 지지
    {
      lfoot_trajectory_support_.translation().setZero();
      lfoot_trajectory_euler_support_.setZero();

      rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
      rfoot_trajectory_support_.translation()(2) = 0;
      rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;
    }     
    else if(foot_step_(current_step_num_,6) == 0) // 오른발 지지
    {
      rfoot_trajectory_support_.translation().setZero();
      rfoot_trajectory_euler_support_.setZero();

      lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
      lfoot_trajectory_support_.translation()(2) = 0;
      lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;
    } 

    lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(F_T_L_y_input)*DyrosMath::rotateWithX(-F_T_L_x_input);
    rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(F_T_R_y_input)*DyrosMath::rotateWithX(-F_T_R_x_input);
  }
  
  else if(walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)  
  {
    double t_rest_temp = 0.00*hz_;
    if(walking_tick_mj == t_start_ + t_rest_init_ + t_double1_)
    {
      // rfoot_support_traj_init_.translation() = rfoot_support_current_.translation();
      // lfoot_support_traj_init_.translation() = lfoot_support_current_.translation();
      rfoot_support_traj_init_.translation() = current_rfoot_control.translation();
      lfoot_support_traj_init_.translation() = current_lfoot_control.translation();
    }
    if(foot_step_(current_step_num_,6) == 1)//왼발 지지
    {
      // lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();             
      lfoot_trajectory_support_.translation() = lfoot_support_traj_init_.translation();             
      lfoot_trajectory_euler_support_.setZero();
       
      lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(F_T_L_y_input)*DyrosMath::rotateWithX(-F_T_L_x_input);
      
      if(walking_tick_mj < t_start_ + t_rest_init_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_)/2.0)
      // { rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj,t_start_+ t_rest_init_ + t_double1_ + t_rest_temp, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_)/2.0,0,foot_height_,0.0,0.0); }  
      // { rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj,t_start_+ t_rest_init_ + t_double1_ + t_rest_temp, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_)/2.0,rfoot_support_init_.translation()(2),foot_height_,0.0,0.0); }  
      { 
        rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj,t_start_+ t_rest_init_ + t_double1_ + t_rest_temp, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_)/2.0,rfoot_support_traj_init_.translation()(2),rfoot_support_traj_init_.translation()(2)+foot_height_,0.0,0.0); 
        rd_.link_[Right_Foot].v_traj(2) = DyrosMath::cubicDot(walking_tick_mj,t_start_+ t_rest_init_ + t_double1_ + t_rest_temp, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_)/2.0,rfoot_support_traj_init_.translation()(2),rfoot_support_traj_init_.translation()(2)+foot_height_,0.0,0.0,2000.0);
      }  
      else
      // { rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_,foot_height_,target_swing_foot(2) - 0.005,0.0,0.0); }
      //{ rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_,foot_height_,rfoot_support_init_.translation()(2),0.0,0.0); }
      { 
        rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_,rfoot_support_traj_init_.translation()(2)+foot_height_,0.0,0.0,0.0); 
        rd_.link_[Right_Foot].v_traj(2) = DyrosMath::cubicDot(walking_tick_mj,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_,rfoot_support_traj_init_.translation()(2)+foot_height_,0.0,0.0,0.0,2000.0); 
      }
      
      for(int i=0; i<2; i++)  
      // { rfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_mj,t_start_real_ + t_double1_ + t_rest_temp, t_start_+t_total_-t_rest_last_-t_double2_, rfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0); }
      { rfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_mj,t_start_real_ + t_double1_ + t_rest_temp, t_start_+t_total_-t_rest_last_-t_double2_, rfoot_support_traj_init_.translation()(i),target_swing_foot(i),0.0,0.0); }
      
      rfoot_trajectory_euler_support_(1) = 0;
      rfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_mj,t_start_ + t_rest_init_ + t_double1_,t_start_ + t_total_ - t_rest_last_ - t_double2_,rfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0);
      rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(F_T_R_y_input)*DyrosMath::rotateWithX(-F_T_R_x_input);
    }
    else if(foot_step_(current_step_num_,6) == 0)// 오른발 지지
    {
      // rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
      rfoot_trajectory_support_.translation() = rfoot_support_traj_init_.translation();
      rfoot_trajectory_euler_support_.setZero();
      
      rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(F_T_R_y_input)*DyrosMath::rotateWithX(-F_T_R_x_input);
 
      if(walking_tick_mj < t_start_ + t_rest_init_ + t_double1_ + (t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0)
      // { lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj,t_start_real_ + t_double1_ + t_rest_temp, t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,0,foot_height_,0.0,0.0); }
      // { lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj,t_start_real_ + t_double1_ + t_rest_temp, t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,lfoot_support_init_.translation()(2),foot_height_,0.0,0.0); }
      { 
        lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj,t_start_real_ + t_double1_ + t_rest_temp, t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,lfoot_support_traj_init_.translation()(2),lfoot_support_traj_init_.translation()(2)+foot_height_,0.0,0.0); 
        rd_.link_[Left_Foot].v_traj(2) = DyrosMath::cubicDot(walking_tick_mj,t_start_+ t_rest_init_ + t_double1_ + t_rest_temp, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_)/2.0,lfoot_support_traj_init_.translation()(2),lfoot_support_traj_init_.translation()(2)+foot_height_,0.0,0.0,2000.0);
      }
      else
      // { lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj,t_start_real_ + t_double1_ + (t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_,foot_height_,target_swing_foot(2) - 0.005,0.0,0.0); }
      // { lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj,t_start_real_ + t_double1_ + (t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_,foot_height_,lfoot_support_init_.translation()(2),0.0,0.0); }
      { 
        lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj,t_start_real_ + t_double1_ + (t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_,lfoot_support_traj_init_.translation()(2)+foot_height_,0.0,0.0,0.0); 
        rd_.link_[Left_Foot].v_traj(2) = DyrosMath::cubicDot(walking_tick_mj,t_start_+ t_rest_init_ + t_double1_ + t_rest_temp, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_)/2.0,lfoot_support_traj_init_.translation()(2)+foot_height_,0.0,0.0,0.0,2000.0);
      }
         
      for(int i=0; i<2; i++)
      // { lfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_mj,t_start_real_+t_double1_ + t_rest_temp,t_start_+t_total_-t_rest_last_-t_double2_,lfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0); }
      { lfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_mj,t_start_real_+t_double1_ + t_rest_temp,t_start_+t_total_-t_rest_last_-t_double2_,lfoot_support_traj_init_.translation()(i),target_swing_foot(i),0.0,0.0); }
      
      lfoot_trajectory_euler_support_(1) = 0;  
      lfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_mj,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_,lfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0);
      lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(F_T_L_y_input)*DyrosMath::rotateWithX(-F_T_L_x_input);
    }
  }
  else
  {
    if(foot_step_(current_step_num_,6) == 1)
    {
      lfoot_trajectory_euler_support_.setZero();
   
      lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(F_T_L_y_input)*DyrosMath::rotateWithX(-F_T_L_x_input);
      
      for(int i=0; i<3; i++)
      {
        rfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
        rfoot_trajectory_euler_support_(i) = target_swing_foot(i+3);
      }
   
      rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(F_T_R_y_input)*DyrosMath::rotateWithX(-F_T_R_x_input);
    }
    else if (foot_step_(current_step_num_,6) == 0)
    {
      rfoot_trajectory_euler_support_.setZero();
      rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(F_T_R_y_input)*DyrosMath::rotateWithX(-F_T_R_x_input);

      for(int i=0; i<3; i++)
      {
        lfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
        lfoot_trajectory_euler_support_(i) = target_swing_foot(i+3);
      } 
      lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(F_T_L_y_input)*DyrosMath::rotateWithX(-F_T_L_x_input);
    }
  } 
}

void CustomController::getPelvTrajectory()
{
  double z_rot = foot_step_support_frame_(current_step_num_,5);
  
  pelv_trajectory_support_.translation()(0) = pelv_support_current_.translation()(0) + 0.7*(com_desired_(0) - com_support_current_(0));//0.7*(com_desired_(0) - 0.15*damping_x - com_support_current_(0));//- 0.01 * zmp_err_(0) * 0;
  pelv_trajectory_support_.translation()(1) = pelv_support_current_.translation()(1) + 0.7*(com_desired_(1) - com_support_current_(1));//0.7*(com_desired_(1) - 0.6*damping_y - com_support_current_(1)) ;//- 0.01 * zmp_err_(1) * 0;
  pelv_trajectory_support_.translation()(2) = com_desired_(2);
  // MJ_graph << com_desired_(0) << "," << com_support_current_(0) << "," << com_desired_(1) << "," << com_support_current_(1) << endl;
  Eigen::Vector3d Trunk_trajectory_euler;
  Trunk_trajectory_euler.setZero();

  if(walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
  { Trunk_trajectory_euler(2) = pelv_support_euler_init_(2); }
  else if(walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)
  { Trunk_trajectory_euler(2) = DyrosMath::cubic(walking_tick_mj,t_start_real_+t_double1_,t_start_+t_total_-t_double2_-t_rest_last_, pelv_support_euler_init_(2),z_rot/2.0,0.0,0.0); }
  else
  { Trunk_trajectory_euler(2) = z_rot/2.0; } 

  if(aa == 0 && walking_tick_mj == 0)
  { P_angle_input = 0; R_angle_input = 0; }
  
  P_angle_input_dot = 1.5*(0.0 - P_angle) - 0.01*P_angle_input;
  
  P_angle_input = P_angle_input + P_angle_input_dot*del_t;

  if(P_angle_input > 0.05)
  { P_angle_input = 0.05; }
  else if(P_angle_input < -0.05)
  { P_angle_input = -0.05; }

  Trunk_trajectory_euler(1) = P_angle_input;

  pelv_trajectory_support_.linear() = DyrosMath::rotateWithZ(Trunk_trajectory_euler(2))*DyrosMath::rotateWithY(Trunk_trajectory_euler(1))*DyrosMath::rotateWithX(Trunk_trajectory_euler(0));
     
}

void CustomController::calculateFootStepTotal_MJ()
{
  double initial_rot = 0.0;
  double final_rot = 0.0;
  double initial_drot = 0.0;
  double final_drot = 0.0;

  initial_rot = atan2(target_y_, target_x_);

  if(initial_rot > 0.0)
    initial_drot = 10*DEG2RAD;
  else
    initial_drot = -10*DEG2RAD;

  unsigned int initial_total_step_number = initial_rot/initial_drot;
  double initial_residual_angle = initial_rot - initial_total_step_number*initial_drot;

  final_rot = target_theta_ - initial_rot;
  if(final_rot > 0.0)
    final_drot = 10*DEG2RAD;
  else
    final_drot = -10*DEG2RAD;

  unsigned int final_total_step_number = final_rot/final_drot;
  double final_residual_angle = final_rot - final_total_step_number*final_drot;
  double length_to_target = sqrt(target_x_*target_x_ + target_y_*target_y_);
  double dlength = step_length_x_;
  unsigned int middle_total_step_number = length_to_target/dlength;
  double middle_residual_length = length_to_target - middle_total_step_number*dlength;

  double step_width_init;
  double step_width;

  step_width_init = 0.01;   
  step_width = 0.02;
 
  if(length_to_target == 0)
  {
    middle_total_step_number = 20; //
    dlength = 0;
  }

  unsigned int number_of_foot_step;

  int del_size;

  del_size = 1;
  number_of_foot_step = 2 + initial_total_step_number*del_size + middle_total_step_number*del_size + final_total_step_number*del_size;

  if(initial_total_step_number != 0 || abs(initial_residual_angle) >= 0.0001)
  {
    if(initial_total_step_number % 2 == 0)
      number_of_foot_step = number_of_foot_step + 2*del_size;
    else
    {
      if(abs(initial_residual_angle)>= 0.0001)
        number_of_foot_step = number_of_foot_step + 3*del_size;
      else
        number_of_foot_step = number_of_foot_step + del_size;
    }
  }

  if(middle_total_step_number != 0 || abs(middle_residual_length)>=0.0001)
  {
    if(middle_total_step_number % 2 == 0)
      number_of_foot_step = number_of_foot_step + 2*del_size;
    else
    {
      if(abs(middle_residual_length)>= 0.0001)
        number_of_foot_step = number_of_foot_step + 3*del_size;
      else
        number_of_foot_step = number_of_foot_step + del_size;
    }
  }

  if(final_total_step_number != 0 || abs(final_residual_angle) >= 0.0001)
  {
    if(abs(final_residual_angle) >= 0.0001)
      number_of_foot_step = number_of_foot_step + 2*del_size;
    else
      number_of_foot_step = number_of_foot_step + del_size;
  }


  foot_step_.resize(number_of_foot_step, 7);
  foot_step_.setZero();
  foot_step_support_frame_.resize(number_of_foot_step, 7);
  foot_step_support_frame_.setZero();

  int index = 0;
  int temp, temp2, temp3, is_right;

  if(is_right_foot_swing_ == true)
    is_right = 1;
  else
    is_right = -1;


  temp = -is_right;
  temp2 = -is_right;
  temp3 = -is_right;

  int temp0;
  temp0 = -is_right;

  double initial_dir = 0.0;

  if(aa == 0)
  {
    for (int i = 0 ; i < 2; i++)
    {
      temp0 *= -1;

      if(i == 0)
      {
        foot_step_(index,0) = cos(initial_dir)*(0.0) + temp0*sin(initial_dir)*(0.1025 + step_width_init*(i+1));
        foot_step_(index,1) = sin(initial_dir)*(0.0) - temp0*cos(initial_dir)*(0.1025 + step_width_init*(i+1));
      }
      else if(i == 1)
      {
        foot_step_(index,0) = cos(initial_dir)*(0.0) + temp0*sin(initial_dir)*(0.1025 + step_width_init*(i+1));
        foot_step_(index,1) = sin(initial_dir)*(0.0) - temp0*cos(initial_dir)*(0.1025 + step_width_init*(i+1));
      }     
      
      foot_step_(index,5) = initial_dir;
      foot_step_(index,6) = 0.5 + 0.5*temp0;
      index ++;
    }
  }
  else if(aa == 1)
  {
    for (int i = 0 ; i < 2; i++)
    {
      temp0 *= -1;

      foot_step_(index,0) = cos(initial_dir)*(0.0) + temp0*sin(initial_dir)*(0.1025 + step_width);
      foot_step_(index,1) = sin(initial_dir)*(0.0) - temp0*cos(initial_dir)*(0.1025 + step_width);
      foot_step_(index,5) = initial_dir;
      foot_step_(index,6) = 0.5 + 0.5*temp0;
      index ++;
    }
  }

  if(initial_total_step_number != 0 || abs(initial_residual_angle) >= 0.0001) // 첫번째 회전
  {
    for (int i =0 ; i < initial_total_step_number; i++)
    {
      temp *= -1;
      foot_step_(index,0) = temp*(0.1025 + step_width)*sin((i+1)*initial_drot);
      foot_step_(index,1) = -temp*(0.1025 + step_width)*cos((i+1)*initial_drot);
      foot_step_(index,5) = (i+1)*initial_drot;
      foot_step_(index,6) = 0.5 + 0.5*temp;
      index++;
    }

    if(temp == is_right)
    {
      if(abs(initial_residual_angle) >= 0.0001)
      {
        temp *= -1;

        foot_step_(index,0) = temp*(0.1025 + step_width)*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,1) = -temp*(0.1025 + step_width)*cos((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
        foot_step_(index,6) = 0.5 + 0.5*temp;
        index++;

        temp *= -1;

        foot_step_(index,0) = temp*(0.1025 + step_width)*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,1) = -temp*(0.1025 + step_width)*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
        foot_step_(index,6) = 0.5 + 0.5*temp;
        index++;

        temp *= -1;

        foot_step_(index,0) = temp*(0.1025 + step_width)*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,1) = -temp*(0.1025 + step_width)*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
        foot_step_(index,6) = 0.5 + 0.5*temp;
        index++;

      }
      else
      {
        temp *= -1;

        foot_step_(index,0) = temp*(0.1025 + step_width)*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,1) = -temp*(0.1025 + step_width)*cos((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
        foot_step_(index,6) = 0.5 + 0.5*temp;
        index++;
      }
    }
    else if(temp == -is_right)
    {
      temp *= -1;

      foot_step_(index,0) = temp*(0.1025 + step_width)*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
      foot_step_(index,1) = -temp*(0.1025 + step_width)*cos((initial_total_step_number)*initial_drot + initial_residual_angle);
      foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
      foot_step_(index,6) = 0.5 + 0.5*temp;
      index ++;

      temp *= -1;

      foot_step_(index,0) = temp*(0.1025 + step_width)*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
      foot_step_(index,1) = -temp*(0.1025 + step_width)*cos((initial_total_step_number)*initial_drot + initial_residual_angle);
      foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
      foot_step_(index,6) = 0.5 + 0.5*temp;
      index ++;
    }
  }

  if(middle_total_step_number != 0 || abs(middle_residual_length) >= 0.0001) // 직진, 제자리 보행
  {
    
    for (int i = 0 ; i < middle_total_step_number; i++)
    {
      temp2 *= -1;

      foot_step_(index,0) = cos(initial_rot)*(dlength*(i+1)) + temp2*sin(initial_rot)*(0.1025 + step_width);
      foot_step_(index,1) = sin(initial_rot)*(dlength*(i+1)) - temp2*cos(initial_rot)*(0.1025 + step_width);
      foot_step_(index,5) = initial_rot;
      foot_step_(index,6) = 0.5 + 0.5*temp2;
      index ++;
    }

    if(temp2 == is_right)
    {
      if(abs(middle_residual_length) >= 0.0001)
      {
        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.1025 + step_width);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.1025 + step_width);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5 + 0.5*temp2;

        index++;

        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.1025 + step_width);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.1025 + step_width);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5 + 0.5*temp2;
        index++;

        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.1025 + step_width);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.1025 + step_width);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5 + 0.5*temp2;
        index++;
      }
      else
      {
        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.1025 + step_width);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.1025 + step_width);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5 + 0.5*temp2;
        index++;
      }
    }
    else if(temp2 == -is_right)
    {
      temp2 *= -1;

      foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.1025 + step_width);
      foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.1025 + step_width);
      foot_step_(index,5) = initial_rot;
      foot_step_(index,6) = 0.5 + 0.5*temp2;
      index++;

      temp2 *= -1;

      foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.1025 + step_width);
      foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.1025 + step_width);
      foot_step_(index,5) = initial_rot;
      foot_step_(index,6) = 0.5 + 0.5*temp2;
      index++;
    }
  }

  double final_position_x = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length);
  double final_position_y = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length);

  if(final_total_step_number != 0 || abs(final_residual_angle) >= 0.0001)
  {
    for(int i = 0 ; i < final_total_step_number; i++)
    {
      temp3 *= -1;

      foot_step_(index,0) = final_position_x + temp3*(0.1025 + step_width)*sin((i+1)*final_drot + initial_rot);
      foot_step_(index,1) = final_position_y - temp3*(0.1025 + step_width)*cos((i+1)*final_drot + initial_rot);
      foot_step_(index,5) = (i+1)*final_drot + initial_rot;
      foot_step_(index,6) = 0.5 + 0.5*temp3;
      index++;
    }

    if(abs(final_residual_angle) >= 0.0001)
    {
      temp3 *= -1;

      foot_step_(index,0) = final_position_x + temp3*(0.1025 + step_width)*sin(target_theta_);
      foot_step_(index,1) = final_position_y - temp3*(0.1025 + step_width)*cos(target_theta_);
      foot_step_(index,5) = target_theta_;
      foot_step_(index,6) = 0.5 + 0.5*temp3;
      index++;

      temp3 *= -1;

      foot_step_(index,0) = final_position_x + temp3*(0.1025 + step_width)*sin(target_theta_);
      foot_step_(index,1) = final_position_y - temp3*(0.1025 + step_width)*cos(target_theta_);
      foot_step_(index,5) = target_theta_;
      foot_step_(index,6) = 0.5 + 0.5*temp3;
      index++;
    }
    else
    {
      temp3 *= -1;

      foot_step_(index,0) = final_position_x + temp3*(0.1025 + step_width)*sin(target_theta_);
      foot_step_(index,1) = final_position_y - temp3*(0.1025 + step_width)*cos(target_theta_);
      foot_step_(index,5) = target_theta_;
      foot_step_(index,6) = 0.5 + 0.5*temp3;
      index++;
    }
  }

  //std::cout << "footstep: " << foot_step_ << std::endl;
}

void CustomController::walkingContactState()
{
  if (walking_tick_mj < t_start_ + t_rest_init_) //DSP
  {
    WBC::SetContact(rd_, 1, 1);
    contactcheck = 0;
  }
  else if (walking_tick_mj >= t_start_ + t_rest_init_ && walking_tick_mj < t_start_ + t_rest_init_ + t_double1_) // 0.03 s  DSP
  {
    WBC::SetContact(rd_, 1, 1);
    contactcheck = 0;
  }
  else if (walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_) // SSP
  {
    if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
    {
      WBC::SetContact(rd_, 1, 0);
      contactcheck = 1;
    }
    else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
    {
      WBC::SetContact(rd_, 0, 1);
      contactcheck = 2;
    }
  }
  else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ && walking_tick_mj < t_start_ + t_total_ - t_rest_last_) //DSP
  {
    WBC::SetContact(rd_, 1, 1);
    contactcheck = 0;
  }
  else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ && walking_tick_mj < t_start_ + t_total_) //DSP
  {
    WBC::SetContact(rd_, 1, 1);
    contactcheck = 0;
  }
  else if (walking_tick_mj >= t_start_ + t_total_) //마지막발
  {
    WBC::SetContact(rd_, 1, 1);
    contactcheck = 0;
  }
}

void CustomController::ContactStateChangeTorque()
{
  double contact_gain = 0.0;
   
  if(walking_tick_mj < t_start_ + t_rest_init_ )
  {
    contact_gain = 1.0;
    if(foot_step_(current_step_num_,6) == 1) // 왼발 지지
    { Contact_torque = WBC::ContactForceRedistributionTorqueJS(rd_, Gravity_torque + Task_torque, contact_gain, 1); CF_distributioncheck = 0;}
    else if(foot_step_(current_step_num_,6) == 0) // 오른발 지지
    { Contact_torque = WBC::ContactForceRedistributionTorqueJS(rd_, Gravity_torque + Task_torque, contact_gain, 0); CF_distributioncheck = 0;}  
  }
  else if(walking_tick_mj >= t_start_ + t_rest_init_ && walking_tick_mj < t_start_ + t_rest_init_ + t_double1_ ) // 0.03 s  
  {
    contact_gain = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_, t_start_ + t_rest_init_ + t_double1_, 1.0, 0.0, 0.0, 0.0);
    if(foot_step_(current_step_num_,6) == 1) // 왼발 지지
    { Contact_torque = WBC::ContactForceRedistributionTorqueJS(rd_, Gravity_torque + Task_torque, contact_gain, 1); CF_distributioncheck = 1;}
    else if(foot_step_(current_step_num_,6) == 0) // 오른발 지지
    { Contact_torque = WBC::ContactForceRedistributionTorqueJS(rd_, Gravity_torque + Task_torque, contact_gain, 0); CF_distributioncheck = 2;}
  }
  else if(walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_) // SSP
  {
    Contact_torque.setZero();
  }
  else if(walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ && walking_tick_mj < t_start_ + t_total_ - t_rest_last_)
  // else if(walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ && walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ + 20)
  {
    contact_gain = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ , t_start_ + t_total_ - t_rest_last_ , 0.0, 1.0, 0.0, 0.0);
    // contact_gain = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ , t_start_ + t_total_ - t_rest_last_ - t_double2_ + 20 , 0.0, 1.0, 0.0, 0.0);
    if(foot_step_(current_step_num_,6) == 1) // 왼발 지지
    {   
      Contact_torque = WBC::ContactForceRedistributionTorqueJS(rd_, Gravity_torque + Task_torque, contact_gain, 1); CF_distributioncheck = 1;
    }
    else if(foot_step_(current_step_num_,6) == 0) // 오른발 지지
    {
      Contact_torque = WBC::ContactForceRedistributionTorqueJS(rd_, Gravity_torque + Task_torque, contact_gain, 0); CF_distributioncheck = 2;
    }    
  }
  else if(walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ && walking_tick_mj < t_start_ + t_total_)
  {
    if(foot_step_(current_step_num_,6) == 1) // 왼발 지지
    { Contact_torque = WBC::ContactForceRedistributionTorqueJS(rd_, Gravity_torque + Task_torque, 1.0, 1); CF_distributioncheck = 0;}
    else if(foot_step_(current_step_num_,6) == 0) // 오른발 지지
    { Contact_torque = WBC::ContactForceRedistributionTorqueJS(rd_, Gravity_torque + Task_torque, 1.0, 0); CF_distributioncheck = 0;}
  }
  else if(walking_tick_mj >= t_start_ + t_total_)//마지막발
  {
    if(foot_step_(current_step_num_,6) == 1) // 왼발 지지
    { Contact_torque = WBC::ContactForceRedistributionTorqueJS(rd_, Gravity_torque + Task_torque, 1.0, 1); CF_distributioncheck = 0;}
    else if(foot_step_(current_step_num_,6) == 0) // 오른발 지지
    { Contact_torque = WBC::ContactForceRedistributionTorqueJS(rd_, Gravity_torque + Task_torque, 1.0, 0); CF_distributioncheck = 0;}
  }
}

void CustomController::walkingFstarJacSet()
{
  if (walking_tick_mj == t_start_ + t_total_ - 2)
  {
    t_step_change = t_start_ + t_total_;
  }
  if (walking_tick_mj == t_step_change - 1)
  {
    step_error_before = pelv_trajectory_support_.translation() - pelv_support_current_.translation();
  }

  if (walking_tick_mj == t_step_change)
  {
    step_error_after = pelv_trajectory_support_.translation() - pelv_support_current_.translation();
    step_error_comp += step_error_after - step_error_before;
  }

  if (walking_tick_mj < t_start_ + t_rest_init_) //DSP
  {
    task_number = 6 + 3 + 12; //com + upperbody orientation + 2 hands
    rd_.J_task.resize(task_number, MODEL_DOF_VIRTUAL);
    rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Pelvis].Jac();
    rd_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac().block(3, 0, 3, MODEL_DOF_VIRTUAL);
    rd_.J_task.block(9, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Right_Hand].Jac();
    rd_.J_task.block(15, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Left_Hand].Jac();

    f_star.resize(task_number);
    rd_.link_[Pelvis].v_traj.setZero();
    f_star.segment(0, 3) = WBC::GetFstarPosJS(rd_.link_[Pelvis], pelv_trajectory_support_.translation() - step_error_comp, pelv_support_current_.translation(), rd_.link_[Pelvis].v_traj, pelv_float_current_dot);
    f_star.segment(3, 3) = WBC::GetFstarRot(rd_.link_[Pelvis]);
    f_star.segment(6, 3) = WBC::GetFstarRot(rd_.link_[Upper_Body]);
    f_star.segment(9, 3) = WBC::GetFstarPosJS(rd_.link_[Right_Hand], rd_.link_[Right_Hand].x_traj, rd_.link_[Right_Hand].xpos - rd_.link_[Upper_Body].xpos, rd_.link_[Right_Hand].v_traj, rd_.link_[Right_Hand].v - rd_.link_[Upper_Body].v);
    f_star.segment(12, 3) = WBC::GetFstarRot(rd_.link_[Right_Hand]);
    f_star.segment(15, 3) = WBC::GetFstarPosJS(rd_.link_[Left_Hand], rd_.link_[Left_Hand].x_traj, rd_.link_[Left_Hand].xpos - rd_.link_[Upper_Body].xpos, rd_.link_[Left_Hand].v_traj, rd_.link_[Left_Hand].v - rd_.link_[Upper_Body].v);
    f_star.segment(18, 3) = WBC::GetFstarRot(rd_.link_[Left_Hand]);
  }
  else if (walking_tick_mj >= t_start_ + t_rest_init_ && walking_tick_mj < t_start_ + t_rest_init_ + t_double1_) // 0.03 s  DSP
  {
    task_number = 6 + 3 + 12; //com + upperbody orientation + 2 hands
    rd_.J_task.resize(task_number, MODEL_DOF_VIRTUAL);
    rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Pelvis].Jac();
    rd_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac().block(3, 0, 3, MODEL_DOF_VIRTUAL);
    rd_.J_task.block(9, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Right_Hand].Jac();
    rd_.J_task.block(15, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Left_Hand].Jac();

    f_star.resize(task_number);
    rd_.link_[Pelvis].v_traj.setZero();
    f_star.segment(0, 3) = WBC::GetFstarPosJS(rd_.link_[Pelvis], pelv_trajectory_support_.translation() - step_error_comp, pelv_support_current_.translation(), rd_.link_[Pelvis].v_traj, pelv_float_current_dot);
    f_star.segment(3, 3) = WBC::GetFstarRot(rd_.link_[Pelvis]);
    f_star.segment(6, 3) = WBC::GetFstarRot(rd_.link_[Upper_Body]);
    f_star.segment(9, 3) = WBC::GetFstarPosJS(rd_.link_[Right_Hand], rd_.link_[Right_Hand].x_traj, rd_.link_[Right_Hand].xpos - rd_.link_[Upper_Body].xpos, rd_.link_[Right_Hand].v_traj, rd_.link_[Right_Hand].v - rd_.link_[Upper_Body].v);
    f_star.segment(12, 3) = WBC::GetFstarRot(rd_.link_[Right_Hand]);
    f_star.segment(15, 3) = WBC::GetFstarPosJS(rd_.link_[Left_Hand], rd_.link_[Left_Hand].x_traj, rd_.link_[Left_Hand].xpos - rd_.link_[Upper_Body].xpos, rd_.link_[Left_Hand].v_traj, rd_.link_[Left_Hand].v - rd_.link_[Upper_Body].v);
    f_star.segment(18, 3) = WBC::GetFstarRot(rd_.link_[Left_Hand]);
  }
  else if (walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_) // SSP
  {
    task_number = 6 + 3 + 6 + 12; //com + upperbody orientation + leg + 2 hands
    rd_.J_task.resize(task_number, MODEL_DOF_VIRTUAL);
    if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
    {
      rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Pelvis].Jac();
      rd_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac().block(3, 0, 3, MODEL_DOF_VIRTUAL);
      rd_.J_task.block(9, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Right_Foot].Jac();
      rd_.J_task.block(15, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Right_Hand].Jac();
      rd_.J_task.block(21, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Left_Hand].Jac();

      f_star.resize(task_number);
      f_star.setZero();
      rd_.link_[Pelvis].v_traj.setZero();
      rd_.link_[Right_Foot].rot_init = rd_.link_[Right_Foot].rotm;
      rd_.link_[Right_Foot].rot_desired = Matrix3d::Identity();
      rd_.link_[Right_Foot].SetTrajectoryRotation(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + 100);
      rd_.link_[Right_Foot].v_traj.setZero();
      
      f_star.segment(0, 3) = WBC::GetFstarPosJS(rd_.link_[Pelvis], pelv_trajectory_support_.translation() - step_error_comp, pelv_support_current_.translation(), rd_.link_[Pelvis].v_traj, pelv_float_current_dot);
      f_star.segment(3, 3) = WBC::GetFstarRot(rd_.link_[Pelvis]);
      f_star.segment(6, 3) = WBC::GetFstarRot(rd_.link_[Upper_Body]);
      //f_star.segment(9, 3) = WBC::GetFstarPosJS(rd_.link_[Right_Foot], rfoot_trajectory_support_.translation(), rfoot_support_current_.translation(), rd_.link_[Right_Foot].v_traj, rfoot_float_current_dot);
      f_star(9) = 2800.0 * (rfoot_trajectory_support_.translation()(0) - current_rfoot_control.translation()(0)) + 180.0 * ( - rfoot_float_current_dot(0));
      f_star(10) = 1400.0 * (rfoot_trajectory_support_.translation()(1) - current_rfoot_control.translation()(1)) + 10.0 * ( - rfoot_float_current_dot(1));
      f_star(11) = 700.0 * (rfoot_trajectory_support_.translation()(2) - current_rfoot_control.translation()(2)) + 20.0 * ( - rfoot_float_current_dot(2));
      Eigen::Vector3d ad = DyrosMath::getPhi(rd_.link_[Right_Foot].rotm, rd_.link_[Right_Foot].r_traj);
      f_star(12) = -400.0 * ad(0) + 10.0 * ( - rd_.link_[Right_Foot].w(0));
      f_star(13) = -800.0 * ad(1) + 40.0 * ( - rd_.link_[Right_Foot].w(1));
      f_star(14) = -400.0 * ad(2) + 10.0 * ( - rd_.link_[Right_Foot].w(2));
      f_star.segment(15, 3) = WBC::GetFstarPosJS(rd_.link_[Right_Hand], rd_.link_[Right_Hand].x_traj, rd_.link_[Right_Hand].xpos - rd_.link_[Upper_Body].xpos, rd_.link_[Right_Hand].v_traj, rd_.link_[Right_Hand].v - rd_.link_[Upper_Body].v);
      f_star.segment(18, 3) = WBC::GetFstarRot(rd_.link_[Right_Hand]);
      f_star.segment(21, 3) = WBC::GetFstarPosJS(rd_.link_[Left_Hand], rd_.link_[Left_Hand].x_traj, rd_.link_[Left_Hand].xpos - rd_.link_[Upper_Body].xpos, rd_.link_[Left_Hand].v_traj, rd_.link_[Left_Hand].v - rd_.link_[Upper_Body].v);
      f_star.segment(24, 3) = WBC::GetFstarRot(rd_.link_[Left_Hand]);
    }
    else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
    {
      rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Pelvis].Jac();
      rd_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac().block(3, 0, 3, MODEL_DOF_VIRTUAL);
      rd_.J_task.block(9, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Left_Foot].Jac();
      rd_.J_task.block(15, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Right_Hand].Jac();
      rd_.J_task.block(21, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Left_Hand].Jac();

      f_star.resize(task_number);
      f_star.setZero();
      rd_.link_[Pelvis].v_traj.setZero();
      rd_.link_[Left_Foot].rot_init = rd_.link_[Left_Foot].rotm;
      rd_.link_[Left_Foot].rot_desired = Matrix3d::Identity();
      rd_.link_[Left_Foot].SetTrajectoryRotation(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + 100);
      rd_.link_[Left_Foot].v_traj.setZero();
      
      f_star.segment(0, 3) = WBC::GetFstarPosJS(rd_.link_[Pelvis], pelv_trajectory_support_.translation() - step_error_comp, pelv_support_current_.translation(), rd_.link_[Pelvis].v_traj, pelv_float_current_dot);
      f_star.segment(3, 3) = WBC::GetFstarRot(rd_.link_[Pelvis]);
      f_star.segment(6, 3) = WBC::GetFstarRot(rd_.link_[Upper_Body]);
      //f_star.segment(9, 3) = WBC::GetFstarPosJS(rd_.link_[Left_Foot], lfoot_trajectory_support_.translation(), lfoot_support_current_.translation(), rd_.link_[Left_Foot].v_traj, lfoot_float_current_dot);
      // f_star(9) = 1400.0 * (lfoot_trajectory_support_.translation()(0) - lfoot_support_current_.translation()(0)) + 180.0 * ( - lfoot_float_current_dot(0));
      // f_star(10) = 1400.0 * (lfoot_trajectory_support_.translation()(1) - lfoot_support_current_.translation()(1)) + 10.0 * ( - lfoot_float_current_dot(1));
      // f_star(11) = 700.0 * (lfoot_trajectory_support_.translation()(2) - lfoot_support_current_.translation()(2)) + 20.0 * ( - lfoot_float_current_dot(2));
      f_star(9) = 2800.0 * (lfoot_trajectory_support_.translation()(0) - current_lfoot_control.translation()(0)) + 180.0 * ( - lfoot_float_current_dot(0));
      f_star(10) = 1400.0 * (lfoot_trajectory_support_.translation()(1) - current_lfoot_control.translation()(1)) + 10.0 * ( - lfoot_float_current_dot(1));
      f_star(11) = 700.0 * (lfoot_trajectory_support_.translation()(2) - current_lfoot_control.translation()(2)) + 20.0 * ( - lfoot_float_current_dot(2));
      Eigen::Vector3d ad = DyrosMath::getPhi(rd_.link_[Left_Foot].rotm, rd_.link_[Left_Foot].r_traj);
      f_star(12) = -400.0 * ad(0) + 10.0 * ( - rd_.link_[Left_Foot].w(0));
      f_star(13) = -800.0 * ad(1) + 40.0 * ( - rd_.link_[Left_Foot].w(1));
      f_star(14) = -400.0 * ad(2) + 10.0 * ( - rd_.link_[Left_Foot].w(2));
      f_star.segment(15, 3) = WBC::GetFstarPosJS(rd_.link_[Right_Hand], rd_.link_[Right_Hand].x_traj, rd_.link_[Right_Hand].xpos - rd_.link_[Upper_Body].xpos, rd_.link_[Right_Hand].v_traj, rd_.link_[Right_Hand].v - rd_.link_[Upper_Body].v);
      f_star.segment(18, 3) = WBC::GetFstarRot(rd_.link_[Right_Hand]);
      f_star.segment(21, 3) = WBC::GetFstarPosJS(rd_.link_[Left_Hand], rd_.link_[Left_Hand].x_traj, rd_.link_[Left_Hand].xpos - rd_.link_[Upper_Body].xpos, rd_.link_[Left_Hand].v_traj, rd_.link_[Left_Hand].v - rd_.link_[Upper_Body].v);
      f_star.segment(24, 3) = WBC::GetFstarRot(rd_.link_[Left_Hand]);
    }
  }
  else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ && walking_tick_mj < t_start_ + t_total_ - t_rest_last_) //DSP
  {
    task_number = 6 + 3 + 12; //com + upperbody orientation + 2 hands
    rd_.J_task.resize(task_number, MODEL_DOF_VIRTUAL);
    rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Pelvis].Jac();
    rd_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac().block(3, 0, 3, MODEL_DOF_VIRTUAL);
    rd_.J_task.block(9, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Right_Hand].Jac();
    rd_.J_task.block(15, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Left_Hand].Jac();

    f_star.resize(task_number);
    rd_.link_[Pelvis].v_traj.setZero();
    f_star.segment(0, 3) = WBC::GetFstarPosJS(rd_.link_[Pelvis], pelv_trajectory_support_.translation() - step_error_comp, pelv_support_current_.translation(), rd_.link_[Pelvis].v_traj, pelv_float_current_dot);
    f_star.segment(3, 3) = WBC::GetFstarRot(rd_.link_[Pelvis]);
    f_star.segment(6, 3) = WBC::GetFstarRot(rd_.link_[Upper_Body]);
    f_star.segment(9, 3) = WBC::GetFstarPosJS(rd_.link_[Right_Hand], rd_.link_[Right_Hand].x_traj, rd_.link_[Right_Hand].xpos - rd_.link_[Upper_Body].xpos, rd_.link_[Right_Hand].v_traj, rd_.link_[Right_Hand].v - rd_.link_[Upper_Body].v);
    f_star.segment(12, 3) = WBC::GetFstarRot(rd_.link_[Right_Hand]);
    f_star.segment(15, 3) = WBC::GetFstarPosJS(rd_.link_[Left_Hand], rd_.link_[Left_Hand].x_traj, rd_.link_[Left_Hand].xpos - rd_.link_[Upper_Body].xpos, rd_.link_[Left_Hand].v_traj, rd_.link_[Left_Hand].v - rd_.link_[Upper_Body].v);
    f_star.segment(18, 3) = WBC::GetFstarRot(rd_.link_[Left_Hand]);
  }
  else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ && walking_tick_mj < t_start_ + t_total_) //DSP
  {
    task_number = 6 + 3 + 12; //com + upperbody orientation + 2 hands
    rd_.J_task.resize(task_number, MODEL_DOF_VIRTUAL);
    rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Pelvis].Jac();
    rd_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac().block(3, 0, 3, MODEL_DOF_VIRTUAL);
    rd_.J_task.block(9, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Right_Hand].Jac();
    rd_.J_task.block(15, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Left_Hand].Jac();

    f_star.resize(task_number);
    rd_.link_[Pelvis].v_traj.setZero();
    f_star.segment(0, 3) = WBC::GetFstarPosJS(rd_.link_[Pelvis], pelv_trajectory_support_.translation() - step_error_comp, pelv_support_current_.translation(), rd_.link_[Pelvis].v_traj, pelv_float_current_dot);
    f_star.segment(3, 3) = WBC::GetFstarRot(rd_.link_[Pelvis]);
    f_star.segment(6, 3) = WBC::GetFstarRot(rd_.link_[Upper_Body]);
    f_star.segment(9, 3) = WBC::GetFstarPosJS(rd_.link_[Right_Hand], rd_.link_[Right_Hand].x_traj, rd_.link_[Right_Hand].xpos - rd_.link_[Upper_Body].xpos, rd_.link_[Right_Hand].v_traj, rd_.link_[Right_Hand].v - rd_.link_[Upper_Body].v);
    f_star.segment(12, 3) = WBC::GetFstarRot(rd_.link_[Right_Hand]);
    f_star.segment(15, 3) = WBC::GetFstarPosJS(rd_.link_[Left_Hand], rd_.link_[Left_Hand].x_traj, rd_.link_[Left_Hand].xpos - rd_.link_[Upper_Body].xpos, rd_.link_[Left_Hand].v_traj, rd_.link_[Left_Hand].v - rd_.link_[Upper_Body].v);
    f_star.segment(18, 3) = WBC::GetFstarRot(rd_.link_[Left_Hand]);
  }
  else if (walking_tick_mj >= t_start_ + t_total_) //마지막발
  {
    task_number = 6 + 3 + 12; //com + upperbody orientation + 2 hands
    rd_.J_task.resize(task_number, MODEL_DOF_VIRTUAL);
    rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Pelvis].Jac();
    rd_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac().block(3, 0, 3, MODEL_DOF_VIRTUAL);
    rd_.J_task.block(9, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Right_Hand].Jac();
    rd_.J_task.block(15, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[Left_Hand].Jac();

    f_star.resize(task_number);
    rd_.link_[Pelvis].v_traj.setZero();
    f_star.segment(0, 3) = WBC::GetFstarPosJS(rd_.link_[Pelvis], pelv_trajectory_support_.translation() - step_error_comp, pelv_support_current_.translation(), rd_.link_[Pelvis].v_traj, pelv_float_current_dot);
    f_star.segment(3, 3) = WBC::GetFstarRot(rd_.link_[Pelvis]);
    f_star.segment(6, 3) = WBC::GetFstarRot(rd_.link_[Upper_Body]);
    f_star.segment(9, 3) = WBC::GetFstarPosJS(rd_.link_[Right_Hand], rd_.link_[Right_Hand].x_traj, rd_.link_[Right_Hand].xpos - rd_.link_[Upper_Body].xpos, rd_.link_[Right_Hand].v_traj, rd_.link_[Right_Hand].v - rd_.link_[Upper_Body].v);
    f_star.segment(12, 3) = WBC::GetFstarRot(rd_.link_[Right_Hand]);
    f_star.segment(15, 3) = WBC::GetFstarPosJS(rd_.link_[Left_Hand], rd_.link_[Left_Hand].x_traj, rd_.link_[Left_Hand].xpos - rd_.link_[Upper_Body].xpos, rd_.link_[Left_Hand].v_traj, rd_.link_[Left_Hand].v - rd_.link_[Upper_Body].v);
    f_star.segment(18, 3) = WBC::GetFstarRot(rd_.link_[Left_Hand]);
  }
}