#include "dspSolver.h"
#include <iostream>
#include <cmath>

// Implement this function
dspSolver::dspSolver(dspLinkConfig& linkConfig) : LinkConfig(linkConfig) {
    /// Solver variable initialization
    /// The link with index 0 is the ground link
    /// There are only two joint types : revolute and prismatic
    int link_num = LinkConfig.GetNumLink();
    int joint_num = LinkConfig.GetNumJoint();

    /// You have to initialize following variables
    q.setZero(3 * (link_num - 1));
    qdot.setZero(3 * (link_num - 1));
    qddot.setZero(3 * (link_num - 1));
    M.setZero(3 * (link_num - 1), 3 * (link_num -1));
    M_inv.setZero(3 * (link_num - 1), 3 * (link_num - 1));
    J.setZero(2 * joint_num, 3 * (link_num - 1));
    J_dot.setZero(2 * joint_num, 3 * (link_num - 1));
    F_ext.setZero(3 * (link_num - 1));
    lambda.setZero(2 * joint_num);

}

// Implement this function
bool dspSolver::CalculateConstraintError(Eigen::VectorXd& error_c) {
    /// calculate error what you define as constraint
    int link_num = LinkConfig.GetNumLink();
    int joint_num = LinkConfig.GetNumJoint();
    for (int i = 0; i < joint_num; i++) 
    {
        dspJoint* joint = LinkConfig.GetJoint(i);

        int link1_id = joint->GetLink1ID();
        int link2_id = joint->GetLink2ID();

        dspLink* link1 = LinkConfig.GetLink(link1_id);
        dspLink* link2 = LinkConfig.GetLink(link2_id);

        Eigen::Vector2d p1;
        Eigen::Vector2d q1;
        Eigen::Vector2d p2;
        Eigen::Vector2d q2;

        joint->GetP1_Link1_LCS(p1);
        joint->GetQ1_Link1_LCS(q1);
        joint->GetP2_Link2_LCS(p2);
        joint->GetQ2_Link2_LCS(q2); 
        
        Eigen::Vector2d P1;
        Eigen::Vector2d P2;
        double theta1;
        double theta2;

        if(link1_id == 0)
        {
            P1.setZero();
            theta1 = 0;
            P2(0) = q(3*link2_id - 3);
            P2(1) = q(3*link2_id - 2);
            theta2 = q(3*link2_id - 1);
        }
        else if(link2_id == 0)
        {
            P2.setZero();
            theta2 = 0;
            P1(0) = q(3*link1_id - 3);
            P1(1) = q(3*link1_id - 2);
            theta1 = q(3*link1_id - 1);
        }
        else
        {
            P1(0) = q(3*link1_id - 3);
            P1(1) = q(3*link1_id - 2);
            theta1 = q(3*link1_id - 1);
            P2(0) = q(3*link2_id - 3);
            P2(1) = q(3*link2_id - 2);
            theta2 = q(3*link2_id - 1);
        }

        Eigen::Matrix2d R1;
        Eigen::Matrix2d R2;

        R1 << cos(theta1), -sin(theta1),
              sin(theta1),  cos(theta1);
        R2 << cos(theta2), -sin(theta2),
              sin(theta2),  cos(theta2);       
        
        Eigen::Vector2d p1_GCS;
        Eigen::Vector2d q1_GCS;
        Eigen::Vector2d p2_GCS;
        Eigen::Vector2d q2_GCS;

        p1_GCS = P1 + R1*p1;
        q1_GCS = P1 + R1*q1;
        p2_GCS = P2 + R2*p2;
        q2_GCS = P2 + R2*q2;

        Eigen::Matrix2d R;
        R << 0, -1,
             1,  0;
        
        if(joint->GetType() == 0)
        {
            Eigen::Vector2d C = p2_GCS - p1_GCS;
            error_c(2*i) = C(0);
            error_c(2*i + 1) = C(1);
        }
        else if(joint->GetType() == 1)
        {
            Eigen::Vector2d vi = q1_GCS - p1_GCS;
            Eigen::Vector2d vi_v = R*vi;
            error_c(2*i) = vi_v.dot(p2_GCS - p1_GCS);
            error_c(2*i + 1) = vi_v.dot(q2_GCS - p2_GCS);
        }
    }
    return true;
}

// Implement this function
bool dspSolver::Make_M(void) {
    int link_num = LinkConfig.GetNumLink();
    int joint_num = LinkConfig.GetNumJoint();
    for (int i = 1; i < link_num; ++i) {

        dspLink* link = LinkConfig.GetLink(i);
        double mass = link->GetMass();
        double inertia = link->GetInertia();

        M(3*i - 3, 3*i - 3) = mass;     
        M(3*i - 2, 3*i - 2) = mass; 
        M(3*i - 1, 3*i - 1) = inertia; 
    }

    M_inv = M.inverse();

    return true;
}

// Implement this function
bool dspSolver::Make_J(void) {
    /// make Jacobian matrix which has first-order partial derivatives of vector q's components : J
    int link_num = LinkConfig.GetNumLink();
    int joint_num = LinkConfig.GetNumJoint();
    for (int i = 0; i < joint_num; i++) 
    {
        dspJoint* joint = LinkConfig.GetJoint(i);

        int link1_id = joint->GetLink1ID();
        int link2_id = joint->GetLink2ID();

        dspLink* link1 = LinkConfig.GetLink(link1_id);
        dspLink* link2 = LinkConfig.GetLink(link2_id);

        Eigen::Vector2d p1;
        Eigen::Vector2d q1;
        Eigen::Vector2d p2;
        Eigen::Vector2d q2;

        joint->GetP1_Link1_LCS(p1);
        joint->GetQ1_Link1_LCS(q1);
        joint->GetP2_Link2_LCS(p2);
        joint->GetQ2_Link2_LCS(q2); 
        
        Eigen::Vector2d P1;
        Eigen::Vector2d P2;
        double theta1;
        double theta2;

        if(link1_id == 0)
        {
            P1.setZero();
            theta1 = 0;
            P2(0) = q(3*link2_id - 3);
            P2(1) = q(3*link2_id - 2);
            theta2 = q(3*link2_id - 1);
        }
        else if(link2_id == 0)
        {
            P2.setZero();
            theta2 = 0;
            P1(0) = q(3*link1_id - 3);
            P1(1) = q(3*link1_id - 2);
            theta1 = q(3*link1_id - 1);
        }
        else
        {
            P1(0) = q(3*link1_id - 3);
            P1(1) = q(3*link1_id - 2);
            theta1 = q(3*link1_id - 1);
            P2(0) = q(3*link2_id - 3);
            P2(1) = q(3*link2_id - 2);
            theta2 = q(3*link2_id - 1);
        }

        Eigen::Matrix2d R1;
        Eigen::Matrix2d R2;

        R1 << cos(theta1), -sin(theta1),
              sin(theta1),  cos(theta1);
        R2 << cos(theta2), -sin(theta2),
              sin(theta2),  cos(theta2);       
        
        Eigen::Vector2d p1_GCS;
        Eigen::Vector2d q1_GCS;
        Eigen::Vector2d p2_GCS;
        Eigen::Vector2d q2_GCS;

        p1_GCS = P1 + R1*p1;
        q1_GCS = P1 + R1*q1;
        p2_GCS = P2 + R2*p2;
        q2_GCS = P2 + R2*q2;

        Eigen::Matrix2d R;
        R << 0, -1,
             1,  0;

        Eigen::Vector2d M = R*R1*p1;
        Eigen::Vector2d N = R*R2*p2;

        if(joint->GetType() == 0)
        {
            if(link1_id == 0)
            {
                J(2*i, 3*link2_id - 3) = 1;
                J(2*i, 3*link2_id - 1) = N(0);

                J(2*i + 1, 3*link2_id - 2) = 1;
                J(2*i + 1, 3*link2_id - 1) = N(1);
            }
            
            else if(link2_id == 0)
            {
                J(2*i, 3*link1_id - 3) = -1;
                J(2*i, 3*link1_id - 1) = -M(0);

                J(2*i + 1, 3*link1_id - 2) = -1;
                J(2*i + 1, 3*link1_id - 1) = -M(1);
            }

            else
            {
                J(2*i, 3*link1_id - 3) = -1;
                J(2*i, 3*link1_id - 1) = -M(0);

                J(2*i, 3*link2_id - 3) = 1;
                J(2*i, 3*link2_id - 1) = N(0);

                J(2*i + 1, 3*link1_id - 2) = -1;
                J(2*i + 1, 3*link1_id - 1) = -M(1);

                J(2*i + 1, 3*link2_id - 2) = 1;
                J(2*i + 1, 3*link2_id - 1) = N(1);
            }
        }
        else if(joint->GetType() == 1)
        {

            Eigen::Vector2d vi = q1_GCS - p1_GCS;
            Eigen::Vector2d delP = p2_GCS - p1_GCS;
            Eigen::Vector2d delQP = q2_GCS - p2_GCS;
            Eigen::Vector2d vi_v = R*vi;

            if(link1_id == 0)
            {
                J(2*i, 3*link2_id - 3) = vi_v(0);
                J(2*i, 3*link2_id - 2) = vi_v(1);
                J(2*i, 3*link2_id - 1) = N.dot(vi_v);

                J(2*i + 1, 3*link2_id - 1) = (R*delQP).dot(vi_v);
            }
            
            else if(link2_id == 0)
            {
                J(2*i, 3*link1_id - 3) = -vi_v(0);
                J(2*i, 3*link1_id - 2) = -vi_v(1);
                J(2*i, 3*link1_id - 1) = (-M).dot(vi_v) + delP.dot(R*vi_v);

                J(2*i + 1, 3*link1_id - 1) = delQP.dot(R*vi_v);
            }

            else
            {
                J(2*i, 3*link1_id - 3) = -vi_v(0);
                J(2*i, 3*link1_id - 2) = -vi_v(1);
                J(2*i, 3*link1_id - 1) = (-M).dot(vi_v) + delP.dot(R*vi_v);

                J(2*i, 3*link2_id - 3) = vi_v(0);
                J(2*i, 3*link2_id - 2) = vi_v(1);
                J(2*i, 3*link2_id - 1) = N.dot(vi_v);

                J(2*i + 1, 3*link1_id - 1) = delQP.dot(R*vi_v);

                J(2*i + 1, 3*link2_id - 1) = (R*delQP).dot(vi_v);
            }
        }
    }
    return true;
}

// Implement this function
bool dspSolver::Make_J_dot(void) {
    /// make time derivative of Jacobian matrix  : J_dot
    int link_num = LinkConfig.GetNumLink();
    int joint_num = LinkConfig.GetNumJoint();
    for (int i = 0; i < joint_num; i++) 
    {
        dspJoint* joint = LinkConfig.GetJoint(i);

        int link1_id = joint->GetLink1ID();
        int link2_id = joint->GetLink2ID();

        dspLink* link1 = LinkConfig.GetLink(link1_id);
        dspLink* link2 = LinkConfig.GetLink(link2_id);

        Eigen::Vector2d p1;
        Eigen::Vector2d q1;
        Eigen::Vector2d p2;
        Eigen::Vector2d q2;

        joint->GetP1_Link1_LCS(p1);
        joint->GetQ1_Link1_LCS(q1);
        joint->GetP2_Link2_LCS(p2);
        joint->GetQ2_Link2_LCS(q2); 
        
        Eigen::Vector2d P1;
        Eigen::Vector2d P2;
        double theta1;
        double theta2;
        Eigen::Vector2d Pdot1;
        Eigen::Vector2d Pdot2;
        double omega1;
        double omega2;

        if(link1_id == 0)
        {
            P1.setZero();
            theta1 = 0;
            P2(0) = q(3*link2_id - 3);
            P2(1) = q(3*link2_id - 2);
            theta2 = q(3*link2_id - 1);
            Pdot2(0) = qdot(3*link2_id - 3);
            Pdot2(1) = qdot(3*link2_id - 2);
            omega2 = qdot(3*link2_id - 1);
        }
        else if(link2_id == 0)
        {
            P2.setZero();
            theta2 = 0;
            P1(0) = q(3*link1_id - 3);
            P1(1) = q(3*link1_id - 2);
            theta1 = q(3*link1_id - 1);
            Pdot1(0) = qdot(3*link1_id - 3);
            Pdot1(1) = qdot(3*link1_id - 2);
            omega1 = qdot(3*link1_id - 1);
        }
        else
        {
            P1(0) = q(3*link1_id - 3);
            P1(1) = q(3*link1_id - 2);
            theta1 = q(3*link1_id - 1);
            P2(0) = q(3*link2_id - 3);
            P2(1) = q(3*link2_id - 2);
            theta2 = q(3*link2_id - 1);

            Pdot1(0) = qdot(3*link1_id - 3);
            Pdot1(1) = qdot(3*link1_id - 2);
            omega1 = qdot(3*link1_id - 1);
            Pdot2(0) = qdot(3*link2_id - 3);
            Pdot2(1) = qdot(3*link2_id - 2);
            omega2 = qdot(3*link2_id - 1);
        }
        
        Eigen::Matrix2d R1;
        Eigen::Matrix2d R2;

        R1 << cos(theta1), -sin(theta1),
              sin(theta1),  cos(theta1);
        R2 << cos(theta2), -sin(theta2),
              sin(theta2),  cos(theta2);       
        
        Eigen::Vector2d p1_GCS;
        Eigen::Vector2d q1_GCS;
        Eigen::Vector2d p2_GCS;
        Eigen::Vector2d q2_GCS;

        p1_GCS = P1 + R1*p1;
        q1_GCS = P1 + R1*q1;
        p2_GCS = P2 + R2*p2;
        q2_GCS = P2 + R2*q2;

        Eigen::Matrix2d R;
        R << 0, -1,
             1,  0;

        Eigen::Vector2d M = R*R1*p1;
        Eigen::Vector2d N = R*R2*p2;
        Eigen::Vector2d dM = R*R*R1*p1;
        Eigen::Vector2d dN = R*R*R2*p2;

        if(joint->GetType() == 0)
        {
            if(link1_id == 0)
            {
                J_dot(2*i, 3*link2_id - 1) = omega2*dN(0);

                J_dot(2*i + 1, 3*link2_id - 1) = omega2*dN(1);
            }
            
            else if(link2_id == 0)
            {
                J_dot(2*i, 3*link1_id - 1) = -omega1*dM(0);

                J_dot(2*i + 1, 3*link1_id - 1) = -omega1*dM(1);
            }

            else
            {
                J_dot(2*i, 3*link1_id - 1) = -omega1*dM(0);

                J_dot(2*i, 3*link2_id - 1) = omega2*dN(0);

                J_dot(2*i + 1, 3*link1_id - 1) = -omega1*dM(1);

                J_dot(2*i + 1, 3*link2_id - 1) = omega2*dN(1);
            }
        }
        else if(joint->GetType() == 1)
        {

            Eigen::Vector2d vi = q1_GCS - p1_GCS;
            Eigen::Vector2d delP = p2_GCS - p1_GCS;
            Eigen::Vector2d delQP = q2_GCS - p2_GCS;
            Eigen::Vector2d vi_v = R*vi;
            Eigen::Vector2d dvi_v = R*vi_v;
            Eigen::Vector2d ddvi_v = R*dvi_v;

            if(link1_id == 0)
            {
                J_dot(2*i, 3*link2_id - 3) = omega1*dvi_v(0);
                J_dot(2*i, 3*link2_id - 2) = omega1*dvi_v(1);
                J_dot(2*i, 3*link2_id - 1) = (omega2*dN).dot(vi_v) + N.dot(omega1*dvi_v);

                J_dot(2*i + 1, 3*link2_id - 1) = (omega2*R*R*delQP).dot(vi_v) + (R*delQP).dot(omega1*dvi_v);
            }
            
            else if(link2_id == 0)
            {
                J_dot(2*i, 3*link1_id - 3) = -omega1*dvi_v(0);
                J_dot(2*i, 3*link1_id - 2) = -omega1*dvi_v(1);
                J_dot(2*i, 3*link1_id - 1) = (-omega1*dM).dot(vi_v) + (-M).dot(omega1*dvi_v) + (Pdot2 + omega2*N - Pdot1 - omega1*M).dot(dvi_v) + delP.dot(omega1*ddvi_v);

                J_dot(2*i + 1, 3*link1_id - 1) = (omega2*R*delQP).dot(dvi_v) + delQP.dot(omega1*ddvi_v);
            }

            else
            {
                J_dot(2*i, 3*link1_id - 3) = -omega1*dvi_v(0);
                J_dot(2*i, 3*link1_id - 2) = -omega1*dvi_v(1);
                J_dot(2*i, 3*link1_id - 1) = (-omega1*dM).dot(vi_v) + (-M).dot(omega1*dvi_v) + (Pdot2 + omega2*N - Pdot1 - omega1*M).dot(dvi_v) + delP.dot(omega1*ddvi_v);

                J_dot(2*i, 3*link2_id - 3) = omega1*dvi_v(0);
                J_dot(2*i, 3*link2_id - 2) = omega1*dvi_v(1);
                J_dot(2*i, 3*link2_id - 1) = (omega2*dN).dot(vi_v) + N.dot(omega1*dvi_v);

                J_dot(2*i + 1, 3*link1_id - 1) = (omega2*R*delQP).dot(dvi_v) + delQP.dot(omega1*ddvi_v);

                J_dot(2*i + 1, 3*link2_id - 1) = (omega2*R*R*delQP).dot(vi_v) + (R*delQP).dot(omega1*dvi_v);
            }
        }
    }
    return true;
}

// Implement this function
bool dspSolver::Make_F_ext(void) {
    /// make a n-dimensional vector for external force/torque : F_ext
    /// it contains x direction of force, y direction of force, and torque
    int link_num = LinkConfig.GetNumLink();
    int joint_num = LinkConfig.GetNumJoint();

    for(int i = 1; i < link_num; i++)
    {
        dspLink* link = LinkConfig.GetLink(i);
        double M = link->GetMass();
        F_ext(3*i -2) = -9.81 * M;
    }
    return true;
}

// Implement this function
bool dspSolver::CalcLinAlg(void) {
    /// calculate second derivative of vector q : qddot
    int link_num = LinkConfig.GetNumLink();
    int joint_num = LinkConfig.GetNumJoint();

    int size = 3*(link_num - 1) + 2*(joint_num);
    int size1 = 3*(link_num - 1);
    int size2 = 2*(joint_num);

    Eigen::MatrixXd MatLeft = Eigen::MatrixXd::Zero(size, size);
    Eigen::MatrixXd J_T = J.transpose();

    MatLeft.topLeftCorner(size1, size1) = M;
    MatLeft.topRightCorner(size1, size2) = J_T;
    MatLeft.bottomLeftCorner(size2, size1) = J;

    Eigen::MatrixXd MatLeft_inv = MatLeft.inverse();

    VecRight = Eigen::VectorXd::Zero(size);
    VecRight.head(size1) = F_ext;
    VecRight.tail(size2) = -J_dot * qdot;

    Eigen::VectorXd qddot_lambda = MatLeft_inv * VecRight;
    for(int i = 0; i < size1; i++)
    {
        qddot(i) = qddot_lambda(i);
    }
    return true;
}

// Implement this function
bool dspSolver::UpdateCurrentInfo(void) {
    /// load and update information what you need in class variables
    /// current vector q and its derivative should be loaded : q, q_dot
    int link_num = LinkConfig.GetNumLink();
    int joint_num = LinkConfig.GetNumJoint();

    for(int i = 1; i < link_num; i++)
    {
        dspLink* link = LinkConfig.GetLink(i);
        link->GetQ(q(3*i - 3), q(3*i - 2), q(3*i - 1));
        link->GetQDot(qdot(3*i - 3), qdot(3*i - 2), qdot(3*i - 1));
    }
    Make_M();
    Make_J();
    Make_J_dot();
    Make_F_ext();
    return true;
}

// Implement this function
bool dspSolver::UpdateNextInfo(double timestep) {
    /// update vector q and its derivative : q, q_dot
    /// also save them in class variables for nex step
    //std::cout << qddot(0) << std::endl;
    qdot = qdot + timestep * qddot;
    q = q + timestep * qdot;
    SetQDot();
    SetQ();
    return true;
}

// Implement this function
bool dspSolver::SetQDot(void) {
    /// update new q_dot information in each link structure
    int link_num = LinkConfig.GetNumLink();
    int joint_num = LinkConfig.GetNumJoint();
    for(int i = 1; i < link_num; i++)
    {
        dspLink* link = LinkConfig.GetLink(i);
        link->SetQDot(qdot(3*i - 3), qdot(3*i - 2), qdot(3*i - 1));
    }
    return true;
}

// Implement this function
bool dspSolver::SetQ(void) {
    /// update new q information in each link structure
    int link_num = LinkConfig.GetNumLink();
    int joint_num = LinkConfig.GetNumJoint();
    for(int i = 1; i < link_num; i++)
    {
        dspLink* link = LinkConfig.GetLink(i);
        link->SetQ(q(3*i - 3), q(3*i - 2), q(3*i - 1));
    }
    return true;
}

// Do not change this function
bool dspSolver::GetQ(Eigen::VectorXd& q_record) {
    /// get current q information
    q_record = q;
    return true;
}

// Do not change this function
bool dspSolver::GetQDot(Eigen::VectorXd& qdot_record) {
    /// get current q_dot information 
    qdot_record = qdot;
    return true;
}
