//===========================================================================


#include "InverseKinematics.h"

#include <pmp/mat_vec.h>
#if __APPLE__
#else
#if _OPENOMP
#include <omp.h>
#endif
#endif
#include <iostream>
#include <cfloat>
#include <fstream>

#include "PropertyNames.h"
#include "Types.h"


//===========================================================================
namespace pmp_character {
//===========================================================================


Inverse_kinematics::
Inverse_kinematics(double lambda)
    : skeleton_(nullptr),
      lambda_(lambda),
      clamp_(false)
{
    map_effectors_ = std::map<std::string, std::vector<Inverse_kinematics::Effector *> >();

    effectors_ = std::vector<Inverse_kinematics::Effector *>();
}


//-----------------------------------------------------------------------------


Inverse_kinematics::
~Inverse_kinematics()
{
    clear_effectors();
}


//-----------------------------------------------------------------------------

bool
Inverse_kinematics::set_targets_and_positions_by_vertex(
    const std::vector<pmp::vec3>& targets,
    const std::vector<pmp::vec3>& source,
    const std::vector<pmp::Vertex>& positions, pmp::SurfaceMesh& mesh
) {
    auto vdepends = mesh.get_vertex_property<ivec8>(properties::vertexSkinningDepends);
    auto vweights = mesh.get_vertex_property<vec8>(properties::vertexSkinningWeights);
    if ((targets.size() != positions.size()))
    {
        std::cerr << "Inverse_kinematics::set_targets_and_positions: [ERROR] #targets does not match #positions." << std::endl;
        return false;
    }

    clear_effectors();

    auto global_position = mesh.get_vertex_property<pmp::vec3>(properties::vertexGlobalPosition);
    for (size_t i = 0; i < positions.size(); ++i)
    {
        // find joint closest to i-th position
        auto v = positions[i];
        Joint* closest_ik_joint = NULL;
        closest_ik_joint = skeleton_->get_joint(vdepends[v][0]);

        if (closest_ik_joint != NULL)
        {
            double weight = 1.0;
            add_effector(closest_ik_joint, source[i], targets[i], weight);
        }
    }
    return true;
}


bool
Inverse_kinematics::set_targets_and_positions(const std::vector<pmp::Point>& targets,
                                              const std::vector<pmp::Point>& positions,
                                              const std::vector<double>& weights)
{
    bool weights_available = weights.size() > 0;

    if ((targets.size() != positions.size()) ||
        (weights_available && (targets.size() != weights.size())))
    {
        std::cerr << "Inverse_kinematics::set_targets_and_positions: [ERROR] #targets does not match #positions." << std::endl;
        return false;
    }

    clear_effectors();

    Joint* closest_ik_joint;

    for (size_t i = 0; i < positions.size(); ++i)
    {
        // find joint closest to i-th position
        closest_ik_joint = NULL;
        float closest_dist = FLT_MAX;

        for (size_t j = 0; j <  skeleton_->joints_.size(); ++j)
        {
            Joint* ik_joint = skeleton_->joints_[j];

            const pmp::Point& joint_position = ik_joint->get_global_translation();
            const float dist = norm(joint_position - positions[i]);

            if (dist < closest_dist)
            {
                closest_ik_joint = ik_joint;
                closest_dist = dist;
            }
        }

        if (closest_ik_joint != NULL)
        {
            double weight = weights_available ? weights[i] : 1.0;
            add_effector(closest_ik_joint, positions[i], targets[i], weight);
        }
    }

    return true;
}

bool
Inverse_kinematics::
set_targets_and_positions_by_index(
    const std::vector<pmp::Point>& targets,
    const std::vector<int>& joint_indices,
    const std::vector<double>& weights
) {
    bool weights_available = !weights.empty();

    if ((targets.size() != joint_indices.size()) ||
        (weights_available && (targets.size() != weights.size())))
    {
        std::cerr << "Inverse_kinematics::set_targets_and_positions: [ERROR] #targets does not match #positions." << std::endl;
        return false;
    }

    clear_effectors();

    Joint* closest_ik_joint;

    for (size_t i = 0; i < joint_indices.size(); ++i)
    {
        // take the position of the joint specified
        closest_ik_joint = skeleton_->get_joint(joint_indices[i]);

        if (closest_ik_joint != nullptr)
        {
            double weight = weights_available ? weights[i] : 1.0;
            add_effector(closest_ik_joint, closest_ik_joint->get_global_translation(), targets[i], weight);
        }
    }

    return true;
}

//-----------------------------------------------------------------------------


bool
Inverse_kinematics::
solve_IK()
{
    //--- Step 1: collect effectors ---
    //--- Step 2: collect degrees of freedom ---
    if (collect_dofs())
    {
        if (setup())
        {
            // solve linear system for the update dtheta
            if (compute_update())
            {
                // apply update dtheta to skeleton matrices
                apply_update();
            }
        }
    }

    return true;
}


//-----------------------------------------------------------------------------
long Inverse_kinematics::getNumberOfDofs()
{
    long n_dofs = 0;
    for (auto& [joint, dof] : dofs_)
    {
        // add number of dofs of joint to overall number of dofs.
        n_dofs += dof.size();
    }
    return n_dofs;
}

bool
Inverse_kinematics::
setup()
{
    // number of degrees of freedom
    size_t n_dofs = getNumberOfDofs();
    // how many constraints? three per effector (x,y,z)
    const long n_targets = 3 * effectors_.size();



    // if #constraints or #dofs is zero -> stop
    if (n_targets * n_dofs == 0)
    {
        std::cerr << "IK: matrix is degenerate!\n";
        return false;
    }

    // Jacobian matrix and vector of errors
    errors_   = Eigen::VectorXd::Zero(n_targets);
    // Jacobian has degrees of freedom + 3 as we use the translation as well
    jacobian_ = Eigen::MatrixXd::Zero(n_targets, n_dofs);

    // iterate over constraints
    const std::vector<Inverse_kinematics::Effector*>& effectors = effectors_;

    for(long i = 0; i < static_cast<long>(effectors.size()); ++i)
    {
        const Inverse_kinematics::Effector* e = effectors[i];

        const pmp::Point& position = e->position_;
        const pmp::Point& target   = e->target_;

        // compute errors
        errors_(3 * i + 0) = e->weight_ * (target[0] - position[0]);
        errors_(3 * i + 1) = e->weight_ * (target[1] - position[1]);
        errors_(3 * i + 2) = e->weight_ * (target[2] - position[2]);

        // go up kinematic chain, add entries to Jacobian
        const Joint* joint = e->joint_;
        while (joint != nullptr)
        {
            // get all local axes defined for this joint
            const std::vector<Inverse_kinematics::DoF>& joint_dofs = dofs_[joint->name_];

            // get rotation center of current joint
            const pmp::Point& c = joint->get_global_translation();

            for (const auto& dof : joint_dofs)
            {
                // rotate rotation axis of joint by its global transformation
                const pmp::Point axis = pmp::linear_transform(joint->global_, dof.axis_);

                // calculate gradient
                pmp::Point grad(0);
                if(dof.is_translation)
                {
                    grad = axis;
                } else
                {
                    grad = e->weight_ * cross(axis, position - c);
                }

                // update Jacobian matrix
                jacobian_(3 * i + 0, dof.index_) = grad[0];
                jacobian_(3 * i + 1, dof.index_) = grad[1];
                jacobian_(3 * i + 2, dof.index_) = grad[2];
            }

            joint = joint->parent_;
        }
    }

    return true;
}


//-----------------------------------------------------------------------------


Eigen::MatrixXd
Inverse_kinematics::
damping_matrix()
{
    const long n_dofs = getNumberOfDofs();

    Eigen::MatrixXd invW = Eigen::MatrixXd::Identity(n_dofs, n_dofs);

    const double e = 1e-5;
    if (hprev_.size() != n_dofs)
    {
        hprev_ = std::vector<double>(n_dofs, e);
    }
    for (int i = 0; i < n_dofs; ++i)
    {
        const Inverse_kinematics::DoF& info = get_joint_info(i);
        if (info.index_ == -1)
        {
            std::cerr << "Inverse_kinematics::dampling_matrix: [ERROR] info.index_ == -1" << std::endl;
            exit(1);
        }

        if (!info.has_limits_)
            continue;

        double qmax = info.max_angle;
        double qmin = info.min_angle;

        double q = current_[i];

        // DEBUG
        //const float three_deg_in_radians = 3.0f * M_PI / 180.0f;
        //const float to_degrees = 180.0f / M_PI;
        //if (qmax != qmin &&
        //   qmax - qmin >= three_deg_in_radians &&
        //   (std::abs(qmax - q) < three_deg_in_radians ||
        //    std::abs(qmin - q) < three_deg_in_radians))
        //{
        //   std::cerr << "[WARNING] Joint " << info.joint_->name_ << " approaches limits on axis " << info.axis_ << std::endl;
        //   std::cerr << "[WARNING] Joint " << qmin * to_degrees << " < " << q * to_degrees << " < " << qmax * to_degrees << std::endl;
        //}

        double h = fabs( std::pow(qmax - qmin, 2) * (2 * q - qmax - qmin)
                    / ( 4 * std::pow(qmax - q, 2) * std::pow(q - qmin, 2) ) );

        double delta = h - hprev_[i];
        hprev_[i] = h;

        invW(i, i) = delta >= 0 ? (1.0 + h) : 1.0;
    }

    // apply damping
    for (int i = 0; i < jacobian_.cols(); ++i)
    {
        invW(i, i) *= lambda_;
    }

    return invW;
}


//-----------------------------------------------------------------------------


bool
Inverse_kinematics::
compute_update()
{
    dtheta_.setZero();

    const unsigned int n = jacobian_.cols();
    const Eigen::MatrixXd jacobianT = jacobian_.transpose();

    // coompute largest eigenvalue, note: set lambda to this value!
    // Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian_, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // Eigen::VectorXd singular_values = svd.singularValues();
    // std::cout << "IK Jocobi matrix largest singular value = " << singular_values(0) << std::endl;

    // joint limit avoidance
    if (clamp_)
    {
        // solve system with regularized damped pseudo-inverse:
        //  dt = (J^T * J + lambda*W)^-1 * J^T * e
        auto dampingMatrix = damping_matrix();
        Eigen::MatrixXd DLS = jacobianT * jacobian_ + dampingMatrix;
        dtheta_ = DLS.llt().solve( jacobianT * errors_ );
    }
    else // standard solution
    {
        // solve system with damped pseudo-inverse:
        //  dt = (J^T * J + lambda*I)^-1 * J^T * e
        Eigen::MatrixXd DLS = jacobianT * jacobian_ + lambda_ * Eigen::MatrixXd::Identity(n, n);
        dtheta_ = DLS.llt().solve( jacobianT * errors_ );
    }
    return true;
}


//-----------------------------------------------------------------------------


bool
Inverse_kinematics::
apply_update()
{
    // go through map of all DoFs
    for (auto const &jdofs : dofs_)
    {
        // go through all DoFs of one joint
        for (auto const &dof : jdofs.second)
        {
            if(dof.is_translation)
            {
                dof.joint_->local_ =  dof.joint_->local_ * translation_matrix(dof.axis_ * dtheta_[dof.index_]);
                // dof.joint_->local_ =  dof.joint_->local_ * translation_matrix(pmp::linear_transform(dof.joint_->local_, dof.axis_) * dtheta_[dof.index_]);
                // std::cout << "Translation: " << dtheta_[dof.index_] << std::endl;
            } else
            {
                // compute rotation matrix
                pmp::mat4 rot = rotation_matrix(
                    dof.axis_, static_cast<float>(dtheta_[dof.index_] * 180.0 / M_PI));

                // apply it to local transformation
                dof.joint_->local_ = dof.joint_->local_ * rot;
            }

        }
    }

    // update forward kinematics (do not call update() to avoid skinning
    // computation for characters)
    skeleton_->forward_kinematics();

    return true;
}


//-----------------------------------------------------------------------------


void
Inverse_kinematics::
move_effectors()
{
    #if 0
    for (size_t i = 0; i < skeleton_->joints_.size(); ++i)
    {
        Joint* joint = skeleton_->joints_[i];

        if (has_effectors(joint->name_))
        {
            std::vector<Inverse_kinematics::Effector *>& es = map_effectors_[joint->name_];

            const mat4& transf = joint->global_;

            for (size_t j = 0; j < es.size(); ++j)
            {
                const vec3& offset = es[j]->offset_;

                es[j]->position_ = linear_transform(transf, offset);
            }
        }
    }
    #endif
}


//-----------------------------------------------------------------------------


void
Inverse_kinematics::
add_effector(Joint* joint,
             const pmp::Point& position,
             const pmp::Point target,
             const double weight)
{
    Effector* effector = new Effector(joint, position, target, weight);

    std::string name = effector->joint_->name_;

    map_effectors_[name].push_back(effector);
    effectors_.push_back(effector);
}


//-----------------------------------------------------------------------------


void
Inverse_kinematics::
clear_effectors()
{
    if (effectors_.size() == 0)
    {
        return;
    }

    for (size_t i = 0; i < effectors_.size(); ++i)
    {
        delete effectors_[i];
    }

    effectors_.clear();
    map_effectors_.clear();
}


//-----------------------------------------------------------------------------


bool
Inverse_kinematics::
has_effectors(const std::string &joint)
{
    return map_effectors_.find(joint) != map_effectors_.end();
}


//-----------------------------------------------------------------------------


bool
Inverse_kinematics::
set_parameters(const std::vector<double>& parameters, const pmp::Point& translation)
{
    reset();

    if (!move(parameters, translation))
    {
        std::cerr << "Inverse_kinematics::set: [ERROR] !move(parameters)" << std::endl;
        return false;
    }
    // TODO Check if needed
    skeleton_->root_->forward_kinematics();

    // move effectors of the skeleton
    move_effectors();

    return true;
}


//-----------------------------------------------------------------------------


bool
Inverse_kinematics::
move(const std::vector<double>& parameters, const pmp::Point& translation)
{
    if (parameters.size() != current_.size())
    {
        std::cerr << "Inverse_kinematics::move: [ERROR] parameters.size() != current_.size()" << std::endl;
        std::cerr << "parameters.size(): "<< parameters.size() << std::endl;
        std::cerr << "current_.size(): " << current_.size() << std::endl;
        return false;
    }

    const size_t n_dofs = current_.size();

    // separate rotation axes and pose parameters
    std::vector<double> rotateX(n_dofs, 0); // flexion
    std::vector<double> rotateZ(n_dofs, 0); // abduction
    std::vector<double> rotateY(n_dofs, 0); // twist

    for (size_t i = 0; i < n_dofs; ++i)
    {
        const DoF& info = get_joint_info(i);
        if (info.index_ == -1)
        {
            std::cerr << "Inverse_kinematics::move: [ERROR] info.index_ == -1" << std::endl;
            return false;
        }

        const pmp::Point& axis = info.axis_;
        if (axis == pmp::Point(1,0,0))
        {
            rotateX[i] = parameters[i];    // flexion
        }
        else if (axis == pmp::Point(0,1,0))
        {
            rotateY[i] = parameters[i];    // abduction
        }
        else if (axis == pmp::Point(0,0,1))
        {
            rotateZ[i] = parameters[i];    // twist
        }
        else
        {
            rotateY[i] = parameters[i];    // other

        }
    }

    // transform joints separately
    transform_joints(rotateX); // flexion
    transform_joints(rotateZ); // abduction
    transform_joints(rotateY); // twist

    skeleton_->root_->global_(0,3) = translation[0];
    skeleton_->root_->global_(1,3) = translation[1];
    skeleton_->root_->global_(2,3) = translation[2];

    return true;
}


//-----------------------------------------------------------------------------


void
Inverse_kinematics::
reset()
{
    for (size_t i = 0; i < skeleton_->joints_.size(); ++i)
    {
        Joint* joint = skeleton_->joints_[i];

        if (joint != NULL)
        {
            joint->local_ = joint->local_initial_;
        }
    }

    current_ = std::vector<double>(current_.size(), 0);
}


//-----------------------------------------------------------------------------


const std::vector<double>&
Inverse_kinematics::
get_current_parameters()
{
    return current_;
}


//-----------------------------------------------------------------------------


void
Inverse_kinematics::
transform_joints(const std::vector<double>& parameters)
{
    // transform joints specified in parameters
    for (size_t i = 0; i < parameters.size(); ++i)
    {
        const Inverse_kinematics::DoF& info = get_joint_info(i);
        if (info.index_ == -1)
        {
            std::cerr << "Inverse_kinematics::transform_joints: [ERROR] info.index_ == -1" << std::endl;
            exit(1);
        }

        Joint* joint = info.joint_;
        if (joint == NULL)
            continue;

        const double p_rad = parameters[i];
        const double p_deg = p_rad * (180.0f/M_PI);

        joint->local_ = joint->local_ *  pmp::rotation_matrix(info.axis_, pmp::Scalar(p_deg));

        current_[i] += p_rad;
    }
}


//-----------------------------------------------------------------------------


const Inverse_kinematics::DoF&
Inverse_kinematics::
get_joint_info(int index) const
{
    for (const auto& vec_dofs : dofs_)
    {
        for (const auto& dof : vec_dofs.second)
        {
            if (dof.index_ == index)
            {
                return dof;
            }
        }
    }
    return invalid_dof;
}

std::vector<double> Inverse_kinematics::getFullParameterList()
{
    std::vector<double> angles;
    std::map<Joint*, std::vector<double>> perJointParams;
    for(size_t i =0; i < current_.size(); i++)
    {
        auto info = get_joint_info(i);
        // This joint has params
        perJointParams[info.joint_].emplace_back(current_[i]);
    }
    for(auto joint : skeleton_->joints_)
    {
        // Get the joint info
        // If joint info has params take them]
        if(perJointParams.contains(joint))
        {
            for(auto angle : perJointParams[joint])
            {
                angles.emplace_back(angle);
            }
        } else
        {
            angles.emplace_back(0);
            angles.emplace_back(0);
            angles.emplace_back(0);
        }
    }
    // current_ = angles;
    return angles;
}

std::map<std::string, std::vector<Inverse_kinematics::DoF>> Inverse_kinematics::getDofs()
{
    return dofs_;
}

std::map<std::string, std::pair<pmp::Point, pmp::Point>> Inverse_kinematics::getLimits()
{
    return joint_limits_;
}

//-----------------------------------------------------------------------------


std::vector<double>
Inverse_kinematics::
constrain(const std::vector<double>& param) const
{
    std::vector<double> result(param.size());

    for (size_t i = 0; i < param.size(); ++i)
    {
        const Inverse_kinematics::DoF& info = get_joint_info(i);
        if (info.index_ == -1)
        {
            std::cerr << "Inverse_kinematics::constrain: [ERROR] info.index_ == -1" << std::endl;
            exit(1);
        }

        double qmax = info.max_angle;
        double qmin = info.min_angle;

        const double q = param[i];
        const double e = 0.001;

        if (q < qmin)
        {
            result[i] = qmin + e;
        }
        else if (q > qmax)
        {
            result[i] = qmax - e;
        }
        else
        {
            result[i] = q;
        }
    }

    return result;
}


//-----------------------------------------------------------------------------


bool
Inverse_kinematics::
load_dof(const std::string& filename)
{
    std::ifstream ifs(filename.c_str());
    if (!ifs)
    {
        std::cerr << "Inverse_kinematics::load_dof: [ERROR] Could not load DoF for IK. Filename: \"" << filename << "\"." << std::endl;
        return false;
    }

    free_joints_.clear();
    joint_limits_.clear();

    std::stringstream ss;
    while (ifs)
    {
        std::string line;
        std::getline(ifs, line);
        if (!ifs)
            break;

        if (line.find("#") == 0 || line.find("//") == 0)
            continue;

        ss.clear();
        ss.str(line);

        std::string joint_name;
        ss >> joint_name;

        free_joints_.push_back(joint_name);

        if (ss.str().size() > joint_name.size())
        {
            std::pair<pmp::Point, pmp::Point>& p = joint_limits_[joint_name];
            pmp::Point& lmin = p.first;
            pmp::Point& lmax = p.second;
            for (int i = 0; i < 3; ++i)
            {
                ss >> lmin[i];
            }
            for (int i = 0; i < 3; ++i)
            {
                ss >> lmax[i];
            }
            //convert from degree to radians
            lmax *= M_PI / 180.0f;
            lmin *= M_PI / 180.0f;
            lmin *= -1;
        }
    }

    ifs.close();

    return true;
}


//-----------------------------------------------------------------------------
    template <typename Scalar>
    pmp::Vector<Scalar,3>
    to_euler_XYZ(const pmp::Mat4<Scalar> &mat)
{
    Scalar angle_x,angle_y,angle_z,C,D,tx,ty;

    pmp::Mat4<Scalar> tmat = pmp::transpose(mat);

    angle_y = D =  asin( tmat[2]);        /* Calculate Y-axis angle */
    C           =  cos( angle_y );
    //angle_y    *=  RADIANS;
    if ( fabs( C ) > 0.005 )             /* Gimball lock? */
    {
        tx      =  tmat[10] / C;           /* No, so get X-axis angle */
        ty      = -tmat[6]  / C;
        angle_x  = atan2( ty, tx );// * RADIANS;
        tx      =  tmat[0] / C;            /* Get Z-axis angle */
        ty      = -tmat[1] / C;
        angle_z  = atan2( ty, tx );// * RADIANS;
    }
    else                                 /* Gimball lock has occurred */
    {
        angle_x  = 0;                      /* Set X-axis angle to zero */
        tx      =  tmat[5];                 /* And calculate Z-axis angle */
        ty      =  tmat[4];
        angle_z  = atan2( ty, tx );// * RADIANS;
    }
    return pmp::Vector<Scalar, 3>(angle_x, angle_y, angle_z);
}

bool
Inverse_kinematics::
collect_dofs()
{
    if (free_joints_.empty())
    {
        std::cerr << "Inverse_kinematics::collect_dofs: [ERROR] No free joints!" << std::endl;
        return false;
    }

    current_.clear();

    dofs_.clear();

    int dof_idx = 0;

    const pmp::vec3 X_AXIS(1,0,0);
    const pmp::vec3 Y_AXIS(0,1,0);
    const pmp::vec3 Z_AXIS(0,0,1);
    const std::array<const pmp::vec3,3> AXES = {X_AXIS, Y_AXIS, Z_AXIS};

    for (size_t i = 0; i < free_joints_.size(); ++i)
    {
        Joint* joint = skeleton_->get_joint(free_joints_[i]);

        pmp::Mat4<pmp::Scalar> m1 = joint->local_;
        m1[12] = m1[13] = m1[14] = 0.0f;
        pmp::Mat4<pmp::Scalar> m2 = joint->local_initial_;
        m2[12] = m2[13] = m2[14] = 0.0f;

        pmp::Point current_joint_angles = to_euler_XYZ(pmp::inverse(m2) * m1);

        std::vector<Inverse_kinematics::DoF>& joint_dofs = dofs_[free_joints_[i]];
        if (joint_limits_.count(free_joints_[i]) > 0) // there are joint limits
        {
            const std::pair<pmp::Point, pmp::Point>& limits = joint_limits_[free_joints_[i]];
            const pmp::Point& min_angles = limits.first;
            const pmp::Point& max_angles = limits.second;
            for(int axis = 0; axis < 3; axis++)
            {
                if(min_angles[axis] != 0 and max_angles[axis] != 0)
                {
                    current_.emplace_back(current_joint_angles[axis]);
                    joint_dofs.emplace_back(dof_idx++, joint, AXES[axis], min_angles[axis], max_angles[axis]);
                }
            }
            if(joint == skeleton_->root_)
            {
                auto translation = joint->get_global_translation();
                current_.emplace_back(translation[0]);
                current_.emplace_back(translation[1]);
                current_.emplace_back(translation[2]);
                joint_dofs.emplace_back(dof_idx++, joint, X_AXIS, -0.2, +0.2, true);
                joint_dofs.emplace_back(dof_idx++, joint, Y_AXIS, -0.2, +0.2, true);
                joint_dofs.emplace_back(dof_idx++, joint, Z_AXIS, -0.2, +0.2, true);

            }
        }
        else // or no joint limits
        {
            joint_dofs.emplace_back(dof_idx++, joint, X_AXIS);
            joint_dofs.emplace_back(dof_idx++, joint, Y_AXIS);
            joint_dofs.emplace_back(dof_idx++, joint, Z_AXIS);
            current_.emplace_back(current_joint_angles[0]);
            current_.emplace_back(current_joint_angles[1]);
            current_.emplace_back(current_joint_angles[2]);
        }
    }

    //current_.resize(dof_idx, 0);

    return true;
}


//-----------------------------------------------------------------------------


void
Inverse_kinematics::
set_skeleton(Skeleton* skeleton)
{
    skeleton_ = skeleton;
}

void Inverse_kinematics::load_max_dof()
{
    for(auto joint : skeleton_->joints_)
    {
        free_joints_.push_back(joint->name_);
        std::pair<pmp::Point, pmp::Point>& p = joint_limits_[joint->name_];
        pmp::Point& lmin = p.first;
        lmin = -pmp::Point(M_PI);
        pmp::Point& lmax = p.second;
        lmax = pmp::Point(M_PI);
        //convert from degree to radians
    }

}


//===========================================================================
} // namespace ik
//===========================================================================
