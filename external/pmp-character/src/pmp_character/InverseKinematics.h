#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <Eigen/Geometry>
#include <Eigen/Dense>


#include <map>
#include <vector>

#include <pmp/surface_mesh.h>

#include "Joint.h"
#include "Skeleton.h"
#include <pmp/surface_mesh.h>


namespace pmp_character {

class Inverse_kinematics
{
private:

    struct Effector
    {
        Effector(Joint* joint,
                 const pmp::Point& position,
                 const pmp::Point& target,
                 double weight = 1.0)
            : joint_(joint), position_(position), target_(target), weight_(weight)
        {
            // compute offset
            pmp::Vector<pmp::Scalar, 3> dir = position - joint_->get_global_translation();
            offset_ = linear_part(inverse(joint_->global_)) * dir;
        }

        Joint* joint_;
        pmp::Point position_;
        pmp::Point target_;
        pmp::Point offset_;
        double          weight_;
    };

    struct DoF
    {
        DoF()
            : index_(-1), joint_(nullptr), axis_(pmp::vec3(0)),
              min_angle(-2*M_PI),
              max_angle(2*M_PI),
              has_limits_(false)
        {}

        DoF(int idx,
            Joint* j,
            const pmp::vec3& a)
            : index_(idx), joint_(j), axis_(a),
              min_angle(-2*M_PI),
              max_angle(2*M_PI),
              has_limits_(false)
        {}

        DoF(int idx,
            Joint* j,
            const pmp::vec3& a,
            float min,
            float max)
            : index_(idx), joint_(j), axis_(a),
              min_angle(min), max_angle(max),
              has_limits_(true)
        {}
        DoF(int idx,
                Joint* j,
                const pmp::vec3& a,
                float min,
                float max,
                bool _is_translation)
                : index_(idx), is_translation(_is_translation),
                  joint_(j), axis_(a),
                  min_angle(min), max_angle(max),
                  has_limits_(true)
        {}
        /// the index of this DoF (column index for Jacobian matrix)
        int index_;

        bool is_translation = false;
        /// the joint
        Joint* joint_;

        /// the rotation axis (x, y, or z) of this DoF
        pmp::Point axis_;

        double min_angle;
        double max_angle;

        bool has_limits_;
    };

    DoF invalid_dof;

public:

    Inverse_kinematics(double lambda = 1.0);

    ~Inverse_kinematics();

    bool set_targets_and_positions(const std::vector<pmp::Point>& targets,
                                   const std::vector<pmp::Point>& positions,
                                   const std::vector<double>& weights = std::vector<double>());

    bool set_targets_and_positions_by_index(const std::vector<pmp::Point>& targets,
                                            const std::vector<int>& joint_indices,
                                            const std::vector<double>& weights = std::vector<double>());
    bool set_targets_and_positions_by_vertex(const std::vector<pmp::vec3>& targets,
                                                                const std::vector<pmp::vec3>& source,
                                                                const std::vector<pmp::Vertex>& positions, pmp::SurfaceMesh& mesh);
    bool solve_IK();
    long getNumberOfDofs();

    void set_clamping(bool clamp) { clamp_ = clamp; }

    void set_lambda(double lambda) { lambda_ = lambda; }

    void clear_effectors();

    const std::vector<double>& get_current_parameters();

    bool set_parameters(const std::vector<double>& parameters, const pmp::Point& translation = pmp::Point(0));

    std::vector<double> constrain(const std::vector<double>& param) const;

    bool load_dof(const std::string& filename);

    bool collect_dofs();

    void set_skeleton(Skeleton* skeleton);

    void load_max_dof();

    std::vector<double> getFullParameterList();

    std::map<std::string, std::vector<Inverse_kinematics::DoF>> getDofs();
    std::map<std::string, std::pair<pmp::Point, pmp::Point>> getLimits();
    std::vector<Effector *> effectors_;
    std::vector<std::string> free_joints_;
    const DoF& get_joint_info(int index) const;

private:


    void add_effector(Joint* joint,
                      const pmp::Point& position,
                      const pmp::Point target,
                      const double weight = 1.0);

    bool has_effectors(const std::string &joint);

    void reset();

    void transform_joints(const std::vector<double>& parameters);

    bool setup();
    Eigen::MatrixXd damping_matrix();
    bool compute_update();
    bool apply_update();

    void move_effectors();

    bool move(const std::vector<double>& parameters, const pmp::Point& translation = pmp::Point(0));


private:

    Skeleton* skeleton_;

    // mapping from joint name to vector of effectors
    std::map<std::string, std::vector<Effector *> > map_effectors_;

    double lambda_;

    Eigen::VectorXd errors_;
    Eigen::MatrixXd jacobian_;

    // keep track of current pose parameters for joint limit avoidance
    std::vector<double> current_;

    /// update to joint angles
    Eigen::VectorXd dtheta_;

    bool clamp_;

    std::vector<double> hprev_;

    // mapping from joint name to vector of DoFs
    // todo: map this via joint->id
    std::map<std::string, std::vector<DoF>> dofs_;

    // free joints as specified by filename when load_dof(filename) is called
    // joint limits as specified by filename when load_dof(filename) is called
    // joint_limits_ may be larger than free_joints since the free joints
    // may or may not have joint limits
    std::map<std::string, std::pair<pmp::Point, pmp::Point>> joint_limits_;


public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

} // namespace ik

#endif
