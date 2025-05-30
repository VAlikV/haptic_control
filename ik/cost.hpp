#ifndef COST
#define COST

#include <Eigen/Dense>

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/common/find_resource.h>
#include <drake/math/rigid_transform.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/common/drake_assert.h>
#include <drake/geometry/scene_graph.h>
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/solvers/solve.h"
#include "drake/math/rotation_matrix.h"

class PotentialEnergyCost {
    public:
        PotentialEnergyCost(const drake::multibody::MultibodyPlant<double>* plant, std::unique_ptr<drake::systems::Context<double>> context):
        plant_(plant), 
        context_(std::move(context))
        {

        }
    
        int numInputs() const 
        {
            return plant_->num_positions();
        }
    
        int numOutputs() const
        {
            return 1;
        }
    
        // Шаблонный eval — поддерживает и double, и AutoDiffScalar
        template <typename T>
        void eval(const Eigen::Ref<const Eigen::Matrix<T, -1, 1>>& q, Eigen::Matrix<T, -1, 1>* y) const 
        {
            drake::multibody::MultibodyPlant<T> plant_autodiff(0.0);
            drake::multibody::Parser(&plant_autodiff).AddModels("../robots/iiwa.urdf");
            plant_autodiff.Finalize();

            std::unique_ptr<drake::systems::Context<T>> context_autodiff =
                plant_autodiff.CreateDefaultContext();

            plant_autodiff.SetPositions(context_autodiff.get(), q);
            *y = Eigen::Matrix<T, -1, 1>::Constant(1, -plant_autodiff.CalcPotentialEnergy(*context_autodiff));
        }
    
    private:
        const drake::multibody::MultibodyPlant<double>* plant_;
        std::unique_ptr<drake::systems::Context<double>> context_;
    };

#endif