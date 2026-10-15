#include <iostream>
#include <memory>
#include <stdexcept>

#include <ignition/gazebo/System.hh>
#include <ignition/math/Rand.hh>
#include <ignition/plugin/Register.hh>
#include <sdf/Element.hh>

namespace linorobot2_gazebo
{
class G4RandomSeed final : public ignition::gazebo::System,
                           public ignition::gazebo::ISystemConfigure
{
  public: void Configure(
    const ignition::gazebo::Entity &,
    const std::shared_ptr<const sdf::Element> &sdf,
    ignition::gazebo::EntityComponentManager &,
    ignition::gazebo::EventManager &) override
  {
    if (!sdf || !sdf->HasElement("seed")) {
      throw std::runtime_error("G4 random-seed system requires a <seed>");
    }

    const auto seed = sdf->Get<unsigned int>("seed");
    ignition::math::Rand::Seed(seed);
    std::clog << "G4_RANDOM_SEED=" << seed << '\n';
  }
};
}  // namespace linorobot2_gazebo

IGNITION_ADD_PLUGIN(
  linorobot2_gazebo::G4RandomSeed,
  ignition::gazebo::System,
  ignition::gazebo::ISystemConfigure)
IGNITION_ADD_PLUGIN_ALIAS(
  linorobot2_gazebo::G4RandomSeed,
  "linorobot2_gazebo::G4RandomSeed")