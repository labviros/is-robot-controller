#ifndef __ROBOT_PARAMETERS_HPP__
#define __ROBOT_PARAMETERS_HPP__

#include <yaml-cpp/yaml.h>
#include <armadillo>
namespace YAML {
template <>
struct convert<arma::vec> {
  static Node encode(arma::vec const& vector) {
    Node node;
    for (const auto& value : vector) {
      node.push_back(value);
    }
    return node;
  }

  static bool decode(Node const& node, arma::vec& vector) {
    if (!node.IsSequence()) {
      return false;
    }
    std::vector<double> values;
    for (unsigned int i = 0; i < node.size(); ++i) {
      values.push_back(node[i].as<double>());
    }
    vector = arma::vec(values);
    return true;
  }
};
}

namespace robot {
struct Parameters {
  double center_offset;
  arma::vec L;
  arma::vec K;
  arma::vec fence;
  double w_max;

  Parameters() {}
  Parameters(std::string const& filename) {
    YAML::Node node = YAML::LoadFile(filename);
    this->center_offset = node["center_offset"].as<double>();
    this->K = node["K"].as<arma::vec>();
    this->L = node["L"].as<arma::vec>();
    this->fence = node["fence"].as<arma::vec>();
    this->w_max = node["w_max"].as<double>();
  }
};
}  // ::robot

#endif  // __ROBOT_PARAMETERS_HPP__