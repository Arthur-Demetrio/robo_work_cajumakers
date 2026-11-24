// Register and parse real Nav2 BT plugins without instantiating action clients or ticking a tree.
#include <iostream>
#include <fstream>
#include <behaviortree_cpp/xml_parsing.h>
#include <sstream>
#include <string>
#include <behaviortree_cpp/bt_factory.h>
#include <nav2_behavior_tree/plugins_list.hpp>
int main(int argc, char **argv) {
  BT::BehaviorTreeFactory factory;
  std::stringstream plugins(nav2::details::BT_BUILTIN_PLUGINS);
  std::string name;
  while (std::getline(plugins, name, ';')) {
    factory.registerFromPlugin("/opt/ros/jazzy/lib/lib" + name + ".so");
  }
  if (argc == 1) { std::cout << BT::writeTreeNodesModelXML(factory); return 0; }
  for (int i=1; i<argc; ++i) {
    factory.clearRegisteredBehaviorTrees();
    std::ifstream input(argv[i]);
    std::string xml((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    std::unordered_map<std::string, BT::NodeType> registered;
    for (const auto &item : factory.manifests()) registered[item.first] = item.second.type;
    BT::VerifyXML(xml, registered);
    factory.registerBehaviorTreeFromFile(argv[i]);
    std::cout << "registered_and_parsed=" << argv[i] << std::endl;
  }
}
