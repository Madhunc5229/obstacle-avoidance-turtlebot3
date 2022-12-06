
#include <memory>
#include "obsAvoid.h"

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObsAvoid>("bot_node"));
    rclcpp::shutdown();
    return 0;
}