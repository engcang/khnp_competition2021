#include "main.h"


int main(int argc, char **argv)
{
    QApplication app(argc, argv);

    ros::init(argc, argv, "comp_main_node");
    ros::NodeHandle n("~");

    khnp_comp kcp(n);

    signal(SIGINT, signal_handler); // to exit program when ctrl+c

    ros::AsyncSpinner spinner(5); // Use 7 threads -> 3 callbacks + 4 Timer callbacks
    spinner.start();
    // ros::waitForShutdown();

    return app.exec();
}