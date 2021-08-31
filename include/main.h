#ifndef KHNP_MAIN_H
#define KHNP_MAIN_H

///// Qt GUI elements
#include <QApplication>
#include <QtWidgets>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QImage>
#include <QPixmap>
#include <QPushButton>
#include <QIcon>

//QT utils
#include <QFrame>
#include <QString>
#include <QPalette>
#include <QFont>

///// common headers
#include <ros/ros.h>
#include <ros/package.h> // get path
#include <iostream> //cout
#include <fstream>
#include <string>
#include <math.h> // pow
#include <vector>

///// Utils
#include <tf/LinearMath/Quaternion.h> // to Quaternion_to_euler
#include <tf/LinearMath/Matrix3x3.h> // to Quaternion_to_euler

///// time for Gazebo
#include <rosgraph_msgs/Clock.h>

///// headers
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SpawnModel.h>
#include <std_srvs/Empty.h>

///// image processing
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

///// utils
#include <signal.h>
void signal_handler(sig_atomic_t s) {
  std::cout << "You pressed Ctrl + C, exiting" << std::endl;
  exit(1);
}
double euclidean_dist(double x1, double y1, double z1, double x2, double y2, double z2){
	return sqrt(pow(x1-x2,2) + pow(y1-y2,2) + pow(z1-z2,2));
}

using namespace std;



////////////////////////////////////////////////////////////////////////////////////////////////////
class khnp_comp: public QWidget{
  private:
    // no meaning for private, just separate QT variables
    QHBoxLayout *main_hbox;
    QVBoxLayout *left_vbox, *right_vbox;
    QHBoxLayout *right_hbox_btns, *right_hbox1, *right_hbox2, *right_hbox3, *right_hbox4, *right_hbox5, *right_hbox_result;
    QLabel *left_text1, *left_text2, *left_3rd_img, *left_1st_img;
    QLabel *right_text1, *right_text2, *right_text3, *right_text4;
    QLabel *right_text5, *right_text6, *right_text7, *right_text8;
    QLabel *right_creator, *right_logo, *right_result;
    QPushButton *falldown_button, *pause_button, *reset_button, *skip_button;

    QPalette palette;
    QFont font;
    QColor cyan=QColor(121,215,252);
    QColor palegreen=QColor(172,252,186);
    QColor palepurple=QColor(228,191,255);
    QColor lightred=QColor(255,77,115);
    int iconsize=100;

    cv_bridge::CvImagePtr third_cam_cv_img_ptr, first_cam_cv_img_ptr;
    cv::Mat logo_img, pause_img, paused_img, falldown_img, fell_img, reset_img, skip_img;
    string path;
    bool paused_check=false;

    void qt_img_update(QLabel *label, cv::Mat img);
    void qt_icon_update(QPushButton *btn, cv::Mat img);
    void falldown_button_callback();
    void pause_button_callback();
    void reset_button_callback();
    void skip_button_callback();
    void finish_result();

  public:
    // no meaning for public, just separate ROS and main variables
    gazebo_msgs::ModelStates states;
    gazebo_msgs::ModelState cam_pose;
    gazebo_msgs::SetModelState model_move_srv;
    std_srvs::Empty empty_srv;
    sensor_msgs::CompressedImage third_cam_img_msg, first_cam_img_msg;
    rosgraph_msgs::Clock real_current_time, fixed_current_time;

    bool initialized=false, qt_initialized=false, state_check=false, clock_check=false, third_cam_check=false, first_cam_check=false;
    bool first_clock_in=false, if_felldown_flag=false, if_finished=false;
    std::string robot_name, third_cam_name, third_cam_topic, first_cam_topic;
    int idx=0, img_width, img_height;

    ///// ros and tf
    ros::NodeHandle nh;
    ros::Subscriber states_sub, third_cam_sub, first_cam_sub, clock_sub;
    ros::ServiceClient model_mover, model_spawner, pauser, unpauser, resetter;
    ros::Timer main_timer;

    void states_callback(const gazebo_msgs::ModelStates::ConstPtr& msg);
    void third_cam_callback(const sensor_msgs::CompressedImage::ConstPtr& msg);
    void first_cam_callback(const sensor_msgs::CompressedImage::ConstPtr& msg);
    void clock_callback(const rosgraph_msgs::Clock::ConstPtr& msg);
    void main_timer_func(const ros::TimerEvent& event);
    void main_initialize();
    void QT_initialize();
    void cam_move(geometry_msgs::Pose pose);
    void if_felldown(geometry_msgs::Pose pose);

    khnp_comp(ros::NodeHandle& n, QWidget *parent=0) : nh(n), QWidget(parent){

      ///// params
      nh.param("/img_width", img_width, 480);
      nh.param("/img_height", img_height, 320);
      nh.param<std::string>("/robot_name", robot_name, "/");
      nh.param<std::string>("/third_cam_name", third_cam_name, "third_camera");
      nh.param<std::string>("/third_cam_topic", third_cam_topic, "/third_camera/rgb/image_raw/compressed");
      nh.param<std::string>("/first_cam_topic", first_cam_topic, "/d455/depth/rgb_image_raw/compressed");

      ///// sub pub
      states_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, &khnp_comp::states_callback, this);
      third_cam_sub = nh.subscribe<sensor_msgs::CompressedImage>(third_cam_topic, 10, &khnp_comp::third_cam_callback, this);
      first_cam_sub = nh.subscribe<sensor_msgs::CompressedImage>(first_cam_topic, 10, &khnp_comp::first_cam_callback, this);
      clock_sub = nh.subscribe<rosgraph_msgs::Clock>("/clock", 10, &khnp_comp::clock_callback, this);
      main_timer = nh.createTimer(ros::Duration(1/24.0), &khnp_comp::main_timer_func, this); // every 1/3 second.
      model_mover = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
      pauser = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
      unpauser = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
      resetter = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");

      ///// Init
      path = ros::package::getPath("khnp_competition");
      main_initialize();
      QT_initialize();

      ROS_WARN("class heritated, starting node...");
    }
};

void khnp_comp::main_timer_func(const ros::TimerEvent& event){
  if (initialized && qt_initialized && state_check && third_cam_check && first_cam_check){
    cam_move(states.pose[idx]);

    if(!if_felldown_flag){
      if_felldown(states.pose[idx]);
    }
    qt_img_update(left_3rd_img, third_cam_cv_img_ptr->image);
    qt_img_update(left_1st_img, first_cam_cv_img_ptr->image);

    if(!if_finished){
      auto temp = real_current_time.clock-fixed_current_time.clock;
      right_text8->setText(QString::number(temp.sec + temp.nsec*1e-9,'g',7));
    }
  }
  else{
    cout << initialized << qt_initialized << state_check << third_cam_check << first_cam_check << endl;
  }
}

void khnp_comp::main_initialize(){
  cam_pose.model_name=third_cam_name;
  initialized=true;
  ROS_WARN("%s initialized!", third_cam_name.c_str());
}

void khnp_comp::clock_callback(const rosgraph_msgs::Clock::ConstPtr& msg){
  real_current_time = *msg;
  if(!first_clock_in){
    fixed_current_time=real_current_time;
    first_clock_in=true;
  }
}

void khnp_comp::cam_move(geometry_msgs::Pose pose){
  cam_pose.pose = pose;
  cam_pose.pose.position.z += 5.0;
  cam_pose.pose.orientation.x = 0.0; cam_pose.pose.orientation.y = 0.0; cam_pose.pose.orientation.z = 0.0; cam_pose.pose.orientation.w = 1.0;
  model_move_srv.request.model_state = cam_pose;
  model_mover.call(model_move_srv);
}
void khnp_comp::if_felldown(geometry_msgs::Pose pose){
  double curr_roll, curr_pitch, curr_yaw;
  tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(curr_roll, curr_pitch, curr_yaw);

  if (abs(curr_roll)>1.309 or abs(curr_pitch)>1.309){
    ROS_WARN("Fell down");
    qt_icon_update(falldown_button, fell_img);
    if_felldown_flag=true;
  }
  // else if(){

  // }
  // else if(){

  // }
}

void khnp_comp::states_callback(const gazebo_msgs::ModelStates::ConstPtr& msg){
  states = *msg;
  for (int i = 0; i < states.name.size(); ++i)
  {
    if (states.name[i]==robot_name){
      idx=i;
      state_check=true;
    }
  }
}

void khnp_comp::third_cam_callback(const sensor_msgs::CompressedImage::ConstPtr& msg){
  third_cam_img_msg = *msg;
  third_cam_cv_img_ptr = cv_bridge::toCvCopy(third_cam_img_msg);
  third_cam_check=true;
}
void khnp_comp::first_cam_callback(const sensor_msgs::CompressedImage::ConstPtr& msg){
  first_cam_img_msg = *msg;
  first_cam_cv_img_ptr = cv_bridge::toCvCopy(first_cam_img_msg);
  first_cam_check=true;
}



void khnp_comp::qt_img_update(QLabel *label, cv::Mat img){
  cv::Mat vis_img;
  if (img.cols != img_width or img.rows != img_height){
    cv::resize(img, vis_img, cv::Size(img_width, img_height));
    cv::cvtColor(vis_img, vis_img, CV_BGR2RGB);
  }
  else{
    cv::cvtColor(img, vis_img, CV_BGR2RGB);
  }
  QImage imgIn= QImage((uchar*) vis_img.data, vis_img.cols, vis_img.rows, vis_img.step, QImage::Format_RGB888);
  QPixmap pixmap = QPixmap::fromImage(imgIn);
  label->setPixmap(pixmap);
}

void khnp_comp::qt_icon_update(QPushButton *btn, cv::Mat img){
  QImage imgIn= QImage((uchar*) img.data, img.cols, img.rows, img.step, QImage::Format_RGB888);
  QPixmap pixmap = QPixmap::fromImage(imgIn);
  btn->setIcon(pixmap);
}

void khnp_comp::falldown_button_callback(){
  if(!if_felldown_flag){
    qt_icon_update(falldown_button, fell_img);
    ROS_WARN("Fell down from user");
    if_felldown_flag=true;
  }
}
void khnp_comp::pause_button_callback(){
  QImage pause_imgIn;
  if (paused_check){
    unpauser.call(empty_srv);
    qt_icon_update(pause_button, pause_img);
    ROS_WARN("Resumed!!");
    paused_check=false;
  }
  else{
    pauser.call(empty_srv);
    qt_icon_update(pause_button, paused_img);
    ROS_WARN("Paused!!");
    paused_check=true;
  }
}
void khnp_comp::reset_button_callback(){
  resetter.call(empty_srv);
  fixed_current_time=real_current_time;
  if_felldown_flag=false;
  qt_icon_update(falldown_button, falldown_img);
  ROS_WARN("Resetted!!!");
}
void khnp_comp::skip_button_callback(){
  ROS_WARN("skip");
  finish_result();
}

void khnp_comp::finish_result(){
  if(!if_finished){
    right_text1->setText(tr("Total score"));
    palette = right_text1->palette();
    palette.setColor(QPalette::Window, lightred);
    right_text1->setPalette(palette);

    right_text4->setText(tr("Finished time"));
    palette = right_text4->palette();
    palette.setColor(QPalette::Window, lightred);
    right_text4->setPalette(palette);

    char result_string[200];
    sprintf(result_string, "Results:\n\nRefracted corridor: %d \nRough terrain: %d \nManiuplator: %d \nDisturbance: %d \nStair: %d \nPanelty(falldown):  %d", false, true, false, true, false, -120);
    QString result = QString::fromStdString(result_string);
    right_result->setText(result);
    right_result->setAlignment(Qt::AlignCenter);
    right_result->setAutoFillBackground(true);
    right_result->setFixedSize(QSize(400,180));
    palette = right_result->palette();
    palette.setColor(QPalette::Window, lightred);
    right_result->setPalette(palette);
    font = right_result->font();
    font.setPointSize(11);
    right_result->setFont(font);
    right_result->setFrameStyle(QFrame::Panel | QFrame::Raised);
    right_result->setLineWidth(3);

    auto temp = real_current_time.clock-fixed_current_time.clock;
    right_text8->setText(QString::number(temp.sec + temp.nsec*1e-9,'g',7));

    if_finished=true;

    ROS_WARN("Finished!!!");
  }
}


void khnp_comp::QT_initialize(){
  logo_img = cv::imread(path + "/resources/khnp.png");
  pause_img = cv::imread(path + "/resources/pause.png");
  paused_img = cv::imread(path + "/resources/paused.png");
  falldown_img = cv::imread(path + "/resources/falldown.png");
  fell_img = cv::imread(path + "/resources/fell.png");
  reset_img = cv::imread(path + "/resources/reset.png");
  skip_img = cv::imread(path + "/resources/skip.png");
  cv::cvtColor(logo_img, logo_img, CV_BGR2RGB);
  cv::cvtColor(pause_img, pause_img, CV_BGR2RGB);
  cv::cvtColor(paused_img, paused_img, CV_BGR2RGB);
  cv::cvtColor(falldown_img, falldown_img, CV_BGR2RGB);
  cv::cvtColor(fell_img, fell_img, CV_BGR2RGB);
  cv::cvtColor(reset_img, reset_img, CV_BGR2RGB);
  cv::cvtColor(skip_img, skip_img, CV_BGR2RGB);

  left_text1 = new QLabel();
  left_text2 = new QLabel();
  left_3rd_img = new QLabel();
  left_1st_img = new QLabel();
  falldown_button = new QPushButton();
  pause_button = new QPushButton();
  reset_button = new QPushButton();
  skip_button = new QPushButton();
  right_text1 = new QLabel();
  right_text2 = new QLabel();
  right_text3 = new QLabel();
  right_text4 = new QLabel();
  right_text5 = new QLabel();
  right_text6 = new QLabel();
  right_text7 = new QLabel();
  right_text8 = new QLabel();
  right_creator = new QLabel();
  right_logo = new QLabel();
  right_result = new QLabel();
  right_hbox_btns = new QHBoxLayout();
  right_hbox1 = new QHBoxLayout();
  right_hbox2 = new QHBoxLayout();
  right_hbox3 = new QHBoxLayout();
  right_hbox4 = new QHBoxLayout();
  right_hbox5 = new QHBoxLayout();
  right_hbox_result = new QHBoxLayout();
  left_vbox = new QVBoxLayout();
  right_vbox = new QVBoxLayout();
  main_hbox = new QHBoxLayout();



  QImage falldown_imgIn= QImage((uchar*) falldown_img.data, falldown_img.cols, falldown_img.rows, falldown_img.step, QImage::Format_RGB888);
  QPixmap falldown_pixmap = QPixmap::fromImage(falldown_imgIn);
  falldown_button->setIcon(falldown_pixmap);
  falldown_button->setIconSize(QSize(iconsize, iconsize));

  QImage pause_imgIn= QImage((uchar*) pause_img.data, pause_img.cols, pause_img.rows, pause_img.step, QImage::Format_RGB888);
  QPixmap pause_pixmap = QPixmap::fromImage(pause_imgIn);
  pause_button->setIcon(pause_pixmap);
  pause_button->setIconSize(QSize(iconsize, iconsize));

  QImage reset_imgIn= QImage((uchar*) reset_img.data, reset_img.cols, reset_img.rows, reset_img.step, QImage::Format_RGB888);
  QPixmap reset_pixmap = QPixmap::fromImage(reset_imgIn);
  reset_button->setIcon(reset_pixmap);
  reset_button->setIconSize(QSize(iconsize, iconsize));

  QImage skip_imgIn= QImage((uchar*) skip_img.data, skip_img.cols, skip_img.rows, skip_img.step, QImage::Format_RGB888);
  QPixmap skip_pixmap = QPixmap::fromImage(skip_imgIn);
  skip_button->setIcon(skip_pixmap);
  skip_button->setIconSize(QSize(iconsize, iconsize));


  left_text1->setText(tr("3rd person view image"));
  left_text1->setAlignment(Qt::AlignCenter);
  left_text1->setAutoFillBackground(true);
  left_text1->setFixedSize(QSize(img_width,50));
  palette = left_text1->palette();
  palette.setColor(QPalette::Window, cyan);
  left_text1->setPalette(palette);
  font = left_text1->font();
  font.setPointSize(14);
  left_text1->setFont(font);
  left_text1->setFrameStyle(QFrame::Panel | QFrame::Raised);
  left_text1->setLineWidth(3);

  left_text2->setText(tr("1st person view image"));
  left_text2->setAlignment(Qt::AlignCenter);
  left_text2->setAutoFillBackground(true);
  left_text2->setFixedSize(QSize(img_width,50));
  palette = left_text2->palette();
  palette.setColor(QPalette::Window, cyan);
  left_text2->setPalette(palette);
  font = left_text2->font();
  font.setPointSize(14);
  left_text2->setFont(font);
  left_text2->setFrameStyle(QFrame::Panel | QFrame::Raised);
  left_text2->setLineWidth(3);



  right_text1->setText(tr("Current score"));
  right_text1->setAlignment(Qt::AlignCenter);
  right_text1->setAutoFillBackground(true);
  right_text1->setFixedSize(QSize(260,50));
  palette = right_text1->palette();
  palette.setColor(QPalette::Window, palegreen);
  right_text1->setPalette(palette);
  font = right_text1->font();
  font.setPointSize(14);
  right_text1->setFont(font);
  right_text1->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_text1->setLineWidth(3);

  right_text2->setText(tr("Current stage"));
  right_text2->setAlignment(Qt::AlignCenter);
  right_text2->setAutoFillBackground(true);
  right_text2->setFixedSize(QSize(260,50));
  palette = right_text2->palette();
  palette.setColor(QPalette::Window, palegreen);
  right_text2->setPalette(palette);
  font = right_text2->font();
  font.setPointSize(14);
  right_text2->setFont(font);
  right_text2->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_text2->setLineWidth(3);

  right_text3->setText(tr("Current stage time"));
  right_text3->setAlignment(Qt::AlignCenter);
  right_text3->setAutoFillBackground(true);
  right_text3->setFixedSize(QSize(260,50));
  palette = right_text3->palette();
  palette.setColor(QPalette::Window, palegreen);
  right_text3->setPalette(palette);
  font = right_text3->font();
  font.setPointSize(14);
  right_text3->setFont(font);
  right_text3->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_text3->setLineWidth(3);

  right_text4->setText(tr("Current total time"));
  right_text4->setAlignment(Qt::AlignCenter);
  right_text4->setAutoFillBackground(true);
  right_text4->setFixedSize(QSize(260,50));
  palette = right_text4->palette();
  palette.setColor(QPalette::Window, palegreen);
  right_text4->setPalette(palette);
  font = right_text4->font();
  font.setPointSize(14);
  right_text4->setFont(font);
  right_text4->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_text4->setLineWidth(3);

  right_text5->setText(QString::number(0.0,'g',7));
  right_text5->setAlignment(Qt::AlignCenter);
  right_text5->setAutoFillBackground(true);
  right_text5->setFixedSize(QSize(260,50));
  font = right_text5->font();
  font.setPointSize(13);
  right_text5->setFont(font);

  right_text6->setText(tr("corridor-1"));
  right_text6->setAlignment(Qt::AlignCenter);
  right_text6->setAutoFillBackground(true);
  right_text6->setFixedSize(QSize(260,50));
  font = right_text6->font();
  font.setPointSize(13);
  right_text6->setFont(font);

  right_text7->setText(QString::number(0.0,'g',7));
  right_text7->setAlignment(Qt::AlignCenter);
  right_text7->setAutoFillBackground(true);
  right_text7->setFixedSize(QSize(260,50));
  font = right_text7->font();
  font.setPointSize(13);
  right_text7->setFont(font);

  right_text8->setText(QString::number(0.0,'g',7));
  right_text8->setAlignment(Qt::AlignCenter);
  right_text8->setAutoFillBackground(true);
  right_text8->setFixedSize(QSize(260,50));
  font = right_text8->font();
  font.setPointSize(13);
  right_text8->setFont(font);

  QString creator = "Maintainers (Report any bugs please)\n\nEungchang Mason Lee (email: eungchang_mason@kaist.ac.kr)\nJunho Choi (email: cjh6685kr@kaist.ac.kr)";
  right_creator->setText(creator);
  right_creator->setAlignment(Qt::AlignCenter);
  right_creator->setAutoFillBackground(true);
  right_creator->setFixedSize(QSize(360,80));
  palette = right_creator->palette();
  palette.setColor(QPalette::Window, palepurple);
  right_creator->setPalette(palette);
  font = right_creator->font();
  font.setPointSize(8);
  right_creator->setFont(font);
  right_creator->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_creator->setLineWidth(3);

  QImage imgIn= QImage((uchar*) logo_img.data, logo_img.cols, logo_img.rows, logo_img.step, QImage::Format_RGB888);
  QPixmap pixmap = QPixmap::fromImage(imgIn);
  right_logo->setPixmap(pixmap);



  left_vbox->addWidget(left_text1);
  left_vbox->addWidget(left_3rd_img);
  left_vbox->addWidget(left_text2);
  left_vbox->addWidget(left_1st_img);

  right_hbox_btns->addWidget(falldown_button);
  right_hbox_btns->addWidget(pause_button);
  right_hbox_btns->addWidget(reset_button);
  right_hbox_btns->addWidget(skip_button);
  right_hbox_btns->setAlignment(Qt::AlignCenter);
  right_hbox1->addWidget(right_text1);
  right_hbox1->addWidget(right_text5);
  right_hbox2->addWidget(right_text2);
  right_hbox2->addWidget(right_text6);
  right_hbox3->addWidget(right_text3);
  right_hbox3->addWidget(right_text7);
  right_hbox4->addWidget(right_text4);
  right_hbox4->addWidget(right_text8);
  right_hbox5->addWidget(right_creator);
  right_hbox5->addWidget(right_logo);
  right_hbox5->setAlignment(Qt::AlignCenter);
  right_hbox_result->addWidget(right_result);
  right_hbox_result->setAlignment(Qt::AlignCenter);

  right_vbox->addLayout(right_hbox_btns);
  right_vbox->addLayout(right_hbox1);
  right_vbox->addLayout(right_hbox2);
  right_vbox->addLayout(right_hbox3);
  right_vbox->addLayout(right_hbox4);
  right_vbox->addLayout(right_hbox5);
  right_vbox->setAlignment(Qt::AlignCenter);
  right_vbox->addLayout(right_hbox_result);


  main_hbox->addLayout(left_vbox);
  main_hbox->addLayout(right_vbox);

  setLayout(main_hbox);

  show();
  setWindowTitle(tr("KHNP competition window"));

  connect(falldown_button, &QPushButton::clicked, this, &khnp_comp::falldown_button_callback);
  connect(pause_button, &QPushButton::clicked, this, &khnp_comp::pause_button_callback);
  connect(reset_button, &QPushButton::clicked, this, &khnp_comp::reset_button_callback);
  connect(skip_button, &QPushButton::clicked, this, &khnp_comp::skip_button_callback);
  qt_initialized=true;
}

#endif