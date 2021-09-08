#ifndef COURSE_A_H
#define COURSE_A_H

#include <ros/ros.h>
#include <string>
#include <vector>

using namespace std;

struct position3d{
	double x=0.0;
	double y=0.0;
	double z=0.0;
};

struct course{
  position3d start_position;
  position3d finish_position;
  bool heading_opposite=false;
};


struct map_of_courses{
	string name;
	vector<course> courses;
	ros::Duration time_limit;
};

vector<position3d> cubes_poses, spheres_poses;
vector<string> cubes_names, spheres_names;
map_of_courses refracted_corridor_map, manipulator_map, rough_terrain_map, disturbance_map, stair_map, finish_map;
vector<map_of_courses> courseAB;
int current_map=0;
int current_course=0;



///////////////////////////////////////////////////////////////////////////////////////////////
void course_initilization(){
	cubes_poses.push_back({34.6, -5.6, 0.28}); cubes_poses.push_back({34.85, -5.6, 0.28});
	cubes_poses.push_back({35.1, -5.6, 0.28}); cubes_poses.push_back({35.35, -5.6, 0.28});
	cubes_poses.push_back({35.6, -5.6, 0.28});
	cubes_names.push_back("b1"); cubes_names.push_back("b2"); cubes_names.push_back("b3");
	cubes_names.push_back("b4"); cubes_names.push_back("b5");

	spheres_poses.push_back({58.75, -4, 0.6}); spheres_poses.push_back({64.75, -4, 0.6});
	spheres_poses.push_back({70.75, -4, 0.6}); spheres_poses.push_back({76.75, -4, 0.6});
	spheres_poses.push_back({82.75, -4, 0.6});
	spheres_names.push_back("ds1"); spheres_names.push_back("ds2");
	spheres_names.push_back("ds3"); spheres_names.push_back("ds4");
	spheres_names.push_back("ds5");

	refracted_corridor_map.name = "corridor ";
	manipulator_map.name = "manipulator ";
	rough_terrain_map.name = "rough terrain ";
	disturbance_map.name = "disturbance ";
	stair_map.name = "stair ";
	finish_map.name = "finish ";

	refracted_corridor_map.time_limit = ros::Duration(600.0);
	manipulator_map.time_limit = ros::Duration(600.0);
	rough_terrain_map.time_limit = ros::Duration(600.0);
	disturbance_map.time_limit = ros::Duration(900.0);
	stair_map.time_limit = ros::Duration(1200.0);
	finish_map.time_limit = ros::Duration(300.0);


	course c1; c1.start_position={3.7, 0.0, 0.0}; c1.finish_position={6.35, -10.0, 0.0};
	course c2; c2.start_position={8.7, -10.0, 0.0}; c2.finish_position={11.35, -3.0, 0.0};
	course c3; c3.start_position={13.7, -3.0, 0.0}; c3.finish_position={16.35, -8.0, 0.0};
	course c4; c4.start_position={18.7, -8.0, 0.0}; c4.finish_position={21.35, -1.0, 0.0};
	course c5; c5.start_position={23.7, -1.0, 0.0}; c5.finish_position={26.35, -6.0, 0.0};
	refracted_corridor_map.courses.push_back(c1);
	refracted_corridor_map.courses.push_back(c2);
	refracted_corridor_map.courses.push_back(c3);
	refracted_corridor_map.courses.push_back(c4);
	refracted_corridor_map.courses.push_back(c5);

	course m1; m1.start_position={33.55, -6.0, 0.0}; m1.finish_position={36.75, -6.0, 0.0};
	manipulator_map.courses.push_back(m1);

	course r1; r1.start_position={38.6, -6.0, 0.0}; r1.finish_position={42.3, -6.0, 0.0};
	course r2; r2.start_position={44.5, -6.0, 0.0}; r2.finish_position={50.5, -6.0, 0.0};
	course r3; r3.start_position={52.5, -6.0, 0.0}; r3.finish_position={55.35, -6.0, 0.0};
	rough_terrain_map.courses.push_back(r1);
	rough_terrain_map.courses.push_back(r2);
	rough_terrain_map.courses.push_back(r3);

	course d1; d1.start_position={57.7, -6.0, 0.0}; d1.finish_position={61.45, -6.0, 0.0};
	course d2; d2.start_position={63.7, -6.0, 0.0}; d2.finish_position={67.45, -6.0, 0.0};
	course d3; d3.start_position={69.7, -6.0, 0.0}; d3.finish_position={73.45, -6.0, 0.0};
	course d4; d4.start_position={75.7, -6.0, 0.0}; d4.finish_position={79.45, -6.0, 0.0};
	course d5; d5.start_position={81.7, -6.0, 0.0}; d5.finish_position={85.45, -6.0, 0.0};
	disturbance_map.courses.push_back(d1);
	disturbance_map.courses.push_back(d2);
	disturbance_map.courses.push_back(d3);
	disturbance_map.courses.push_back(d4);
	disturbance_map.courses.push_back(d5);

	course s1; s1.start_position={88.45, -6.0, 0.0}; s1.finish_position={91.25, -6.0, 2.35};
	course s2; s2.start_position={91.3, -5.15, 2.35}; s2.finish_position={88.55, -5.15, 4.45};
	s2.heading_opposite=true;
	stair_map.courses.push_back(s1);
	stair_map.courses.push_back(s2);

	course f1; f1.start_position={88.55, -5.15, 4.45}; f1.finish_position={85.0, -5.15, 4.45};
	f1.heading_opposite=true;
	finish_map.courses.push_back(f1);
	
	courseAB.push_back(refracted_corridor_map);
	courseAB.push_back(manipulator_map);
	courseAB.push_back(rough_terrain_map);
	courseAB.push_back(disturbance_map);
	courseAB.push_back(stair_map);
	courseAB.push_back(finish_map);
}


#endif