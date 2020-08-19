#pragma once
#pragma once
#include<iostream>
#include<opencv2\opencv.hpp>
#include<fstream>
#include<cmath>
# include <time.h>     // or   # include <ctime>
# include <stdlib.h>   // or   # include <cstdlib>

#define M_PI 3.14159265358979323846
using namespace std;
using namespace cv;
class contour {
public: contour(string pathway, vector<double> world_coord);
public:contour();
private:vector<vector<Point2d>> readtxt(string path);
private: string path;
private: vector<vector<Point2d>> allcontourcoord; //coordinates of the original dxf (dxf world coordinates)
private: double min_angle;
private: int sorting_direction; //0:CCW, 1:CW
private: double lineAngle(Point2d p1, Point2d p2, Point2d p3);
private: vector<Point2d> make_sure_CCW(vector<Point2d> c);
private: void boundary_sorting(vector<Point2d> boundary1, vector<Point2d> boundary2, vector<Point2d>& output);
public: vector<Point2d> boundingbox;// coordinates of the bounding box in the world coordinates
public: Point2d center;// coordinates of the center of the bounding box
public: vector<Point2d> outercontour_world; // coordinates of the contour in the world coordinates
public: vector<Point2d> outercontour_object; // relative coordinates of the contour to the center
public: vector<Point2d> min_box_world;
public: vector<Point2d> min_box_object;
public:vector<Point2d> upper_boundary;
public:vector<Point2d> bottom_boundary;
public:vector<Point2d> left_boundary;
public:vector<Point2d> right_boundary;
public: double rotate_angle;
public:vector<Point2d> tippingpoint = { {0,0},{0,0},{0,0},{0,0} }; //top->left->bottom->right
public: double min_boundingbox_angle(contour X);
public: void shift(vector<double> shift_amount);
public: contour rotate(double angle);
private: contour inirotate(double angle);
public: void get_boundary();
};
class group {
public: group();
public: group(double x, double y, contour first);
public: group(double x, double y, contour first, bool turn_off_optimize_pose);
public: vector<contour> contour_inside_boundary;
public: double xlimit;
public: double ylimit;
private: double optimized_pose(contour first);
public: vector<Point2d> upper_boundary;
public: vector<Point2d> right_boundary;
public: vector<Point2d> bottom_boundary;
public: vector<Point2d> left_boundary;
public: vector<Point2d> boundingbox;
public: vector<Point> convex_hull;
public: void shift(vector<double> shift_amount);
public: void merge(contour newcontour);
};
class container {
public:container();
public:container(vector<double> limit, group first);
public: int xlimit;
public: int ylimit;
public: vector<group> group_inside_container;
public: int add_group_vertically(group upper, double cutting_gap, double x_shift = 1);
public: void shift(vector<double> shift_amount);
public: void output();
};
class canvas {
public:canvas();
public: canvas(int w, int h, int set, int shift);
public: Mat img;
private: int x_shift;
private: int y_shift;
private: int width;
private: int height;
private: int setTo_value;
private: Mat blank;
public: void refresh();
public: void write();
public: void draw_center(Scalar color, contour& cont, double zoom_ratio, int center_radius = 10);
public: void draw_outercontour(Scalar color, contour& cont, double zoom_ratio, int line_width = 1);
public: void draw_boundingbox(Scalar color, contour& cont, double zoom_ratio, int line_width = 1);
public: void draw_minboundingbox(Scalar color, contour& cont, double zoom_ratio, int line_width = 1);
public: void draw_convexHull(Scalar color, vector<Point>& hull, double zoom_ratio, int line_width = 1);
public: void draw_boundary(Scalar color, vector<Point2d>& bounary, double zoom_ratio, int line_width = 1);
public: void draw_all_element(contour& cont, double zoom_ratio);
public: void draw_all_element(group& b, double zoom_ratio);
public: void draw_all_element(container& b, double zoom_ratio);
public: void show(string windowname, int waittime);
public: void draw_last_element(group& b, double zoom_ratio);
public: void draw_limit(vector<double> limit, double zoom_ratio, int line_width = 1);
};

double approach_from_top_minarea(group& bottom, contour top, double angle, contour& output, vector<double> limit, double cutting_gap, int waittime = 1, double x_shift = 1, int zoom_ratio = 5);
double approach_from_right_minarea(group& left, contour right, double angle, contour& output, vector<double> limit, double cutting_gap, int waittime = 1, double y_shift = -1, int zoom_ratio = 5);
double approach_from_bottom_minarea(group& top, contour bottom, double angle, contour& output, vector<double> limit, double cutting_gap, int waittime = 1, double x_shift = 1, int zoom_ratio = 5);
double approach_from_left_minarea(group& right, contour left, double angle, contour& output, vector<double> limit, double cutting_gap, int waittime = 1, double y_shift = -1, int zoom_ratio = 5);
contour approach_from_right(group& left, contour right, double cutting_gap, int waittime = 1, int zoom_ratio = 5);
group  approach_two_group_vertically(group upper, group lower, vector<int> limit, double cutting_gap, double x_shift = 1);
container get_all_possible_optimized_group(contour contourB, double material_width, double cutting_gap, int num_of_possibility, double zoom_ratio = 0.3);