#include"core.h"

contour::contour() {

};
contour::contour(string pathway, vector<double> world_coord) {
    path = pathway;
    allcontourcoord = readtxt(path);
    allcontourcoord[1] = make_sure_CCW(allcontourcoord[1]);
    vector<vector<Point2d>> temp = allcontourcoord;
    center = allcontourcoord[0][0];
    for (int i = 0; i < allcontourcoord.size(); i++) {
        for (int j = 0; j < allcontourcoord[i].size(); j++) {
            allcontourcoord[i][j].x = allcontourcoord[i][j].x - center.x;
            allcontourcoord[i][j].y = allcontourcoord[i][j].y - center.y;
        }
    }
    outercontour_object = allcontourcoord[1];
    for (int i = 0; i < allcontourcoord.size(); i++) {
        for (int j = 0; j < allcontourcoord[i].size(); j++) {
            allcontourcoord[i][j].x = allcontourcoord[i][j].x + world_coord[0];
            allcontourcoord[i][j].y = allcontourcoord[i][j].y + world_coord[1];
        }
    }
    center = allcontourcoord[0][0];
    boundingbox = { {allcontourcoord[0][1].x, allcontourcoord[0][1].y},{allcontourcoord[0][1].x, allcontourcoord[0][2].y},
                  {allcontourcoord[0][2].x, allcontourcoord[0][2].y},{allcontourcoord[0][2].x, allcontourcoord[0][1].y},
                  {allcontourcoord[0][1].x, allcontourcoord[0][1].y} };
    outercontour_world = allcontourcoord[1];
    allcontourcoord = temp;

    min_angle = min_boundingbox_angle(*this);
    *this = inirotate(min_angle);
    min_box_world = boundingbox;
    min_box_object = boundingbox;
    for (int j = 0; j < min_box_world.size(); j++) {
        min_box_object[j].x = min_box_world[j].x - center.x;
        min_box_object[j].y = min_box_world[j].y - center.y;
    }
}
vector<vector<Point2d>> contour::readtxt(string path) {
    ifstream fin(path);
    if (!fin) { cout << "讀檔失敗" << endl; }    // 檢查讀檔成功與否
    string s;
    vector<vector<double>> list;
    vector<vector<Point2d>> list_xy;
    vector<double> currentoutline;
    while (getline(fin, s)) {

        if (s == "") {
            list.push_back(currentoutline);
            currentoutline.clear();

        }
        else
        {
            currentoutline.push_back(stod(s));
        }
    };
    fin.close();   // 關閉檔案
    for (int i = 0; i < list.size(); i++) {
        vector<Point2d> x;
        for (int j = 0; j < list[i].size(); j = j + 2) {
            Point2d coord;
            coord.x = list[i][j];
            coord.y = list[i][j + 1];
            x.push_back(coord);
        }
        list_xy.push_back(x);
    }
    return list_xy;
}
double contour::lineAngle(Point2d p1, Point2d p2, Point2d p3) {
    double tmp = atan2((p2.y - p1.y), (p2.x - p1.x)) - atan2((p3.y - p2.y), (p3.x - p2.x));
    return (tmp < -M_PI) ? tmp + M_PI * 2 : (tmp > M_PI) ? tmp - M_PI * 2 : tmp;
}
vector<Point2d>contour::make_sure_CCW(vector<Point2d> c) {
    double angleCount = 0;
    for (unsigned int i = 0; i < c.size(); i++)
        angleCount += lineAngle(c[i], c[(i + 1) % c.size()], c[(i + 2) % c.size()]);
    if (angleCount > 0)  //original <0
    {
        reverse(c.begin(), c.end());
    }
    return c;
}
void contour::boundary_sorting(vector<Point2d> boundary1, vector<Point2d> boundary2, vector<Point2d>& output) {
    if (boundary1[0] == boundary2[0]) {
        reverse(boundary2.begin(), boundary2.end());
        output = boundary2;
        output.insert(output.end(), boundary1.begin(), boundary1.end());
    }
    else if (boundary1[0] == boundary2[boundary2.size() - 1]) {
        output = boundary2;
        output.insert(output.end(), boundary1.begin(), boundary1.end());
    }
    else if (boundary1[boundary1.size() - 1] == boundary2[0]) {
        output = boundary1;
        output.insert(output.end(), boundary2.begin(), boundary2.end());
    }
    else if (boundary1[boundary1.size() - 1] == boundary2[boundary2.size() - 1]) {
        reverse(boundary2.begin(), boundary2.end());
        output = boundary1;
        output.insert(output.end(), boundary2.begin(), boundary2.end());
    }
}
double contour::min_boundingbox_angle(contour X) {
    double area = INFINITY;
    double min_degree;
    for (int i = 0; i < 180; i += 5) {
        vector<Point2d> cur_boundingbox = this->inirotate(i).boundingbox;
        double cur_area = (cur_boundingbox[3].x - cur_boundingbox[0].x) * (cur_boundingbox[0].y - cur_boundingbox[1].y);
        if (cur_area <= area) {
            if ((cur_boundingbox[3].x - cur_boundingbox[0].x) < (cur_boundingbox[0].y - cur_boundingbox[1].y)) {
                area = cur_area;
                min_degree = i;
            }
        }
    }
    return min_degree;
};
void contour::shift(vector<double> shift_amount) {
    for (int j = 0; j < outercontour_world.size(); j++) {
        outercontour_world[j].y = outercontour_world[j].y + shift_amount[1];
        outercontour_world[j].x = outercontour_world[j].x + shift_amount[0];
    }
    center.x = center.x + shift_amount[0];
    center.y = center.y + shift_amount[1];
    for (int j = 0; j < boundingbox.size(); j++) {
        boundingbox[j].y = boundingbox[j].y + shift_amount[1];
        boundingbox[j].x = boundingbox[j].x + shift_amount[0];
    }
    for (int j = 0; j < min_box_world.size(); j++) {
        min_box_world[j].y = min_box_world[j].y + shift_amount[1];
        min_box_world[j].x = min_box_world[j].x + shift_amount[0];
    }
    for (int j = 0; j < tippingpoint.size(); j++) {
        tippingpoint[j].x += shift_amount[0];
        tippingpoint[j].y += shift_amount[1];
    }
}
contour contour::rotate(double angle) {
    contour rot = *this;
    rot.rotate_angle = angle;
    angle += min_angle;
    double theta = angle / 180 * M_PI;
    for (int j = 0; j < rot.outercontour_object.size(); j++) {
        rot.outercontour_world[j].x = rot.outercontour_object[j].x * cos(theta) -
            rot.outercontour_object[j].y * sin(theta) + center.x;
        rot.outercontour_world[j].y = rot.outercontour_object[j].y * cos(theta) +
            rot.outercontour_object[j].x * sin(theta) + center.y;
    }
    double right = rot.outercontour_world[0].x;
    double left = rot.outercontour_world[0].x;
    double top = rot.outercontour_world[0].y;
    double bottom = rot.outercontour_world[0].y;
    for (int j = 0; j < rot.outercontour_world.size(); j++)
    {
        if (rot.outercontour_world[j].y >= top) {
            top = rot.outercontour_world[j].y;
            rot.tippingpoint[0] = rot.outercontour_world[j];
        }
        if (rot.outercontour_world[j].y <= bottom) {
            bottom = rot.outercontour_world[j].y;
            rot.tippingpoint[2] = rot.outercontour_world[j];
        }
        if (rot.outercontour_world[j].x >= right) {
            right = rot.outercontour_world[j].x;
            rot.tippingpoint[3] = rot.outercontour_world[j];
        }
        if (rot.outercontour_world[j].x <= left) {
            left = rot.outercontour_world[j].x;
            rot.tippingpoint[1] = rot.outercontour_world[j];
        }
    }
    rot.boundingbox = { {left,top},{left,bottom},{right,bottom},{right,top},{left,top} };
    angle -= min_angle;
    theta = angle / 180 * M_PI;
    for (int j = 0; j < min_box_object.size(); j++) {
        rot.min_box_world[j].x = rot.min_box_object[j].x * cos(theta) -
            rot.min_box_object[j].y * sin(theta) + center.x;
        rot.min_box_world[j].y = rot.min_box_object[j].y * cos(theta) +
            rot.min_box_object[j].x * sin(theta) + center.y;
    }
    return rot;
}
contour contour::inirotate(double angle) {
    contour rot = *this;
    double theta = angle / 180 * M_PI;
    for (int j = 0; j < rot.outercontour_object.size(); j++) {
        rot.outercontour_world[j].x = rot.outercontour_object[j].x * cos(theta) -
            rot.outercontour_object[j].y * sin(theta) + center.x;
        rot.outercontour_world[j].y = rot.outercontour_object[j].y * cos(theta) +
            rot.outercontour_object[j].x * sin(theta) + center.y;
    }
    double right = rot.outercontour_world[0].x;
    double left = rot.outercontour_world[0].x;
    double top = rot.outercontour_world[0].y;
    double bottom = rot.outercontour_world[0].y;
    for (int j = 0; j < rot.outercontour_world.size(); j++)
    {
        if (rot.outercontour_world[j].y >= top) {
            top = rot.outercontour_world[j].y;
            rot.tippingpoint[0] = rot.outercontour_world[j];
        }
        if (rot.outercontour_world[j].y <= bottom) {
            bottom = rot.outercontour_world[j].y;
            rot.tippingpoint[2] = rot.outercontour_world[j];
        }
        if (rot.outercontour_world[j].x >= right) {
            right = rot.outercontour_world[j].x;
            rot.tippingpoint[3] = rot.outercontour_world[j];
        }
        if (rot.outercontour_world[j].x <= left) {
            left = rot.outercontour_world[j].x;
            rot.tippingpoint[1] = rot.outercontour_world[j];
        }
    }
    rot.boundingbox = { {left,top},{left,bottom},{right,bottom},{right,top},{left,top} };
    return rot;
}
void contour::get_boundary() {
    vector<Point2d>::iterator left_pos = find(outercontour_world.begin(), outercontour_world.end(), tippingpoint[1]);
    if (left_pos == outercontour_world.end()) {
        cout << "left error\n";
        system("pause");
    }
    int left_pos_index = distance(outercontour_world.begin(), left_pos);
    vector<Point2d>::iterator right_pos = find(outercontour_world.begin(), outercontour_world.end(), tippingpoint[3]);
    if (right_pos == outercontour_world.end()) {
        cout << "right error\n";
        system("pause");
    }
    int right_pos_index = distance(outercontour_world.begin(), right_pos);
    vector<Point2d>::iterator top_pos = find(outercontour_world.begin(), outercontour_world.end(), tippingpoint[0]);
    if (top_pos == outercontour_world.end()) {
        cout << "top error\n";
        system("pause");
    }
    int top_pos_index = distance(outercontour_world.begin(), top_pos);
    vector<Point2d>::iterator bottom_pos = find(outercontour_world.begin(), outercontour_world.end(), tippingpoint[2]);
    if (bottom_pos == outercontour_world.end()) {
        cout << "bottom error\n";
        system("pause");
    }
    int bottom_pos_index = distance(outercontour_world.begin(), bottom_pos);
    /*upper_boundary.clear();
    bottom_boundary.clear();
    right_boundary.clear();
    left_boundary.clear();*/
    vector<Point2d> left_top;
    if (left_pos_index >= top_pos_index) left_top.assign(outercontour_world.begin() + top_pos_index, outercontour_world.begin() + left_pos_index + 1);
    else {
        left_top.assign(outercontour_world.begin(), outercontour_world.begin() + left_pos_index + 1);
        vector<Point2d> temp1;
        temp1.assign(outercontour_world.begin() + top_pos_index, outercontour_world.end());
        left_top.insert(left_top.begin(), temp1.begin(), temp1.end());
    }
    vector<Point2d> left_bottom;
    if (bottom_pos_index >= left_pos_index) left_bottom.assign(outercontour_world.begin() + left_pos_index, outercontour_world.begin() + bottom_pos_index + 1);
    else {
        left_bottom.assign(outercontour_world.begin(), outercontour_world.begin() + bottom_pos_index + 1);
        vector<Point2d> temp1;
        temp1.assign(outercontour_world.begin() + left_pos_index, outercontour_world.end());
        left_bottom.insert(left_bottom.begin(), temp1.begin(), temp1.end());
    }
    vector<Point2d> right_top;
    if (top_pos_index >= right_pos_index) right_top.assign(outercontour_world.begin() + right_pos_index, outercontour_world.begin() + top_pos_index + 1);
    else {
        right_top.assign(outercontour_world.begin(), outercontour_world.begin() + top_pos_index + 1);
        vector<Point2d> temp1;
        temp1.assign(outercontour_world.begin() + right_pos_index, outercontour_world.end());
        right_top.insert(right_top.begin(), temp1.begin(), temp1.end());
    }
    vector<Point2d> right_bottom;
    if (right_pos_index >= bottom_pos_index) right_bottom.assign(outercontour_world.begin() + bottom_pos_index, outercontour_world.begin() + right_pos_index + 1);
    else {
        right_bottom.assign(outercontour_world.begin(), outercontour_world.begin() + right_pos_index + 1);
        vector<Point2d> temp1;
        temp1.assign(outercontour_world.begin() + bottom_pos_index, outercontour_world.end());
        right_bottom.insert(right_bottom.begin(), temp1.begin(), temp1.end());
    }
    upper_boundary = right_top;
    upper_boundary.insert(upper_boundary.end(), left_top.begin(), left_top.end());
    bottom_boundary = left_bottom;
    bottom_boundary.insert(bottom_boundary.end(), right_bottom.begin(), right_bottom.end());
    left_boundary = left_top;
    left_boundary.insert(left_boundary.end(), left_bottom.begin(), left_bottom.end());
    right_boundary = right_bottom;
    right_boundary.insert(right_boundary.end(), right_top.begin(), right_top.end());
    vector<Point2d> copy_upper_boundary;
    for (int i = 0; i < upper_boundary.size(); i++) {
        double x = upper_boundary[i].x;
        int flag = -1;
        double local_y = -1;
        for (int j = 0; j < upper_boundary.size() - 1; j++) {
            if (((upper_boundary[j].x < x && x < upper_boundary[j + 1].x) ||
                (upper_boundary[j].x > x && x > upper_boundary[j + 1].x))) {
                double a1 = upper_boundary[j].x;
                double b1 = upper_boundary[j].y;
                double a2 = upper_boundary[j + 1].x;
                double b2 = upper_boundary[j + 1].y;
                double ycross = (b2 - b1) / (a2 - a1) * (x - a1) + b1;
                local_y = upper_boundary[i].y - ycross;
                flag = 1;
                if (local_y < 0) break;
            }
        }
        if (flag == -1 || local_y >= 0) {
            copy_upper_boundary.push_back(upper_boundary[i]);
        }
    }
    upper_boundary = copy_upper_boundary;

    vector<Point2d> copy_bottom_boundary;
    for (int i = 0; i < bottom_boundary.size(); i++) {
        double x = bottom_boundary[i].x;
        int flag = -1;
        double local_y = 1;
        for (int j = 0; j < bottom_boundary.size() - 1; j++) {
            if (((bottom_boundary[j].x < x && x < bottom_boundary[j + 1].x) ||
                (bottom_boundary[j].x > x && x > bottom_boundary[j + 1].x))) {
                double a1 = bottom_boundary[j].x;
                double b1 = bottom_boundary[j].y;
                double a2 = bottom_boundary[j + 1].x;
                double b2 = bottom_boundary[j + 1].y;
                double ycross = (b2 - b1) / (a2 - a1) * (x - a1) + b1;
                local_y = bottom_boundary[i].y - ycross;
                flag = 1;
                if (local_y > 0) break;
            }
        }
        if (flag == -1 || local_y < 0) {
            copy_bottom_boundary.push_back(bottom_boundary[i]);
        }
    }
    bottom_boundary = copy_bottom_boundary;

    vector<Point2d> copy_right_boundary;
    for (int i = 0; i < right_boundary.size(); i++) {
        double y = right_boundary[i].y;
        int flag = -1;
        double local_x = -1;
        for (int j = 0; j < right_boundary.size() - 1; j++) {
            if ((right_boundary[j].y < y && y < right_boundary[j + 1].y) ||
                (right_boundary[j].y > y && y > right_boundary[j + 1].y)) {
                double a1 = right_boundary[j].x;
                double b1 = right_boundary[j].y;
                double a2 = right_boundary[j + 1].x;
                double b2 = right_boundary[j + 1].y;
                double xcross = (y - b1) * (a2 - a1) / (b2 - b1) + a1;
                local_x = right_boundary[i].x - xcross;
                flag = 1;
                if (local_x < 0) break;
            }
        }
        if (flag == -1 || local_x > 0) {
            copy_right_boundary.push_back(right_boundary[i]);
        }
    }
    right_boundary = copy_right_boundary;

    vector<Point2d> copy_left_boundary;
    for (int i = 0; i < left_boundary.size(); i++) {
        double y = left_boundary[i].y;
        int flag = -1;
        double local_x = 1;
        for (int j = 0; j < left_boundary.size() - 1; j++) {
            if ((left_boundary[j].y < y && y < left_boundary[j + 1].y) ||
                (left_boundary[j].y > y && y > left_boundary[j + 1].y)) {
                double a1 = left_boundary[j].x;
                double b1 = left_boundary[j].y;
                double a2 = left_boundary[j + 1].x;
                double b2 = left_boundary[j + 1].y;
                double xcross = (y - b1) * (a2 - a1) / (b2 - b1) + a1;
                local_x = left_boundary[i].x - xcross;
                flag = 1;
                if (local_x > 0) break;
            }
        }
        if (flag == -1 || local_x < 0) {
            copy_left_boundary.push_back(left_boundary[i]);
        }
    }
    left_boundary = copy_left_boundary;
}


group::group() {

};
group::group(double x, double y, contour first) {
    xlimit = x;
    ylimit = y;
    double optimized_angle = optimized_pose(first);
    first = first.rotate(optimized_angle);
    first.shift({ -first.boundingbox[1].x, -first.boundingbox[1].y });
    contour_inside_boundary.push_back(first);
    first.get_boundary();
    upper_boundary = first.upper_boundary;
    right_boundary = first.right_boundary;
    bottom_boundary = first.bottom_boundary;
    left_boundary = first.left_boundary;
    boundingbox = first.boundingbox;
    //all_contour_coord = first.outercontour_world;
    upper_boundary.insert(upper_boundary.begin(), { upper_boundary[0].x,0 });
    upper_boundary.insert(upper_boundary.begin(), { xlimit,0 });
    upper_boundary.push_back({ upper_boundary[upper_boundary.size() - 1].x, 0 });
    upper_boundary.push_back({ 0,0 });

    right_boundary.push_back({ 0,right_boundary[right_boundary.size() - 1].y });
    right_boundary.push_back({ 0,ylimit });
    right_boundary.insert(right_boundary.begin(), { 0,right_boundary[0].y });
    right_boundary.insert(right_boundary.begin(), { 0,0 });

    bottom_boundary.push_back({ bottom_boundary[bottom_boundary.size() - 1].x,ylimit });
    bottom_boundary.push_back({ xlimit,ylimit });
    bottom_boundary.insert(bottom_boundary.begin(), { bottom_boundary[0].x,ylimit });
    bottom_boundary.insert(bottom_boundary.begin(), { 0,ylimit });

    left_boundary.insert(left_boundary.begin(), { xlimit,left_boundary[0].y });
    left_boundary.insert(left_boundary.begin(), { xlimit,ylimit });
    left_boundary.push_back({ xlimit,left_boundary[left_boundary.size() - 1].y });
    left_boundary.push_back({ xlimit,0 });
    convex_hull.assign(first.outercontour_world.begin(), first.outercontour_world.end());
}
group::group(double x, double y, contour first, bool turn_off_optimize_pose) {
    xlimit = x;
    ylimit = y;
    first.shift({ -first.boundingbox[1].x, -first.boundingbox[1].y });
    contour_inside_boundary.push_back(first);
    first.get_boundary();
    upper_boundary = first.upper_boundary;
    right_boundary = first.right_boundary;
    bottom_boundary = first.bottom_boundary;
    left_boundary = first.left_boundary;
    boundingbox = first.boundingbox;
    //all_contour_coord = first.outercontour_world;
    upper_boundary.insert(upper_boundary.begin(), { upper_boundary[0].x,0 });
    upper_boundary.insert(upper_boundary.begin(), { xlimit,0 });
    upper_boundary.push_back({ upper_boundary[upper_boundary.size() - 1].x, 0 });
    upper_boundary.push_back({ 0,0 });

    right_boundary.push_back({ 0,right_boundary[right_boundary.size() - 1].y });
    right_boundary.push_back({ 0,ylimit });
    right_boundary.insert(right_boundary.begin(), { 0,right_boundary[0].y });
    right_boundary.insert(right_boundary.begin(), { 0,0 });

    bottom_boundary.push_back({ bottom_boundary[bottom_boundary.size() - 1].x,ylimit });
    bottom_boundary.push_back({ xlimit,ylimit });
    bottom_boundary.insert(bottom_boundary.begin(), { bottom_boundary[0].x,ylimit });
    bottom_boundary.insert(bottom_boundary.begin(), { 0,ylimit });

    left_boundary.insert(left_boundary.begin(), { xlimit,left_boundary[0].y });
    left_boundary.insert(left_boundary.begin(), { xlimit,ylimit });
    left_boundary.push_back({ xlimit,left_boundary[left_boundary.size() - 1].y });
    left_boundary.push_back({ xlimit,0 });
    convex_hull.assign(first.outercontour_world.begin(), first.outercontour_world.end());
}
double group::optimized_pose(contour first) {
    double x_sum = 0;
    double min_degree = 0;
    first.get_boundary();
    vector<Point2d> cur_leftcontour = first.left_boundary;
    for (int i = 0; i < cur_leftcontour.size(); i++) {
        x_sum += cur_leftcontour[i].x;
    }
    double x_sum_average = x_sum / first.left_boundary.size();
    first = first.rotate(180);
    first.shift({ -first.boundingbox[1].x, -first.boundingbox[1].y });
    first.get_boundary();
    cur_leftcontour = first.left_boundary;
    double flip_xsum = 0;
    for (int i = 0; i < cur_leftcontour.size(); i++) {
        flip_xsum += cur_leftcontour[i].x;
    }
    double flip_xsum_average = flip_xsum / first.left_boundary.size();
    if (flip_xsum_average < x_sum_average) {
        min_degree = 180;
    }
    return min_degree;
}
void group::shift(vector<double> shift_amount) {
    for (int i = 0; i < upper_boundary.size(); i++) {
        upper_boundary[i].x += shift_amount[0];
        upper_boundary[i].y += shift_amount[1];
    }
    for (int i = 0; i < right_boundary.size(); i++) {
        right_boundary[i].x += shift_amount[0];
        right_boundary[i].y += shift_amount[1];
    }
    for (int i = 0; i < bottom_boundary.size(); i++) {
        bottom_boundary[i].x += shift_amount[0];
        bottom_boundary[i].y += shift_amount[1];
    }
    for (int i = 0; i < left_boundary.size(); i++) {
        left_boundary[i].x += shift_amount[0];
        left_boundary[i].y += shift_amount[1];
    }
    for (int i = 0; i < boundingbox.size(); i++) {
        boundingbox[i].x += shift_amount[0];
        boundingbox[i].y += shift_amount[1];
    }
    for (int i = 0; i < contour_inside_boundary.size(); i++) {
        contour_inside_boundary[i].shift(shift_amount);
    }
}
void group::merge(contour newcontour) {
    contour_inside_boundary.push_back(newcontour);
    newcontour.get_boundary();
    vector<Point2d> temp;
    Point2d cross;
    if (upper_boundary[0].x > upper_boundary[upper_boundary.size() - 1].x) reverse(upper_boundary.begin(), upper_boundary.end());
    if (newcontour.upper_boundary[0].x > newcontour.upper_boundary[newcontour.upper_boundary.size() - 1].x) {
        reverse(newcontour.upper_boundary.begin(), newcontour.upper_boundary.end());
    }
    int new_start = -1;
    int upper_end = -1;
    double local_y = -1;
    for (int i = 0; i < newcontour.upper_boundary.size(); i++) {
        double x = newcontour.upper_boundary[i].x;
        int flag = -1;
        for (int j = 0; j < upper_boundary.size() - 1; j++) {
            if (upper_boundary[j].x <= x && x < upper_boundary[j + 1].x) {
                double a1 = upper_boundary[j].x;
                double b1 = upper_boundary[j].y;
                double a2 = upper_boundary[j + 1].x;
                double b2 = upper_boundary[j + 1].y;
                double ycross = (b2 - b1) / (a2 - a1) * (x - a1) + b1;
                local_y = newcontour.upper_boundary[i].y - ycross;
                upper_end = j;//**
                flag = 1;
                cross = { x,ycross };
                break;
            }
        }
        if (local_y >= 0) {
            new_start = i;
            break;
        }
        else if (flag == -1) {
            upper_end = upper_boundary.size() - 1;
            new_start = i;
            if (i > 0) {
                x = upper_boundary[upper_end].x;
                double a1 = newcontour.upper_boundary[i - 1].x;
                double b1 = newcontour.upper_boundary[i - 1].y;
                double a2 = newcontour.upper_boundary[i].x;
                double b2 = newcontour.upper_boundary[i].y;
                double ycross = (b2 - b1) / (a2 - a1) * (x - a1) + b1;
                cross = { x,ycross };
            }
            break;
        }
    }
    bool whether_be_covered = false;
    if (new_start > 0) {
        double x = upper_boundary[upper_end].x;
        double a1 = newcontour.upper_boundary[new_start - 1].x;
        double b1 = newcontour.upper_boundary[new_start - 1].y;
        double a2 = newcontour.upper_boundary[new_start].x;
        double b2 = newcontour.upper_boundary[new_start].y;
        double ycross = (b2 - b1) / (a2 - a1) * (x - a1) + b1;
        if (upper_boundary[upper_end].y - ycross < 0) whether_be_covered = true;
    }
    while ((whether_be_covered && new_start > 0) && upper_end >= 0) {
        double x = upper_boundary[upper_end].x;
        double a1 = newcontour.upper_boundary[new_start - 1].x;
        double b1 = newcontour.upper_boundary[new_start - 1].y;
        double a2 = newcontour.upper_boundary[new_start].x;
        double b2 = newcontour.upper_boundary[new_start].y;
        double ycross = (b2 - b1) / (a2 - a1) * (x - a1) + b1;
        if (upper_boundary[upper_end].y - ycross < 0) {
            upper_end--;
        }
        else {
            whether_be_covered = false;
            cross = { x,ycross };
        }
    }

    if (new_start != -1) {
        temp.assign(upper_boundary.begin(), upper_boundary.begin() + upper_end + 1);
        temp.insert(temp.end(), cross);
        temp.insert(temp.end(), newcontour.upper_boundary.begin() + new_start, newcontour.upper_boundary.end());
    }
    if (upper_boundary[upper_boundary.size() - 1].x > newcontour.upper_boundary[newcontour.upper_boundary.size() - 1].x && new_start != -1) {
        for (int j = newcontour.upper_boundary.size() - 1; j >= 0; j--) {
            double x = newcontour.upper_boundary[j].x;
            double y = newcontour.upper_boundary[j].y;
            int break_mark = 0;
            for (int i = upper_end; i < upper_boundary.size() - 1; i++) {//**
                if (upper_boundary[i].x <= x && x < upper_boundary[i + 1].x)
                {
                    double a1 = upper_boundary[i].x;
                    double b1 = upper_boundary[i].y;
                    double a2 = upper_boundary[i + 1].x;
                    double b2 = upper_boundary[i + 1].y;
                    double ycross = (b2 - b1) / (a2 - a1) * (x - a1) + b1;
                    cross = { x,ycross };
                    if (ycross - y > 0) temp.pop_back();
                    else {
                        temp.insert(temp.end(), cross);
                        temp.insert(temp.end(), upper_boundary.begin() + i + 1, upper_boundary.end());
                        break_mark = 1;
                        break;
                    }
                }
            }
            if (break_mark == 1) break;
        }
    }
    if (new_start == -1) {
        temp = upper_boundary;
    }
    upper_boundary = temp;

    temp.clear();
    int new_right_start = -1;
    int right_end = -1;
    double local_x = -1;
    for (int i = 0; i < newcontour.right_boundary.size(); i++) {
        double y = newcontour.right_boundary[i].y;
        int flag = -1;
        for (int j = 0; j < right_boundary.size() - 1; j = j++) {
            if (right_boundary[j].y <= y && y < right_boundary[j + 1].y) {
                double a1 = right_boundary[j].x;
                double b1 = right_boundary[j].y;
                double a2 = right_boundary[j + 1].x;
                double b2 = right_boundary[j + 1].y;
                double xcross = (y - b1) * (a2 - a1) / (b2 - b1) + a1;
                local_x = newcontour.right_boundary[i].x - xcross;
                right_end = j;//**
                flag = 1;
                cross = { xcross,y };
                break;
            }
        }
        if (local_x >= 0) {
            new_right_start = i;
            break;
        }
        else if (flag == -1) {
            new_right_start = i;
            right_end = right_boundary.size() - 1;
            if (i > 0) {
                y = right_boundary[right_end].y;
                double a1 = newcontour.right_boundary[i - 1].x;
                double b1 = newcontour.right_boundary[i - 1].y;
                double a2 = newcontour.right_boundary[i].x;
                double b2 = newcontour.right_boundary[i].y;
                double xcross = (y - b1) * (a2 - a1) / (b2 - b1) + a1;
                cross = { xcross,y };
            }
            break;
        }
    }
    whether_be_covered = false;
    if (new_right_start > 0) {
        double y = right_boundary[right_end].y;
        double a1 = newcontour.right_boundary[new_right_start - 1].x;
        double b1 = newcontour.right_boundary[new_right_start - 1].y;
        double a2 = newcontour.right_boundary[new_right_start].x;
        double b2 = newcontour.right_boundary[new_right_start].y;
        double xcross = (y - b1) * (a2 - a1) / (b2 - b1) + a1;
        if (right_boundary[right_end].x - xcross < 0) whether_be_covered = true;
    }
    while ((whether_be_covered && new_right_start > 0) && right_end >= 0) {
        double y = right_boundary[right_end].y;
        double a1 = newcontour.right_boundary[new_right_start - 1].x;
        double b1 = newcontour.right_boundary[new_right_start - 1].y;
        double a2 = newcontour.right_boundary[new_right_start].x;
        double b2 = newcontour.right_boundary[new_right_start].y;
        double xcross = (y - b1) * (a2 - a1) / (b2 - b1) + a1;
        if (right_boundary[right_end].x - xcross < 0) {

            right_end--;
        }
        else {
            whether_be_covered = false;
            cross = { xcross,y };
        }
    }

    if (new_right_start != -1) {
        temp.assign(right_boundary.begin(), right_boundary.begin() + right_end + 1);
        temp.insert(temp.end(), cross);
        temp.insert(temp.end(), newcontour.right_boundary.begin() + new_right_start, newcontour.right_boundary.end());
    }

    if (right_boundary[right_boundary.size() - 1].y > newcontour.right_boundary[newcontour.right_boundary.size() - 1].y && new_right_start != -1) {
        for (int j = newcontour.right_boundary.size() - 1; j >= 0; j--) {
            double x = newcontour.right_boundary[j].x;
            double y = newcontour.right_boundary[j].y;
            int break_mark = 0;
            for (int i = right_end; i < right_boundary.size() - 1; i++) {//**
                if (right_boundary[i].y <= y && y < right_boundary[i + 1].y)
                {
                    double a1 = right_boundary[i].x;
                    double b1 = right_boundary[i].y;
                    double a2 = right_boundary[i + 1].x;
                    double b2 = right_boundary[i + 1].y;
                    double xcross = (y - b1) * (a2 - a1) / (b2 - b1) + a1;
                    cross = { xcross,y };
                    if (xcross - x > 0) temp.pop_back();
                    else {
                        temp.insert(temp.end(), cross);
                        temp.insert(temp.end(), right_boundary.begin() + i + 1, right_boundary.end());
                        break_mark = 1;
                        break;
                    }
                }
            }
            if (break_mark == 1) break;
        }
    }

    if (new_right_start == -1) {
        temp = right_boundary;
    }
    right_boundary = temp;

    temp.clear();

    if (bottom_boundary[0].x > bottom_boundary[bottom_boundary.size() - 1].x) reverse(bottom_boundary.begin(), bottom_boundary.end());
    if (newcontour.bottom_boundary[0].x > newcontour.bottom_boundary[newcontour.bottom_boundary.size() - 1].x) {
        reverse(newcontour.bottom_boundary.begin(), newcontour.bottom_boundary.end());
    }
    new_start = -1;
    int bottom_end = -1;
    local_y = -1;
    for (int i = 0; i < newcontour.bottom_boundary.size(); i++) {
        double x = newcontour.bottom_boundary[i].x;
        int flag = -1;
        for (int j = 0; j < bottom_boundary.size() - 1; j++) {
            if (bottom_boundary[j].x <= x && x < bottom_boundary[j + 1].x) {
                double a1 = bottom_boundary[j].x;
                double b1 = bottom_boundary[j].y;
                double a2 = bottom_boundary[j + 1].x;
                double b2 = bottom_boundary[j + 1].y;
                double ycross = (b2 - b1) / (a2 - a1) * (x - a1) + b1;
                local_y = newcontour.bottom_boundary[i].y - ycross;
                bottom_end = j;//**
                flag = 1;
                cross = { x,ycross };
            }
        }
        if (local_y <= 0) {
            new_start = i;
            break;
        }
        else if (flag == -1) {
            bottom_end = bottom_boundary.size() - 1;
            new_start = i;
            if (i > 0) {
                x = bottom_boundary[bottom_end].x;
                double a1 = newcontour.bottom_boundary[i - 1].x;
                double b1 = newcontour.bottom_boundary[i - 1].y;
                double a2 = newcontour.bottom_boundary[i].x;
                double b2 = newcontour.bottom_boundary[i].y;
                double ycross = (b2 - b1) / (a2 - a1) * (x - a1) + b1;
                cross = { x,ycross };
            }
            break;
        }
    }
    whether_be_covered = false;
    if (new_start > 0) {
        double x = bottom_boundary[bottom_end].x;
        double a1 = newcontour.bottom_boundary[new_start - 1].x;
        double b1 = newcontour.bottom_boundary[new_start - 1].y;
        double a2 = newcontour.bottom_boundary[new_start].x;
        double b2 = newcontour.bottom_boundary[new_start].y;
        double ycross = (b2 - b1) / (a2 - a1) * (x - a1) + b1;
        if (bottom_boundary[bottom_end].y - ycross > 0) whether_be_covered = true;
    }
    while ((whether_be_covered && new_start > 0) && bottom_end >= 0) {
        double x = bottom_boundary[bottom_end].x;
        double a1 = newcontour.bottom_boundary[new_start - 1].x;
        double b1 = newcontour.bottom_boundary[new_start - 1].y;
        double a2 = newcontour.bottom_boundary[new_start].x;
        double b2 = newcontour.bottom_boundary[new_start].y;
        double ycross = (b2 - b1) / (a2 - a1) * (x - a1) + b1;
        if (bottom_boundary[bottom_end].y - ycross > 0) {
            bottom_end--;
        }
        else {
            whether_be_covered = false;
            cross = { x,ycross };
        }
    }

    if (new_start != -1) {
        temp.assign(bottom_boundary.begin(), bottom_boundary.begin() + bottom_end + 1);
        temp.insert(temp.end(), cross);
        temp.insert(temp.end(), newcontour.bottom_boundary.begin() + new_start, newcontour.bottom_boundary.end());
    }

    if (bottom_boundary[bottom_boundary.size() - 1].x > newcontour.bottom_boundary[newcontour.bottom_boundary.size() - 1].x && new_start != -1) {
        for (int j = newcontour.bottom_boundary.size() - 1; j >= 0; j--) {
            double x = newcontour.bottom_boundary[j].x;
            double y = newcontour.bottom_boundary[j].y;
            int break_mark = 0;
            for (int i = bottom_end; i < bottom_boundary.size() - 1; i++) {//**
                if (bottom_boundary[i].x <= x && x < bottom_boundary[i + 1].x)
                {
                    double a1 = bottom_boundary[i].x;
                    double b1 = bottom_boundary[i].y;
                    double a2 = bottom_boundary[i + 1].x;
                    double b2 = bottom_boundary[i + 1].y;
                    double ycross = (b2 - b1) / (a2 - a1) * (x - a1) + b1;
                    cross = { x,ycross };
                    if (ycross - y < 0) temp.pop_back();
                    else {
                        temp.insert(temp.end(), cross);
                        temp.insert(temp.end(), bottom_boundary.begin() + i + 1, bottom_boundary.end());
                        break_mark = 1;
                        break;
                    }
                }
            }
            if (break_mark == 1) break;
        }
    }
    if (new_start == -1) {
        temp = bottom_boundary;
    }
    bottom_boundary = temp;

    temp.clear();
    if (left_boundary[0].y > left_boundary[left_boundary.size() - 1].y) reverse(left_boundary.begin(), left_boundary.end());
    if (newcontour.left_boundary[0].y > newcontour.left_boundary[newcontour.left_boundary.size() - 1].y) reverse(newcontour.left_boundary.begin(), newcontour.left_boundary.end());
    int new_left_start = -1;
    int left_end = -1;
    local_x = -1;
    for (int i = 0; i < newcontour.left_boundary.size(); i++) {
        double y = newcontour.left_boundary[i].y;
        int flag = -1;
        for (int j = 0; j < left_boundary.size() - 1; j = j++) {
            if (left_boundary[j].y <= y && y < left_boundary[j + 1].y) {
                double a1 = left_boundary[j].x;
                double b1 = left_boundary[j].y;
                double a2 = left_boundary[j + 1].x;
                double b2 = left_boundary[j + 1].y;
                double xcross = (y - b1) * (a2 - a1) / (b2 - b1) + a1;
                local_x = newcontour.left_boundary[i].x - xcross;
                left_end = j;//**
                flag = 1;
                cross = { xcross,y };
            }
        }
        if (local_x <= 0) {
            new_left_start = i;
            break;
        }
        else if (flag == -1) {
            new_left_start = i;
            left_end = left_boundary.size() - 1;
            if (i > 0) {
                y = left_boundary[left_end].y;
                double a1 = newcontour.left_boundary[i - 1].x;
                double b1 = newcontour.left_boundary[i - 1].y;
                double a2 = newcontour.left_boundary[i].x;
                double b2 = newcontour.left_boundary[i].y;
                double xcross = (y - b1) * (a2 - a1) / (b2 - b1) + a1;
                cross = { xcross,y };
            }
            break;
        }
    }
    whether_be_covered = false;
    if (new_left_start > 0) {
        double y = left_boundary[left_end].y;
        double a1 = newcontour.left_boundary[new_left_start - 1].x;
        double b1 = newcontour.left_boundary[new_left_start - 1].y;
        double a2 = newcontour.left_boundary[new_left_start].x;
        double b2 = newcontour.left_boundary[new_left_start].y;
        double xcross = (y - b1) * (a2 - a1) / (b2 - b1) + a1;
        if (left_boundary[left_end].x - xcross > 0) whether_be_covered = true;
    }
    while ((whether_be_covered && new_left_start > 0) && left_end >= 0) {
        double y = left_boundary[left_end].y;
        double a1 = newcontour.left_boundary[new_left_start - 1].x;
        double b1 = newcontour.left_boundary[new_left_start - 1].y;
        double a2 = newcontour.left_boundary[new_left_start].x;
        double b2 = newcontour.left_boundary[new_left_start].y;
        double xcross = (y - b1) * (a2 - a1) / (b2 - b1) + a1;
        if (left_boundary[left_end].x - xcross > 0) {

            left_end--;
        }
        else {
            whether_be_covered = false;
            cross = { xcross,y };
        }
    }

    if (new_left_start != -1) {
        temp.assign(left_boundary.begin(), left_boundary.begin() + left_end + 1);
        temp.insert(temp.end(), cross);
        temp.insert(temp.end(), newcontour.left_boundary.begin() + new_left_start, newcontour.left_boundary.end());
    }

    if (left_boundary[left_boundary.size() - 1].y > newcontour.left_boundary[newcontour.left_boundary.size() - 1].y && new_left_start != -1) {
        for (int j = newcontour.left_boundary.size() - 1; j >= 0; j--) {
            double x = newcontour.left_boundary[j].x;
            double y = newcontour.left_boundary[j].y;
            int break_mark = 0;
            for (int i = left_end; i < left_boundary.size() - 1; i++) {//**
                if (left_boundary[i].y <= y && y < left_boundary[i + 1].y)
                {
                    double a1 = left_boundary[i].x;
                    double b1 = left_boundary[i].y;
                    double a2 = left_boundary[i + 1].x;
                    double b2 = left_boundary[i + 1].y;
                    double xcross = (y - b1) * (a2 - a1) / (b2 - b1) + a1;
                    cross = { xcross,y };
                    if (xcross - x < 0) temp.pop_back();
                    else {
                        temp.insert(temp.end(), cross);
                        temp.insert(temp.end(), left_boundary.begin() + i + 1, left_boundary.end());
                        break_mark = 1;
                        break;
                    }
                }
            }
            if (break_mark == 1) break;
        }
    }
    if (new_left_start == -1) {
        temp = left_boundary;
    }
    left_boundary = temp;
    boundingbox[0].y = max(boundingbox[0].y, newcontour.boundingbox[0].y);
    boundingbox[2].x = max(boundingbox[2].x, newcontour.boundingbox[2].x);
    boundingbox[3].x = boundingbox[2].x;
    boundingbox[3].y = boundingbox[0].y;
    boundingbox[4] = boundingbox[0];
    //all_contour_coord.insert(all_contour_coord.end(),newcontour.outercontour_world.begin(), newcontour.outercontour_world.end());

    /*int right_top_right_connction_index;
    for (int i = 0; i < right_boundary.size(); i++) {
        if (right_boundary[i].y == boundingbox[0].y) {
            right_top_right_connction_index = i;
            break;
        }
   }
    int top_top_right_connction_index;
    for (int i = upper_boundary.size() - 1; i >= 0; i--) {
        if (upper_boundary[i].y == boundingbox[0].y) {
            top_top_right_connction_index = i;
            break;
        }
    }
    temp.clear();
    temp.assign(upper_boundary.begin(), upper_boundary.begin() + top_top_right_connction_index);
    vector<Point2d> ttemp;
    ttemp.assign(right_boundary.begin(), right_boundary.begin() + right_top_right_connction_index);
    reverse(ttemp.begin(), ttemp.end());
    temp.insert(temp.end(), ttemp.begin(), ttemp.end());
    upper_right_boundary = temp;*/
}

container::container() {

};
container::container(vector<double> limit, group first) {
    xlimit = limit[0];
    ylimit = limit[1];
    group_inside_container.push_back(first);
};
int container::add_group_vertically(group upper, double cutting_gap, double x_shift) {
    group lower = group_inside_container[group_inside_container.size() - 1];
    upper.shift({ -upper.boundingbox[1].x,lower.boundingbox[0].y - upper.boundingbox[1].y });
    double min_area = INFINITY;
    group output;
    if (lower.upper_boundary[lower.upper_boundary.size() - 1].x < lower.upper_boundary[0].x) reverse(lower.upper_boundary.begin(), lower.upper_boundary.end());
    if (upper.bottom_boundary[upper.bottom_boundary.size() - 1].x < upper.bottom_boundary[0].x) reverse(upper.bottom_boundary.begin(), upper.bottom_boundary.end());
    while (upper.boundingbox[3].x < xlimit) {
        double maxdistance = INFINITY;
        for (int i = 0; i < upper.bottom_boundary.size(); i++) {
            double x = upper.bottom_boundary[i].x;
            int left = 0;
            int right = lower.upper_boundary.size() - 1;
            int result = -1;
            int pp = 0;
            while (left <= right) {
                int mid = (left + right) / 2;
                if (mid + 1 > lower.upper_boundary.size() - 1) break;
                if (lower.upper_boundary[mid].x <= x && x < lower.upper_boundary[mid + 1].x) {
                    result = mid;
                    pp = -1;
                    break;
                }
                else if (lower.upper_boundary[mid].x == x && x == lower.upper_boundary[mid + 1].x) {
                    result = mid;
                    pp = 1;
                    break;
                }
                if (lower.upper_boundary[mid].x < x && lower.upper_boundary[mid + 1].x <= x) {
                    left = mid + 1;
                }
                else {
                    right = mid - 1;
                }
            }
            if (pp == -1) {
                double a1 = lower.upper_boundary[result].x;
                double b1 = lower.upper_boundary[result].y;
                double a2 = lower.upper_boundary[result + 1].x;
                double b2 = lower.upper_boundary[result + 1].y;
                double ycross = (b2 - b1) / (a2 - a1) * (x - a1) + b1;
                double local_distance = abs(upper.bottom_boundary[i].y - ycross);
                if (local_distance < maxdistance) {
                    maxdistance = local_distance;
                }
            }
            else if (pp == 1) {
                double y_upper = max(lower.upper_boundary[result].y, lower.upper_boundary[result + 1].y);
                double local_distance = abs(upper.bottom_boundary[i].y - y_upper);
                if (local_distance < maxdistance) {
                    maxdistance = local_distance;
                }
            }
        }
        for (int i = 0; i < lower.upper_boundary.size(); i++) {
            double x = lower.upper_boundary[i].x;
            int left = 0;
            int right = upper.bottom_boundary.size() - 1;
            int result = -1;
            int pp = 0;
            while (left <= right) {
                int mid = (left + right) / 2;
                if (mid + 1 > upper.bottom_boundary.size() - 1) break;
                if (upper.bottom_boundary[mid].x <= x && x < upper.bottom_boundary[mid + 1].x) {
                    result = mid;
                    pp = -1;
                    break;
                }
                else if (upper.bottom_boundary[mid].x == x && x == upper.bottom_boundary[mid + 1].x) {
                    result = mid;
                    pp = 1;
                    break;
                }
                if (upper.bottom_boundary[mid].x < x && upper.bottom_boundary[mid + 1].x <= x) {
                    left = mid + 1;
                }
                else {
                    right = mid - 1;
                }
            }

            if (pp == -1) {
                double a1 = upper.bottom_boundary[result].x;
                double b1 = upper.bottom_boundary[result].y;
                double a2 = upper.bottom_boundary[result + 1].x;
                double b2 = upper.bottom_boundary[result + 1].y;
                double ycross = (b2 - b1) / (a2 - a1) * (x - a1) + b1;
                double local_distance = abs(lower.upper_boundary[i].y - ycross);
                if (local_distance < maxdistance) {
                    maxdistance = local_distance;
                }
            }
            else if (pp == 1) {
                double y_upper = min(upper.bottom_boundary[result].y, upper.bottom_boundary[result + 1].y);
                double local_distance = abs(lower.upper_boundary[i].y - y_upper);
                if (local_distance < maxdistance) {
                    maxdistance = local_distance;
                }
            }
        }
        /*canv.draw_all_element(upper, 0.5);
        canv.draw_all_element(lower, 0.5);
        canv.show("whiteboard", NULL);*/
        upper.shift({ 0,-maxdistance + cutting_gap });
        /*canv.draw_all_element(upper, 0.5);
        canv.show("whiteboard", NULL);
        canv.refresh();*/
        double local_area;
        if (upper.boundingbox[0].y > ylimit || upper.boundingbox[3].x > xlimit) local_area = INFINITY;
        else {
            local_area = (upper.boundingbox[0].y - upper.boundingbox[1].y) * (upper.boundingbox[3].x - upper.boundingbox[0].x);
            local_area += (lower.boundingbox[0].y - lower.boundingbox[1].y) * (lower.boundingbox[3].x - lower.boundingbox[0].x);
            double overlap_width = min(upper.boundingbox[3].x, lower.boundingbox[3].x) - max(upper.boundingbox[0].x, lower.boundingbox[0].x);
            double overlap_height = min(upper.boundingbox[3].y, lower.boundingbox[3].y) - max(upper.boundingbox[1].y, lower.boundingbox[1].y);
            local_area -= overlap_height * overlap_width;
        }
        if (local_area < min_area) {
            min_area = local_area;
            output = upper;
        }
        upper.shift({ 0,maxdistance - cutting_gap });
        upper.shift({ x_shift,0 });
    }
    if (min_area == INFINITY) {
        return -1;
    }
    else {
        group_inside_container.push_back(output);
        return 1;
    }
}
void container::shift(vector<double> shift_amount) {
    for (int i = 0; i < group_inside_container.size(); i++) {
        group_inside_container[i].shift(shift_amount);
    }
};
void container::output() {
    fstream fp;
    fp.open("C://Users/wades/Desktop/DXF Decomposer/output.txt", ios::out);//開啟檔案
    if (!fp) {//如果開啟檔案失敗，fp為0；成功，fp為非0
        cout << "Fail to open file\n";
    }
    for (int i = 0; i < group_inside_container.size(); i++) {
        for (int j = 0; j < group_inside_container[i].contour_inside_boundary.size(); j++) {
            for (int k = 0; k < group_inside_container[i].contour_inside_boundary[j].outercontour_world.size(); k++) {
                fp << group_inside_container[i].contour_inside_boundary[j].outercontour_world[k].x << " " << group_inside_container[i].contour_inside_boundary[j].outercontour_world[k].y << "\n";
            }
            fp << "\n";
        }
    }
}

canvas::canvas() {};
canvas::canvas(int w, int h, int set, int shift) {
    width = w;
    height = h;
    y_shift = h - shift;
    x_shift = 0 + shift;
    setTo_value = set;
    img = Mat::zeros(Size(width, height), CV_8UC3);
    img.setTo(setTo_value);
    img.copyTo(blank);
}
void canvas::refresh() {
    blank.copyTo(img);
}
void canvas::write() {
    imwrite("result.png", img);
}
void canvas::draw_center(Scalar color, contour& cont, double zoom_ratio, int center_radius) {
    Point2d p1(x_shift + cont.center.x * zoom_ratio, y_shift - cont.center.y * zoom_ratio);
    circle(img, p1, center_radius, color, 0);
}
void canvas::draw_outercontour(Scalar color, contour& cont, double zoom_ratio, int line_width) {
    for (int i = 0; i < cont.outercontour_world.size() && i + 1 < cont.outercontour_world.size(); i++) {
        Point2d p1(x_shift + cont.outercontour_world[i].x * zoom_ratio, y_shift - cont.outercontour_world[i].y * zoom_ratio);
        Point2d p2(x_shift + cont.outercontour_world[i + 1].x * zoom_ratio, y_shift - cont.outercontour_world[i + 1].y * zoom_ratio);
        line(img, p1, p2, color, line_width);
        circle(img, p1, 1, (0, 0, 0), 0);
    }
}
void canvas::draw_boundingbox(Scalar color, contour& cont, double zoom_ratio, int line_width) {
    for (int i = 0; i < cont.boundingbox.size() && i + 1 < cont.boundingbox.size(); i++) {
        Point2d p1(x_shift + cont.boundingbox[i].x * zoom_ratio, y_shift - cont.boundingbox[i].y * zoom_ratio);
        Point2d p2(x_shift + cont.boundingbox[i + 1].x * zoom_ratio, y_shift - cont.boundingbox[i + 1].y * zoom_ratio);
        line(img, p1, p2, color, line_width);
        circle(img, p1, 1, (0, 0, 0), 0);
    }
}
void canvas::draw_minboundingbox(Scalar color, contour& cont, double zoom_ratio, int line_width) {
    for (int i = 0; i < cont.min_box_world.size() && i + 1 < cont.min_box_world.size(); i++) {
        Point2d p1(x_shift + cont.min_box_world[i].x * zoom_ratio, y_shift - cont.min_box_world[i].y * zoom_ratio);
        Point2d p2(x_shift + cont.min_box_world[i + 1].x * zoom_ratio, y_shift - cont.min_box_world[i + 1].y * zoom_ratio);
        line(img, p1, p2, color, line_width);
        circle(img, p1, 1, (0, 0, 0), 0);
    }
}
void canvas::draw_convexHull(Scalar color, vector<Point>& hull, double zoom_ratio, int line_width) {
    for (int i = 0; i < hull.size() && i + 1 < hull.size(); i++) {
        Point2d p1(x_shift + hull[i].x * zoom_ratio, y_shift - hull[i].y * zoom_ratio);
        Point2d p2(x_shift + hull[i + 1].x * zoom_ratio, y_shift - hull[i + 1].y * zoom_ratio);
        line(img, p1, p2, color, line_width);
        circle(img, p1, 1, (0, 0, 0), 0);
    }
}
void canvas::draw_boundary(Scalar color, vector<Point2d>& bounary, double zoom_ratio, int line_width) {
    for (int i = 0; i < bounary.size() && i + 1 < bounary.size(); i++) {
        Point2d p1(x_shift + bounary[i].x * zoom_ratio, y_shift - bounary[i].y * zoom_ratio);
        Point2d p2(x_shift + bounary[i + 1].x * zoom_ratio, y_shift - bounary[i + 1].y * zoom_ratio);
        line(img, p1, p2, color, line_width);
        circle(img, p1, 1, (0, 0, 0), 0);
    }
}
void canvas::draw_all_element(contour& cont, double zoom_ratio) {
    //draw center
    Point p1(x_shift + cont.center.x * zoom_ratio, y_shift - cont.center.y * zoom_ratio);
    circle(img, p1, 5, (0, 0, 0), 0);

    //draw outer contour
    for (int i = 0; i < cont.outercontour_world.size() && i + 1 < cont.outercontour_world.size(); i++) {
        Point p1(x_shift + cont.outercontour_world[i].x * zoom_ratio, y_shift - cont.outercontour_world[i].y * zoom_ratio);
        Point p2(x_shift + cont.outercontour_world[i + 1].x * zoom_ratio, y_shift - cont.outercontour_world[i + 1].y * zoom_ratio);
        line(img, p1, p2, Scalar(255, 0, 0), 1);
        //circle(img, p1, 1, (0, 0, 0), 0);
    }
    /*//draw bounding box
    for (int i = 0; i < cont.boundingbox.size() && i + 1 < cont.boundingbox.size(); i++) {
        Point p1(x_shift + cont.boundingbox[i].x * zoom_ratio, y_shift - cont.boundingbox[i].y * zoom_ratio);
        Point p2(x_shift + cont.boundingbox[i + 1].x * zoom_ratio, y_shift - cont.boundingbox[i + 1].y * zoom_ratio);
        line(img, p1, p2, Scalar(0, 255, 0), 1);
        circle(img, p1, 1, (0, 0, 0), 0);
    }
    //draw min_bounding box
    for (int i = 0; i < cont.min_box_world.size() && i + 1 < cont.min_box_world.size(); i++) {
        Point p1(x_shift + cont.min_box_world[i].x * zoom_ratio, y_shift - cont.min_box_world[i].y * zoom_ratio);
        Point p2(x_shift + cont.min_box_world[i + 1].x * zoom_ratio, y_shift - cont.min_box_world[i + 1].y * zoom_ratio);
        line(img, p1, p2, Scalar(255, 0, 0), 1);
        circle(img, p1, 1, (0, 0, 0), 0);
    }*/
}
void canvas::draw_all_element(group& b, double zoom_ratio) {
    for (int i = 0; i < b.contour_inside_boundary.size(); i++) {
        contour cont = b.contour_inside_boundary[i];
        //draw center
        Point p1(x_shift + cont.center.x * zoom_ratio, y_shift - cont.center.y * zoom_ratio);
        circle(img, p1, 3, (0, 0, 0), 0);
        //draw outer contour
        for (int i = 0; i < cont.outercontour_world.size() && i + 1 < cont.outercontour_world.size(); i++) {
            Point p1(x_shift + cont.outercontour_world[i].x * zoom_ratio, y_shift - cont.outercontour_world[i].y * zoom_ratio);
            Point p2(x_shift + cont.outercontour_world[i + 1].x * zoom_ratio, y_shift - cont.outercontour_world[i + 1].y * zoom_ratio);
            line(img, p1, p2, Scalar(255, 255, 0), 1);
            //circle(img, p1, 1, (0, 0, 0), 0);
        }
        //draw bounding box
        /*for (int i = 0; i < cont.boundingbox.size() && i + 1 < cont.boundingbox.size(); i++) {
            Point p1(x_shift + cont.boundingbox[i].x * zoom_ratio, y_shift - cont.boundingbox[i].y * zoom_ratio);
            Point p2(x_shift + cont.boundingbox[i + 1].x * zoom_ratio, y_shift - cont.boundingbox[i + 1].y * zoom_ratio);
            line(img, p1, p2, Scalar(0, 255, 0), 1);
            circle(img, p1, 1, (0, 0, 0), 0);
        }*/
        //draw min_bounding box
        /*for (int i = 0; i < cont.min_box_world.size() && i + 1 < cont.min_box_world.size(); i++) {
            Point p1(x_shift + cont.min_box_world[i].x * zoom_ratio, y_shift - cont.min_box_world[i].y * zoom_ratio);
            Point p2(x_shift + cont.min_box_world[i + 1].x * zoom_ratio, y_shift - cont.min_box_world[i + 1].y * zoom_ratio);
            line(img, p1, p2, Scalar(255, 0, 0), 1);
            circle(img, p1, 1, (0, 0, 0), 0);
        }*/
    }
    //draw boundary boundingbox
    /*for (int i = 0; i < b.boundingbox.size() && i + 1 < b.boundingbox.size(); i++) {
        Point p1(x_shift + b.boundingbox[i].x * zoom_ratio, y_shift - b.boundingbox[i].y * zoom_ratio);
        Point p2(x_shift + b.boundingbox[i + 1].x * zoom_ratio, y_shift - b.boundingbox[i + 1].y * zoom_ratio);
        line(img, p1, p2, Scalar(0, 255, 0), 1);
        circle(img, p1, 1, (0, 0, 0), 0);
    }*/
    //draw limit
    //Point p1(x_shift,y_shift );
    //circle(img, p1, 10, (0, 0, 0), 0);
    //Point p2(x_shift+b.xlimit*zoom_ratio,y_shift-b.ylimit*zoom_ratio);
    //circle(img, p2, 10, (0, 0, 0), 0);

    //rectangle(img, p1,p2,Scalar(0,0,0),1);

}
void canvas::draw_all_element(container& b, double zoom_ratio) {
    for (int i = 0; i < b.group_inside_container.size(); i++) {
        draw_all_element(b.group_inside_container[i], zoom_ratio);
    }
    //draw limit
    Point p1(x_shift, y_shift);
    //circle(img, p1, 10, (0, 0, 0), 0);
    Point p2(x_shift + b.xlimit * zoom_ratio, y_shift - b.ylimit * zoom_ratio);
    //circle(img, p2, 10, (0, 0, 0), 0);

    rectangle(img, p1, p2, Scalar(0, 0, 0), 1);
}
void canvas::show(string windowname, int waittime) {
    imshow(windowname, img);
    waitKey(waittime);
}
void canvas::draw_last_element(group& b, double zoom_ratio) {
    int k = b.contour_inside_boundary.size() - 1;
    contour cont = b.contour_inside_boundary[k];
    //draw center
    Point p1(x_shift + cont.center.x * zoom_ratio, y_shift - cont.center.y * zoom_ratio);
    circle(img, p1, 3, (0, 0, 0), 0);
    //draw outer contour
    for (int i = 0; i < cont.outercontour_world.size() && i + 1 < cont.outercontour_world.size(); i++) {
        Point p1(x_shift + cont.outercontour_world[i].x * zoom_ratio, y_shift - cont.outercontour_world[i].y * zoom_ratio);
        Point p2(x_shift + cont.outercontour_world[i + 1].x * zoom_ratio, y_shift - cont.outercontour_world[i + 1].y * zoom_ratio);
        line(img, p1, p2, Scalar(255, 255, 0), 1);
        //circle(img, p1, 1, (0, 0, 0), 0);
    }
}
void canvas::draw_limit(vector<double> limit, double zoom_ratio, int line_width) {
    //draw limit
    Point p1(x_shift, y_shift);
    //circle(img, p1, 10, (0, 0, 0), 0);
    Point p2(x_shift + limit[0] * zoom_ratio, y_shift - limit[1] * zoom_ratio);
    //circle(img, p2, 10, (0, 0, 0), 0);

    rectangle(img, p1, p2, Scalar(0, 0, 0), 1);
}

double approach_from_top_minarea(group& bottom, contour top, double angle, contour& output, vector<double> limit, double cutting_gap, int waittime, double x_shift, int zoom_ratio) {
    contour rot = top.rotate(angle);
    rot.shift({ bottom.boundingbox[0].x - rot.boundingbox[1].x,bottom.boundingbox[0].y - rot.boundingbox[1].y });
    double min_area = INFINITY;
    if (bottom.upper_boundary[bottom.upper_boundary.size() - 1].x < bottom.upper_boundary[0].x) reverse(bottom.upper_boundary.begin(), bottom.upper_boundary.end());
    while (rot.boundingbox[0].x <= bottom.boundingbox[2].x) {
        double maxdistance = INFINITY;
        rot.get_boundary();
        for (int i = 0; i < rot.bottom_boundary.size(); i++) {
            double x = rot.bottom_boundary[i].x;
            int left = 0;
            int right = bottom.upper_boundary.size() - 1;
            int result = -1;
            int pp = 0;
            while (left <= right) {
                int mid = (left + right) / 2;
                if (mid + 1 > bottom.upper_boundary.size() - 1) break;
                if (bottom.upper_boundary[mid].x <= x && x < bottom.upper_boundary[mid + 1].x) {
                    result = mid;
                    pp = -1;
                    break;
                }
                else if (bottom.upper_boundary[mid].x == x && x == bottom.upper_boundary[mid + 1].x) {
                    result = mid;
                    pp = 1;
                    break;
                }
                if (bottom.upper_boundary[mid].x < x && bottom.upper_boundary[mid + 1].x <= x) {
                    left = mid + 1;
                }
                else {
                    right = mid - 1;
                }
            }
            if (pp == -1) {
                double a1 = bottom.upper_boundary[result].x;
                double b1 = bottom.upper_boundary[result].y;
                double a2 = bottom.upper_boundary[result + 1].x;
                double b2 = bottom.upper_boundary[result + 1].y;
                double ycross = (b2 - b1) / (a2 - a1) * (x - a1) + b1;
                double local_distance = abs(rot.bottom_boundary[i].y - ycross);
                if (local_distance < maxdistance) {
                    maxdistance = local_distance;
                }
            }
            else if (pp == 1) {
                double y_upper = max(bottom.upper_boundary[result].y, bottom.upper_boundary[result + 1].y);
                double local_distance = abs(rot.bottom_boundary[i].y - y_upper);
                if (local_distance < maxdistance) {
                    maxdistance = local_distance;
                }
            }
        }

        for (int i = 0; i < bottom.upper_boundary.size(); i++) {
            double x = bottom.upper_boundary[i].x;
            int left = 0;
            int right = rot.bottom_boundary.size() - 1;
            int result = -1;
            int pp = 0;
            while (left <= right) {
                int mid = (left + right) / 2;
                if (mid + 1 > rot.bottom_boundary.size() - 1) break;
                if (rot.bottom_boundary[mid].x <= x && x < rot.bottom_boundary[mid + 1].x) {
                    result = mid;
                    pp = -1;
                    break;
                }
                else if (rot.bottom_boundary[mid].x == x && x == rot.bottom_boundary[mid + 1].x) {
                    result = mid;
                    pp = 1;
                    break;
                }
                if (rot.bottom_boundary[mid].x < x && rot.bottom_boundary[mid + 1].x <= x) {
                    left = mid + 1;
                }
                else {
                    right = mid - 1;
                }
            }

            if (pp == -1) {
                double a1 = rot.bottom_boundary[result].x;
                double b1 = rot.bottom_boundary[result].y;
                double a2 = rot.bottom_boundary[result + 1].x;
                double b2 = rot.bottom_boundary[result + 1].y;
                double ycross = (b2 - b1) / (a2 - a1) * (x - a1) + b1;
                double local_distance = abs(bottom.upper_boundary[i].y - ycross);
                if (local_distance < maxdistance) {
                    maxdistance = local_distance;
                }
            }
            else if (pp == 1) {
                double y_upper = min(rot.bottom_boundary[result].y, rot.bottom_boundary[result + 1].y);
                double local_distance = abs(bottom.upper_boundary[i].y - y_upper);
                if (local_distance < maxdistance) {
                    maxdistance = local_distance;
                }
            }
        }
        rot.shift({ 0,-maxdistance + cutting_gap });
        double local_area = 0;
        if (rot.boundingbox[0].y > limit[1] || rot.boundingbox[3].x > limit[0]) local_area = INFINITY;
        else {
            local_area = (rot.boundingbox[0].y - rot.boundingbox[1].y) * (rot.boundingbox[3].x - rot.boundingbox[0].x);
            local_area += (bottom.boundingbox[0].y - bottom.boundingbox[1].y) * (bottom.boundingbox[3].x - bottom.boundingbox[0].x);
            double overlap_width = min(rot.boundingbox[3].x, bottom.boundingbox[3].x) - max(rot.boundingbox[0].x, bottom.boundingbox[0].x);
            double overlap_height = min(rot.boundingbox[3].y, bottom.boundingbox[3].y) - max(rot.boundingbox[1].y, bottom.boundingbox[1].y);
            local_area -= overlap_height * overlap_width;
        }
        if (local_area < min_area) {
            min_area = local_area;
            output = rot;
        }
        rot.shift({ 0,maxdistance - cutting_gap });
        rot.shift({ x_shift,0 });
    }
    return  min_area;
}
double approach_from_right_minarea(group& left, contour right, double angle, contour& output, vector<double> limit, double cutting_gap, int waittime, double y_shift, int zoom_ratio) {
    contour rot = right.rotate(angle);
    //rot.drawcontour(zoom_ratio, img);
    //left.drawcontour(zoom_ratio, img);
    //imshow("whiteboard", img);
    //waitKey(time);
    rot.shift({ left.boundingbox[3].x - rot.boundingbox[1].x,left.boundingbox[3].y - rot.boundingbox[1].y });
    double min_area = INFINITY;

    while (rot.boundingbox[2].y >= left.boundingbox[2].y) {
        double maxdistance = INFINITY;
        rot.get_boundary();
        for (int i = 0; i < rot.left_boundary.size(); i = i++) {
            double y = rot.left_boundary[i].y;
            int l = 0;
            int r = left.right_boundary.size() - 1;
            int result = -1;
            int pp = 0;

            while (l <= r) {
                int mid = (l + r) / 2;
                if (mid + 1 > left.right_boundary.size() - 1) break;
                if (left.right_boundary[mid].y <= y && y < left.right_boundary[mid + 1].y) {
                    result = mid;
                    pp = -1;
                    break;
                }
                else if (left.right_boundary[mid].y == y && y == left.right_boundary[mid + 1].y) {
                    result = mid;
                    pp = 1;
                    break;
                }
                if (left.right_boundary[mid].y < y && left.right_boundary[mid + 1].y <= y) {
                    l = mid + 1;
                }
                else {
                    r = mid - 1;
                }
            }

            if (pp == -1) {
                double a1 = left.right_boundary[result].x;
                double b1 = left.right_boundary[result].y;
                double a2 = left.right_boundary[result + 1].x;
                double b2 = left.right_boundary[result + 1].y;
                double xcross = (y - b1) * (a2 - a1) / (b2 - b1) + a1;
                double local_distance = abs(rot.left_boundary[i].x - xcross);
                if (local_distance < maxdistance) {
                    maxdistance = local_distance;
                }
            }
            else if (pp == 1) {
                double x_righter = max(left.right_boundary[result].x, left.right_boundary[result + 1].x);
                double local_distance = abs(rot.left_boundary[i].x - x_righter);
                if (local_distance < maxdistance) {
                    maxdistance = local_distance;
                }
            }
        }
        reverse(rot.left_boundary.begin(), rot.left_boundary.end());

        for (int i = 0; i < left.right_boundary.size(); i++) {
            double y = left.right_boundary[i].y;
            int l = 0;
            int r = rot.left_boundary.size() - 1;
            int result = -1;
            int pp = 0;

            while (l <= r) {
                int mid = (l + r) / 2;
                if (mid + 1 > rot.left_boundary.size() - 1) break;
                if (rot.left_boundary[mid].y <= y && y < rot.left_boundary[mid + 1].y) {
                    result = mid;
                    pp = -1;
                    break;
                }
                else if (rot.left_boundary[mid].y == y && y == rot.left_boundary[mid + 1].y) {
                    result = mid;
                    pp = 1;
                    break;
                }
                if (rot.left_boundary[mid].y < y && rot.left_boundary[mid + 1].y <= y) {
                    l = mid + 1;
                }
                else {
                    r = mid - 1;
                }
            }
            if (pp == -1) {
                double a1 = rot.left_boundary[result].x;
                double b1 = rot.left_boundary[result].y;
                double a2 = rot.left_boundary[result + 1].x;
                double b2 = rot.left_boundary[result + 1].y;
                double xcross = (y - b1) * (a2 - a1) / (b2 - b1) + a1;
                double local_distance = abs(left.right_boundary[i].x - xcross);
                if (local_distance < maxdistance) {
                    maxdistance = local_distance;
                }
            }
            else if (pp == 1) {
                double x_righter = min(rot.left_boundary[result].x, rot.left_boundary[result + 1].x);
                double local_distance = abs(left.right_boundary[i].x - x_righter);
                if (local_distance < maxdistance) {
                    maxdistance = local_distance;
                }
            }
        }
        rot.shift({ -maxdistance + cutting_gap,0 });
        double local_area = 0;
        if (rot.boundingbox[0].y > limit[1] || rot.boundingbox[3].x > limit[0]) local_area = INFINITY;
        else {
            local_area = (rot.boundingbox[0].y - rot.boundingbox[1].y) * (rot.boundingbox[3].x - rot.boundingbox[0].x);
            local_area += (left.boundingbox[0].y - left.boundingbox[1].y) * (left.boundingbox[3].x - left.boundingbox[0].x);
            double overlap_width = min(rot.boundingbox[3].x, left.boundingbox[3].x) - max(rot.boundingbox[0].x, left.boundingbox[0].x);
            double overlap_height = min(rot.boundingbox[3].y, left.boundingbox[3].y) - max(rot.boundingbox[1].y, left.boundingbox[1].y);
            local_area -= overlap_height * overlap_width;
        }
        if (local_area < min_area) {
            min_area = local_area;
            output = rot;
        }
        rot.shift({ maxdistance - cutting_gap,0 });
        rot.shift({ 0,y_shift });
    }
    return  min_area;
}
double approach_from_bottom_minarea(group& top, contour bottom, double angle, contour& output, vector<double> limit, double cutting_gap, int waittime, double x_shift, int zoom_ratio) {
    contour rot = bottom.rotate(angle);
    rot.shift({ top.boundingbox[1].x - rot.boundingbox[0].x,top.boundingbox[1].y - rot.boundingbox[0].y });
    double min_area = INFINITY;
    if (top.bottom_boundary[top.bottom_boundary.size() - 1].x < top.bottom_boundary[0].x) reverse(top.bottom_boundary.begin(), top.bottom_boundary.end());

    while (rot.boundingbox[0].x <= top.boundingbox[2].x) {
        double maxdistance = INFINITY;
        rot.get_boundary();
        reverse(rot.upper_boundary.begin(), rot.upper_boundary.end());
        for (int i = 0; i < rot.upper_boundary.size(); i++) {
            double x = rot.upper_boundary[i].x;
            int left = 0;
            int right = top.bottom_boundary.size() - 1;
            int result = -1;
            int pp = 0;
            while (left <= right) {
                int mid = (left + right) / 2;
                if (mid + 1 > top.bottom_boundary.size() - 1) break;
                if (top.bottom_boundary[mid].x <= x && x < top.bottom_boundary[mid + 1].x) {
                    result = mid;
                    pp = -1;
                    break;
                }
                else if (top.bottom_boundary[mid].x == x && x == top.bottom_boundary[mid + 1].x) {
                    result = mid;
                    pp = 1;
                    break;
                }
                if (top.bottom_boundary[mid].x < x && top.bottom_boundary[mid + 1].x <= x) {
                    left = mid + 1;
                }
                else {
                    right = mid - 1;
                }
            }
            if (pp == -1) {
                double a1 = top.bottom_boundary[result].x;
                double b1 = top.bottom_boundary[result].y;
                double a2 = top.bottom_boundary[result + 1].x;
                double b2 = top.bottom_boundary[result + 1].y;
                double ycross = (b2 - b1) / (a2 - a1) * (x - a1) + b1;
                double local_distance = abs(rot.upper_boundary[i].y - ycross);
                if (local_distance < maxdistance) {
                    maxdistance = local_distance;
                }
            }
            else if (pp == 1) {
                double y_upper = min(top.bottom_boundary[result].y, top.bottom_boundary[result + 1].y);
                double local_distance = abs(rot.upper_boundary[i].y - y_upper);
                if (local_distance < maxdistance) {
                    maxdistance = local_distance;
                }
            }
        }

        for (int i = 0; i < top.bottom_boundary.size(); i++) {
            double x = top.bottom_boundary[i].x;
            int left = 0;
            int right = rot.upper_boundary.size() - 1;
            int result = -1;
            int pp = 0;
            while (left <= right) {
                int mid = (left + right) / 2;
                if (mid + 1 > rot.upper_boundary.size() - 1) break;
                if (rot.upper_boundary[mid].x <= x && x < rot.upper_boundary[mid + 1].x) {
                    result = mid;
                    pp = -1;
                    break;
                }
                else if (rot.upper_boundary[mid].x == x && x == rot.upper_boundary[mid + 1].x) {
                    result = mid;
                    pp = 1;
                    break;
                }
                if (rot.upper_boundary[mid].x < x && rot.upper_boundary[mid + 1].x <= x) {
                    left = mid + 1;
                }
                else {
                    right = mid - 1;
                }
            }

            if (pp == -1) {
                double a1 = rot.upper_boundary[result].x;
                double b1 = rot.upper_boundary[result].y;
                double a2 = rot.upper_boundary[result + 1].x;
                double b2 = rot.upper_boundary[result + 1].y;
                double ycross = (b2 - b1) / (a2 - a1) * (x - a1) + b1;
                double local_distance = abs(top.bottom_boundary[i].y - ycross);
                if (local_distance < maxdistance) {
                    maxdistance = local_distance;
                }
            }
            else if (pp == 1) {
                double y_upper = max(rot.upper_boundary[result].y, rot.upper_boundary[result + 1].y);
                double local_distance = abs(top.bottom_boundary[i].y - y_upper);
                if (local_distance < maxdistance) {
                    maxdistance = local_distance;
                }
            }
        }

        rot.shift({ 0,maxdistance });
        vector<Point> hull;
        double local_area = 0;
        /*canvas canv(1200, 800, 255);
        canv.draw_all_element(rot, zoom_ratio);
        canv.draw_all_element(top, zoom_ratio);
        canv.show("whiteboard", NULL);*/

        if (rot.boundingbox[0].y > top.ylimit || rot.boundingbox[3].x > top.xlimit || rot.boundingbox[1].y < 0) local_area = INFINITY;
        else {
            vector<Point2d> all_contour_coord = rot.outercontour_world;
            all_contour_coord.insert(all_contour_coord.end(), top.convex_hull.begin(), top.convex_hull.end());
            vector<Point> v(all_contour_coord.begin(), all_contour_coord.end());
            convexHull(v, hull);
            local_area = contourArea(hull);
        }


        if (local_area < min_area) {
            min_area = local_area;
            output = rot;
        }
        /*canvas canv(1200, 800, 255);
        canv.draw_all_element(rot, zoom_ratio);
        canv.draw_all_element(bottom, zoom_ratio);
        canv.draw_convexHull((0, 0, 0), hull, zoom_ratio);
        canv.show("whiteboard",waittime);*/
        rot.shift({ 0,-maxdistance });
        rot.shift({ x_shift,0 });
    }
    return  min_area;
}
double approach_from_left_minarea(group& right, contour left, double angle, contour& output, vector<double> limit, double cutting_gap, int waittime, double y_shift, int zoom_ratio) {
    contour rot = left.rotate(angle);
    //rot.drawcontour(zoom_ratio, img);
    //left.drawcontour(zoom_ratio, img);
    //imshow("whiteboard", img);
    //waitKey(time);
    rot.shift({ right.boundingbox[0].x - rot.boundingbox[2].x,right.boundingbox[0].y - rot.boundingbox[2].y });
    double min_area = INFINITY;
    if (right.left_boundary[(right.left_boundary.size() - 1)].y < right.left_boundary[0].y) reverse(right.left_boundary.begin(), right.left_boundary.end());
    while (rot.boundingbox[2].y >= right.boundingbox[2].y) {
        double maxdistance = INFINITY;
        rot.get_boundary();
        for (int i = 0; i < rot.right_boundary.size(); i = i++) {
            double y = rot.right_boundary[i].y;
            int l = 0;
            int r = right.left_boundary.size() - 1;
            int result = -1;
            int pp = 0;

            while (l <= r) {
                int mid = (l + r) / 2;
                if (mid + 1 > right.left_boundary.size() - 1) break;
                if (right.left_boundary[mid].y <= y && y < right.left_boundary[mid + 1].y) {
                    result = mid;
                    pp = -1;
                    break;
                }
                else if (right.left_boundary[mid].y == y && y == right.left_boundary[mid + 1].y) {
                    result = mid;
                    pp = 1;
                    break;
                }
                if (right.left_boundary[mid].y < y && right.left_boundary[mid + 1].y <= y) {
                    l = mid + 1;
                }
                else {
                    r = mid - 1;
                }
            }

            if (pp == -1) {
                double a1 = right.left_boundary[result].x;
                double b1 = right.left_boundary[result].y;
                double a2 = right.left_boundary[result + 1].x;
                double b2 = right.left_boundary[result + 1].y;
                double xcross = (y - b1) * (a2 - a1) / (b2 - b1) + a1;
                double local_distance = abs(rot.right_boundary[i].x - xcross);
                if (local_distance < maxdistance) {
                    maxdistance = local_distance;
                }
            }
            else if (pp == 1) {
                double x_righter = min(right.left_boundary[result].x, right.left_boundary[result + 1].x);
                double local_distance = abs(rot.right_boundary[i].x - x_righter);
                if (local_distance < maxdistance) {
                    maxdistance = local_distance;
                }
            }
        }

        for (int i = 0; i < right.left_boundary.size(); i++) {
            double y = right.left_boundary[i].y;
            int l = 0;
            int r = rot.right_boundary.size() - 1;
            int result = -1;
            int pp = 0;

            while (l <= r) {
                int mid = (l + r) / 2;
                if (mid + 1 > rot.right_boundary.size() - 1) break;
                if (rot.right_boundary[mid].y <= y && y < rot.right_boundary[mid + 1].y) {
                    result = mid;
                    pp = -1;
                    break;
                }
                else if (rot.right_boundary[mid].y == y && y == rot.right_boundary[mid + 1].y) {
                    result = mid;
                    pp = 1;
                    break;
                }
                if (rot.right_boundary[mid].y < y && rot.right_boundary[mid + 1].y <= y) {
                    l = mid + 1;
                }
                else {
                    r = mid - 1;
                }
            }
            if (pp == -1) {
                double a1 = rot.right_boundary[result].x;
                double b1 = rot.right_boundary[result].y;
                double a2 = rot.right_boundary[result + 1].x;
                double b2 = rot.right_boundary[result + 1].y;
                double xcross = (y - b1) * (a2 - a1) / (b2 - b1) + a1;
                double local_distance = abs(right.left_boundary[i].x - xcross);
                if (local_distance < maxdistance) {
                    maxdistance = local_distance;
                }
            }
            else if (pp == 1) {
                double x_righter = min(rot.right_boundary[result].x, rot.right_boundary[result + 1].x);
                double local_distance = abs(right.left_boundary[i].x - x_righter);
                if (local_distance < maxdistance) {
                    maxdistance = local_distance;
                }
            }
        }
        rot.shift({ maxdistance,0 });
        double local_area = 0;
        vector<Point> hull;
        if (rot.boundingbox[0].y > right.ylimit || rot.boundingbox[3].x > right.xlimit || rot.boundingbox[0].x < 0) local_area = INFINITY;
        else {
            vector<Point2d> all_contour_coord = rot.outercontour_world;
            all_contour_coord.insert(all_contour_coord.end(), right.convex_hull.begin(), right.convex_hull.end());
            vector<Point> v(all_contour_coord.begin(), all_contour_coord.end());

            convexHull(v, hull);
            local_area = contourArea(hull);
        }

        if (local_area < min_area) {
            min_area = local_area;
            output = rot;
        }
        /*canvas canv(1200, 800, 255);
        canv.draw_all_element(rot, zoom_ratio);
        canv.draw_all_element(left, zoom_ratio);
        canv.draw_convexHull((0, 0, 0), hull, zoom_ratio);
        canv.show("whiteboard", waittime);*/
        rot.shift({ -maxdistance,0 });
        rot.shift({ 0,y_shift });
    }
    return  min_area;
}
contour approach_from_right(group& left, contour right, double cutting_gap, int waittime, int zoom_ratio) {
    right.shift({ left.boundingbox[3].x - right.boundingbox[1].x,0 });
    double maxdistance = INFINITY;
    right.get_boundary();
    for (int i = 0; i < right.left_boundary.size(); i = i++) {
        double y = right.left_boundary[i].y;
        int l = 0;
        int r = left.right_boundary.size() - 1;
        int result = -1;
        int pp = 0;

        while (l <= r) {
            int mid = (l + r) / 2;
            if (mid + 1 > left.right_boundary.size() - 1) break;
            if (left.right_boundary[mid].y <= y && y < left.right_boundary[mid + 1].y) {
                result = mid;
                pp = -1;
                break;
            }
            else if (left.right_boundary[mid].y == y && y == left.right_boundary[mid + 1].y) {
                result = mid;
                pp = 1;
                break;
            }
            if (left.right_boundary[mid].y < y && left.right_boundary[mid + 1].y <= y) {
                l = mid + 1;
            }
            else {
                r = mid - 1;
            }
        }

        if (pp == -1) {
            double a1 = left.right_boundary[result].x;
            double b1 = left.right_boundary[result].y;
            double a2 = left.right_boundary[result + 1].x;
            double b2 = left.right_boundary[result + 1].y;
            double xcross = (y - b1) * (a2 - a1) / (b2 - b1) + a1;
            double local_distance = abs(right.left_boundary[i].x - xcross);
            if (local_distance < maxdistance) {
                maxdistance = local_distance;
            }
        }
        else if (pp == 1) {
            double x_righter = max(left.right_boundary[result].x, left.right_boundary[result + 1].x);
            double local_distance = abs(right.left_boundary[i].x - x_righter);
            if (local_distance < maxdistance) {
                maxdistance = local_distance;
            }
        }
    }
    reverse(right.left_boundary.begin(), right.left_boundary.end());

    for (int i = 0; i < left.right_boundary.size(); i++) {
        double y = left.right_boundary[i].y;
        int l = 0;
        int r = right.left_boundary.size() - 1;
        int result = -1;
        int pp = 0;

        while (l <= r) {
            int mid = (l + r) / 2;
            if (mid + 1 > right.left_boundary.size() - 1) break;
            if (right.left_boundary[mid].y <= y && y < right.left_boundary[mid + 1].y) {
                result = mid;
                pp = -1;
                break;
            }
            else if (right.left_boundary[mid].y == y && y == right.left_boundary[mid + 1].y) {
                result = mid;
                pp = 1;
                break;
            }
            if (right.left_boundary[mid].y < y && right.left_boundary[mid + 1].y <= y) {
                l = mid + 1;
            }
            else {
                r = mid - 1;
            }
        }
        if (pp == -1) {
            double a1 = right.left_boundary[result].x;
            double b1 = right.left_boundary[result].y;
            double a2 = right.left_boundary[result + 1].x;
            double b2 = right.left_boundary[result + 1].y;
            double xcross = (y - b1) * (a2 - a1) / (b2 - b1) + a1;
            double local_distance = abs(left.right_boundary[i].x - xcross);
            if (local_distance < maxdistance) {
                maxdistance = local_distance;
            }
        }
        else if (pp == 1) {
            double x_righter = min(right.left_boundary[result].x, right.left_boundary[result + 1].x);
            double local_distance = abs(left.right_boundary[i].x - x_righter);
            if (local_distance < maxdistance) {
                maxdistance = local_distance;
            }
        }
    }
    right.shift({ -maxdistance + cutting_gap,0 });
    return  right;
}
group  approach_two_group_vertically(group upper, group lower, vector<int> limit, double cutting_gap, double x_shift) {
    upper.shift({ -upper.boundingbox[1].x,lower.boundingbox[0].y - upper.boundingbox[1].y });
    double min_area = INFINITY;
    group output;
    if (lower.upper_boundary[lower.upper_boundary.size() - 1].x < lower.upper_boundary[0].x) reverse(lower.upper_boundary.begin(), lower.upper_boundary.end());
    if (upper.bottom_boundary[upper.bottom_boundary.size() - 1].x < upper.bottom_boundary[0].x) reverse(upper.bottom_boundary.begin(), upper.bottom_boundary.end());
    while (upper.boundingbox[3].x < limit[0]) {
        double maxdistance = INFINITY;
        for (int i = 0; i < upper.bottom_boundary.size(); i++) {
            double x = upper.bottom_boundary[i].x;
            int left = 0;
            int right = lower.upper_boundary.size() - 1;
            int result = -1;
            int pp = 0;
            while (left <= right) {
                int mid = (left + right) / 2;
                if (mid + 1 > lower.upper_boundary.size() - 1) break;
                if (lower.upper_boundary[mid].x <= x && x < lower.upper_boundary[mid + 1].x) {
                    result = mid;
                    pp = -1;
                    break;
                }
                else if (lower.upper_boundary[mid].x == x && x == lower.upper_boundary[mid + 1].x) {
                    result = mid;
                    pp = 1;
                    break;
                }
                if (lower.upper_boundary[mid].x < x && lower.upper_boundary[mid + 1].x <= x) {
                    left = mid + 1;
                }
                else {
                    right = mid - 1;
                }
            }
            if (pp == -1) {
                double a1 = lower.upper_boundary[result].x;
                double b1 = lower.upper_boundary[result].y;
                double a2 = lower.upper_boundary[result + 1].x;
                double b2 = lower.upper_boundary[result + 1].y;
                double ycross = (b2 - b1) / (a2 - a1) * (x - a1) + b1;
                double local_distance = abs(upper.bottom_boundary[i].y - ycross);
                if (local_distance < maxdistance) {
                    maxdistance = local_distance;
                }
            }
            else if (pp == 1) {
                double y_upper = max(lower.upper_boundary[result].y, lower.upper_boundary[result + 1].y);
                double local_distance = abs(upper.bottom_boundary[i].y - y_upper);
                if (local_distance < maxdistance) {
                    maxdistance = local_distance;
                }
            }
        }
        for (int i = 0; i < lower.upper_boundary.size(); i++) {
            double x = lower.upper_boundary[i].x;
            int left = 0;
            int right = upper.bottom_boundary.size() - 1;
            int result = -1;
            int pp = 0;
            while (left <= right) {
                int mid = (left + right) / 2;
                if (mid + 1 > upper.bottom_boundary.size() - 1) break;
                if (upper.bottom_boundary[mid].x <= x && x < upper.bottom_boundary[mid + 1].x) {
                    result = mid;
                    pp = -1;
                    break;
                }
                else if (upper.bottom_boundary[mid].x == x && x == upper.bottom_boundary[mid + 1].x) {
                    result = mid;
                    pp = 1;
                    break;
                }
                if (upper.bottom_boundary[mid].x < x && upper.bottom_boundary[mid + 1].x <= x) {
                    left = mid + 1;
                }
                else {
                    right = mid - 1;
                }
            }

            if (pp == -1) {
                double a1 = upper.bottom_boundary[result].x;
                double b1 = upper.bottom_boundary[result].y;
                double a2 = upper.bottom_boundary[result + 1].x;
                double b2 = upper.bottom_boundary[result + 1].y;
                double ycross = (b2 - b1) / (a2 - a1) * (x - a1) + b1;
                double local_distance = abs(lower.upper_boundary[i].y - ycross);
                if (local_distance < maxdistance) {
                    maxdistance = local_distance;
                }
            }
            else if (pp == 1) {
                double y_upper = min(upper.bottom_boundary[result].y, upper.bottom_boundary[result + 1].y);
                double local_distance = abs(lower.upper_boundary[i].y - y_upper);
                if (local_distance < maxdistance) {
                    maxdistance = local_distance;
                }
            }
        }
        //canvas canv(1200, 800, 255);
        /*canv.draw_all_element(upper, 0.5);
        canv.draw_all_element(lower, 0.5);
        canv.show("whiteboard", NULL);*/
        upper.shift({ 0,-maxdistance + cutting_gap });
        /*canv.draw_all_element(upper, 0.5);
        canv.show("whiteboard", NULL);
        canv.refresh();*/


        double local_area;
        if (upper.boundingbox[0].y > limit[1] || upper.boundingbox[3].x > limit[0]) local_area = INFINITY;
        else {
            local_area = (upper.boundingbox[0].y - upper.boundingbox[1].y) * (upper.boundingbox[3].x - upper.boundingbox[0].x);
            local_area += (lower.boundingbox[0].y - lower.boundingbox[1].y) * (lower.boundingbox[3].x - lower.boundingbox[0].x);
            double overlap_width = min(upper.boundingbox[3].x, lower.boundingbox[3].x) - max(upper.boundingbox[0].x, lower.boundingbox[0].x);
            double overlap_height = min(upper.boundingbox[3].y, lower.boundingbox[3].y) - max(upper.boundingbox[1].y, lower.boundingbox[1].y);
            local_area -= overlap_height * overlap_width;
        }
        if (local_area < min_area) {
            min_area = local_area;
            output = upper;
        }
        upper.shift({ 0,maxdistance - cutting_gap });
        upper.shift({ x_shift,0 });
    }
    return output;
}
container get_all_possible_optimized_group(contour contourB, double material_width, double cutting_gap, int num_of_possibility, double zoom_ratio) {
    vector<double> limit = { material_width,0 };
    canvas canv(1200, 800, 255, 20);
    container all_possible_optimized_group;
    for (int k = 0; k < 5; k++)
    {
        limit = { limit[0],(contourB.boundingbox[0].y - contourB.boundingbox[1].y) * (1 + 0.1 * k) + 1 };
        group possible_optimized_group(limit[0], limit[1], contourB);
        double current_area = 0;
        while (current_area != INFINITY) {
            double min_area = INFINITY;
            contour contourC;
            for (int i = 0; i < 360; i += 180) {
                double area;
                contour contourD;
                vector<Point> min_hull;
                area = approach_from_right_minarea(possible_optimized_group, contourB, i, contourD, limit, cutting_gap);
                if (area < min_area) {
                    min_area = area;
                    contourC = contourD;
                }
            }
            current_area = min_area;
            if (current_area != INFINITY) {
                bool whether_merge = true;
                for (int j = 0; j < possible_optimized_group.contour_inside_boundary.size(); j++) {
                    if (contourC.rotate_angle == possible_optimized_group.contour_inside_boundary[j].rotate_angle
                        && abs(contourC.center.y - possible_optimized_group.contour_inside_boundary[j].center.y) < 0.7) {
                        bool whether_within_limit = true;
                        int repeat_end = possible_optimized_group.contour_inside_boundary.size();
                        int repeat_start = j;
                        while (whether_within_limit) {
                            for (int z = repeat_start; z < repeat_end; z++) {
                                contour cloest = approach_from_right(possible_optimized_group, possible_optimized_group.contour_inside_boundary[z], cutting_gap);
                                group before_merge = possible_optimized_group;
                                if (possible_optimized_group.boundingbox[3].x + cutting_gap > limit[0]) {
                                    whether_within_limit = false;
                                    current_area = INFINITY;
                                    break;
                                }
                                possible_optimized_group.merge(cloest);
                                /*canv.refresh();
                                canv.draw_all_element(possible_optimized_group, zoom_ratio);
                                canv.draw_limit({ possible_optimized_group.xlimit,possible_optimized_group.ylimit }, zoom_ratio);
                                canv.show("whiteboard", 1);*/
                                if (possible_optimized_group.boundingbox[3].x > limit[0]) {
                                    whether_within_limit = false;
                                    current_area = INFINITY;
                                    possible_optimized_group = before_merge;
                                    break;
                                }
                            }

                        }
                        whether_merge = false;
                        break;
                    }
                }
                if (whether_merge) {
                    possible_optimized_group.merge(contourC);
                }
            }
            /*canv.refresh();
            canv.draw_all_element(possible_optimized_group, zoom_ratio);
            canv.draw_limit({ possible_optimized_group.xlimit,possible_optimized_group.ylimit }, zoom_ratio);
            canv.show("whiteboard", 1);*/
        }
        all_possible_optimized_group.group_inside_container.push_back(possible_optimized_group);
    }
    vector<vector<double>> element_per_area_and_index;
    container all_possible_optimized_group_temp;
    for (int k = 0; k < all_possible_optimized_group.group_inside_container.size(); k++) {
        double element_per_area;
        group current_group = all_possible_optimized_group.group_inside_container[k];
        limit = { material_width,10000 };
        container temp_container(limit, current_group);
        temp_container.add_group_vertically(current_group, cutting_gap);
        double box_width = material_width;
        double box_height = temp_container.group_inside_container[1].boundingbox[3].y - temp_container.group_inside_container[0].boundingbox[2].y;
        element_per_area = all_possible_optimized_group.group_inside_container[k].contour_inside_boundary.size() / (box_width * box_height);
        element_per_area_and_index.push_back({ element_per_area,(double)k });
    }
    sort(element_per_area_and_index.begin(), element_per_area_and_index.end(), [](const std::vector<double>& a, const std::vector<double>& b)
        {
            return a[0] > b[0];
        });
    double current_element_per_area = 0;
    int deal_with_same_value = 0;
    for (int i = 0; i < num_of_possibility + deal_with_same_value; i++) {
        if (element_per_area_and_index[i][0] == current_element_per_area) {
            deal_with_same_value++;
            if (deal_with_same_value > 2) deal_with_same_value = 2;
        }
        else if (all_possible_optimized_group_temp.group_inside_container.size() <= 3) {
            current_element_per_area = element_per_area_and_index[i][0];
            all_possible_optimized_group_temp.group_inside_container.push_back(all_possible_optimized_group.group_inside_container[(int)element_per_area_and_index[i][1]]);
        }

    }
    return all_possible_optimized_group_temp;
};
