#include "core.h"

int main() {
    double zoom_ratio = 0.3;
    canvas canv(1200, 800, 255,20);
    contour contourB("C://Users/wades/Desktop/OpenSuorce/Contour.txt", { 0,0 });
    contourB.shift({ -contourB.boundingbox[0].x,-contourB.boundingbox[1].y });
    vector<double> limit(2, 0);
    cout << "請輸入材料寬度\n";
    cin >> limit[0];
    double material_width = limit[0];
    cout << "請輸入材料長度\n";
    cin >> limit[1];
    double material_length = limit[1];
    limit = { limit[0],(contourB.boundingbox[0].y - contourB.boundingbox[1].y) + 1 };
    group possible_optimized_group(limit[0], limit[1], contourB);
    double current_area = 0;
    double cutting_gap = 0;
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
            possible_optimized_group.merge(contourC);
        }
        canv.refresh();
        canv.draw_all_element(possible_optimized_group, zoom_ratio);
        canv.draw_limit({ possible_optimized_group.xlimit,possible_optimized_group.ylimit }, zoom_ratio);
        canv.show("whiteboard", 1);
    }
    limit = { material_width,material_length };
    container temp_container(limit, possible_optimized_group);
    int signal = 1;
    while (signal > 0) {
        signal = temp_container.add_group_vertically(possible_optimized_group, cutting_gap);
    }
    canv.refresh();
    canv.draw_all_element(temp_container, zoom_ratio);
    canv.show("whiteboard", NULL);
}