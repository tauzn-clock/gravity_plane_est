#include <vector>

std::vector<std::array<float,3> > get_normal(std::vector<std::array<float,3> > points, int W, int H){
    
    assert (points.size() == W*H);
    
    std::vector<std::array<float,3> > up(W*H), down(W*H), left(W*H), right(W*H);
    
    for (int i = 0; i < H; ++i) {
        for (int j = 0; j < W; ++j) {
            int index = i * W + j;
            if (i > 0) up[index] = points[index - W];
            else up[index] = {0, 0, 0};
            if (i < (H - 1)) down[index] = points[index + W];
            else down[index] = {0, 0, 0};
            if (j > 0) left[index] = points[index - 1];
            else left[index] = {0, 0, 0};
            if (j < (W - 1)) right[index] = points[index + 1];
            else right[index] = {0, 0, 0};
        }
    }

    std::array<float,3>  diff_x, diff_y;
    std::vector<std::array<float,3> > normal(W*H);
    for (int i = 0; i < H; ++i) {
        for (int j = 0; j < W; ++j) {
            int index = i * W + j;
            diff_x = {right[index][0] - left[index][0], right[index][1] - left[index][1], right[index][2] - left[index][2]};
            diff_y = {down[index][0] - up[index][0], down[index][1] - up[index][1], down[index][2] - up[index][2]};

            normal[index] = {diff_x[1] * diff_y[2] - diff_x[2] * diff_y[1],
                            diff_x[2] * diff_y[0] - diff_x[0] * diff_y[2],
                            diff_x[0] * diff_y[1] - diff_x[1] * diff_y[0]};
            normal[index] = normalise(normal[index]);
        }
    }

    return normal;
}

void centre_to_hemisphere(std::vector<std::array<float,3> >& normal, std::array<float, 3> grav){    

    for (int i=0; i < normal.size(); ++i){
        if (dot(normal[i],grav) < 0){
            normal[i][0] *= -1;
            normal[i][1] *= -1;
            normal[i][2] *= -1;
        }
    }
    
}