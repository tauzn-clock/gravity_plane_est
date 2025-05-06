#include <vector>

std::vector<std::array<float,3> > get_normal(std::vector<std::array<float,3> > points, int W, int H){
    
    assert (points.size() == W*H);
    
    std::vector<std::array<float,3> > up(W*H), down(W*H), left(W*H), right(W*H);
    
    for (int i = 0; i < H; ++i) {
        for (int j = 0; j < W; ++j) {
            int index = i * W + j;
            if (i > 0) up[index] = points[index - W];
            if (i < H - 1) down[index] = points[index + W];
            if (j > 0) left[index] = points[index - 1];
            if (j < W - 1) right[index] = points[index + 1];
        }
    }

    std::vector<std::array<float,3> > diff_x(W*H), diff_y(W*H);
    std::vector<std::array<float,3> > normal(W*H);
    for (int i = 0; i < H; ++i) {
        for (int j = 0; j < W; ++j) {
            int index = i * W + j;
            diff_x[index] = {right[index][0] - left[index][0], right[index][1] - left[index][1], right[index][2] - left[index][2]};
            diff_y[index] = {down[index][0] - up[index][0], down[index][1] - up[index][1], down[index][2] - up[index][2]};
            normal[index] = {diff_x[index][1] * diff_y[index][2] - diff_x[index][2] * diff_y[index][1],
                             diff_x[index][2] * diff_y[index][0] - diff_x[index][0] * diff_y[index][2],
                             diff_x[index][0] * diff_y[index][1] - diff_x[index][1] * diff_y[index][0]};
            normal[index] = normalise(normal[index]);
        }
    }

    return normal;
}